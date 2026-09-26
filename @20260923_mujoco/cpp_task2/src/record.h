// 离屏渲染录像：把帧喂给 ffmpeg 管道（对应 Python 侧 scripts/visualization/mujoco_video.py）。
//
// 做法照官方 sample/record.cc（就在官方发布包的 sample/record.cc，同版本可直接对照）：
//   建隐藏窗口拿 GL 上下文 → mjr_makeContext → mjr_setBuffer(mjFB_OFFSCREEN)
//   → 每帧 mjv_updateScene + mjr_render + mjr_readPixels → 写进 ffmpeg 的 stdin。
//
// 三个注意点：
//   1) mjr_readPixels 出来的行序**自下而上**（OpenGL 约定），交给 ffmpeg 的 -vf vflip 翻，
//      比在 C++ 里手工翻省事、也不会写错 stride；
//   2) 不要在 Linux 上调 glfwTerminate()（官方 record.cc 的注释明说在本机驱动上会崩），
//      收尾只销毁窗口，进程退出时由系统回收；
//   3) 不影响物理：出帧时机按**仿真时间** d->time 决定，改 fps 不用动循环，
//      MP4 的时间轴 = 仿真时间，与机器快慢无关（和 Python 侧同一条规则）。
//
// header-only，直接 #include "record.h"。

#pragma once

#include <mujoco/mujoco.h>

#include <GLFW/glfw3.h>

#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

class OffscreenRecorder {
  public:
    OffscreenRecorder(mjModel *m, const std::string &path, int width, int height, double fps,
                      const std::string &camera)
        : path_(std::filesystem::absolute(path).lexically_normal().string()), fps_(fps) {
        // 离屏 framebuffer 的大小就是模型里的 <visual><global offwidth/offheight>（默认 640x480，
        // 本任务场景声明的是 1280x720）—— 不是隐藏窗口的尺寸。直接把它改成请求的输出尺寸，
        // 渲染就**原生**发生在输出分辨率上，不用再缩放（Python 侧 VideoRecorder 也是这么做的）。
        // 必须在 mjr_makeContext 之前改：offscreen framebuffer 是那时按这两个字段分配的。
        m->vis.global.offwidth = width;
        m->vis.global.offheight = height;
        if (!glfwInit())
            mju_error("glfwInit 失败");
        glfwWindowHint(GLFW_VISIBLE, 0); // 隐藏窗口
        glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_FALSE);
        window_ = glfwCreateWindow(width, height, "offscreen", nullptr, nullptr);
        if (window_ == nullptr)
            mju_error("创建隐藏窗口失败（有显示服务吗？）");
        glfwMakeContextCurrent(window_);

        mjv_defaultCamera(&cam_);
        mjv_defaultOption(&opt_);
        mjv_defaultScene(&scn_);
        mjr_defaultContext(&con_);
        mjv_makeScene(m, &scn_, 2000);
        mjr_makeContext(m, &con_, mjFONTSCALE_150);
        mjr_setBuffer(mjFB_OFFSCREEN, &con_);

        // 按**实际视口**（mjr_maxViewport）告诉 ffmpeg 每帧多少字节：正常情况它等于上面设的请求
        // 尺寸；万一驱动把它夹小了，就再 scale 到输出尺寸，否则帧对不齐。
        viewport_ = mjr_maxViewport(&con_);
        out_width_ = width;
        out_height_ = height;
        rgb_.resize(static_cast<size_t>(3) * viewport_.width * viewport_.height);
        depth_.resize(static_cast<size_t>(viewport_.width) * viewport_.height);

        char scale[64] = "";
        if (viewport_.width != width || viewport_.height != height)
            std::snprintf(scale, sizeof(scale), ",scale=%d:%d", width, height);
        char cmd[1024];
        std::snprintf(cmd, sizeof(cmd),
                      "ffmpeg -y -loglevel error -f rawvideo -pix_fmt rgb24 -s %dx%d -r %g -i -"
                      " -vf vflip%s -c:v libx264 -pix_fmt yuv420p -crf 18 \"%s\"",
                      viewport_.width, viewport_.height, fps, scale, path_.c_str());
        pipe_ = ::popen(cmd, "w");
        if (pipe_ == nullptr)
            mju_error("打不开 ffmpeg 管道（PATH 里有 ffmpeg 吗？）");

        if (camera.empty()) { // 默认：自由相机，等距视角（与 Python 侧 iso 一致）
            cam_.type = mjCAMERA_FREE;
            cam_.azimuth = 135.0;
            cam_.elevation = -20.0;
            cam_.distance = 2.0;
            cam_.lookat[0] = 0.0;
            cam_.lookat[1] = 0.0;
            cam_.lookat[2] = 0.15;
        } else {
            const int id = mj_name2id(m, mjOBJ_CAMERA, camera.c_str());
            if (id < 0)
                mju_error("模型里没有名为「%s」的相机", camera.c_str());
            cam_.type = mjCAMERA_FIXED;
            cam_.fixedcamid = id;
        }
    }

    ~OffscreenRecorder() { Close(); }

    OffscreenRecorder(const OffscreenRecorder &) = delete;
    OffscreenRecorder &operator=(const OffscreenRecorder &) = delete;

    // 到点就出一帧；返回值表示这一帧有没有出（mjv_updateScene 要非 const 的 mjData）。
    // 出帧时刻取**严格网格** k/fps_（第 k 帧对应 k/fps_ 仿真秒），而不是“距上次出帧过了 1/fps_ 就出”：
    // 后者在 1/fps_ 不能整除 dt 时会漂——实测 5 仿真秒 @50 fps 只出 236 帧（4.72 s 的片子，比仿真快 6%）、
    // @120 fps 只出 500 帧（4.17 s，快 20%），MP4 的时间轴就不再等于仿真时间。
    // 代价：帧时刻要对齐到最近的物理步（最多晚一个 dt，帧间仍按 1/fps_ 排布、不累积）；
    // fps_ > 1/dt（本模型 500 fps）时物理步不够用，只能丢掉来不及渲染的网格点（同 Python 侧）。
    bool Capture(const mjModel *m, mjData *d) {
        if (have_frame_ && d->time < next_time_ - 1e-12)
            return false;
        mjv_updateScene(m, d, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
        mjr_render(viewport_, &scn_, &con_);

        char stamp[64];
        std::snprintf(stamp, sizeof(stamp), "t = %.2f s", d->time); // 左上角时间戳
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport_, stamp, nullptr, &con_);

        mjr_readPixels(rgb_.data(), depth_.data(), viewport_, &con_);
        std::fwrite(rgb_.data(), 3, rgb_.size() / 3, pipe_);
        have_frame_ = true;
        ++frames_;
        next_time_ = frames_ / fps_; // 下一帧的仿真时刻按帧数推，不累加，避免浮点误差
        if (next_time_ <= d->time)   // 网格点落在当前时刻之前：跳到前方（别在同一时刻连出两帧）
            next_time_ = d->time + 1.0 / fps_;
        return true;
    }

    void Close() {
        if (pipe_ != nullptr) {
            ::pclose(pipe_); // 等 ffmpeg 收尾（写完索引）
            pipe_ = nullptr;
        }
        if (window_ != nullptr) {
            mjr_freeContext(&con_);
            mjv_freeScene(&scn_);
            glfwDestroyWindow(window_); // 不调 glfwTerminate：Linux 下会崩
            window_ = nullptr;
        }
    }

    int frames() const { return frames_; }
    const std::string &path() const { return path_; }
    int viewport_width() const { return viewport_.width; }
    int viewport_height() const { return viewport_.height; }
    // 视口尺寸与输出尺寸不一致（驱动夹小了 / 调用方另外要求缩放）时为真
    bool scaled() const { return out_width_ != viewport_.width || out_height_ != viewport_.height; }

  private:
    std::string path_;
    double fps_ = 50.0;
    int out_width_ = 0;
    int out_height_ = 0;
    GLFWwindow *window_ = nullptr;
    std::FILE *pipe_ = nullptr;
    mjvCamera cam_;
    mjvOption opt_;
    mjvScene scn_;
    mjrContext con_;
    mjrRect viewport_{};
    std::vector<unsigned char> rgb_;
    std::vector<float> depth_;
    double next_time_ = 0.0; // 下一帧的仿真时刻（严格网格 k/fps_）
    bool have_frame_ = false;
    int frames_ = 0;
};

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
#include <string>
#include <vector>

class OffscreenRecorder {
  public:
    OffscreenRecorder(const mjModel *m, const std::string &path, int width, int height, double fps,
                      const std::string &camera)
        : path_(path), fps_(fps) {
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

        // 关键：离屏缓冲的实际大小由驱动/显示缩放决定（本机 960x540 的窗口会拿到 1280x720 的 framebuffer），
        // 必须按**实际视口**告诉 ffmpeg 每帧多少字节，再用 scale 缩到请求尺寸，否则帧对不齐。
        viewport_ = mjr_maxViewport(&con_);
        rgb_.resize(static_cast<size_t>(3) * viewport_.width * viewport_.height);
        depth_.resize(static_cast<size_t>(viewport_.width) * viewport_.height);

        char cmd[1024];
        std::snprintf(cmd, sizeof(cmd),
                      "ffmpeg -y -loglevel error -f rawvideo -pix_fmt rgb24 -s %dx%d -r %g -i -"
                      " -vf vflip,scale=%d:%d -c:v libx264 -pix_fmt yuv420p -crf 18 \"%s\"",
                      viewport_.width, viewport_.height, fps, width, height, path.c_str());
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

    // 到时间就出一帧；返回值表示这一帧有没有出（mjv_updateScene 要非 const 的 mjData）
    bool Capture(const mjModel *m, mjData *d) {
        if (have_frame_ && d->time - last_time_ <= 1.0 / fps_)
            return false;
        mjv_updateScene(m, d, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
        mjr_render(viewport_, &scn_, &con_);

        char stamp[64];
        std::snprintf(stamp, sizeof(stamp), "t = %.2f s", d->time); // 左上角时间戳
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport_, stamp, nullptr, &con_);

        mjr_readPixels(rgb_.data(), depth_.data(), viewport_, &con_);
        std::fwrite(rgb_.data(), 3, rgb_.size() / 3, pipe_);
        last_time_ = d->time;
        have_frame_ = true;
        ++frames_;
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

  private:
    std::string path_;
    double fps_ = 50.0;
    GLFWwindow *window_ = nullptr;
    std::FILE *pipe_ = nullptr;
    mjvCamera cam_;
    mjvOption opt_;
    mjvScene scn_;
    mjrContext con_;
    mjrRect viewport_{};
    std::vector<unsigned char> rgb_;
    std::vector<float> depth_;
    double last_time_ = 0.0;
    bool have_frame_ = false;
    int frames_ = 0;
};

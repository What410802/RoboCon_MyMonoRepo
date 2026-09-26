// C++ 侧的极简仿真：加载场景 → 零力矩跑 N 秒，默认顺带录像。
// 对标 Python 侧 scripts/simulate_record.py：**不加载 keyframe**（从模型默认位形开始，
// 四脚站在地面上，零力矩下自然塌成趴卧），**不做静止判定**（那是 rest_check.cpp 的事），
// 循环只按**仿真时间**收尾——没有窗口就没有 viewer.is_running() 可用。
//
// 用法：dog_sim [scene.xml] [seconds] [--no-record] [--out FILE] [--fps N] [--width N] [--height N] [--camera NAME]
//   默认：../scenes/flat_scene.xml、4 仿真秒、50 fps、960x540，录到 ../output/cpp_record.mp4。

#include <mujoco/mujoco.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>

#include "record.h"

namespace fs = std::filesystem;

namespace {

const char *kUsage =
    "用法：dog_sim [scene.xml] [seconds] [--no-record] [--out FILE] [--fps N] [--width N] "
    "[--height N] [--camera NAME]\n"
    "  默认 ../scenes/flat_scene.xml、4 仿真秒、50 fps、960x540，录到 ../output/cpp_record.mp4\n";

struct Options {
    fs::path scene; // 空 = 默认场景
    double seconds = 4.0;
    bool record = true;
    fs::path out; // 空 = 默认 output/cpp_record.mp4
    double fps = 50.0;
    int width = 960;
    int height = 540;
    std::string camera; // 空 = 自由相机（等距视角，与 Python 侧 iso 一致）
};

} // namespace

int main(int argc, char **argv) {
    Options opt;
    int positional = 0;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        const auto next = [&](const char *name) -> std::string {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "%s 后面缺参数\n", name);
                std::exit(1);
            }
            return argv[++i];
        };
        if (a == "--no-record") {
            opt.record = false;
        } else if (a == "--out") {
            opt.out = next("--out");
        } else if (a == "--fps") {
            opt.fps = std::atof(next("--fps").c_str());
        } else if (a == "--width") {
            opt.width = std::atoi(next("--width").c_str());
        } else if (a == "--height") {
            opt.height = std::atoi(next("--height").c_str());
        } else if (a == "--camera") {
            opt.camera = next("--camera");
        } else if (a == "--help" || a == "-h") {
            std::printf("%s", kUsage);
            return 0;
        } else if (positional++ == 0) {
            opt.scene = a;
        } else {
            opt.seconds = std::atof(a.c_str());
        }
    }

    // 固定路径：可执行文件应为 <任务目录>/cpp_task2/build/dog_sim，往上两层就是任务目录。
    // （成品代码不做向上搜索；要换构建目录就用第一个参数直接给 scene.xml。）
    const fs::path exe_dir = fs::read_symlink("/proc/self/exe").parent_path();
    const fs::path root = exe_dir.parent_path().parent_path();
    if (opt.scene.empty() && !fs::is_directory(root / "scenes")) {
        std::fprintf(stderr, "预期可执行文件在 <任务目录>/cpp_task2/build/ 下，但 %s 里没有 scenes/\n",
                     root.c_str());
        std::fprintf(stderr, "请用第一个参数指定场景，或按 README 的构建命令重新构建。\n");
        return 1;
    }
    const fs::path scene = opt.scene.empty() ? root / "scenes/flat_scene.xml" : opt.scene;
    const fs::path out = opt.out.empty() ? root / "output/cpp_record.mp4" : opt.out;

    std::printf("MuJoCo %s\n", mj_versionString());
    if (mjVERSION_HEADER != mj_version()) {
        std::fprintf(stderr, "头文件与库版本不一致，终止\n");
        return 1;
    }

    char error[1024] = "";
    mjModel *m = mj_loadXML(scene.c_str(), nullptr, error, sizeof(error));
    if (m == nullptr) {
        std::fprintf(stderr, "加载失败：%s\n%s\n", scene.c_str(), error);
        return 1;
    }
    mjData *d = mj_makeData(m);
    mju_zero(d->ctrl, m->nu); // 零力矩
    std::printf("模型：nq=%ld nv=%ld nu=%ld dt=%g s；零力矩跑 %.1f 仿真秒\n", static_cast<long>(m->nq),
                static_cast<long>(m->nv), static_cast<long>(m->nu), m->opt.timestep, opt.seconds);

    std::unique_ptr<OffscreenRecorder> rec;
    if (opt.record) {
        fs::create_directories(out.parent_path());
        rec = std::make_unique<OffscreenRecorder>(m, out.string(), opt.width, opt.height, opt.fps,
                                                  opt.camera);
        std::printf("录像：%.0f fps → %s（离屏缓冲 %dx%d → 输出 %dx%d）\n", opt.fps, out.c_str(),
                    rec->viewport_width(), rec->viewport_height(), opt.width, opt.height);
    }

    const auto t_start = std::chrono::steady_clock::now();
    int steps = 0;
    while (d->time < opt.seconds - 1e-12) {
        mj_step(m, d);
        if (rec)
            rec->Capture(m, d);
        ++steps;
    }
    const double wall_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_start).count();

    if (rec) {
        std::printf("录像：%d 帧 → %s\n", rec->frames(), rec->path().c_str());
        rec->Close(); // 等 ffmpeg 收尾（写完 moov），否则 MP4 播不了
    }
    std::printf("仿真 %.3f s（%d 步，wall %.1f ms，单步 %.4f ms）\n", d->time, steps, wall_ms,
                wall_ms / std::max(1, steps));

    mj_deleteData(d);
    mj_deleteModel(m);
    return 0;
}

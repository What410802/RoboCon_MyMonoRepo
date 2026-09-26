// C++ 侧的极简仿真：加载场景 → 零力矩跑 N 秒，三种模式。
//   --mode sim    只仿真（无窗口、无录像，全速跑完，用来看吞吐或取数）
//   --mode record 仿真 + 离屏录像（无窗口，写 mp4），对标 Python 侧 scripts/simulate_record.py
//   --mode view   仿真 + MuJoCo 官方 Simulate 窗口（与 Python 的默认窗口是同一个界面，
//                 也就是 unitree_mujoco C++ 里那个窗口），可暂停/调速/换相机；跑到关窗
// 三种模式都**不加载 keyframe**（从模型默认位形开始，四脚站在地面上，零力矩下自然塌成趴卧），
// 也**不做静止判定**（那是 rest_check.cpp 的事）。
// sim/record 两个模式的循环只按**仿真时间**收尾——没有窗口就没有 viewer.is_running() 可用。
//
// 用法：dog_sim [scene.xml] [seconds] [--mode sim|record|view] [--out FILE] [--fps N] [--width N] [--height N] [--camera NAME]
//   默认：../scenes/flat_scene.xml、4 仿真秒、--mode view；record 模式录到 ../output/cpp/cpp_record.mp4。

#include <glfw_adapter.h> // mujoco::GlfwAdapter
#include <mujoco/mujoco.h>
#include <simulate.h> // mujoco::Simulate（官方界面，由 mujoco::libmujoco_simulate 提供）

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include "args.h"
#include "record.h"

namespace fs = std::filesystem;

namespace {

const char *kUsage =
    "用法：dog_sim [scene.xml] [seconds] [--mode sim|record|view] [--out FILE] [--fps N] "
    "[--width N] [--height N] [--camera NAME]\n"
    "  --mode sim    只仿真（无窗口无录像，全速）\n"
    "  --mode record 仿真 + 离屏录像（默认录到 ../output/cpp/cpp_record.mp4）\n"
    "  --mode view   仿真 + MuJoCo 官方 Simulate 窗口（默认模式；时长不限、关窗结束，\n"
    "                给了 seconds 就到那个仿真时刻停止推进，窗口仍开着）\n"
    "  默认 ../scenes/flat_scene.xml；sim/record 默认 4 仿真秒\n";

// 本程序认哪些选项；未知选项、缺值、多余的位置参数都在 args.h 里报错
const std::vector<OptionDef> kOptionDefs = {
    {"--mode", true},
    {"--out", true},
    {"--fps", true},
    {"--width", true},
    {"--height", true},
    {"--camera", true},
};

struct Options {
    fs::path scene;       // 空 = 默认场景
    double seconds = 4.0; // sim/record 的时长；view 下只有显式给了才用
    bool seconds_given = false;
    std::string mode = "view"; // sim / record / view
    fs::path out;              // 空 = 默认 output/cpp/cpp_record.mp4
    double fps = 50.0;
    int width = 960;
    int height = 540;
    std::string camera; // 空 = 自由相机（录像用等距视角，窗口用模型默认视角）
};

// 窗口模式的物理线程：先 Load 把模型交给界面，再把仿真时间轴钉在墙钟上推进（不加速，见下）。
// **Load 会阻塞等渲染线程来接模型**（Simulate::Load 里有个条件变量），所以它必须在主线程的
// RenderLoop() 跑起来之后才调用；在 RenderLoop 之前调，就是开一个空白窗口然后死等。
// 官方 main.cc 也是这个顺序（PhysicsThread 里 Load + mj_forward，PhysicsLoop 里推进），
// 只是它的 PhysicsLoop 还做了更细的同步（busywait、余量补偿等），这里只留我们能用的部分。
void PhysicsThreadView(mujoco::Simulate &sim, mjModel *m, mjData *d, std::atomic<int> &steps,
                       const std::string &filename, double stop_time) {
    using Clock = mujoco::Simulate::Clock;
    using Mutex = std::remove_reference_t<decltype(sim.mtx)>;
    const auto seconds = [](Clock::duration dt) { return std::chrono::duration<double>(dt).count(); };
    constexpr int kSpeeds = static_cast<int>(sizeof(mujoco::Simulate::percentRealTime) /
                                             sizeof(mujoco::Simulate::percentRealTime[0]));

    sim.Load(m, d, filename.c_str()); // 交给界面（这几帧窗口里显示 LOADING...）
    {
        const std::lock_guard<Mutex> lock(sim.mtx);
        mj_forward(m, d); // 与官方一致：先算一遍派生量，免得第一帧是空的
    }
    std::printf("窗口：模型已交给界面（暂停/单步/调速/换相机都在窗口里）\n");

    auto wall0 = Clock::now();
    double sim0 = d->time;
    const auto wall_elapsed = [&] { return seconds(Clock::now() - wall0); };
    while (sim.exitrequest == 0) {
        if (stop_time > 0 && d->time >= stop_time) {
            std::printf("窗口：已到 %.3f 仿真秒（wall %.2f s），物理线程收工（窗口还开着）\n", d->time,
                        wall_elapsed());
            return;
        }
        if (sim.run == 0) { // 界面里按了暂停：这段墙钟不计入节流
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            wall0 = Clock::now();
            sim0 = d->time;
            continue;
        }
        {
            const std::lock_guard<Mutex> lock(sim.mtx);
            mj_step(m, d);
        }
        ++steps;

        // 仿真时间轴钉在墙钟上：跑快了就睡掉差值，落后太多就重新对齐。
        // 界面上最快也只有 100%，这里再钳一层，保证不会比实时快。
        const int idx = std::clamp(sim.real_time_index, 0, kSpeeds - 1);
        const double slowdown = std::min(1.0, mujoco::Simulate::percentRealTime[idx] / 100.0);
        const double ahead = (d->time - sim0) - wall_elapsed() * slowdown; // >0 = 跑到实时前面了
        if (ahead > 0) {
            std::this_thread::sleep_for(std::chrono::duration<double>(ahead));
        } else if (ahead < -0.05) { // 落后超过 50 ms 才重新对齐，不做追赶
            wall0 = Clock::now();
            sim0 = d->time;
        }
        sim.measured_slowdown = static_cast<float>(slowdown);
    }
}

} // namespace

int main(int argc, char **argv) {
    const Args args = ParseArgs(argc, argv, kUsage, kOptionDefs, /*max_positional=*/2);
    Options opt;
    if (args.positional.size() > 0)
        opt.scene = args.positional[0];
    if (args.positional.size() > 1) {
        opt.seconds = Args::ParseNum(args.positional[1], "seconds", kUsage);
        opt.seconds_given = true;
    }
    opt.mode = args.value("--mode", opt.mode);
    opt.out = args.value("--out");
    opt.fps = args.number("--fps", opt.fps, kUsage);
    opt.width = args.integer("--width", opt.width, kUsage);
    opt.height = args.integer("--height", opt.height, kUsage);
    opt.camera = args.value("--camera");
    if (opt.mode != "sim" && opt.mode != "record" && opt.mode != "view") {
        std::fprintf(stderr, "--mode 只能是 sim / record / view：%s\n%s", opt.mode.c_str(), kUsage);
        return 1;
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
    const fs::path out = opt.out.empty() ? root / "output/cpp/cpp_record.mp4" : opt.out;

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
    std::printf("模型：nq=%ld nv=%ld nu=%ld dt=%g s；零力矩", static_cast<long>(m->nq),
                static_cast<long>(m->nv), static_cast<long>(m->nu), m->opt.timestep);
    if (opt.mode == "view" && !opt.seconds_given)
        std::printf("，时长不限（关窗结束）\n");
    else
        std::printf("跑 %.1f 仿真秒\n", opt.seconds);

    std::atomic<int> steps{0};
    const auto t_start = std::chrono::steady_clock::now();

    if (opt.mode == "view") {
        // 官方界面：自己起物理线程（零力矩），主线程跑 RenderLoop（MacOS 要求它在主线程）
        mjvCamera cam;
        mjvOption ui_opt;
        mjvPerturb pert;
        mjv_defaultCamera(&cam);
        mjv_defaultOption(&ui_opt);
        mjv_defaultPerturb(&pert);
        if (opt.camera.empty()) {
            mjv_defaultFreeCamera(m, &cam); // 与 MuJoCo/Python 默认窗口同一个视角
        } else {
            const int id = mj_name2id(m, mjOBJ_CAMERA, opt.camera.c_str());
            if (id < 0)
                mju_error("模型里没有名为「%s」的相机", opt.camera.c_str());
            cam.type = mjCAMERA_FIXED;
            cam.fixedcamid = id;
        }

        auto sim = std::make_unique<mujoco::Simulate>(std::make_unique<mujoco::GlfwAdapter>(), &cam,
                                                      &ui_opt, &pert, /*is_passive=*/false);
        if (opt.seconds_given)
            std::printf("窗口模式：界面里可暂停/单步/调速/换相机；到 %.1f 仿真秒停止推进，关窗结束\n",
                        opt.seconds);
        else
            std::printf("窗口模式：界面里可暂停/单步/调速/换相机；时长不限，关掉窗口才结束\n");
        // 顺序要紧：主线程先跑 RenderLoop，物理线程里再 Load（Load 会等渲染线程）
        std::thread physics(PhysicsThreadView, std::ref(*sim), m, d, std::ref(steps), scene.string(),
                            opt.seconds_given ? opt.seconds : -1.0);
        sim->RenderLoop();    // 阻塞到关窗
        sim->exitrequest = 1; // 通知物理线程收工
        physics.join();
    } else {
        std::unique_ptr<OffscreenRecorder> rec;
        if (opt.mode == "record") {
            fs::create_directories(out.parent_path());
            rec = std::make_unique<OffscreenRecorder>(m, out.string(), opt.width, opt.height, opt.fps,
                                                      opt.camera);
            std::printf("录像：%.0f fps → %s（离屏缓冲 %dx%d → 输出 %dx%d）\n", opt.fps, out.c_str(),
                        rec->viewport_width(), rec->viewport_height(), opt.width, opt.height);
        } else {
            std::printf("只仿真：无窗口无录像，全速跑 %.1f 仿真秒\n", opt.seconds);
        }

        while (d->time < opt.seconds - 1e-12) {
            mj_step(m, d);
            if (rec)
                rec->Capture(m, d);
            ++steps;
        }
        if (rec) {
            std::printf("录像：%d 帧 → %s\n", rec->frames(), rec->path().c_str());
            rec->Close(); // 等 ffmpeg 收尾（写完 moov），否则 MP4 播不了
        }
    }

    const double wall_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_start).count();
    std::printf("仿真 %.3f s（%d 步，wall %.1f ms，单步 %.4f ms）\n", d->time, steps.load(), wall_ms,
                wall_ms / std::max(1, steps.load()));

    mj_deleteData(d);
    mj_deleteModel(m);
    return 0;
}

// 四足稳定站立 demo：本文件是 cpp_task2/src/main.cpp 的**副本 + 改动**（不为共享而抽象，
// 两边各自演进；只复用 args.h / record.h 这两个已经存在的头）。
// 与 main.cpp 的差别：
//   1. 每步先调 stand::StanceController（见 control.h）写 d->ctrl，再 mj_step；
//   2. 跑完打印站立指标（四足触地数、机身高度、竖直度、漂移）并给判定（退出码 0 = 站稳、2 = 未站稳）；
//   3. 目标姿态 = 初始化时**搜**出来的屈膝站姿（搜索不读 keyframe）。
//
// 起点（--start）：
//   raw（默认）不摆任何初始姿态，就照模型加载时的原姿态（qpos0：直立、直腿、脚底刚好触地）
//        开始，控制目标由 --ramp 秒的斜坡从“原姿态关节角”推到站姿（同一个 PD，只是 q_des 随时间动）；
//   stance 直接把狗摆在那条站姿上开始（等于没有起步过程，用于复现早期的数字）；
//   rest   起点换成场景自带的 rest keyframe（趴卧），同样用斜坡起身。
//
// 用法：stand [scene.xml] [seconds] [--mode sim|record|view] [--start raw|stance|rest] [--ramp SEC]
//             [--kp N] [--kd N] [--gravity-comp] [--seconds N] [--out FILE] [--fps N]
//             [--width N] [--height N] [--camera NAME]
//   默认：../scenes/flat_scene.xml、5 仿真秒、--mode view、--start raw、--ramp 1.5、kp=200、kd=5。

#include <glfw_adapter.h> // mujoco::GlfwAdapter
#include <mujoco/mujoco.h>
#include <simulate.h> // mujoco::Simulate（官方界面，由 mujoco::libmujoco_simulate 提供）

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include "args.h"
#include "control.h"
#include "record.h"

namespace fs = std::filesystem;

namespace {

const char *kUsage =
    "用法：stand [scene.xml] [seconds] [--mode sim|record|view] [--start raw|stance|rest] [--ramp SEC]\n"
    "             [--kp N] [--kd N] [--gravity-comp] [--seconds N] [--out FILE] [--fps N]\n"
    "             [--width N] [--height N] [--camera NAME]\n"
    "  --mode sim    只仿真（无窗口无录像），跑完打印站立指标与判定\n"
    "  --mode record 仿真 + 离屏录像（默认录到 ../output/cpp/stand.mp4）\n"
    "  --out FILE    改录像输出路径：不给就用上面那个默认（相对可执行文件的固定位置，与当前目录\n"
    "                无关）；给了就按你写的路径解析（相对*当前目录*），终端里打的是绝对路径\n"
    "  --mode view   仿真 + MuJoCo 官方 Simulate 窗口（默认模式，跑到关窗）\n"
    "  --start raw   起点 = 模型加载时的原姿态（qpos0：直立直腿，不读 keyframe、不摆姿态）；默认\n"
    "  --start stance 起点 = 搜出来的站姿（等于没有起步过程）\n"
    "  --start rest  起点 = 场景自带的 rest keyframe（趴卧）\n"
    "  --ramp SEC    raw / rest 时把 q_des 从起点关节角推到站姿的斜坡时长\n"
    "                （默认：raw 0.1 s、rest 1.5 s；原因见 docs/stand.md）\n"
    "  --gravity-comp 在 PD 之上叠加 qfrc_bias（重力/科氏补偿），增益可以给得更小\n"
    "  目标姿态 = 初始化时搜出来的站姿（搜索不读 keyframe；--start 只决定起点）；kp/kd 对所有关节一样\n";

const std::vector<OptionDef> kOptionDefs = {
    {"--mode", true},    {"--start", true}, {"--ramp", true},  {"--kp", true},
    {"--kd", true},      {"--gravity-comp", false},            {"--seconds", true},
    {"--out", true},     {"--fps", true},    {"--width", true}, {"--height", true},
    {"--camera", true},
};

struct Options {
    fs::path scene;             // 空 = 默认场景
    double seconds = 5.0;
    bool seconds_given = false; // view 模式下只有显式给了才停步
    std::string mode = "view";  // sim / record / view
    std::string start = "raw";  // raw / stance / rest：起点是原姿态、站姿，还是 rest keyframe
    double ramp = 1.5;          // raw / rest 时把 q_des 从起点推到站姿的时长（s）
    double kp = 200.0;
    double kd = 5.0;
    bool gravity_comp = false;
    fs::path out;               // 空 = 默认 output/cpp/stand.mp4
    double fps = 50.0;
    int width = 960;
    int height = 540;
    std::string camera;         // 空 = 自由相机（模型默认视角）
};

// 窗口模式的物理线程（与 main.cpp 同结构）：先 Load 把模型交给界面，再把仿真时间轴钉在墙钟上。
// **Load 会阻塞等渲染线程来接模型**，所以它必须在主线程的 RenderLoop() 跑起来之后才调用。
// 与 main.cpp 的唯一差别：每一步 mj_step 之前先调 control(d) 写 ctrl。
void PhysicsThreadView(mujoco::Simulate &sim, mjModel *m, mjData *d, std::atomic<int> *steps,
                       const std::string &filename, double stop_time,
                       const stand::StanceController &control) {
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

    // 计时分两套，千万别混用（混用过的后果见 ../docs/stand.md 踩坑 9）：
    //   * wall0 / sim0 —— 只给**节流**当基准：界面按暂停、或"落后太多重新对齐"时会重置；
    //   * wall_begin / sim_begin / paused_total —— 只给**报告**用，从线程开始到收工永不重置，
    //     暂停时长单独扣掉（否则在界面里暂停一会儿，"倍实时"会被算成 0.5 以下）。
    auto wall0 = Clock::now();
    double sim0 = d->time;
    const auto wall_elapsed = [&] { return seconds(Clock::now() - wall0); };
    const auto wall_begin = Clock::now();
    const double sim_begin = d->time;
    double paused_total = 0.0;
    auto pause_begin = Clock::now();
    bool paused = false;
    // 报告口径：仿真时长 / "活动墙钟"（扣掉暂停），分子分母同一起点
    const auto wall_active = [&] {
        return seconds(Clock::now() - wall_begin) - paused_total -
               (paused ? seconds(Clock::now() - pause_begin) : 0.0);
    };
    const auto sim_done = [&] { return d->time - sim_begin; };
    while (sim.exitrequest == 0) {
        if (stop_time > 0 && d->time >= stop_time) {
            std::printf("窗口：已到 %.3f 仿真秒（wall %.2f s，%.2f 倍实时），物理线程收工（窗口还开着）\n",
                        sim_done(), wall_active(), sim_done() / std::max(1e-9, wall_active()));
            return;
        }
        if (sim.run == 0) { // 界面里按了暂停：这段墙钟既不计入节流，也不计入"活动墙钟"
            if (!paused) {
                paused = true;
                pause_begin = Clock::now();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            wall0 = Clock::now();
            sim0 = d->time;
            continue;
        }
        if (paused) {
            paused = false;
            paused_total += seconds(Clock::now() - pause_begin);
        }
        {
            const std::lock_guard<Mutex> lock(sim.mtx);
            control(d); // 控制律（与 headless 走的是同一个对象）
            mj_step(m, d);
        }
        if (steps)
            ++*steps;

        // 仿真时间轴钉在墙钟上：跑快了就睡掉差值，落后太多就重新对齐；界面上最快 100%。
        const int idx = std::clamp(sim.real_time_index, 0, kSpeeds - 1);
        const double slowdown = std::min(1.0, mujoco::Simulate::percentRealTime[idx] / 100.0);
        const double ahead = (d->time - sim0) - wall_elapsed() * slowdown; // >0 = 跑到实时前面了
        if (ahead > 0)
            std::this_thread::sleep_for(std::chrono::duration<double>(ahead));
        else if (ahead < -0.05) { // 落后超过 50 ms 才重新对齐，不做追赶
            wall0 = Clock::now();
            sim0 = d->time;
        }
        // 界面里那个 Real-time 百分比读的是 measured_slowdown，官方定义是"墙钟/仿真"（界面显示
        // 100/它，并与下拉框目标值比对、偏差超 10% 就告警），所以要写**实测值**——
        // 之前把目标值塞进去，界面等于永远显示"已对齐"，跟不上也不会告警。
        sim.measured_slowdown = static_cast<float>(wall_active() / std::max(1e-9, sim_done()));
    }
    // 实时率只能在这里报：main 那边的 wall 含开窗、加载与收尾，不是物理线程的墙钟
    std::printf("窗口：物理线程收工（仿真 %.3f s，wall %.2f s，%.2f 倍实时）\n", sim_done(), wall_active(),
                sim_done() / std::max(1e-9, wall_active()));
}

// 站立指标：四足触地数（"机器人 geom ↔ 世界体 geom"的接触，按脚去重）、机身高度、竖直度、水平位移
struct Stance {
    int feet = 0;
    double z = 0;
    double tilt_deg = 0;
    double xy = 0;
};

// 水平位移的参考点（世界系 xy）：默认是原点（= m->qpos0 的 xy，与没加 --start 时的行为逐位一致）；
// --start rest 时会在起身斜坡跑完那一刻改成当时的基座位置——起身过程中必然有几厘米挪动，
// 判定要问的是"站起来之后还漂不漂"；离趴卧起点的总位移另打印。
std::array<double, 2> g_ref_xy{0.0, 0.0};

// “四足触地”只认这 4 个足底碰撞球（contype≠0 的 sphere）：趴卧时躯干/小腿也压在地面上，
// 那些接触不能算"足"——否则 --start rest 时"还没站起来"也会被数成四足触地。
std::vector<int> g_feet;

Stance Measure(const mjModel *m, const mjData *d) {
    Stance s;
    s.z = d->qpos[2];
    double rot[9];
    mju_quat2Mat(rot, d->qpos + 3);
    s.tilt_deg = std::acos(std::clamp(rot[8], -1.0, 1.0)) * 180.0 / M_PI; // 机身 z 轴与世界 z 的夹角
    s.xy = std::hypot(d->qpos[0] - g_ref_xy[0], d->qpos[1] - g_ref_xy[1]);
    std::vector<int> hit;
    for (int c = 0; c < d->ncon; ++c) {
        const int g1 = d->contact[c].geom[0], g2 = d->contact[c].geom[1];
        const bool floor1 = m->geom_bodyid[g1] == 0, floor2 = m->geom_bodyid[g2] == 0;
        if (floor1 == floor2)
            continue; // 只看"机器人 ↔ 地面"的接触
        const int foot = floor1 ? g2 : g1;
        if (std::find(g_feet.begin(), g_feet.end(), foot) == g_feet.end())
            continue; // 不是足底球（躯干/小腿）→ 不算触地
        if (std::find(hit.begin(), hit.end(), foot) == hit.end())
            hit.push_back(foot);
    }
    s.feet = static_cast<int>(hit.size());
    return s;
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
    opt.start = args.value("--start", opt.start);
    // 默认斜坡跟起点绑定：原姿态是"质心在足后"的不平衡位形，得很快把腿收进去（≤0.2 s 才站得住，
    // 实测见 docs/stand.md），所以 raw 默认 0.1 s；趴卧起身可以慢（1.5 s）。显式 --ramp 优先。
    opt.ramp = args.has("--ramp") ? args.number("--ramp", opt.ramp, kUsage)
                                  : (opt.start == "raw" ? 0.1 : 1.5);
    opt.kp = args.number("--kp", opt.kp, kUsage);
    opt.kd = args.number("--kd", opt.kd, kUsage);
    opt.gravity_comp = args.has("--gravity-comp");
    if (args.has("--seconds")) {
        opt.seconds = args.number("--seconds", opt.seconds, kUsage);
        opt.seconds_given = true;
    }
    opt.out = args.value("--out");
    opt.fps = args.number("--fps", opt.fps, kUsage);
    opt.width = args.integer("--width", opt.width, kUsage);
    opt.height = args.integer("--height", opt.height, kUsage);
    opt.camera = args.value("--camera");
    if (opt.mode != "sim" && opt.mode != "record" && opt.mode != "view") {
        std::fprintf(stderr, "--mode 只能是 sim / record / view：%s\n%s", opt.mode.c_str(), kUsage);
        return 1;
    }
    if (opt.start != "raw" && opt.start != "stance" && opt.start != "rest") {
        std::fprintf(stderr, "--start 只能是 raw / stance / rest：%s\n%s", opt.start.c_str(), kUsage);
        return 1;
    }
    if (opt.ramp <= 0.0) {
        std::fprintf(stderr, "--ramp 要正数（秒）：%g\n%s", opt.ramp, kUsage);
        return 1;
    }

    // 固定路径：可执行文件应为 <任务目录>/cpp_stand/build/stand，往上两层就是任务目录。
    const fs::path exe_dir = fs::read_symlink("/proc/self/exe").parent_path();
    const fs::path root = exe_dir.parent_path().parent_path();
    if (opt.scene.empty() && !fs::is_directory(root / "scenes")) {
        std::fprintf(stderr, "预期可执行文件在 <任务目录>/cpp_stand/build/ 下，但 %s 里没有 scenes/\n",
                     root.c_str());
        std::fprintf(stderr, "请用第一个参数指定场景，或按 README 的构建命令重新构建。\n");
        return 1;
    }
    const fs::path scene = opt.scene.empty() ? root / "scenes/flat_scene.xml" : opt.scene;
    const fs::path out = opt.out.empty() ? root / "output/cpp/stand.mp4" : opt.out;

    std::printf("MuJoCo %s\n", mj_versionString());
    std::printf("场景：%s\n", scene.c_str());
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
    std::printf("模型：nq=%ld nv=%ld nu=%ld dt=%g s；控制目标 = 初始化搜出来的站姿（不读 keyframe）\n",
                static_cast<long>(m->nq), static_cast<long>(m->nv), static_cast<long>(m->nu),
                m->opt.timestep);
    double mass = 0;
    for (int b = 1; b < m->nbody; ++b)
        mass += m->body_mass[b];
    std::printf("总质量 %.3f kg；执行器 ctrlrange ±%.0f N·m（nu=%ld）\n", mass,
                m->actuator_ctrlrange[1], static_cast<long>(m->nu));

    // 初始化：站姿生成（不加载 keyframe、也不改 XML）
    //  URDF 导出把"所有关节 = 0"当默认位形，但 0 对膝（calf）越界（例：FL_calf range=[-2.5,-0.85]），
    //  一开场就被限位力踢出去：现象是"与增益无关的崩溃"（实测 kp 100→1500、带不带重力补偿，
    //  末态几乎逐位相同，ctrl 却冲到 200+ N·m）。所以这里直接**搜**一个站得住的姿态：
    //  膝取几个合法弯曲量、大腿在其限位内扫一遍，每次把基座平移到最低脚刚好触地，取
    //  "质心水平投影离四足中心最近"的那一组——站得住的条件就是这个距离足够小。
    //  左右腿符号是镜像的，符号由限位区间决定，不靠猜。
    const int nleg = m->nu / 3;
    std::vector<double> calf_sign(nleg, 1.0);
    std::vector<double> q_stand(m->nu);
    for (int leg = 0; leg < nleg; ++leg) {
        const int j = m->actuator_trnid[2 * (3 * leg + 2)];
        calf_sign[leg] = (m->jnt_range[2 * j + 1] < 0.0) ? -1.0 : 1.0;
    }
    // 脚 = 模型里那 4 个半径为 2 cm 的碰撞球（contype≠0、type=SPHERE；group=1 的视觉球 contype=0）。
    // 直接用几何位置（而不是接触点）才能在"脚还没着地"时也做搜索。
    double foot_radius = 0.0;
    for (int g = 0; g < m->ngeom; ++g) {
        if (m->geom_type[g] == mjGEOM_SPHERE && m->geom_contype[g] != 0) {
            g_feet.push_back(g);
            foot_radius = m->geom_size[3 * g];
        }
    }
    // 当前姿态下：最低脚的球心高度、四足中心的水平位置、质心的水平位置
    const auto probe = [&](double *foot_z, double *foot_xy, double *com_xy) {
        double minz = 1e9, fx = 0, fy = 0;
        for (int g : g_feet) {
            minz = std::min(minz, static_cast<double>(d->geom_xpos[3 * g + 2]));
            fx += d->geom_xpos[3 * g];
            fy += d->geom_xpos[3 * g + 1];
        }
        if (g_feet.empty())
            return false;
        *foot_z = minz;
        foot_xy[0] = fx / g_feet.size();
        foot_xy[1] = fy / g_feet.size();
        com_xy[0] = d->subtree_com[3];
        com_xy[1] = d->subtree_com[4];
        return true;
    };
    const double z0 = d->qpos[2];
    double best_err = 1e9, best_bend = 0, best_frac = 0, best_z = z0;
    std::vector<double> best_q = q_stand;
    for (double bend : {0.9, 1.1, 1.3}) {
        for (double frac = 0.1; frac <= 1.0 + 1e-9; frac += 0.05) {
            for (int leg = 0; leg < nleg; ++leg) {
                for (int k = 0; k < 3; ++k) {
                    const int i = 3 * leg + k;
                    const int j = m->actuator_trnid[2 * i];
                    const int adr = m->jnt_qposadr[j];
                    double v = m->qpos0[adr];                     // 髋：保持默认位形
                    if (k == 2)
                        v = calf_sign[leg] * bend;                // 膝：按限位方向弯
                    if (k == 1)
                        v = -calf_sign[leg] * bend * frac;        // 大腿：配对
                    if (m->jnt_limited[j])
                        v = std::clamp(v, m->jnt_range[2 * j], m->jnt_range[2 * j + 1]);
                    q_stand[i] = v;
                    d->qpos[adr] = v;
                }
            }
            d->qpos[2] = z0;
            mj_forward(m, d);
            double fz = 0, fxy[2] = {0, 0}, cxy[2] = {0, 0};
            if (!probe(&fz, fxy, cxy))
                continue;
            d->qpos[2] = z0 - (fz - foot_radius); // 基座平移到最低脚底面刚好贴地
            mj_forward(m, d);
            double fz2 = 0, fxy2[2] = {0, 0}, cxy2[2] = {0, 0};
            if (!probe(&fz2, fxy2, cxy2))
                continue;
            const double err = std::hypot(cxy2[0] - fxy2[0], cxy2[1] - fxy2[1]);
            if (err < best_err) {
                best_err = err;
                best_bend = bend;
                best_frac = frac;
                best_z = d->qpos[2];
                best_q = q_stand;
            }
        }
    }
    if (best_err < 1e9) {
        q_stand = best_q;
        for (int i = 0; i < m->nu; ++i)
            d->qpos[m->jnt_qposadr[m->actuator_trnid[2 * i]]] = best_q[i];
        d->qpos[2] = best_z;
        mj_forward(m, d);
        double fz = 0, fxy[2] = {0, 0}, cxy[2] = {0, 0};
        probe(&fz, fxy, cxy);
        std::printf("初始化：搜到站姿——膝 %.2f rad、大腿 = %.2f×膝 配对；质心 (%.3f, %.3f) vs 四足中心 "
                    "(%.3f, %.3f)，差 %.3f m；基座 z=%.3f、脚底面 z=%.4f、四足触地 %d\n",
                    best_bend, best_frac, cxy[0], cxy[1], fxy[0], fxy[1], best_err, d->qpos[2],
                    fz - foot_radius, Measure(m, d).feet);
    } else {
        std::printf("初始化：没能搜到站姿（模型里没找到脚球？）\n");
    }
    const double stance_z = d->qpos[2]; // 判定基准：搜到的站姿高度（不是 XML 的默认高度）
    // 诊断：守住站姿所需的静态重力力矩（超限幅就注定站不住）——必须在切到趴卧之前量
    std::printf("站姿静态重力力矩：");
    for (int i = 0; i < m->nu; ++i)
        std::printf("%.1f ", d->qfrc_bias[m->jnt_dofadr[m->actuator_trnid[2 * i]]]);
    std::printf("N·m\n");

    stand::StanceController control(m, q_stand, opt.kp, opt.kd, opt.gravity_comp);
    double start_x = d->qpos[0], start_y = d->qpos[1]; // 起点 xy（raw / rest 时用来算总位移）
    const char *start_label = "站姿";
    // --start 只决定**起点**，控制目标始终是上面那组站姿 q_stand：
    //   stance：不动（搜索的最后一步已经把狗摆成那条站姿）；
    //   raw   ：回到模型加载时的原姿态（qpos0），再把 q_des 从它的关节角斜坡推过去；
    //   rest  ：读场景自带的 rest keyframe（趴卧），同样斜坡推过去。
    if (opt.start == "raw") {
        mj_resetData(m, d); // qpos = qpos0（直立直腿、脚底刚好触地）、qvel/ctrl/time 归零
        start_label = "原姿态（导出默认位形：直腿）";
    } else if (opt.start == "rest") {
        const int key = mj_name2id(m, mjOBJ_KEY, "rest");
        if (key < 0) {
            std::fprintf(stderr, "场景里没有名为 rest 的 keyframe，--start rest 用不了：%s\n",
                         scene.c_str());
            return 1;
        }
        mj_resetDataKeyframe(m, d, key); // 趴卧起点（xy 清零、基座 z≈0.145）
        start_label = "rest keyframe 趴卧";
    }
    if (opt.start != "stance") { // raw / rest：起点不是站姿 ⇒ 用同一个 PD 的目标斜坡走过去
        mju_zero(d->ctrl, m->nu);
        start_x = d->qpos[0];
        start_y = d->qpos[1];
        std::vector<double> q_start(m->nu);
        for (int i = 0; i < m->nu; ++i)
            q_start[i] = d->qpos[m->jnt_qposadr[m->actuator_trnid[2 * i]]];
        control.RampFrom(q_start, opt.ramp, opt.start == "raw" ? "原姿态（直腿）" : "趴卧");
        if (opt.seconds < opt.ramp + 1.0)
            std::printf("注意：--seconds %.2f s 比「斜坡 %.2f s + 末段统计 1 s」还短，"
                        "末段指标会把起步过程也算进去\n",
                        opt.seconds, opt.ramp);
    }
    mj_forward(m, d); // 先算一遍派生量，让起始指标有意义
    const Stance begin = Measure(m, d);
    std::printf("起始（%s）：基座 z=%.4f 竖直度 %.2f° 四足触地 %d\n", start_label, begin.z,
                begin.tilt_deg, begin.feet);

    std::atomic<int> steps{0};
    const auto t_start = std::chrono::steady_clock::now();

    if (opt.mode == "view") {
        // 官方界面：物理线程里跑控制 + mj_step，主线程跑 RenderLoop（MacOS 要求它在主线程）
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
        std::thread physics(PhysicsThreadView, std::ref(*sim), m, d, &steps, scene.string(),
                            opt.seconds_given ? opt.seconds : -1.0, std::cref(control));
        sim->RenderLoop();    // 阻塞到关窗
        sim->exitrequest = 1; // 通知物理线程收工
        physics.join();
        std::printf("窗口模式不做判定（指标要在 --mode sim 下跑）\n");
        mj_deleteData(d);
        mj_deleteModel(m);
        return 0;
    }

    // sim / record：主循环里跑控制
    std::unique_ptr<OffscreenRecorder> rec;
    if (opt.mode == "record") {
        fs::create_directories(out.parent_path());
        rec = std::make_unique<OffscreenRecorder>(m, out.string(), opt.width, opt.height, opt.fps,
                                                  opt.camera);
        char off[96];
        if (rec->scaled())
            std::snprintf(off, sizeof(off), "离屏缓冲 %dx%d → 缩放输出 %dx%d", rec->viewport_width(),
                          rec->viewport_height(), opt.width, opt.height);
        else
            std::snprintf(off, sizeof(off), "离屏缓冲 = 输出 %dx%d", rec->viewport_width(),
                          rec->viewport_height());
        std::printf("录像：%.0f fps → %s（%s）\n", opt.fps, rec->path().c_str(), off);
    } else {
        std::printf("只仿真：无窗口无录像，跑满 CPU\n");
    }

    // 末段（最后 1 仿真秒）统计：高度范围、最大倾斜/位移、最少触地数、最大关节速度
    const double tail = std::max(0.0, opt.seconds - 1.0);
    bool in_tail = false;
    bool ref_pending = (opt.start != "stance"); // 斜坡结束时把水平位移参考点挪到当时的位置
    double z_min = 1e9, z_max = -1e9, tilt_max = 0, xy_max = 0, v_max = 0, ctrl_max = 0;
    int feet_min = 4;
    while (d->time < opt.seconds - 1e-12) {
        control(d); // 每步先写 ctrl，再推进物理
        mj_step(m, d);
        if (rec)
            rec->Capture(m, d);
        ++steps;
        if (ref_pending && d->time >= opt.ramp) { // 起身完成（斜坡跑完）
            g_ref_xy = {d->qpos[0], d->qpos[1]};
            ref_pending = false;
        }
        if (d->time >= tail) {
            in_tail = true;
            const Stance s = Measure(m, d);
            z_min = std::min(z_min, s.z);
            z_max = std::max(z_max, s.z);
            tilt_max = std::max(tilt_max, s.tilt_deg);
            xy_max = std::max(xy_max, s.xy);
            feet_min = std::min(feet_min, s.feet);
            for (int v = 0; v < m->nv; ++v)
                v_max = std::max(v_max, std::fabs(d->qvel[v]));
            for (int a = 0; a < m->nu; ++a)
                ctrl_max = std::max(ctrl_max, std::fabs(d->ctrl[a])); // 打满 = 力矩不够（饱和）
        }
    }
    // 仿真循环到此为止：计时先按下——后面的 rec->Close() 要等 ffmpeg 收尾（写完 moov），
    // 那段不属于仿真循环，掺进来会把"单步耗时/实时率"稀释掉。
    const double wall_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_start).count();
    if (rec) {
        std::printf("录像：%d 帧 → %s\n", rec->frames(), rec->path().c_str());
        rec->Close(); // 等 ffmpeg 收尾（写完 moov），否则 MP4 播不了
    }

    const Stance end = Measure(m, d);
    std::printf("末态：基座 z=%.4f（站姿 %.4f，默认位形 %.4f）竖直度 %.2f° 四足触地 %d 水平位移 %.4f m\n",
                end.z, stance_z, m->qpos0[2], end.tilt_deg, end.feet, end.xy);
    if (opt.start != "stance")
        std::printf("起步：末态离起点（%s）共 %.4f m（上面那个「水平位移」是从斜坡结束那一刻算的）\n",
                    opt.start == "raw" ? "原姿态" : "趴卧",
                    std::hypot(d->qpos[0] - start_x, d->qpos[1] - start_y));
    if (in_tail) {
        std::printf("末 %.0f s：z∈[%.4f, %.4f] 最大倾斜 %.2f° 最大位移 %.4f m 最少触地 %d 最大 |qvel| "
                    "%.3f 最大 |ctrl| %.1f\n",
                    opt.seconds - tail, z_min, z_max, tilt_max, xy_max, feet_min, v_max, ctrl_max);
    }
    std::printf("仿真 %.3f s（%d 步，wall %.1f ms，单步 %.4f ms，%.1fx 实时）\n", d->time, steps.load(),
                wall_ms, wall_ms / std::max(1, steps.load()), 1000.0 * d->time / std::max(1e-9, wall_ms));

    // 判定：末段全程四足触地、高度接近站姿、姿态竖直、没有明显漂移 ⇒ 站住
    // （--start rest 时"漂移"从起身完成那一刻算，所以这条同时也是"站起来之后没乱跑"）
    const bool stand = in_tail && feet_min >= 4 && std::fabs(end.z - stance_z) < 0.03 &&
                       tilt_max < 5.0 && xy_max < 0.03;
    std::printf("判定：%s\n", stand ? "四足站稳 ✓" : "未站稳 ✗");

    mj_deleteData(d);
    mj_deleteModel(m);
    return stand ? 0 : 2; // 与 rest_check 一致的约定：0 = 站稳，2 = 判定不通过
}

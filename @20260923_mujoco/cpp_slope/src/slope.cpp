// 四足在**可调倾斜地面**上站稳：本文件是 cpp_stand/src/stand.cpp 的副本 + 倾斜改动
// （control.h 直接复用 cpp_stand 的那一份，不做公共头提取）。
// 与 stand.cpp 的差别：多两个参数 --pitch/--roll（度）把地面法向转过去；
//   * 狗跟着转同一个旋转 ⇒ 相对几何完全不变，站姿搜索结果照旧成立（初始仍四足着地）；
//   * **重力不动**（不动 m->opt.gravity），所以斜面越陡越难过；
//   * 高度/倾斜/漂移都改成相对新的地面法向计算（水平地面时就退化成原来的 z / 竖直度 / xy）；
//   * **本 demo 不追求站在斜面上**（斜面下滑是正常的），判定只看"有没有翻倒/离地"：
//     末段四足全程触地且机身法向朝上即通过（退出码 0），高度/倾斜/漂移只列出来供参考；
//   * 默认场景是 ../scenes/slope_scene.xml：与 flat_scene.xml 同一个世界，只多一张棋盘格地面
//     纹理，否则"又平又无限又无纹理"的地面从重力水平的相机看过去与倾斜地面几乎一模一样
//     （实测与原因见 ../docs/stand.md）。传 ../scenes/flat_scene.xml 就回到纯色地面。
//   * 与 stand.cpp 一样支持 --start rest：起点换成场景自带的 rest keyframe（趴卧），先把它摆到
//     **斜面上**（朝向跟着地面转、基座放到离平面 lie_h 处，相对几何与水平时一致），再由同一个
//     PD 控制器的 target 斜坡（--ramp 秒）起身；此时判定额外要求"真的站起来了"（末段法向高度
//     不低于站姿 3 cm）——趴卧时也有四足触地，光看触地数分不出"没起来"。
//
// 用法：slope [scene.xml] [seconds] [--mode sim|record|view] [--start stance|rest] [--ramp SEC]
//             [--kp N] [--kd N] [--gravity-comp] [--pitch DEG] [--roll DEG] [--seconds N]
//             [--out FILE] [--fps N] [--width N] [--height N] [--camera NAME]
//   默认：../scenes/slope_scene.xml、5 仿真秒、--mode view、--start stance、--ramp 1.5、kp=200、kd=5、地面水平。

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
#include <cstring>
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
    "用法：slope [scene.xml] [seconds] [--mode sim|record|view] [--start stance|rest] [--ramp SEC]\n"
    "             [--kp N] [--kd N] [--gravity-comp] [--pitch DEG] [--roll DEG] [--seconds N]\n"
    "             [--out FILE] [--fps N] [--width N] [--height N] [--camera NAME]\n"
    "  --mode sim    只仿真（无窗口无录像），跑完打印站立指标与判定\n"
    "  --mode record 仿真 + 离屏录像（默认录到 ../output/cpp/slope_stand.mp4）\n"
    "  --out FILE    改录像输出路径：不给就用上面那个默认（相对可执行文件的固定位置，与当前目录\n"
    "                无关）；给了就按你写的路径解析（相对*当前目录*），终端里打的是绝对路径\n"
    "  --mode view   仿真 + MuJoCo 官方 Simulate 窗口（默认模式，跑到关窗）\n"
    "  --start stance 起点 = 搜出来的站姿（默认；零起步过程）\n"
    "  --start raw   起点 = 模型加载时的原姿态（qpos0：直立直腿），自己摆到斜面上\n"
    "  --start rest  起点 = 场景自带的 rest keyframe（趴卧），刚体旋转摆到斜面上\n"
    "  --ramp SEC    raw / rest 时把 q_des 从起点关节角推到站姿的斜坡时长\n"
    "                （默认：raw 0.1 s、其余 1.5 s；原因见 ../docs/stand.md）\n"
    "  --pitch/--roll  地面绕 y / x 轴倾斜的度数（狗跟着转同一角度，**重力不动**）\n"
    "  --gravity-comp  在 PD 之上叠加 qfrc_bias（重力/科氏补偿），增益可以给得更小\n"
    "  站姿：初始化时搜一个合法屈膝站姿（不加载 keyframe、不改 XML）；kp/kd 对所有关节一样\n";

const std::vector<OptionDef> kOptionDefs = {
    {"--mode", true},    {"--start", true}, {"--ramp", true},  {"--kp", true},
    {"--kd", true},      {"--gravity-comp", false},            {"--pitch", true},
    {"--roll", true},    {"--seconds", true}, {"--out", true}, {"--fps", true},
    {"--width", true},   {"--height", true}, {"--camera", true},
};

struct Options {
    fs::path scene;             // 空 = 默认场景
    double seconds = 5.0;
    bool seconds_given = false; // view 模式下只有显式给了才停步
    std::string mode = "view";  // sim / record / view
    std::string start = "stance"; // raw / stance / rest：斜面默认从站姿起（原姿态在斜坡上更容易摔，见 ../docs/stand.md）
    double ramp = 1.5;             // raw / rest 时把 q_des 从起点推到站姿的时长（s）
    double kp = 200.0;
    double kd = 5.0;
    double pitch_deg = 0.0;     // 地面绕 y 轴倾斜（度）
    double roll_deg = 0.0;      // 地面绕 x 轴倾斜（度）
    bool gravity_comp = false;
    fs::path out;               // 空 = 默认 output/cpp/slope_stand.mp4
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

// 站立指标：四足触地数（"机器人 geom ↔ 世界体 geom"的接触，按脚去重）、相对地面的高度/倾斜/漂移
// 地面由两个全局量描述：法向 g_up 与平面上一点 g_floor_pt（世界系）。默认水平地面。
std::array<double, 3> g_up{0.0, 0.0, 1.0};
std::array<double, 3> g_floor_pt{0.0, 0.0, 0.0};
std::array<double, 3> g_base0{0.0, 0.0, 0.0}; // 初始化结束时的基座位置（切向漂移的基准）

struct Stance {
    int feet = 0;
    double z = 0;
    double tilt_deg = 0;
    double xy = 0;
};

// “四足触地”只认这 4 个足底碰撞球（contype≠0 的 sphere）：趴卧时躯干/小腿也压在地面上，
// 那些接触不能算"足"——否则 --start rest 时"还没站起来"也会被数成四足触地。
std::vector<int> g_feet;

Stance Measure(const mjModel *m, const mjData *d) {
    Stance s;
    // 高度/倾斜都相对当前地面法向算：水平地面时 g_up = +z，就退化成原来的 z 与竖直度。
    s.z = mju_dot3(d->qpos, g_up.data()) - mju_dot3(g_floor_pt.data(), g_up.data());
    double rot[9];
    mju_quat2Mat(rot, d->qpos + 3);
    const double body_up[3] = {rot[2], rot[5], rot[8]}; // 机身 z 轴
    s.tilt_deg = std::acos(std::clamp(mju_dot3(body_up, g_up.data()), -1.0, 1.0)) * 180.0 / M_PI;
    // 漂移只算切向（沿斜面滑动的量），法向的沉降不算漂
    const double dx = d->qpos[0] - g_base0[0], dy = d->qpos[1] - g_base0[1],
                 dz = d->qpos[2] - g_base0[2];
    const double dn = dx * g_up[0] + dy * g_up[1] + dz * g_up[2];
    s.xy = std::hypot(dx - dn * g_up[0], dy - dn * g_up[1], dz - dn * g_up[2]);
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
    opt.ramp = args.has("--ramp") ? args.number("--ramp", opt.ramp, kUsage)
                                  : (opt.start == "raw" ? 0.1 : 1.5); // 同 stand：raw 得很快收腿
    opt.kp = args.number("--kp", opt.kp, kUsage);
    opt.kd = args.number("--kd", opt.kd, kUsage);
    opt.pitch_deg = args.number("--pitch", opt.pitch_deg, kUsage);
    opt.roll_deg = args.number("--roll", opt.roll_deg, kUsage);
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
        std::fprintf(stderr, "预期可执行文件在 <任务目录>/cpp_slope/build/ 下，但 %s 里没有 scenes/\n",
                     root.c_str());
        std::fprintf(stderr, "请用第一个参数指定场景，或按 README 的构建命令重新构建。\n");
        return 1;
    }
    // 默认场景带棋盘格地面：坡度才看得出来（原因见 ../docs/stand.md）
    const fs::path scene = opt.scene.empty() ? root / "scenes/slope_scene.xml" : opt.scene;
    const fs::path out = opt.out.empty() ? root / "output/cpp/slope_stand.mp4" : opt.out;

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
    std::printf("模型：nq=%ld nv=%ld nu=%ld dt=%g s；目标姿态 = 默认位形（m->qpos0）\n",
                static_cast<long>(m->nq), static_cast<long>(m->nv), static_cast<long>(m->nu),
                m->opt.timestep);
    double mass = 0;
    for (int b = 1; b < m->nbody; ++b)
        mass += m->body_mass[b];
    std::printf("总质量 %.3f kg；执行器 ctrlrange ±%.0f N·m（nu=%ld）\n", mass,
                m->actuator_ctrlrange[1], static_cast<long>(m->nu));
    // 地面：先找到它的 geom（记住平面上的一个点），--pitch/--roll 稍后改它的法向
    int floor_geom = mj_name2id(m, mjOBJ_GEOM, "floor");
    if (floor_geom < 0) {
        for (int g = 0; g < m->ngeom; ++g)
            if (m->geom_type[g] == mjGEOM_PLANE) {
                floor_geom = g;
                break;
            }
    }
    if (floor_geom >= 0)
        std::memcpy(g_floor_pt.data(), m->geom_pos + 3 * floor_geom, 3 * sizeof(double));
    // 地面材质：flat_scene.xml 是纯色；slope_scene.xml 多定义了一个棋盘格材质，这里挂上去。
    // 只换材质，geom 的类型/尺寸/摩擦都不动，所以物理与 flat_scene 逐位相同；
    // 这样也不用把场景里的 option/visual/灯光/地面复制一份出来（复制的那份必然会漂）。
    if (floor_geom >= 0) {
        const int floor_mat = mj_name2id(m, mjOBJ_MATERIAL, "floor_grid_mat");
        if (floor_mat >= 0)
            m->geom_matid[floor_geom] = floor_mat;
    }

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
    // 倾斜地面：把 floor 的法向转 (roll, pitch)，同时把狗按同一个旋转摆好——相对几何完全不变
    // （平面与狗一起转），所以刚搜出来的站姿照旧成立，只需沿新法向重新"落"到斜面上。
    // **重力不动**（不动 m->opt.gravity），所以斜面越陡越难过。
    const double pitch = opt.pitch_deg * M_PI / 180.0;
    const double roll = opt.roll_deg * M_PI / 180.0;
    const double euler[3] = {roll, pitch, 0.0}; // 绕 x = roll、绕 y = pitch
    double tilt[4];
    mju_euler2Quat(tilt, euler, "xyz"); // 平面的局部 z 轴就是它的法向；全 0 时就是单位四元数
    // 沿地面法向把"最低的足底球"落到平面上，再轻压一点让四只都进接触。
    // "落面"（站姿起点）与 "原姿态起点" 共用这一段。
    const auto ground_feet = [&] {
        double h_min = 1e9;
        for (int g : g_feet)
            h_min = std::min(h_min, mju_dot3(d->geom_xpos + 3 * g, g_up.data()) -
                                         mju_dot3(g_floor_pt.data(), g_up.data()) - foot_radius);
        for (int k = 0; k < 3; ++k)
            d->qpos[k] += -h_min * g_up[static_cast<size_t>(k)];
        mj_forward(m, d);
        for (int i = 0; i < 40 && Measure(m, d).feet < 4; ++i) {
            for (int k = 0; k < 3; ++k)
                d->qpos[k] -= 0.0005 * g_up[static_cast<size_t>(k)];
            mj_forward(m, d);
        }
    };
    if (pitch != 0.0 || roll != 0.0) {
        if (floor_geom < 0) {
            std::fprintf(stderr, "场景里没有平面地面，--pitch/--roll 用不了\n");
            return 1;
        }
        mju_copy4(m->geom_quat + 4 * floor_geom, tilt);
        // **关键**：光改 geom_quat 是没用的。地面 geom 的局部位姿本来是全零/单位四元数，编译时
        // m->geom_sameframe[geom] 被标成 1，mj_kinematics 里的 mj_local2Global 看到它就直接
        // 抄世界体的位姿，根本不去读 geom_pos/geom_quat（源码：engine_core_smooth.c 的
        // mj_kinematics → mj_local2Global(..., m->geom_sameframe[g])）。
        // 所以必须把 sameframe 也清掉，否则“地面转了”只会存在于我们自己的 g_up 里：
        // 碰撞面、渲染出来的地面都不动，狗只是自己歪了 15°。（踩坑记录见 ../docs/stand.md）
        m->geom_sameframe[floor_geom] = 0;
        mju_copy4(d->qpos + 3, tilt); // 狗跟着转同一朝向
        const double z_axis[3] = {0.0, 0.0, 1.0};
        mju_rotVecQuat(g_up.data(), z_axis, tilt); // 新法向（世界系）
        mj_forward(m, d);
        // 自检：地面 geom 在世界系里的**实际**法向（d->geom_xmat 第三列）必须等于 g_up，
        // 否则上面那一切都是自说自话（这正是曾经发生过的错误）。
        const double *floor_xmat = d->geom_xmat + 9 * floor_geom;
        const double up_dot = floor_xmat[2] * g_up[0] + floor_xmat[5] * g_up[1] +
                              floor_xmat[8] * g_up[2];
        if (up_dot < 1.0 - 1e-9) {
            std::fprintf(stderr,
                         "倾斜地面失败：想要的法向 (%.3f, %.3f, %.3f)，地面实际法向 "
                         "(%.3f, %.3f, %.3f)（dot=%.6f）\n",
                         g_up[0], g_up[1], g_up[2], floor_xmat[2], floor_xmat[5], floor_xmat[8],
                         up_dot);
            return 1;
        }
        // 沿新法向把最低的脚底面落到平面上（再轻压让四只都进接触）
        ground_feet();
        std::printf("地面：法向 (%.3f, %.3f, %.3f)，倾角 %.2f°（pitch %.1f° roll %.1f°）；"
                    "重力不动；地面 geom 实测法向 (%.3f, %.3f, %.3f)\n",
                    g_up[0], g_up[1], g_up[2],
                    std::acos(std::clamp(g_up[2], -1.0, 1.0)) * 180.0 / M_PI, opt.pitch_deg,
                    opt.roll_deg, floor_xmat[2], floor_xmat[5], floor_xmat[8]);
        std::printf("落面后：四足触地 %d\n", Measure(m, d).feet);
    }
    // 判定基准：站姿的"法向高度"（水平地面时就是 z）——必须在把状态改成趴卧之前量
    const double stance_z =
        mju_dot3(d->qpos, g_up.data()) - mju_dot3(g_floor_pt.data(), g_up.data());
    // 诊断：守住站姿所需的静态重力力矩（超限幅就注定站不住）——同样在切到趴卧之前量
    std::printf("站姿静态重力力矩：");
    for (int i = 0; i < m->nu; ++i)
        std::printf("%.1f ", d->qfrc_bias[m->jnt_dofadr[m->actuator_trnid[2 * i]]]);
    std::printf("N·m\n");

    stand::StanceController control(m, q_stand, opt.kp, opt.kd, opt.gravity_comp); // 非 const：RampFrom
    double start_x = d->qpos[0], start_y = d->qpos[1]; // 起点 xy（raw / rest 时用来算总位移）
    const char *start_label = "站姿";
    // --start 只决定**起点**，控制目标始终是上面那组站姿 q_stand：
    //   stance：不动（搜索的最后一步已经把狗摆成那条站姿）；
    //   raw   ：回到模型加载时的原姿态（qpos0：直立直腿、脚底刚好触地），
    //           朝向跟着地面转、再沿法向落面，然后 q_des 从它的关节角斜坡推过去；
    //   rest  ：读场景自带的 rest keyframe（趴卧），**刚体旋转**摆到斜面上：
    //           偏移（相对平面上的点）与朝向一起转——别用"基座到平面的法向距离"去算，
    //           那样算出来是 0.1449·cos(pitch)（15° 时差 5 mm），会把它往平面里压。
    if (opt.start == "raw") {
        mj_resetData(m, d);           // qpos = qpos0（不读任何 keyframe）
        mju_copy4(d->qpos + 3, tilt); // 朝向跟着地面转
        mj_forward(m, d);
        ground_feet();                // 沿法向落到斜面上（水平地面时是恒等操作）
        start_label = "原姿态（导出默认位形：直腿）";
    } else if (opt.start == "rest") {
        const int key = mj_name2id(m, mjOBJ_KEY, "rest");
        if (key < 0) {
            std::fprintf(stderr, "场景里没有名为 rest 的 keyframe，--start rest 用不了：%s\n",
                         scene.c_str());
            return 1;
        }
        mj_resetDataKeyframe(m, d, key); // 趴卧起点
        double offset[3], rotated[3], q_rest[4];
        mju_sub3(offset, d->qpos, g_floor_pt.data()); // 水平时就是 (0, 0, 0.1449)
        mju_copy4(q_rest, d->qpos + 3);               // keyframe 自带的朝向（含那点小偏航）
        mju_mulQuat(d->qpos + 3, tilt, q_rest);       // R ∘ q_rest：狗跟着地面转
        mju_rotVecQuat(rotated, offset, tilt);        // 偏移也跟着转
        mju_add3(d->qpos, rotated, g_floor_pt.data());
        mj_forward(m, d);
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
    // 漂移参考点 = 起点（raw / rest 时在斜坡跑完那一刻会挪到站住的位置，见主循环）
    std::memcpy(g_base0.data(), d->qpos, 3 * sizeof(double));

    mj_forward(m, d); // 先算一遍派生量，让起始指标有意义
    const Stance begin = Measure(m, d);
    std::printf("起始（%s）：基座 z=%.4f 竖直度 %.2f° 四足触地 %d\n", start_label, begin.z,
                begin.tilt_deg, begin.feet);
    // 诊断：守住这个姿态所需的静态重力力矩（超限幅就注定站不住）
    std::printf("静态重力力矩：");
    for (int i = 0; i < m->nu; ++i)
        std::printf("%.1f ", d->qfrc_bias[m->jnt_dofadr[m->actuator_trnid[2 * i]]]);
    std::printf("N·m\n");

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
    bool ref_pending = (opt.start != "stance"); // 斜坡结束时把漂移参考点挪到那时侯的位置
    double z_min = 1e9, z_max = -1e9, tilt_max = 0, xy_max = 0, v_max = 0, ctrl_max = 0;
    int feet_min = 4;
    while (d->time < opt.seconds - 1e-12) {
        control(d); // 每步先写 ctrl，再推进物理
        mj_step(m, d);
        if (rec)
            rec->Capture(m, d);
        ++steps;
        if (ref_pending && d->time >= opt.ramp) { // 起身完成（斜坡跑完）
            std::memcpy(g_base0.data(), d->qpos, 3 * sizeof(double));
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
    std::printf("末态：法向高度 %.4f（站姿 %.4f）相对法向倾斜 %.2f° 四足触地 %d 切向漂移 %.4f m\n",
                end.z, stance_z, end.tilt_deg, end.feet, end.xy);
    if (opt.start != "stance")
        std::printf("起步：末态离起点（%s）共 %.4f m（上面那个「切向漂移」是从斜坡结束那一刻算的）\n",
                    opt.start == "raw" ? "原姿态" : "趴卧",
                    std::hypot(d->qpos[0] - start_x, d->qpos[1] - start_y));
    if (in_tail) {
        std::printf("末 %.0f s：z∈[%.4f, %.4f] 最大倾斜 %.2f° 最大位移 %.4f m 最少触地 %d 最大 |qvel| "
                    "%.3f 最大 |ctrl| %.1f\n",
                    opt.seconds - tail, z_min, z_max, tilt_max, xy_max, feet_min, v_max, ctrl_max);
    }
    std::printf("仿真 %.3f s（%d 步，wall %.1f ms，单步 %.4f ms，%.1fx 实时）\n", d->time, steps.load(),
                wall_ms, wall_ms / std::max(1, steps.load()), 1000.0 * d->time / std::max(1e-9, wall_ms));

    // 判定：本 demo **不追求站在斜面上**（斜面会滑是正常的），只看"有没有翻倒/离地"：
    //  末段全程四足触地、机身法向朝上（没有翻过去）。高度/倾斜/漂移列出来供参考，不参与判定。
    // 例外：起点不是站姿（raw / rest）时额外要求"真的站起来了"——趴卧时也有四足触地（膝收着、
    //  足底仍碰地），光看触地数分不出"没起来"，所以要求末段法向高度不低于站姿 3 cm。
    double up_dot = 0.0;
    {
        double rot[9];
        mju_quat2Mat(rot, d->qpos + 3);
        const double body_up[3] = {rot[2], rot[5], rot[8]};
        up_dot = mju_dot3(body_up, g_up.data());
    }
    const bool stood_up = (opt.start == "stance") || z_min > stance_z - 0.03;
    const bool ok = in_tail && feet_min >= 4 && up_dot > 0.0 && stood_up;
    if (opt.start != "stance")
        std::printf("判定：%s\n", ok ? "站起来了且四足没离地（斜面是否滑动属正常，见漂移列）✓"
                                   : "没站起来、或有脚离地/已翻倒 ✗");
    else
        std::printf("判定：%s\n", ok ? "四足没离地（斜面是否滑动属正常，见漂移列）✓"
                                   : "有脚离地或已翻倒 ✗");

    mj_deleteData(d);
    mj_deleteModel(m);
    return ok ? 0 : 2; // 与 rest_check 一致的约定：0 = 通过，2 = 判定不通过
}

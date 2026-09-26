// 零力矩静止判定：读场景自带的 rest keyframe，零力矩跑 N 秒，打印基座漂移、末段 max|qvel|
// 与接触点数；静止则退出码 0、否则 2（数字与 Python 侧 scripts/agent_scripts/rest_check.py 逐项对照）。
// 只管判定、不录像——要录像用同目录的 main.cpp（可执行文件 dog_sim）。任务 4 的落点在 ../cpp/。
//
// 用法：rest_check [scene.xml] [seconds]
//   省略时用 ../scenes/flat_scene.xml（按可执行文件位置往上固定两层），跑 8 s（ctrl=0）。

#include <mujoco/mujoco.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

// keyframe 名字存在 m->names 里，name_keyadr[i] 是偏移
std::string KeyframeName(const mjModel *m, int i) {
    return m->name_keyadr[i] >= 0 ? std::string(m->names + m->name_keyadr[i]) : std::string();
}

double MaxAbs(const std::vector<mjtNum> &a, const std::vector<mjtNum> &b) {
    double v = 0;
    for (size_t i = 0; i < a.size(); ++i)
        v = std::max(v, std::abs(a[i] - b[i]));
    return v;
}

// 命令行：位置参数 = [scene.xml] [seconds]，其余都是 --flag
const char *kUsage = "用法：rest_check [scene.xml] [seconds]\n";

struct Options {
    fs::path scene; // 空 = 默认场景
    double seconds = 8.0;
};

bool ParseArgs(int argc, char **argv, Options &o) {
    int positional = 0;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--help" || a == "-h") {
            std::printf("%s", kUsage);
            return false;
        } else if (positional++ == 0) {
            o.scene = a;
        } else {
            o.seconds = std::atof(a.c_str());
        }
    }
    return true;
}

} // namespace

int main(int argc, char **argv) {
    std::printf("MuJoCo %s\n", mj_versionString());
    if (mjVERSION_HEADER != mj_version()) {
        std::fprintf(stderr, "头文件与库版本不一致，终止\n");
        return 1;
    }

    Options opt;
    if (!ParseArgs(argc, argv, opt))
        return 0;

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

    const fs::path scene = opt.scene.empty() ? root / "scenes/flat_scene.xml" : fs::path(opt.scene);
    const double seconds = opt.seconds;

    char error[1024] = "";
    mjModel *m = mj_loadXML(scene.c_str(), nullptr, error, sizeof(error));
    if (m == nullptr) {
        std::fprintf(stderr, "加载失败：%s\n%s\n", scene.c_str(), error);
        return 1;
    }
    if (error[0] != '\0')
        std::printf("模型编译警告：%s\n", error);

    mjData *d = mj_makeData(m);

    if (m->nkey > 0) {
        mj_resetDataKeyframe(m, d, 0);
        std::printf("已加载 keyframe「%s」（keyframe 依赖说明见 @20260923_mujoco/README.md）\n",
                    KeyframeName(m, 0).c_str());
    } else {
        mj_resetData(m, d);
        std::printf("模型里没有 keyframe，用默认位形\n");
    }
    mju_zero(d->ctrl, m->nu);

    double total_mass = 0;
    for (int i = 0; i < m->nbody; ++i)
        total_mass += m->body_mass[i];
    std::printf("模型：nbody=%ld nq=%ld nv=%ld nu=%ld ngeom=%ld 总质量=%.4f kg\n",
                static_cast<long>(m->nbody), static_cast<long>(m->nq), static_cast<long>(m->nv),
                static_cast<long>(m->nu), static_cast<long>(m->ngeom), total_mass);

    const std::vector<mjtNum> ref(d->qpos, d->qpos + m->nq);
    const double z0 = ref[2];
    std::vector<mjtNum> at_last(d->qpos, d->qpos + m->nq);
    const double t_last = std::max(0.0, seconds - 1.0);
    bool have_last = false;
    double max_xy_drift = 0;
    int steps = 0;

    const auto t_start = std::chrono::steady_clock::now();
    while (d->time < seconds - 1e-12) {
        if (!have_last && d->time >= t_last) {
            at_last.assign(d->qpos, d->qpos + m->nq);
            have_last = true;
        }
        mj_step(m, d);
        ++steps;
        max_xy_drift = std::max(max_xy_drift, std::hypot(d->qpos[0] - ref[0], d->qpos[1] - ref[1]));
    }
    const auto t_end = std::chrono::steady_clock::now();
    const double wall_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    double max_v = 0;
    for (int i = 0; i < m->nv; ++i)
        max_v = std::max(max_v, std::abs(d->qvel[i]));

    const double last1_xy =
        std::hypot(d->qpos[0] - at_last[0], d->qpos[1] - at_last[1]);
    const double last1_q = MaxAbs(std::vector<mjtNum>(d->qpos, d->qpos + m->nq), at_last);

    std::printf("\n仿真 %.3f s（%d 步，wall %.1f ms，单步 %.4f ms）\n", d->time, steps, wall_ms,
                wall_ms / std::max(1, steps));
    std::printf("  初始基座 z        %.4f m\n", z0);
    std::printf("  末态基座 z        %.4f m\n", d->qpos[2]);
    std::printf("  全程基座 xy 漂移  %.4f m\n", max_xy_drift);
    std::printf("  末 1 s xy 漂移    %.3e m\n", last1_xy);
    std::printf("  末 1 s 最大 qpos 变化 %.3e\n", last1_q);
    std::printf("  末态 max|qvel|    %.3e\n", max_v);
    std::printf("  末态接触点数      %d\n", d->ncon);

    const bool still = max_v < 1e-3 && max_xy_drift < 1e-3;
    std::printf("判定：%s\n", still ? "静止趴住 ✓" : "未静止 ✗");

    mj_deleteData(d);
    mj_deleteModel(m);
    return still ? 0 : 2;
}

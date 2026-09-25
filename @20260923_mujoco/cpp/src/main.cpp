// C++ 侧仿真程序（任务 4）。当前是 Stage 0 的「工具链 spike + 静止判定复现」：
// 先钉死三件事，再往上长成完整程序（结构与线程设计见本目录 README 与
// docs/learn/unitree-mujoco.md）：
//   1) pixi 环境里能编译并链接 conda-forge 的 libmujoco（find_package(mujoco)）；
//   2) 能加载本项目场景（scenes/flat_scene.xml → models/ → meshdir="meshes/" 软链接），
//      即验证「meshdir 相对顶层文件解析」这条规则在 C++ 下同样成立；
//   3) C++ 侧算出的数字与 Python 侧 scripts/agent_scripts/rest_check.py 一致。
//
// 用法：dog_sim [scene.xml] [seconds]
//   省略时用 scenes/flat_scene.xml，跑 8 s（ctrl=0，零力矩）。

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

// 与 Python 侧同一条规则：向上找**同时**含 scenes/ 与 models/ 的目录，不写死层数。
fs::path FindProjectRoot(fs::path start) {
  for (fs::path p = start; !p.empty(); p = p.parent_path()) {
    if (fs::is_directory(p / "scenes") && fs::is_directory(p / "models")) return p;
    if (p == p.parent_path()) break;
  }
  return {};
}

// keyframe 名字存在 m->names 里，name_keyadr[i] 是偏移
std::string KeyframeName(const mjModel* m, int i) {
  return m->name_keyadr[i] >= 0 ? std::string(m->names + m->name_keyadr[i]) : std::string();
}

double MaxAbs(const std::vector<mjtNum>& a, const std::vector<mjtNum>& b) {
  double v = 0;
  for (size_t i = 0; i < a.size(); ++i) v = std::max(v, std::abs(a[i] - b[i]));
  return v;
}

}  // namespace

int main(int argc, char** argv) {
  std::printf("MuJoCo %s\n", mj_versionString());
  if (mjVERSION_HEADER != mj_version()) {
    std::fprintf(stderr, "头文件与库版本不一致，终止\n");
    return 1;
  }

  fs::path exe_dir = fs::read_symlink("/proc/self/exe").parent_path();
  const fs::path root = FindProjectRoot(exe_dir);
  if (root.empty()) {
    std::fprintf(stderr, "找不到项目根目录（应含 scenes/ 与 models/）\n");
    return 1;
  }

  const fs::path scene = argc > 1 ? fs::path(argv[1]) : root / "scenes/flat_scene.xml";
  const double seconds = argc > 2 ? std::atof(argv[2]) : 8.0;

  char error[1024] = "";
  mjModel* m = mj_loadXML(scene.c_str(), nullptr, error, sizeof(error));
  if (m == nullptr) {
    std::fprintf(stderr, "加载失败：%s\n%s\n", scene.c_str(), error);
    return 1;
  }
  if (error[0] != '\0') std::printf("模型编译警告：%s\n", error);

  mjData* d = mj_makeData(m);

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
  for (int i = 0; i < m->nbody; ++i) total_mass += m->body_mass[i];
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
  for (int i = 0; i < m->nv; ++i) max_v = std::max(max_v, std::abs(d->qvel[i]));

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

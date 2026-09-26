# 任务 2 结果：平地 + 零力矩静止趴卧

> 模型怎么来、为什么这么转，见 [`model.md`](model.md)；通用知识（穿模成因、足底最低点怎么求）见 [`../../docs/learn/mujoco.md`](../../docs/learn/mujoco.md) §6.2。

```bash
cd ..                                # 仓库根目录（有 pixi.toml）
pixi run python @20260923_mujoco/scripts/agent_scripts/rest_check.py              # 验证 keyframe 静止
pixi run python @20260923_mujoco/scripts/agent_scripts/rest_check.py --mode drop  # 重新求趴卧姿态
pixi run python @20260923_mujoco/scripts/visualization/examples/example_attach.py  # 录像（需要 ffmpeg）
pixi run python @20260923_mujoco/scripts/simulate_record.py                      # 自己的循环 + 录像
```

实测（`timestep=0.002`，`ctrl` 全 0）：

| 阶段 | 结果 |
|---|---|
| 站立姿态放到地面 | 脚底最低点相对基座 -0.578580 m，基座 z=0.5786 m |
| **默认位形自由落下 8 s**（最简脚本的走法） | 自然塌成四肢收拢的趴卧姿态：基座 z=**0.1449 m**，8 个接触点，末段 max\|qvel\|=3.45e-08 |
| 从收敛姿态（keyframe）重新开始 8 s | **总漂移 0.0000 m，末 1 s 漂移 4.4e-10 m，末段 max\|qvel\|=3.6e-08** → 静止趴住 ✓ |

关键点：**初始姿态不是拍脑袋写的常数**，而是先由脚底球体几何反算出“脚刚好触地”的基座高度，让它自由落下收敛，再把收敛后的 qpos 写回 `../scenes/flat_scene.xml` 的 `<keyframe name="rest">`，这样模型一加载就已经是静态平衡位形，不需要“先掉一下”。

C++ 侧同一判据（`../cpp_task2/`）：`rest_check` 跑 8 s → 末 1 s 漂移 **4.440e-10 m**、末态 max|qvel| 6.06e-09、接触点数 8 → 判“静止趴住 ✓”，与 Python 侧数字一致。用法见任务 [README](../README.md) 的「C++ 程序」。

**A/B 对照**（同一平地、模型默认位形、`ctrl=0`、2 s）—— 证明“穿模”确实是弹跳的根因：

| 场景 | 初始脚底 z | 初始基座 z | 2 s 内最高基座 z | 2 s 内最高 \|qvel\| |
|---|---|---|---|---|
| `flat_scene_raw.xml`（原始导出，`trunk pos="0 0 0"`） | **-0.5786** | 0.0000 | **4.0444** | **49.36** |
| `flat_scene.xml`（补丁后） | -0.0000 | 0.5786 | 0.5786 | 15.86 |

对照脚本：`pixi run python @20260923_mujoco/scripts/agent_scripts/ab_initial_state.py`。

## 录像与截图

产物按**语言**分目录（`output/python/`、`output/cpp/`），每个脚本的默认输出路径就写在脚本里；下表左边相对 `@20260923_mujoco/`，右边是从仓库根执行的产出命令：

| 产物 | 产出命令 |
|---|---|
| `output/python/simulate_record.mp4` | `pixi run python @20260923_mujoco/scripts/simulate_record.py`（无窗口、50 fps、4.0 s） |
| `output/python/example_attach.mp4` | `pixi run python @20260923_mujoco/scripts/visualization/examples/example_attach.py`（把录像接进一个“已有”循环） |
| `output/python/record_with_viewer.mp4` | `pixi run python @20260923_mujoco/scripts/visualization/examples/example_with_viewer.py`（一边开窗口一边录；10 fps，否则跟不上实时） |
| `output/python/preview_iso.png`、`output/python/preview_side.png` | `pixi run python @20260923_mujoco/scripts/visualization/render_preview.py [--camera iso\|side]` |
| `output/cpp/cpp_record.mp4` | `pixi run @20260923_mujoco/cpp_task2/build/dog_sim`（C++ 离屏渲染 → ffmpeg；4 仿真秒、50 fps、**191 帧 / 3.82 s / 960×540**，画面为全程趴卧） |

# 模型来源与 URDF → MJCF 转换

> 通用规则（`<include>` 之后 `meshdir` 怎么解析、`<keyframe>` 不会自动加载、足底最低点怎么求）在 [`../../docs/learn/mujoco.md`](../../docs/learn/mujoco.md) §6.1 / §6.2；本文只记本任务的模型来源、取舍与具体数字。

## 来源

原始文件：`N-W-wolf/Training_Materials@main: 第二次培训/black/black_description.urdf`（`N-W-wolf` 下的四足组培训资料仓库；`rl_sar-black-W` 中另有一份不同版本的 `src/robots/black_description/urdf/black_description.urdf`）。

`assets/urdf/` 里的副本已用 sha256 与上游核对一致：`e337a9d619154b98c0484bf4c41e20dd44b48c877d64916a16cb664b4cd12f26`。

## 两条路线的差异

URDF→MJCF 有两条现成路线：网站（urdf.enkeebot.com）导出，以及用 MuJoCo 自带的 `mj_saveLastXML` / `MjSpec.to_xml` 转。**两者的产物差别很大**，本任务用的是前者；下面列清差异，便于以后再换模型时判断该走哪条（两条路线都还需要自己组装平地场景与自由基座，都不是“开箱可跑”）。

| 模型 | 体积(B) | nbody | njnt | 自由关节 | nq | nv | nu | ngeom | nmesh | nmat | 总质量(kg) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `black_description.urdf`（MuJoCo 直接读） | 18765 | 13 | 12 | 0 | 12 | 12 | 0 | 30 | 0 | 0 | **7.4700** |
| 网站旧导出 `…skeleton.xml`（现 .bak） | 9378 | 20 | 13 | **1** | 19 | 18 | 12 | 19 | 14 | 0 | 13.2472 |
| 网站旧导出 `…description.xml`（现 .bak，默认选项） | 17971 | 20 | **12** | **0** | 12 | 12 | 12 | 49 | 14 | 19 | 13.2472 |
| 网站新导出 `black_description.xml`（Floating Base ON + Torque） | 17942 | 20 | 13 | **1** | 19 | 18 | 12 | 49 | 14 | 19 | 13.2472 |
| MuJoCo `saveLastXML` / `spec` | 7786 | 13 | 12 | 0 | 12 | 12 | 0 | 30 | 0 | 0 | **7.4700** |

（复现：`pixi run python @20260923_mujoco/scripts/agent_scripts/compare_mjcf.py <文件...>`）

表里最后一行**不入库**：它可由脚本一键复现，而且两种方法的输出**逐字节相同**（sha256 `8bc83581660f9d494f932c3795446c0ad1fb66c2296e68feae78677970a58d2e`）：

```bash
pixi run python @20260923_mujoco/scripts/agent_scripts/urdf_to_mjcf.py \
    @20260923_mujoco/assets/urdf/black_description.urdf /tmp/converted.xml   # 也可加 --method spec
```

**MuJoCo 自带转换的特点（对照用）：**

1. 只导入 URDF 的 `<collision>`，**完全丢弃 `<visual>` mesh**，因此没有 `<asset>`、没有材质；
2. **根 link（`trunk`）被并入 `worldbody`**：基座被“焊死”在世界上，还丢掉了基座的质量与惯量（总质量 13.2472 → 7.47 kg，差值 5.7772 kg 正好是 trunk+imu_link+d435_link）——用于四足仿真时必须自己补一个带 `<freejoint/>` 的 body；
3. `fixed` 关节的子 link 被合并（4 个 `*_foot` 的质量并入对应 `*_calf`）；
4. 没有 actuator（URDF 里本来也没有执行器概念）；
5. 会把 URDF 的 `effort` 限制转成 `actuatorfrcrange`。

**网站转换的特点：**

1. 视觉 mesh 作为 `group="1"`、`contype=0 conaffinity=0 mass=0` 的“只显示不碰撞”geom 保留；collision 作为 `group="3"` 的 box/cylinder/sphere；
2. 每个 link 的 `<inertial>` 完整保留 ⇒ 总质量与 URDF 一致；
3. 默认选项的导出（现 `assets/black_description.bak/black_description.xml`）会生成 12 个 `<position>`（位置伺服，kp=10）、**且没有 `<freejoint/>`**；
4. `…_skeleton.xml`（现也在 `.bak/`）是“去掉碰撞体的骨架版”：带 `<freejoint name="trunk"/>`、`<default>` 里是 `<motor ctrlrange="-1 1"/>`（力矩模式，但 ±1 N·m 太小），且**所有 geom 都无碰撞**，只能当参考模板；
5. 按正确选项重新导出（Floating Base ON + Torque + 骨架关）得到现在的 `assets/black_description/black_description.xml`：freejoint + 12 个 `<motor gear="1" ctrlrange="-20 20">` + 完整视觉/碰撞几何 + 材质，总质量 13.2472 kg。

**结论**：本任务选网站导出（几何/惯量保真度明显更好：保留 mesh 与原始惯性）；两条路线都**不会**自动给出能直接跑的“平地 + 自由基座 + 力矩电机”模型，这部分需要自己组装（见 `../scenes/` 与 `../models/`）。

## 网站导出选项怎么选（实测结论）

`urdf.enkeebot.com` 的选项里，只要调对三个开关，就能得到「可直接用」的模型：

| 选项 | 取值 | 为什么 |
|---|---|---|
| Floating Base | **开** | 否则根 body 被焊死在 world（nq=12、无 `<freejoint/>`） |
| Actuator Type | **Torque** | 得到 `<motor ctrlrange="-20 20" gear="1">`（力矩模式，限幅已与 URDF `effort=20` 一致）；Position 会给 `<position kp=10>` |
| Include Skeleton | **关** | 骨架版 **19 个 geom 全是 `contype=0 conaffinity=0`（完全无碰撞）**，拿它做“趴在地上”会直接穿过地面 |
| 其余（Mesh Directory / Mesh Format / STL Quality / Shared Mesh Reuse） | 默认 | 保持相对路径与最高保真 |

导出后需要手工处理的只剩**一件事**，已固化成可复现补丁 `../scripts/onetime_tools/measure_and_fix_base_height.py`：

1. 基座默认高度 `pos="0 0 0"` → `pos="0 0 0.578580"`（本模型的具体数字），理由见下面的穿模；
   这个高度**不是硬编码常数**：脚本用脚底球体几何实时算出（算式见 [`../../docs/learn/mujoco.md`](../../docs/learn/mujoco.md) §6.2），最后重新加载产物断言“默认状态脚底 z ≥ 0”。

**`meshdir` 不改**：导出自带 `meshdir="meshes/"`，网格重复问题用**目录软链接**解决：真实 STL 只在 `assets/urdf/meshes` 存一份，每个会用到它的目录（`assets/black_description/`、`models/`、`scenes/`）各放一个 `meshes` 软链接；`measure_and_fix_base_height.py` 会把这三个链接建好（缺失就补，指向不对就报错）。为什么 `scenes/` 也要一个，见下面那条坑。

场景（地面/灯光）仍需自己写 —— 见 `../scenes/flat_scene.xml`（正常仿真）与 `../scenes/flat_scene_raw.xml`（对照用，直接 include 原始导出）。

## 本模型遇到的两条坑

### 1. `<include>` 之后 `meshdir` 是相对**顶层文件**解析的

通用规则与实测见 [`../../docs/learn/mujoco.md`](../../docs/learn/mujoco.md) §6.1。落到本模型：`scenes/flat_scene.xml` 去 include 模型、模型写 `meshdir="meshes/"` 时，会去找 `scenes/meshes/...` 而报错；同一个模型单独加载却正常。**所以 `scenes/` 旁也必须有一个 `meshes` 软链接**（这就是为什么三个目录都有）。

### 2. 默认位形就穿模，求解器会把狗弹飞

导出把 `trunk` 放在 `pos="0 0 0"`，而零位形下**脚底在基座下方 0.5786 m**，于是整只狗沉进地面，求解器第一步释放穿透恢复力、把狗一下弹到空中（量化见 [`task2.md`](task2.md) 的 A/B 表）。

修好之后两种起手方式都可用（模型已把基座抬到触地高度）：

* 默认位形 → 四脚站在地面上，零力矩下**自然塌成趴卧**（`../scripts/visualization/examples/example_attach.py` 默认录的就是这一段，C++ 侧 `../cpp_task2/src/main.cpp` 同理）；
* 想一开始就是静止趴卧 → `mujoco.mj_resetDataKeyframe(model, data, 0)`（对应 `scenes/flat_scene.xml` 里的 `<keyframe name="rest">`）。

**关于 `rest` keyframe**（2026-09-25 复核）：它还在、也还有用——`rest_check.py` 默认模式、`example_attach.py --start rest`、`render_preview.py` 都在读它；重跑一次自由落体（`rest_check.py --mode drop`）得到的 qpos 与它**逐位一致**，说明没有过期。它只写了 `qpos`（`qvel` 默认 0），且 xy 清零（平面上平移等价）。**什么时候要重做**：改了模型的惯性/几何/执行器，或动了 `timestep`/`solver` 导致平衡位形变化时，重跑 `--mode drop` 把新 qpos 粘回去；默认模式（`keyframe`）就是它的回归测试（要求末段 max|qvel| < 1e-3 且漂移 < 1 mm）。

另外，`inertiafromgeom="auto"` 这里**刻意不改**：它只在 body 没有显式 `<inertial>` 时才生效，而本模型 20 个 body 全都有 ⇒ 改了也不会有任何行为差异，属于纯防御性改动，只会让补丁清单变长、diff 变脏。

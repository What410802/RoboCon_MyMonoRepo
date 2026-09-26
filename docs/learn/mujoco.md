# MuJoCo 知识点与坑点

> 环境：**mujoco 3.12.0**（conda-forge，经仓库根的 `pixi.toml` 管理）。 本文中标「实测」的结论都是跑出来的，不是抄文档；复现命令见文末。 相关任务记录：[`@20260923_mujoco/README.md`](../../@20260923_mujoco/README.md) 图形栈（OpenGL / Skia / EGL / GLFW 都是什么）：[`graphics-stack.md`](graphics-stack.md) 本机的环境搭建与踩坑（pixi 环境、镜像、显卡与后端选择、编辑器提示）：[`../pitfalls/environment.md`](../pitfalls/environment.md) 上游 `unitree_mujoco` 研读笔记：[`unitree-mujoco.md`](unitree-mujoco.md)；文档索引见 [`../../README.md`](../../README.md)。

**阅读顺序建议**：§1（两大对象）→ §2~§5（XML 属性速查）→ §6（坑点）→ §7（渲染）→ §8（复现命令）。

---

## 1. 两个核心对象：`MjModel` 与 `MjData`

MuJoCo 的 API 只有两个主角：

* **`MjModel`：模型的「结构描述」** —— 由 XML 编译而来，每个数组的长度都固定，`mj_step` **不会**改它；
* **`MjData`：运行时的「状态 + 草稿纸」** —— 每步都在变：广义坐标/速度、接触点、求解器中间量。

```python
model = mujoco.MjModel.from_xml_path("scene.xml")   # 编译一次（贵）
data  = mujoco.MjData(model)                        # 按 model 的尺寸分配（便宜，可以开很多份）
mujoco.mj_step(model, data)                         # 几乎所有 API 都要同时传这两个
```

这种「结构 / 状态分离」是 MuJoCo 的设计核心：同一份 `MjModel` 可以配多份 `MjData`（并行跑多条 轨迹、随机重置），模型本身也可序列化/复用。

### 1.1 两大对象的规模，以及「平行数组」风格

| 对象 | 成员数（本机 3.12.0 实测） |
|---|---|
| `MjModel` | **622** |
| `MjData` | **215** |

它们绝大多数不是「一个对象列表」，而是**平行数组（SoA, structure-of-arrays）**：没有 `bodies[0].mass`，而是 `model.body_mass[0]`；没有 `joints[0].type`，而是 `model.jnt_type[0]`。 C 层靠这个拿到连续内存、方便向量化；代价是初学时看着发散（记住「**字段名_属性名**」 这个命名习惯就好读了：`body_mass`、`jnt_range`、`geom_size`、`actuator_ctrlrange`）。

想看完整字段表（含类型与头文件注释），可以直接让 MuJoCo 把自己吐出来：

```bash
pixi run python -c "import mujoco; print(mujoco.mj_printSchema(0, 1))" | less      # 纯文本
pixi run python -c "import mujoco; print(mujoco.mj_printSchema(1, 0))" > schema.html  # HTML
```

### 1.2 `MjModel` 中值得先记住的字段

**（1）尺寸类 —— 永远是第一个该看的**（括号内是本仓库 `scenes/flat_scene.xml` 的实测值）

| 字段 | 含义 | 实测 |
|---|---|---|
| `nbody` | body 个数（含虚拟的 `world`） | 20 |
| `njnt` | 关节数 | 13（1 free + 12 hinge） |
| `nq` | **广义坐标数** = `len(qpos)` | 19 |
| `nv` | **自由度（广义速度）数** = `len(qvel)` | 18 |
| `nu` | 执行器数 = `len(ctrl)` | 12 |
| `na` | 执行器内部状态数 = `len(act)` | 0 |
| `ngeom` | geom 数（含地面） | 50 |
| `nmesh` / `nmat` / `ntex` | 网格 / 材质 / 贴图 | 14 / 19 / 0 |
| `nkey` | `<keyframe>` 个数 | 1 |
| `nsite` / `nsensor` / `nsensordata` | site / 传感器 | 0 / 0 / 0 |
| `nmocap` | mocap body 数 | 0 |
| `npair` | 显式接触对 `<pair>` 数 | 0 |

> **`nq` 为什么不等于 `nv`？** 见 §1.5 —— 只要模型里有四元数关节（free / ball）， `nq` 就会比 `nv` 各多 1。

**（2）结构 / 几何 / 惯性 / 执行器**

| 字段 | 形状 | 含义 |
|---|---|---|
| `body_parentid` | `(nbody,)` | 运动树拓扑：每个 body 的父 id（`world` 是 0） |
| `body_pos` / `body_quat` | `(nbody,3)` / `(nbody,4)` | **相对父体**的位姿（不是世界坐标！世界坐标看 `data.xpos`） |
| `body_mass` / `body_inertia` / `body_ipos` / `body_iquat` | `(nbody,)` / `(nbody,3)` ×3 | 惯性参数（body 坐标系） |
| `jnt_type` / `jnt_axis` / `jnt_pos` / `jnt_range` / `jnt_bodyid` | `(njnt,…)` | 关节类型（`mjtJoint`）/ 轴 / 位置 / 范围 / 所属 body |
| `geom_type` / `geom_size` / `geom_friction` / `geom_condim` / `geom_contype` / `geom_conaffinity` | `(ngeom,…)` | 几何与接触参数（见 §2~§5） |
| `actuator_trnid` / `actuator_gear` / `actuator_ctrlrange` / `actuator_gainprm` / `actuator_biasprm` | `(nu,…)` | 执行器：作用对象 / 传动比 / 控制范围 / 增益 / 偏置 |
| `key_qpos` / `key_qvel` | `(nkey,nq)` / `(nkey,nv)` | `<keyframe>` 的内容（**不会自动加载**，见 §6.2） |

**（3）选项与可视化**

* `model.opt`：`timestep`、`gravity`、`integrator`、`solver`、`iterations`、`o_friction` … （本模型：`0.002` / `[0,0,-9.81]` / `implicitfast` / `Newton` / `100`）
* `model.vis`：离屏尺寸 `global_.offwidth/offheight`（本模型 1280×720）、机位、灯光、各类渲染开关
* `model.stat`：位置/质心/包围盒等统计量

### 1.3 `MjData` 中值得先记住的字段

**（1）时间与状态**

| 字段 | 形状 | 含义 |
|---|---|---|
| `time` | 标量 | 仿真时间（每个 `mj_step` 加一个 `timestep`） |
| `qpos` | `(nq,)` | 广义坐标（freejoint 的 7 个量就写在这儿；本模型 `qpos[:3]` 就是基座位置） |
| `qvel` | `(nv,)` | 广义速度（不含四元数分量，所以比 qpos 短） |
| `qacc` | `(nv,)` | 广义加速度 |
| `ctrl` | `(nu,)` | **控制输入**（本模型 12 个 `<motor>`，即关节力矩） |
| `act` | `(na,)` | 执行器内部状态（仅带 activation/dynamics 的执行器才有；本模型 `(0,)`） |
| `qfrc_actuator` | `(nv,)` | 执行器实际施加的广义力（`gear` 已乘进去） |
| `xpos` / `xquat` / `xmat` | `(nbody,3)` / `(nbody,4)` / `(nbody,9)` | **世界坐标**下每个 body 的位姿（与 `body_pos` 相对父体要分清） |
| `xipos` / `ximat` | `(nbody,3)` / `(nbody,9)` | 惯性参考点的世界位姿 |
| `cvel` | `(nbody,6)` | body 质心的线速度 + 角速度（世界系） |
| `subtree_com` | `(nbody,3)` | 子树质心（算总质心、重力力矩用） |
| `mocap_pos` / `mocap_quat` | `(nmocap,3)` / `(nmocap,4)` | mocap body 的位姿（人为写入，不受仿真影响） |

**（2）接触与约束 —— 求解器的产物**

| 字段 | 含义 | 实测 |
|---|---|---|
| `ncon` | 当前接触点个数 | 默认位形 4；趴卧 keyframe 8 |
| `contact` | 长度 `ncon` 的 `mjContact`：`dist`（**负值代表穿透深度**）、`pos`、`frame`、`geom1`/`geom2`、`dim`、摩擦系数… | `dist=-0.0000000`、`dim=3` |
| `nefc` | 标量约束个数（接触 + 关节限位 + 等式约束…） | 默认 20；keyframe 44 |
| `solver_niter` | 本次求解实际迭代次数（上限 `opt.iterations`） | 默认 3；keyframe 1 |
| `efc_J` / `efc_b` / `efc_force` / `efc_type` / `efc_pos` / `efc_vel` | 约束的雅可比 / 偏置 / 约束力 / 类型 / 位置与速度误差 | — |
| `qfrc_constraint` / `qfrc_bias` / `qfrc_passive` | 约束力 / 科氏离心项 / 被动力（阻尼、弹簧） | — |

**（3）缓存与「草稿纸」**

`M`（质量矩阵，**稀疏格式**：配合 `M_rowadr`/`M_colind`；旧版本里叫 `qM`）、`qLD`/`qLDiagInv`、 `qacc_warmstart`、`cinert`/`crb`/`cdof`/`cacc`、 `cfrc_int`/`cfrc_ext`、`efc_AR`/`efm_*`（约束求解的分解结果）…… 这些是给求解器复用的内存，平时不用碰，但**它们就是 `MjData` 为什么这么大、为什么每步 `mj_step` 要传同一个 `data` 的原因**。

**（4）警告：MuJoCo 不抛异常，它记账**

```python
print(data.warning)   # 7 个条目，各有 lastinfo（最近一次的 id）与 number（累计次数）
```

实测的 7 种警告：`mjWARN_BADQPOS`、`mjWARN_BADQVEL`、`mjWARN_BADQACC`、`mjWARN_BADCTRL`、 `mjWARN_CONTACTFULL`、`mjWARN_CNSTRFULL`、`mjWARN_INERTIA`。 遇到 NaN / 越界 / 非法数值时，MuJoCo **不会报错中断**，而是把非法值换掉、把计数加一。 所以「仿真怎么不动了」的第一件事就是看 `data.warning`。

### 1.4 `mj_forward` / `mj_step` / 直接改字段：谁更新什么

| | 做了什么 | 什么时候用 |
|---|---|---|
| `mj_forward(model, data)` | 正向计算一轮：运动学 + 碰撞 + 约束求解 + 加速度。**不推进时间** | 改完 `qpos`/`ctrl` 后想立刻看 `xpos`/`ncon`/`qacc` |
| `mj_step(model, data)` | `forward` + 积分一步，`time += timestep` | 仿真主循环 |
| `mj_resetData` / `mj_resetDataKeyframe` | 重置状态（后者加载第 i 个 keyframe） | 起手 / 实验前复位 |
| 直接写 `data.qpos[...]`、`data.ctrl[...]` | 可以，但**派生量还是旧的**，直到下一次 `forward`/`step` | 设初始姿态、给控制量 |

两个小坑：每步「先 `forward` 再 `step`」是重复计算；`mj_step` 之后 `data.time` 已经前进了， 用 `while data.time < T` 做循环条件时不用自己加时间。

**改字段还要分清「引擎什么时候读它」**：`opt.gravity`、`geom_friction`、`geom_matid` 这类是每步/每帧现读的，加载后改了立刻生效（实测：运行时换掉地面的 `geom_matid` 就能把纹理换掉）；但**位姿类**的 `geom_pos`/`geom_quat` 会被编译期定下的 `geom_sameframe` 挡住——同一个 geom 上「改了没反应」与「改了生效」都可能，见 §6.7。

### 1.5 广义坐标 vs 广义速度：`nq` 与 `nv` 为什么会不一样

**会，而且只要模型里有「四元数关节」就一定会不一样。** 实测各关节类型的贡献：

| 关节类型 | `nq` 贡献 | `nv` 贡献 | 差 |
|---|---|---|---|
| `free`（`<freejoint/>`） | **7**（3 位置 + 4 四元数） | **6**（3 线速度 + 3 角速度） | +1 |
| `ball` | **4**（一个四元数） | **3**（角速度） | +1 |
| `hinge` / `slide` | 1 | 1 | 0 |

所以 `nq - nv` = 模型里 free/ball 关节的**个数**：本模型 `1 free + 12 hinge` → `19 - 18 = 1` ✓ （实测另一个 `free + ball` 的模型是 `11 / 9`，差 2）。所以是 **`nq` ≥ `nv`**，而不是"可能不同"这么含糊。

**为什么？** 因为姿态没法用 3 个数无奇异、无冗余地表示（欧拉角有万向锁），MuJoCo 用 **单位四元数**：4 个数表示 3 个旋转自由度，多的那个不是独立变量（被"模长=1"约束定死）。 速度层面没有这个问题，直接用**角速度**（3 个数）。

**三个很容易踩的后果：**

1. **`qvel` 不是 `qpos` 的逐元素导数**：`qpos[3:7]` 是四元数、`qvel[3:6]` 是角速度，分量根本不对应。
   实测：令 `qvel = [1,0,0, 0,0,π]`（绕 z 轴 π rad/s）积分 1 s，得到 `qpos[3:7] = [0,0,0,1]` —— 即绕 z 轴转 180°，走的是四元数**指数映射**；若按"线性叠加"算成 `[1,0,0,π]`， 模长 3.297，根本不是四元数。
2. 在两个 `qpos` 之间插值/比较要用专门的 API：`mj_integratePos(model, qpos, qvel, dt)` 往前推；
   反方向是 `mj_differentiatePos(model, qvel, dt, qpos1, qpos2)`（实测前者积出来的 `[1,0,0,0,0,π]` 能被后者精确还原）。直接对 `qpos` 线性插值会得到非单位四元数。
3. 力/力矩、雅可比、质量矩阵全在 `nv` 维空间（`qfrc_*`、`efc_J`、`data.M`），
   只有 `qpos`/`key_qpos` 是 `nq` 维。看代码时先看 `nq` 还是 `nv`，就知道自己在哪个空间。

> 顺带：`qpos` 的布局由 `jnt_qposadr[j]` 给出、`qvel` 由 `jnt_dofadr[j]` 给出。本模型 freejoint 是关节 0， `qposadr = dofadr = 0`，所以 `qpos[0:3]` 是基座位置、`qpos[3:7]` 是基座姿态、`qpos[7:]` 才是那 12 个关节角。

---

## 2. `geom` 的 `type` 有哪些可能值

XML 里**能写**的只有下面 9 个（对应 `mjtGeom` 里 0–8 的值）：

| `type` | 含义 | 必需的其它属性 |
|---|---|---|
| `plane` | 无限大平面 | `size` 必须为正（通常只给前两个） |
| `hfield` | 高度场 | 要有有效的 `hfield` 指向 `<asset><hfield>` |
| `sphere` | 球 | `size[0]` = 半径 |
| `capsule` | 胶囊 | `size[0]` = 半径，`size[1]` = 半长 |
| `ellipsoid` | 椭球 | `size` 为三个半轴 |
| `cylinder` | 圆柱 | `size[0]` = 半径，`size[1]` = 半长 |
| `box` | 长方体 | `size` 为三个半长 |
| `mesh` | 三角网格 | 要有 `mesh` 指向 `<asset><mesh>` |
| `sdf` | 有符号距离场 | 要有 `mesh`（或 SDF 插件） |

`mjtGeom` 里还有一批 **100 以上**的值，它们是**可视化 / 内部**用的，写进 XML 会直接报 `XML Error: invalid keyword`：

```
arrow(100) arrow1(101) arrow2(102) line(103) linebox(104) flex(105)
skin(106) label(107) triangle(108) none(1001)
```

**实测**：怎么区分「关键字合法但属性没给全」和「关键字非法」——

```
type="arrow"                → XML Error: invalid keyword: 'arrow'      ← 名字就不合法
type="plane" size 不全      → Error: plane size(3) must be positive    ← 名字合法，缺属性
type="mesh"  没有 meshid    → Error: mesh geom '' (id = 0) must have valid meshid
```

## 3. `friction` 的语法

不同地方要求的**个数不一样**，这是最容易记错的一点（出处：装好的 `include/mujoco/mjmodel.h` 注释）：

| 写在哪 | 几个数 | 含义 | 默认值 |
|---|---|---|---|
| `<geom friction="...">` | **3** | `slide spin roll`（滑动 / 自转 / 滚动） | `1 0.005 0.0001` |
| `<contact><pair friction="...">` | **5** | `tangent1 tangent2 spin roll1 roll2`（可各向异性） | 同 geom 的 3 个 + 补默认 |
| `<option o_friction="...">` | **5** | 全局覆盖，同上 5 维 | `1 1 0.005 0.0001 0.0001` |

`mjmodel.h` 原文：

```c
mjtNum* geom_friction;   // friction for (slide, spin, roll)      (ngeom x 3)
mjtNum* pair_friction;   // tangent1, 2, spin, roll1, 2            (npair x 5)
mjtNum  o_friction[5];   // friction
```

**实测（少给会补齐，多给报错）**：

```
<geom friction="2">            OK -> [2.    0.005 0.0001]   ← 没给的用默认补齐
<geom friction="2 0.1">        OK -> [2.    0.1   0.0001]
<geom friction="2 0.1 0.2">    OK -> [2.    0.1   0.2   ]
<geom friction="2 0.1 0.2 0.3">NG  -> XML Error: attribute 'friction' has too much data
```

## 4. `condim` 的全称与取值

- 全称：**contact dimensionality**（接触维数），即一个接触点上独立的力/力矩分量个数。
- `mjmodel.h` 原文：`int* geom_condim; // contact dimensionality (1, 3, 4, 6)`；`pair_dim` 同义。
- 取值只有 4 个：

| 值 | 含义 |
|---|---|
| `1` | 只有法向力 → 无摩擦接触 |
| `3` | 法向 + 2 个切向 → 常见的滑动摩擦（默认） |
| `4` | 3 + 自转力矩（spin / 扭转） |
| `6` | 4 + 滚动阻力（roll） |

**实测**：`condim=1/3/4/6` 合法；`condim=2`、`condim=5` 报 `Error: invalid condim in geom`。

## 5. 接触相关默认值速查（实测，`<geom size="0.1"/>` 一个 geom 的模型）

| 字段 | MJCF 属性 | 默认值 |
|---|---|---|
| `geom_condim` | `condim` | `3` |
| `geom_friction` | `friction` | `[1, 0.005, 0.0001]` |
| `geom_solref` | `solref` | `[0.02, 1]`（timeconst, dampratio） |
| `geom_solimp` | `solimp` | `[0.9, 0.95, 0.001, 0.5, 2.0]` |
| `geom_margin` | `margin` | `0` |
| `geom_gap` | `gap` | `0` |
| `opt.o_friction` | `<option o_friction>` | `[1, 1, 0.005, 0.0001, 0.0001]` |

## 6. 坑点

### 6.1 `<include>` 之后 `meshdir` 是相对**顶层文件**解析的

场景与机器人分目录时，模型里的 `meshdir="meshes/"` **不会**相对模型自己解析，而是相对 顶层（scene）文件所在目录。**实测**：`scenes/flat_scene.xml` include `models/xxx.xml`， 模型写 `meshdir="meshes/"` → 报错去找 `scenes/meshes/...`；同一个模型**单独**加载却正常。

**做法**：给每个会用到 `meshdir="meshes/"` 的目录各放一个 `meshes` **目录软链接**， 指向唯一一份真实 mesh。详见 `@20260923_mujoco/scripts/onetime_tools/measure_and_fix_base_height.py` 里的 `MESH_LINKS`。

### 6.2 网站导出的模型「默认位形就穿模」，求解器会把狗弹飞

`urdf.enkeebot.com` 把根 body 放在原点，而零位形下脚底在基座下方 0.5786 m ⇒ 默认状态整只 狗沉进地面。**症状与修法**：不修的话，第一步就会看到狗被弹到几米高（本任务的实测数字与 A/B 脚本见 [`../../@20260923_mujoco/docs/task2.md`](../../@20260923_mujoco/docs/task2.md)）。修法：把基座默认高度抬到「脚底刚好触地」；或在脚本里 `mj_resetDataKeyframe(model, data, 0)` / 显式设 `data.qpos[2]`。
注意**场景里的 `<keyframe>` 不会自动加载**，最简 `viewer.launch_passive` 循环就会踩到。

**“脚底刚好触地”的高度怎么求**（本模型足底是球体，所以有闭式解）：`mj_forward()` 之后遍历足底 geom，
取 `基座高度 = -min(球心世界坐标 z − size[0])`，即“基座到最低足底点的距离”；把基座默认 z 设成它即可。
两个脚本用的都是这条思路：`scripts/onetime_tools/measure_and_fix_base_height.py`（算出后写回 XML）与
`scripts/agent_scripts/rest_check.py --mode drop`（用同一条算式反算基座高度，再自由落体求趴卧姿态）。
足型不是球体时，改用 `mj_ray` 向下打射线，或对 geom 的顶点/包围盒求最小值。

### 6.3 同一份 mesh 被复制三份

原始 URDF、网站导出、旧导出各带一份 STL，每份 34 MB 且 md5 完全相同。 **做法**：真实文件只留 `assets/urdf/meshes`，其余目录用软链接；git 里只多几条几十字节的 软链接条目（git 能直接存软链接）。

### 6.4 MuJoCo **没有**原生录像功能

官方 `simulate` GUI 只能存单帧截图；Python 侧 `mujoco.Renderer` 只返回 RGB 帧数组， 编码要自己接（本仓库的做法是离屏渲染 + 管道给系统 ffmpeg，Python 与 C++ 各一份，用法与参数见 [`../../@20260923_mujoco/docs/recording.md`](../../@20260923_mujoco/docs/recording.md)）。

### 6.5 URDF→MJCF 有两条路线，产物差别很大

MuJoCo 自带的 `mj_saveLastXML()` / `MjSpec.from_file().to_xml()` 都能转（两者输出**逐字节相同**），但产物与网站（urdf.enkeebot.com）导出不同：只导 `<collision>`、**visual mesh 全丢**、**根 link 被并进 `worldbody`**（于是没有 `<freejoint/>`，还丢掉基座质量惯量）、没有 actuator。具体差异、选项组合与本任务的选择见 [`../../@20260923_mujoco/docs/model.md`](../../@20260923_mujoco/docs/model.md)。

### 6.6 离屏渲染尺寸受模型 XML 限制（默认只有 640×480）

`mujoco.Renderer` 用的是模型里声明的**离屏 framebuffer**，默认 `offwidth=640 offheight=480`， 由 XML 的 `<visual><global offwidth=".." offheight=".."/>` 控制。要的画面比它大就直接报错：

```
ValueError: Image width 960 > framebuffer width 640. Either reduce the image width
or specify a larger offscreen framebuffer in the model XML ...
```

**实测**：同一套代码、同样请求 960x540 —— `scenes/flat_scene.xml` 能跑（它写了 `offwidth="1280" offheight="720"`），`scenes/flat_scene_raw.xml` 直接报错（它没写，用默认 640×480）。

**做法**：两种都行——(a) 在 XML 里声明够大的 `offwidth/offheight`；(b) 加载后改 `model.vis.global_.offwidth/offheight` 再建 `Renderer`（**录像库 `VideoRecorder` 就是自动这么做的**，所以任意场景都能按你给的 `width/height` 录）。注意改必须在 `Renderer(...)` 构造**之前**。

C++ 侧同一条规则：`cpp_task2/src/record.h` 的 `OffscreenRecorder` 也在 `mjr_makeContext` **之前**把 `m->vis.global.offwidth/offheight` 设成 `--width/--height`，所以录像现在是**原生**跑在输出分辨率上、不再从场景声明的 1280×720 放大（放大版会糊）。实测 960×540 / 1920×1080 / 3840×2160 三档：`mjr_maxViewport` 返回的视口与输出尺寸逐项一致（`ffprobe` 实测分辨率也对），`ffmpeg -vf` 里只剩 `vflip`。

### 6.7 运行时改 `geom_pos` / `geom_quat` 可能**静默无效**：`geom_sameframe`

把地面转个角度、把障碍物挪个位置，直觉写法是加载后直接写 `m->geom_quat[...]` / `m->geom_pos[...]`。**实测（MuJoCo 3.12）这样会静默失效**：模型里字段读回来确实是新值，但 `d->geom_xpos` / `d->geom_xmat` 一点不变，碰撞面也不动——把地面转 15° 后，放在 `x=1` 的自由小球跑 2 s 仍停在原地（落点 `x=1.000`、`z=0.050`），地面 geom 的实测法向仍是 `(0, 0, 1)`。

原因在运动学：`mj_kinematics2` 算 geom 世界位姿时调 `mj_local2Global(d, d->geom_xpos+3*g, d->geom_xmat+9*g, m->geom_pos+3*g, m->geom_quat+4*g, m->geom_bodyid[g], m->geom_sameframe[g])`（`src/engine/engine_core_smooth.c:214`），而 `mj_local2Global`（同文件 `:975`）在 `sameframe` 为 `mjSAMEFRAME_BODY` 时**直接抄它所属 body 的位姿**（位置分支就是一句 `mji_copy3(xpos, d->xpos+3*body)`），根本不看传进去的 `pos`/`quat`。`geom_sameframe` 是**编译期**推断的：geom 的 `pos` 为零、`quat` 为单位四元数（也就是「与 body 同帧」）时被标成 `mjSAMEFRAME_BODY`（枚举见 `mjtSameFrame`，`mjtype.h:457`）；场景里不写 `pos/quat` 的地面（`<geom name="floor" type="plane" .../>`）正是这样，实测它的 `geom_sameframe=1`。

**做法**：改 `geom_pos`/`geom_quat` 的同时把 `m->geom_sameframe[g] = 0` 清掉（一行）。**实测**：只改 quat → 小球落点 `x=1.000`、法向 `(0, 0, 1)`；quat + 清 sameframe → 法向变成 `(0.259, 0, 0.966)`、小球沿坡滚到 `x=4.303`。不想在运行时改也行：走 `mj_parseXML` → 改 `mjsGeom` → `mj_compile` 把姿态烘进编译期（`mjSpec` 系列是 3.2+ 的公开 API），或者干脆写在 XML 里。

**配套习惯**：这类「自己算的几何 ≠ 引擎真正用的几何」不会报任何错，只会让结论整段错。凡是自己维护「地面法向 / 平面上一点」这类派生量的地方，都在改完 + `mj_forward` 之后断言一次 `d->geom_xmat` 的第三列（世界系法向）与自己的期望一致，不一致就报错退出。

### 6.8 「地面斜了，画面却看不出」：先查地面到底转没转，再谈相机

**现象**：把地面倾斜 15° 后，窗口里地面看起来仍与画面底边平行（像根本没转），自然会怀疑「相机跟着地面一起转了」。**两个结论都实测过**：

- **相机不可能跟着转**：自由相机的 up 由世界 z 构造（`mjv_updateCamera`），`up` 与世界 z 的夹角就等于 `elevation`（本次场景 `<visual><global azimuth="120" elevation="-20"/>` ⇒ `up=(-0.171, 0.296, 0.940)`、`up·z=0.93969=cos 20°`）；`mjvCamera` 里根本没有 roll 字段。实测 pitch 0° 与 15° 两次算出的相机 `pos/forward/up` **逐位相同**。
- **真正的原因是地面压根没转**：那种写法被 §6.7 的 `geom_sameframe` 吃掉了。实测同一次比较里，`d->geom_xmat` 显示地面法向仍是 `(0, 0, 1)`，画面与水平地面基准的**下部条带**（该区域只有地面）几乎逐像素相同（平均 |ΔRGB| = **1.98**，上限 765）；清掉 `sameframe` 后同一个比较变成 **185.73**——画面立刻就变了。所以「看起来像没转」是字面意义上的「真的没转」，不是视角问题。

**做法**：先用 §6.7 的办法确认地面转成了（断言 `d->geom_xmat`），再看画面。顺带一条：纯色地面即使真转了，画面里也只有亮度/边界的变化，看不出「斜多少、往哪斜」；要一眼看清坡度就给地面加**可见参照**——本次用的是 MuJoCo 自带的程序化纹理（`<texture type="2d" builtin="checker"/>` + `<material texture="..."/>`，不需要外部图片；地面仍是 `type="plane"`，物理一点没变），斜面 demo 的默认场景 `@20260923_mujoco/scenes/slope_scene.xml` 就是「flat_scene + 一张棋盘格材质」。

---

## 7. 渲染后端（`MUJOCO_GL`）与开销

### 7.1 那些后端名字是什么

> 完整的分层图（OpenGL / Vulkan / Direct3D / Skia / ANGLE / Mesa … 各在哪一层）见 [`graphics-stack.md`](graphics-stack.md)；下面只讲 `MUJOCO_GL` 会用到的这几个。

| 名字 | 全称 | 是什么 |
|---|---|---|
| `glfw` | **GLFW** = *Graphics Library Framework* | 一个跨平台的**窗口/上下文/输入**库（Windows、macOS、Wayland、X11），负责"给你一个窗口 + 一个 OpenGL context"，还管键盘鼠标手柄。它的 FAQ 里明确说自己**不**实现 OpenGL、不渲染任何东西——只负责把 context 交给你。MuJoCo 的 `launch_passive` 窗口就是它建的。<br>注：官方站点只写 `GLFW`、不展开缩写，这个展开见 Wikipedia 等资料。 |
| `egl` | **EGL** = *Khronos Native Platform Graphics Interface*（EGL 1.2 起的规范名）；1.2 之前叫 *OpenGL ES Native Platform Graphics Interface*；X.Org 的术语表则把 EGL 展开为 **Embedded-System Graphics Library** | Khronos 定义的"渲染 API（OpenGL / OpenGL ES / OpenVG）↔ 原生窗口系统"之间的**接口层**：管 context、surface 绑定、渲染同步。关键区别是它**可以直接渲染到离屏 buffer**，不需要真的开窗口——所以无显示器/纯录像场景用它。<br>Android、Wayland 客户端也都用 EGL。 |
| `osmesa` | Mesa 的离屏渲染库 **`libOSMesa`**（一般理解为 *off-screen Mesa*） | Mesa 的**纯软件**离屏实现：没 GPU、没显示器也能出图，但慢，多用于 CI。 |
| `glx` | **GLX** = *OpenGL Extension to the X Window System* | X11 上的 OpenGL 接口，EGL 在 X11 场景下的"同类竞品"。 |
| `wgl` / `cgl` | WGL（Windows 的 GL 接口）/ **CGL** = *Core OpenGL*（macOS 的） | 各自平台上的对应接口，本机（Linux）用不到。 |

一句话理清层次：**EGL / GLX / WGL / CGL 是同一层**的东西（"把 GL 接到某个窗口系统上"，或接到离屏 buffer 上）；**GLFW 是它们上面一层**的便利库（帮你建窗口、拿到 context）。所以 MuJoCo 的 `MUJOCO_GL` 选的是"**用哪条路拿到 GL context**"。

MuJoCo 认可的取值（mujoco 3.12，linux-64，见 `mujoco/rendering/classic/gl_context.py`）： `""`（默认，等价 `glfw`） | `glfw` | `glx` | `egl` | `osmesa`，以及 `disable/disabled/off/false/0`（禁用 GL，此时不能渲染）。 非法值会在 **import mujoco** 时直接报 `invalid value for environment variable MUJOCO_GL`。

怎么选（本机实测）：

* **离屏渲染 / 录像** → `egl`：不需要显示服务；而且在这台 Optimus 笔记本上，`egl` 的离屏 context 落在 **NVIDIA MX350（独显）** 上 —— 出一帧 5.6 ms；
* **要交互窗口**（`launch_passive`）→ `egl` 也可以（实测 99% 实时、窗口正常）；
* 没有 GPU 的容器 / CI → `osmesa`（纯软件光栅，很慢）；
* 默认（`glfw`）→ 走"显示的 GL"，也就是**核显**（本机 Intel UHD），还要 X/Wayland 显示服务； 出一帧 21 ms，**慢 3.8 倍**。

> **同一台机器、不同后端可能落在不同的 GPU 上**（Optimus 笔记本的典型现象）。 所以测出"后端快慢"时，先别急着归因给后端本身——先把 `GL_RENDERER` 打出来看看是谁在画 （§7.2 的做法）。

### 7.2 开销与实时性（实测，本机 i5-1035G1，960×540，`MUJOCO_GL=egl` → MX350）

MuJoCo 本体是 CPU 引擎，仿真极快；**成本主要在渲染**。数字由 `scripts/agent_scripts/render_cost.py` 实测（该脚本会顺便打印 `GL_RENDERER`）：

| 调用 | 单次开销 | 说明 |
|---|---|---|
| `mujoco.mj_step()` | **0.04 ms/步** | 500 Hz 只占个位数 % CPU 时间 |
| `update_scene()` | 0.01 ms | 组装场景（几乎不要钱） |
| 离屏 `render()` + 回读 | **5.5 ms/帧** | 大头（960×540 = 1.5 MiB/帧 的读回也在里面） |
| `tobytes()` | 0.06 ms/帧 | |
| **「出一帧」合计** | **5.6 ms/帧** | 上面三项之和 |
| 录像整链路 `capture()` | **7.1 ms/帧** | ⇒ 50 fps 约占 **35%** CPU 时间 |
| ↳ 其中管道 + x264 编码 | ≈ **1~2 ms/帧** | 比渲染便宜得多（复测 3 次：1.3 / 1.3 / 2.3 ms）|
| `Viewer.sync()` | 7.7 ~ 20 ms/次 | 交换/刷新节流，随负载浮动 |

**“后端”其实就是“哪块显卡”**（同一台机器实测，所以不能只看“后端”两个字）：

| `MUJOCO_GL` | 实际渲染的 GPU | 出一帧 | 录像一帧 | 50 fps 录像占用 |
|---|---|---|---|---|
| `egl` | **NVIDIA GeForce MX350**（GL 4.6，max texture 32768） | 5.6 ms | 7.1 ms | 35% ✓ |
| `glfw` | **Intel UHD (ICL GT1) / Mesa 23.2.1**（max texture 16384） | 21.1 ms | 22.2 ms | **111%** ✗ 跟不上 |

于是顺序很清楚：**渲染 ≫ 编码 ≫ 仿真**。想提速先动渲染（换 GPU / 降分辨率 / 降帧率）， 而不是去调 x264 参数。

> 踩过的坑：`mujoco.Renderer.render()` 之前**必须先 `update_scene(data)`**，否则画的是 **空场景**——快好几倍，测出来的"渲染成本"完全失真（本项目早期写下的 1.17 ms/帧就是这么来的， 真实值是 5.5 ms）。

> 量级 vs 小数：这些数字随系统负载浮动，**看量级、别抠小数**。尤其编码那一项受采样次数影响： `--n 100` 复测稳定在 1.3~2.3 ms，而 `--n 20` 的短跑曾测出 4.3 ms（ffmpeg 尚在预热 / 管道未吃满）。两个数量级的结论（渲染 ≫ 编码 ≫ 仿真）在两种情况下都成立。

**`MUJOCO_GL` 必须在 `import mujoco` 之前设置**：后端在那一刻就选定，之后再改 `os.environ` 无效。 Linux 下**不设时默认是 `glfw`**——它走"显示的 GL"，在 Optimus 笔记本上就是**核显**， 既需要显示服务、又比 `egl`（走独显）慢好几倍。 本仓库把它写在仓库根 `pixi.toml` 的 `[activation.env]` 里。注意 pixi 的激活环境会**覆盖** 你在命令行前缀给的值，临时换后端得这样写：

```bash
pixi run env MUJOCO_GL=glfw python xxx.py     # 有效
MUJOCO_GL=glfw pixi run python xxx.py         # 无效（被 activation.env 覆盖）
```

实测「跑完 3 s 仿真需要的墙钟」（`timestep=0.002`；开窗口时 `sync` 每 25 步一次）：

| 配置 | 3 s 仿真所需墙钟 | 实时比例 |
|---|---|---|
| 纯离屏录像 50 fps（不开窗口） | 4 s 仿真只花 2.87 s（含启动） | **>100%** |
| 只开窗口，不录像 | 3.02 s | 99% |
| 开窗口 + 录像 10 fps | 3.17 s | 95% |
| 开窗口 + 录像 50 fps | 3.58 s | 84% |
| 开窗口 + 录像 50 fps（**GLFW 后端**） | 7.14 s | **42%** |
| 开窗口 + **每步** `sync` | 约 30 s（外推） | **约 10%** |

要点：

1. **只录视频就不要开窗口**。不开窗口时 50 fps 也能跑得比实时快，而且没有下面那两个坑。
2. 开窗口时 `viewer.sync()` 必须**降频**（每 10~25 步一次）。它每次要 8~20 ms，按 500 Hz 的
   仿真步调它，等于把循环钉在刷新率上、仿真只剩 ~10% 实时。
3. 开窗口时的实时节流要用「目标墙钟时刻 = 起点 + `data.time`」**自我纠偏**；
   `sleep(dt - 本步耗时)` 这种只补本步亏欠的写法会累积误差 （`sleep(1.7 ms)` 实际睡 4~5 ms），实测 69% → 100%。
4. 开窗口的进程退出时偶发 `segmentation fault`，或输出 `GLFWError: EGL: Failed to clear current
   context ...` 之类的清理告警（GLFW 与已存在的 EGL 上下文在同进程收尾时的冲突）； MP4 在崩溃前已由 `close()` 写完，产物不受影响；纯离屏不出现。
5. 自己 new 的 `mujoco.Renderer` 一定要 `close()`，否则解释器退出时会打印
   `Exception ignored in: Renderer.__del__` 加一串 OpenGL 报错（录像库内部会显式 close， 正常使用看不到）。

### 7.3 分辨率、"DPI" 与尺寸上限

**MuJoCo 没有"DPI"这个概念** —— DPI（一英寸多少像素）是显示/输出端的属性； 离屏渲染里能调的只有**像素数**：`mujoco.Renderer(model, height, width)`。 "保持视角范围不变、只改分辨率"完全可行，因为**视角由相机决定、与像素数无关**： 自由机位的视场角是 `model.vis.global_.fovy`（本模型 45°），`azimuth/elevation/distance` 决定机位。

同一机位扫描（本机 `egl` → MX350）：

| 分辨率 | 像素数 | 出一帧 | 归一化包围盒 |
|---|---|---|---|
| 240×135 | 0.03 Mpx | 4.5 ms | 1.000 × 0.822 |
| 480×270 | 0.13 Mpx | 5.0 ms | 1.000 × 0.819 |
| 960×540 | 0.52 Mpx | 5.6 ms | 1.000 × 0.817 |
| 1920×1080 | 2.07 Mpx | 8.9 ms | 1.000 × 0.817 |
| 3840×2160 | 8.29 Mpx | 22.3 ms | 1.000 × 0.816 |

归一化包围盒几乎不变 ⇒ **画面内容一模一样、只是像素更多**（就是"高 DPI"想要的效果）。 代价：像素数 ×256，一帧从 4.5 ms 涨到 22 ms（小尺寸下固定开销占主导，大尺寸下像素量/回读量占主导）。

**上限在哪？**

* MuJoCo **自己**只有一条要求：模型里声明的离屏 buffer 必须 ≥ 请求尺寸 （`<visual><global offwidth/offheight>`，默认 640×480；录像库 `VideoRecorder` 会自动调大， 见 §6.6；C++ 的 `OffscreenRecorder` 同样会把它调到请求尺寸）；
* 真正的天花板是 **GL / 驱动 / 显存**：本机 `egl`(NVIDIA) `GL_MAX_TEXTURE_SIZE = 32768`、 `glfw`(Intel) 只有 16384。实测 8192×8192 能跑（**192 MiB/帧**，一帧 3.6 s —— 瓶颈已经是回读带宽）， 40960×40960 直接报 `FatalError: Offscreen framebuffer is not complete, error 0x8cd6`（FBO 不完整）。

所以"分辨率上限" = **min(GL/显存上限, 你能接受的带宽)**，不是 MuJoCo 设的。

### 7.4 送给 ffmpeg 的是什么流

**完全没压缩的裸像素**。`VideoRecorder.capture()` 每帧做的是：

```
update_scene → render()（GPU 画进离屏 FBO）→ tobytes()
   → 写进 ffmpeg 的 stdin（-f rawvideo -pixel_format rgb24 -video_size 960x540 -framerate 50）
```

也就是说 **RGB 转换、色彩空间、压缩、封装全都还没发生**，只是把 `960×540×3 = 1,555,200` 字节（1.5 MiB）的裸数据塞进管道；50 fps 意味着 **74 MiB/s** 的管道流量。 ffmpeg 那边才做：

1. `-pix_fmt yuv420p`：RGB → YUV，并按 4:2:0 做色度下采样（每 4 个像素共享一组色度）；
2. `libx264` 编码：`-preset veryfast -crf 20`，运动估计 / DCT / 熵编码 → I/P/B 帧；
3. MP4 封装：写上时间戳。注意**时间戳不是逐帧传过去的**，而是靠 `-framerate 50` 声明为恒定帧率
   （第 N 帧 = N/50 秒）——这是 `capture()` 必须按**仿真时间**节流的原因，否则视频和仿真对不上。

实测（960×540 / 50 fps / crf 20 / veryfast）：输出 **310 kbps**（4.02 s 共 156 KB）， 相对 74 MiB/s 的输入约 **2000:1** 的压缩（场景基本静止，所以这么夸张）。

本机 ffmpeg 另外还带 `h264_nvenc`（MX350 的 NVENC 硬编码）/ `h264_vaapi` / `h264_qsv`， 改编码器只需动 `VideoRecorder` 里的 ffmpeg 参数；但按 §7.2 的账，**编码只占 ~1~2 ms，换了也省不了多少**。

### 7.5 视频帧率与“渲染性能”无关（可复现实验）

**`fps=` 只是一个“采样网格”，与渲染耗时、机器快慢完全无关。** `VideoRecorder` 的做法：

* 视频时间轴由 `fps` 决定：第 N 帧 = `N/fps` 秒（ffmpeg 侧按恒定帧率写时间戳）；
* 什么时候**取帧**由**仿真时间** `data.time` 决定（`capture()` 内部维护 `_next_t`），不是墙钟 —— 所以机器慢只意味着**录制过程更久**，不改变产物的帧率/帧数/时间戳。

实测（同一场景、`fps=50`、录 2 s 仿真，跑两遍；第二遍每步额外 `sleep(20 ms)` 模拟“慢机器”）：

| | 墙钟 | 帧数 | 时长 | 逐帧时间戳 |
|---|---|---|---|---|
| A 正常 | 1.15 s | 100 | 2.000 s | 基准 |
| B 每步 +20 ms | **23.39 s**（慢 20×） | **100** | **2.000 s** | **`diff` 完全一致 ✓** |

两个前提（否则会“掉帧”——时间轴仍然正确，但画面会跳）：

1. 调用频率要够：**每个 `1/fps` 的仿真区间内至少调一次 `capture()`**（最省心的做法是每步都调，
   反正内部会节流）；
2. `fps ≤ 1/timestep`：一个步长只能采一帧，本模型 `timestep=0.002` ⇒ 上限 500 fps（C++ 侧 `record.h` 同一条限制），
   想更高只能减小 `timestep`。

### 7.6 这些结论驱动了哪些配置决策

上面几条不只是"知识"，仓库里有具体落点。**改这些配置前先读对应小节**（反向索引）：

| 配置 / 决策 | 落在哪 | 依据 |
|---|---|---|
| 全局 `MUJOCO_GL = "egl"` | `pixi.toml` 的 `[activation.env]` | §7.1（必须在 import 之前生效）+ §7.2（egl 不依赖显示服务，且在双显卡机器上落到更合适的那块 GPU）|
| 库里再 `setdefault("MUJOCO_GL", "egl")` 兜底 | `visualization/mujoco_video.py` | §7.1（不依赖调用方的 import 顺序）|
| 建 `Renderer` 前自动调大 `model.vis.global_.offwidth/offheight` | `visualization/mujoco_video.py` | §6.6 / §7.3（默认离屏只有 640×480，否则直接报错）|
| 同上（C++）：建 context 前调大 `m->vis.global.off*`，尺寸一致就不加 `scale`；输出路径打印成绝对路径 | `cpp_task2/src/record.h` | §6.6（同一规则，两边都能按 `--width/--height` 原生录）|
| 提供 `--width/--height`（分辨率与视角解耦）| `example_attach.py`、`example_with_viewer.py`、`render_preview.py` | §7.3 |
| 开窗口时 `viewer.sync()` 默认每 25 步一次（`--sync-every`）| `example_with_viewer.py` | §7.2（每步 sync 会被显示刷新钉住）|
| 开窗口时建议把录像降到 10 fps | `example_with_viewer.py` docstring | §7.2（渲染是链路大头，50 fps + 窗口跟不上实时）|
| 实时节流用“目标墙钟 = 起点 + `data.time`”自我纠偏 | `example_with_viewer.py` 的 `--pacing deadline`（默认）| §7.2（`sleep(dt − 本步耗时)` 会累积误差）|
| 离线录制不加 sleep、跑满 CPU | `example_attach.py` | §7.5（帧率与墙钟无关，慢机器只是录得久）|
| 用 libx264 软件编码，不换硬件编码 | `visualization/mujoco_video.py` 的 ffmpeg 参数 | §7.4（编码只占一小部分）|
| `meshdir` 不改，改用目录软链接 | `onetime_tools/measure_and_fix_base_height.py` 的 `MESH_LINKS` | §6.1 |
| 场景/模型/导出目录各放一个 `meshes` 软链接 | `scenes/`、`models/`、`assets/black_description/` | §6.1（`meshdir` 相对顶层文件解析）|
| 模型里把基座抬到触地高度；不改 `inertiafromgeom` | `onetime_tools/measure_and_fix_base_height.py` | §6.2 |

上面这些落点都在 `@20260923_mujoco/scripts/` 下：录像库核心是 `visualization/mujoco_video.py`，包入口 `visualization/__init__.py` 负责导出 `VideoRecorder`（脚本里直接 `from visualization import VideoRecorder`）。

---

## 8. 复现命令

```bash
cd ~/BiS.d/Code.d/RoboCon/MyMonoRepo.d

# geom type / friction / condim 的全部实测数据
pixi run python @20260923_mujoco/scripts/agent_scripts/mujoco_facts.py

# 第 6.7/6.8 节：运行时改地面 geom_quat 会被 geom_sameframe 吃掉；相机不会跟地面转（需要离屏渲染）
pixi run python @20260923_mujoco/scripts/agent_scripts/geom_sameframe_check.py --pitch 15

# 原始导出 vs 打过补丁模型的初始状态 A/B
pixi run python @20260923_mujoco/scripts/agent_scripts/ab_initial_state.py

# 字段注释的权威出处（本地头文件，随 mujoco 一起装好）
grep -n 'geom_friction\|geom_condim\|pair_friction' .pixi/envs/default/include/mujoco/mjmodel.h

# 渲染/录像的单次开销（第 7 节的全部数字，--viewer 会弹窗）
pixi run python @20260923_mujoco/scripts/agent_scripts/render_cost.py --viewer
pixi run env MUJOCO_GL=glfw python @20260923_mujoco/scripts/agent_scripts/render_cost.py   # 对比后端

# 实测实时比例（对照着第 7 节看）
pixi run python @20260923_mujoco/scripts/visualization/examples/example_with_viewer.py --seconds 3 --no-record
pixi run python @20260923_mujoco/scripts/visualization/examples/example_with_viewer.py --seconds 3 --fps 10
pixi run python @20260923_mujoco/scripts/visualization/examples/example_with_viewer.py --seconds 3 --fps 50
pixi run python @20260923_mujoco/scripts/visualization/examples/example_with_viewer.py --seconds 3 --no-viewer --fps 50

# 把录像接进自己的循环（只加一行 rec.capture(data)，不需要窗口）
pixi run python @20260923_mujoco/scripts/visualization/examples/example_attach.py
```

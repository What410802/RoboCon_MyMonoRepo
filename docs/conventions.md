# 项目约定

本仓库在培训过程中形成的约定，来源有三处：验收规范（`四足组培训任务提交与验收规范.md`，只读材料）、对提交方式的明确要求、以及踩坑后写进 [`pitfalls/environment.md`](pitfalls/environment.md) 的经验。改动约定时请同步更新本文档。

## 1. 提交信息（commit message）

- **一行英文**，不写正文；"这次做了什么、做到哪一步"由仓库文档本身体现。
- 形如 `type(scope): 一句话`，`type` 取 `feat` / `fix` / `docs` / `refactor` / `build` / `chore` / `test`，`scope` 可省略。
- 例：`feat(cpp): add dog_sim (toolchain, model load, rest-pose check)`、`docs: explain why pixi is used`。
- 更早的提交用了 `work:` 前缀（如 `work: training02 task 2 done`），属历史遗留；**新提交请用上面的类型**。
- **分阶段提交**（验收规范 §3.1）：搭环境 / 加载模型 / 加控制 / 整理结构 / 补文档分别提交，不攒到最后一次。
- 提交前先确认信息内容与待提交文件；**AI 助手只准备信息与命令，不代为提交**。
- **不改写已推送的历史**（验收规范 §5 与 §11 要求保留过程）；确需改写时（例如撤回刚提交的内容），先说明原因并确认。

## 2. 文档

- Markdown **每段单行**，不要按宽度自动换行；代码围栏内的内容逐字节保持原样。
- 需要图示时用 **Mermaid**，不要并排写 ASCII 或纯文字版。（PlantUML 的 timing/gantt 需要额外装扩展 + 服务端/Java，GitHub 也不渲染，所以不用。）要画「各方案在时间轴上占了多少」就用 Mermaid `gantt`，写法照 [`learn/runtime-timing.md`](learn/runtime-timing.md) §11.2 的样例抄：`dateFormat X` + `axisFormat %s`（刻度数字即毫秒）、**每个方案只画一轮、按轮长降序排**、每条任务给 id 且用 `id, 0, 时长` 起头 + `after <上一条id>, 时长` 顺序接力（**`id, 起点, 终点` 里的起点会被忽略**，条一律从 0 起算——实测过）、每轮用 `milestone` 收尾。
- 根 `README.md` 只做**入口**：定位、任务记录、快速开始、文档索引；具体内容一律放 `docs/` 下。
- 文档**分类存放**在 `docs/` 的子目录下，类别可以新增、合并或调整（当前是 `learn/` 研究与学习、`pitfalls/` 踩坑记录）；`docs/` 根下只留本文（`conventions.md`）。
- **改动文档路径（移动 / 重命名）时**，必须全仓搜一遍指向它的引用（文档、代码与配置注释），把对方那一侧的引用一并改对；新增引用时也要让双向链接成对（对方没有针对性位置，就在首部或尾部放反向链接）。
- 配置文件里的注释**引用文档**、不重复结论：例如 `pixi.toml` 的注释只写"见 `docs/pitfalls/environment.md` 某节"，把"为什么"留在文档里。
- 本机专属的结论（性能数据、显卡型号、本机路径、编辑器配置）集中写在 [`pitfalls/environment.md`](pitfalls/environment.md)；任务目录的 README 只描述任务本身并给出指针。
- 每个正式任务目录都有一份 `README.md`，至少说明：做了什么、环境与依赖、如何编译或运行、主要文件与目录的作用（验收规范 §4.2）。
- 文档里出现的路径、目录树**必须与仓库实际一致**——"文档写了但不存在"同样算错。
- 文档改动后自检：代码围栏是否配对、相对链接与锚点是否有效、双向引用是否成对、目录树是否与实际一致。

## 3. 目录与命名

- 任务目录用 `@<日期>_<主题>`（当前：`@20260922_robot_cpp_training`、`@20260923_mujoco`）；这与验收规范建议的 `01_/02_…` 序号方式等价，映射关系见根 [`README.md`](../README.md) 的「任务记录」。
- 目录与文件命名**统一用下划线**（如 `agent_scripts`、`onetime_tools`），不用连字符。
- **C/C++ 与 CMake 缩进统一用 4 空格**（不用 Tab、不用 2 空格）：根目录放了 `.clang-format`（`IndentWidth: 4`、`ColumnLimit: 0` 即不自动折行），格式化直接 `clang-format -i <file>`。
- 按**语言**分目录：语言无关的资源（`assets/`、`models/`、`scenes/`）放任务根目录；产物放 `output/<语言>/`（如 `output/python/`、`output/cpp/`，默认路径写在产出脚本里）；仿真与可视化代码放 `python/`；C++ 工程放 `cpp/`；诊断脚本与一次性工具留在 `scripts/`。
- 构建产物、缓存、大二进制不入库（验收规范 §4.4）：`.gitignore` 覆盖 `build/`、`.cache/`、`__pycache__/`、`.pixi/` 等；重复的大资源用软链接共用一份真实文件。

## 4. 环境与依赖

- Python / C++ / MuJoCo 的依赖都在根 `pixi.toml` 里声明，用 `pixi run …` 执行；不写死绝对路径、不把环境相关参数全部硬编码（验收规范 §9）。
- 版本一致性优先：C++ 与 Python 用同一个 MuJoCo 版本（当前 3.12.0）。
- 必须在进程早期生效的环境变量写进 `[activation.env]`（如 `MUJOCO_GL`）。
- 环境选择的理由集中在 [`pitfalls/environment.md`](pitfalls/environment.md)，任务 README 只写怎么用。

## 5. 验证与证据

- 每个结论都给**可复现命令**与实测数字，不写"感觉更快"这类表述。
- 引用外部代码或仓库时给出**可核对的位置**（`文件:行`，例：`simulate/src/main.cc:413`），不要只写"在 main.cc 里"；结论性说法要能顺着这个位置查到。
- 改脚本后跑一遍回归（`scripts/agent_scripts/` 的诊断脚本 + 示例脚本 + `cpp/` 的程序），并在 README 里更新当前状态。
- C++ 与 Python 共用同一份 MJCF，两边结果可以做逐项对照，这是任务 3/4 的验收手段。

## 6. 外部代码与许可

- 使用外部代码或资料时保留来源说明（验收规范 §8）：本仓库引用了 `unitreerobotics/unitree_mujoco`（BSD-3-Clause）与 MuJoCo 官方示例（Apache-2.0）。
- 不整段照搬：验收会要求现场解释并按需小幅修改自己的代码（验收规范 §6），所以结构要小而清楚。

## 7. 文档职责分工（任务目录 vs 仓库 `docs/`）

- **仓库 `docs/` = 适用于本环境**：MuJoCo 知识点与坑点、图形栈、编辑器提示、环境/镜像/工具链依据，以及跳任务也成立的通用踩坑（如 `<include>` 后 `meshdir` 的解析规则、keyframe 不会自动加载）。
- **任务目录（`@<日期>_<主题>/`）= 只适用于本任务**：模型来源与转换取舍、本任务的过程与产出、面向本任务的运行方式与结论表、只对这份模型成立的数字。
- 判定标准：**换个任务还成立吗？** 成立 → 写进 `docs/`；只对这份模型/这次验收成立 → 写任务目录。同一条内容不写第二遍，父文档只放飞指针。
- 任务 README 是**入口**：做完什么、怎么跑、结果在哪、往哪跳；过程细节沉淀到自己的 `docs/` 子目录，长的操作步骤不要堆在 README 里。
- 现有可瘦身项（**2026-09-26 已执行**）：任务 README 拆成了入口 + `@20260923_mujoco/docs/` 下四份（`model.md` / `task2.md` / `recording.md` / `replication.md`），对应的通用部分留在 `docs/learn/mujoco.md`、`docs/pitfalls/environment.md` 里只留结论 + 指针。新任务请沿用这个分层。

# RoboCon 培训学习仓库

项目约定（提交信息、文档、目录、Git 用法、环境）见 [`docs/conventions.md`](docs/conventions.md)。

## 任务记录

+ 2026-09-22 `robot_cpp_training` [目录](@20260922_robot_cpp_training/robot_cpp_oop_cmake_training/)：[文档](@20260922_robot_cpp_training/验收.md)
+ 2026-09-23 URDF, MuJoCo [目录](@20260923_mujoco)：[文档](@20260923_mujoco/README.md)


## 环境与踩坑记录

### 工具链选择：为什么用 pixi

结论：**Python、MuJoCo（C 库 + Python 绑定）与 C++ 工具链都装在同一个 pixi 环境里**，不用系统 apt 包，也不用单独的 venv。

- **跨语言共用同一份 MuJoCo**：任务 4 要写 C++，而 C++ 需要的是 `libmujoco`（头文件 + 库 + CMake 配置）；conda-forge 的 `mujoco` 包同时提供 C 库与 Python 绑定，两边**就是同一个 3.12.0**，行为一致、数字可比（C++/Python 对照正是任务 4 的验收方式）。pip/venv 只能拿到 Python 轮子，拿不到配套头文件与 CMake 配置。
- **一次装齐 C++ 工具链**：`cxx-compiler` / `cmake` / `ninja` 都在 conda-forge，与 `libmujoco` 出自同一条工具链，避免“系统 gcc 编、conda 库链接”的 ABI / libstdc++ 错配。
- **可复现**：`pixi.lock` 锁死确切版本与哈希，别人 clone 后 `pixi install` 得到同一个环境，“在我机器上能跑”这件事有据可查。
- **不污染系统**：一切装在仓库内的 `.pixi/`（已 gitignore），不需要 sudo，删掉目录即回滚；系统只负责显卡驱动这类必须由系统管的东西。
- **能承载“必须提前生效”的环境变量**：`MUJOCO_GL` 必须在 `import mujoco` 之前生效（见下面“图形后端”一节），`[activation.env]` 正好解决这件事。
- **两种包源都可用**：conda 侧走 conda-forge（实测直连 conda.anaconda.org 最快，见下一小节），PyPI 侧由内置的 uv 负责，并可用 `extra-index-urls` 配国内回退源。
- 代价：多一层工具（要习惯用 `pixi run` 而不是直接 `python`），且 conda-forge 的 `mujoco` 版本略落后于 PyPI（当前 3.12.0 vs 3.14.0）——对本项目不构成问题。

### 2026-09-24 pixi / conda / PyPI 镜像

本工作区用 `pixi` 统一管理 Python 与 MuJoCo（`pixi.toml` 在仓库根目录）。初始化时遇到下面几个坑，均已处理，记录备查。

**1）清华 TUNA 镜像对本机整体 403（conda 与 pypi 都是）**

```bash
curl -s -o /dev/null -w '%{http_code}\n' \
  https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/conda-forge/linux-64/repodata.json   # 403
curl -s -o /dev/null -w '%{http_code}\n' \
  https://pypi.tuna.tsinghua.edu.cn/simple/mujoco/                                          # 403
```

现象：`pixi search` / `pixi install` 直接失败（`HTTP status client error (403 Forbidden)`）； 只有 `pixi search --no-config ...` 才能绕过配置拿到数据。

- 配置位置是 `~/.pixi/config.toml`（**不是** `~/.config/pixi/config.toml`），原内容把 `conda-forge` / `pytorch` 映射到 TUNA，并把 pypi `index-url` 也指向 TUNA。
- 处理：**删除 `[mirrors]` 段**，pypi `index-url` 改成 USTC。原因见第 2 条。

**2）国内镜像都不支持 sharded repodata，走镜像反而更慢**

conda 的完整 `repodata.json` 是几十 MB；conda.anaconda.org 额外提供 `repodata_shards.msgpack.zst`（linux-64 约 **570 KB**），pixi 会优先用它。实测：

| 源 | `repodata_shards.msgpack.zst` |
|---|---|
| conda.anaconda.org（官方） | **200**，570 KB / 2.3 s |
| USTC | 302（跳 NJU） |
| NJU / aliyun / TUNA | 404 |

所以结论是反直觉的：**conda 侧不用镜像，直连 conda.anaconda.org 最快**。这也顺便解释了 第 1 条里为什么「删掉 mirrors」是正确解法。

**3）PyPI 上游本身达不到 1 MiB/s，所以仍然需要镜像**

同一文件（`mujoco-3.14.0-cp312-...whl`）8~20 s 限时下载实测：

| 源 | 实测速度 |
|---|---|
| files.pythonhosted.org（上游） | 4～21 KB/s |
| USTC | 31～356 KB/s（最快） |
| NJU | 157 KB/s |
| aliyun | 索引 200，但文件路径 404（布局不同） |
| pypi.org | 索引可访问，但 20 s 超时 |

结论：**没有任何一端达到 1 MiB/s**，因此保留镜像；`pixi.toml` 用多索引回退链：

```toml
[pypi-options]
extra-index-urls = [
    "https://mirrors.ustc.edu.cn/pypi/simple",
    "https://mirrors.aliyun.com/pypi/simple",
    "https://mirror.nju.edu.cn/pypi/web/simple",
]
```

- pixi 底层是 uv，**采用 first-index 策略**（不是 pip 的「所有索引一起选最优版本」）： 按顺序取**第一个命中该包**的索引，且 `extra-index-urls` **优先于** `index-url`。 上面 `index-url` 留空 ⇒ 默认 `https://pypi.org/simple` 作为最后兜底。
- 因此 `extra-index-urls` 的顺序 = 优先级顺序，**第一个应该写最快的源**（此处 USTC）。
- 注意 pip/uv 需要的是 PEP 503 的 `.../simple` 路径；`https://mirrors.ustc.edu.cn/pypi/` 会 301，不能直接当 index 用。

**4）其它小坑**

| 问题 | 说明 |
|---|---|
| `pixi add --dry-run` | pixi 0.77.0 **没有**这个参数 |
| `pixi --no-config search ...` | 报 `unexpected argument`；`--no-config` 必须放在**子命令之后**：`pixi search --no-config ...` |
| conda-forge 的 `mujoco` 版本落后 | conda-forge 最高 **3.12.0**，PyPI 已到 **3.14.0**；本项目选 conda-forge 以统一 C++/Python |
| `raw.githubusercontent.com` | 一度超时（curl exit 28）；稍后恢复。URDF 来源用 sha256 校验确认 |
| pixi 识别到 `__cuda=13.0` | 来自驱动 580；但本机 MX350 是 sm_61，**不支持 CUDA 13**，后续装 CUDA 相关包时要注意 |

**测速/验源可复用的命令：**

```bash
# conda 镜像是否支持 sharded repodata（200 才值得用）
curl -s -o /dev/null -w '%{http_code}\n' \
  "$MIRROR/conda-forge/linux-64/repodata_shards.msgpack.zst"

# 真实吞吐（B/s；>= 1048576 才算 1 MiB/s）
curl -s -o /dev/null -m 10 -w '%{http_code} %{speed_download}\n' "$FILE_URL"
```

### 2026-09-24 URDF → MJCF 转换与 git 索引

详见 [`@20260923_mujoco/README.md`](@20260923_mujoco/README.md)，这里只记最容易再踩的几条：

1. **MuJoCo 自带转换会把根 link 并进 `worldbody`**。`mj_saveLastXML()` / `MjSpec.from_file().to_xml()`读 URDF 得到的结果里，`trunk` 不再是 body ⇒ **没有 `<freejoint/>`，而且基座质量惯量整块丢失**（本模型 13.2472 → 7.47 kg，差值 5.7772 kg 正好是 trunk+imu_link+d435_link）。它还只导 URDF 的 `<collision>`，visual mesh 全部丢弃。
2. **urdf.enkeebot.com 的「Include Skeleton」版没有碰撞体**：19 个 geom 全是`contype=0 conaffinity=0`，用它做“趴在地上”会直接穿过地面。正确组合是**Floating Base 开 + Actuator Type = Torque + Include Skeleton 关**。
3. **同一份 mesh 被复制了 3 份**（原始 URDF、网站导出、旧导出），每份 34 MB 且 md5 完全相同。处理：模型用 `meshdir` 指向 `assets/urdf/meshes` 这一份，其余在 `.gitignore` 里排除。
4. **`.gitignore` 对已经 `git add` 过的文件无效**。重复的 meshes 与已删除的旧 skeleton 仍然留在索引里，必须 `git rm -r --cached <path>` 才能真正排除（磁盘文件不受影响）。
5. **导出模型的默认位形穿模**。urdf.enkeebot.com 把根 body 放在原点，而零位形下脚底在基座下方约 0.58 m ⇒ 任何“建完 `MjData` 直接 `mj_step`”的脚本都会看到求解器把狗弹到空中（场景里的 `<keyframe>` 不会被自动加载）。两条修法：把模型基座默认高度抬到“脚底刚好触地”（推荐，模型自洽），或在脚本里`mj_resetDataKeyframe(model, data, 0)` / 显式设 `data.qpos[2]`。
6. **`<include>` 之后 `meshdir` 是相对顶层文件解析的**。scene 与模型分目录时，模型里的`meshdir="meshes/"` 会去 scene 所在目录找 `meshes/`，而同一模型单独加载却正常。做法：每个会用到它的目录各放一个 `meshes` 目录软链接指向唯一一份实际在存储的 mesh （git 也能直接存软链接）；当然，直接在xml中改引用路径也可以。

> MuJoCo 知识点（`geom` 的 `type` 取值、`friction` / `condim` 语法与默认值、接触参数速查） 见 [`docs/mujoco-notes.md`](docs/mujoco-notes.md)。


### 2026-09-25 图形后端 / GPU / 渲染性能（本机实测）

> 数据与复现方式见 [`docs/mujoco-notes.md`](docs/mujoco-notes.md) 第 6、7 节， 图形栈背景见 [`docs/graphics-stack.md`](docs/graphics-stack.md)。

- 本机是 **Optimus 双显卡**（Intel UHD 核显负责显示 + NVIDIA MX350 2 GB 独显），实测 **`MUJOCO_GL` 会决定用哪块卡**：
  * `egl` → 离屏 context 落在 **NVIDIA MX350**（GL 4.6，max texture 32768）；
  * `glfw` → 走“显示的 GL”，即 **Intel 核显**（Mesa 23.2.1，max texture 16384）。

  所以两侧的性能差异**首先是 GPU 差异**，不是后端本身的开销。
- 渲染成本（960×540，同一套代码）：MX350 出一帧 ~5.5 ms、整条录像链路 ~7.1 ms； 核显则分别是 ~21 ms / ~22 ms（50 fps 录像要 ~111% CPU，跟不上）。
- 仓库把 `MUJOCO_GL=egl` 写进 `pixi.toml` 的 `[activation.env]`，对所有脚本生效； 临时换后端要 `pixi run env MUJOCO_GL=glfw python …`（命令行前缀会被激活环境覆盖）。
- 本机显卡算力（Pascal sm_61 + 驱动 580）**不够跑 MJX / MJWarp**，所以只用 CPU 仿真。
- 本机特有的小现象：开窗口的进程退出时偶发 `segmentation fault` 或 `GLFWError: EGL: Failed to clear current context`（MP4 在崩溃前已写完，产物不受影响）； Wayland/XWayland 会把 GLFW 的窗口位置警告打到 stderr。
- 这些结论**驱动了哪些配置**（反向索引）：见 [`docs/mujoco-notes.md` 第 7.6 节](docs/mujoco-notes.md)。

### 2026-09-25 C++ 工具链（pixi 提供）与编辑器提示

> 用途与任务背景见 [`@20260923_mujoco/README.md`](@20260923_mujoco/README.md) 的「C++ 程序」一节。

- **用 pixi 的编译器，不用系统 gcc**：conda-forge 的 `libmujoco` 是 conda 工具链编出来的，混搭系统 gcc 容易踩 ABI / libstdc++ 版本问题，所以 `cxx-compiler` / `cmake` / `ninja` 都装进同一个 env（本机实测 gcc 15.3.0 / cmake 4.4.3 / ninja 1.13.2）。
- conda 的 `mujoco` 包**已经带齐 C++ 需要的东西**：`include/mujoco/*.h`、`libmujoco.so`、`lib/cmake/mujoco/mujocoConfig.cmake`（目标 `mujoco::mujoco` 与 `mujoco::libmujoco_simulate`），所以既不用自带 MuJoCo 源码，也不用宇树 readme 里那套“下载官方包解压到 `~/.mujoco` 再 `ln -s`”。
- 配置时把 `-DCMAKE_PREFIX_PATH="$CONDA_PREFIX"` 传进去即可，`find_package(mujoco)` 就能找到上面的 CMake 配置。
- **换 C++ 不会更快**：`mj_step` 两边调的是同一份 C 库，本机实测 C++ 循环 0.0394 ms/步、Python 0.04 ms/步；渲染依旧是 5.5 ms/帧（由 GPU 决定）。
- **编辑器（clangd）要配 `--query-driver`**：CMake 把 `$CONDA_PREFIX/include` 当作“隐式包含目录”而**不写进** `compile_commands.json`，clangd 于是找不到 `mujoco/mujoco.h`、也拿不到 conda 的 libstdc++ 头，满屏标红（一次自检可报出 21 条错误）。让 clangd 去问 pixi 的编译器即可消除：`--query-driver=**/.pixi/envs/*/bin/*`（glob 必须 `**/` 开头，实测不带就匹配不上）。排查工具：`clangd --check=<file>`，它打印的就是编辑器同源的诊断。
- **这个参数要写在「工作区文件」或用户设置里，不能写在文件夹级 `.vscode/settings.json`**：clangd 扩展把 `clangd.arguments` 声明为 **window scope**，而多根工作区下 window 级设置只认工作区文件 / 用户设置，放文件夹里不生效（现象：clangd 进程参数是空的，仍然标红）。本项目放在同级的 `RoboCon.code-workspace`（该文件不入库）。
- 改完要重启语言服务器（命令面板 → `clangd: Restart language server`），否则跑的还是旧进程；想确认可以直接看进程参数：`ps -eo args | grep clangd`。
- clangd 把索引缓存写在 `<project>/.cache/clangd/`（它把含 `compile_commands.json` 的上级目录当作 project），已加进 `.gitignore`。

## 其他知识点记录 <!-- *（[我的问答链接](https://yuanbao.tencent.com/chat/naQivTmsDa/0Qgx9qPyAvQ?projectId=3daea2310a624f939a9e427e121d9c47)）* -->

### C++

#### Grammar

- *Q:* `virtual` & `explicit` & `override`：每个加或不加有什么区别、虚函数（虚类）在内存中是什么形态、虚函数和虚类有哪些性质、为何可以赋值等于0、除赋值0外还有哪些初始化方法。

	*A:*

	**1）三个关键字：加 / 不加的区别**

	| 关键字 | 加了 | 不加 | 备注 |
	|---|---|---|---|
	| `virtual` | 动态绑定（运行期按对象真实类型分派） | 静态绑定（编译期按指针/引用的**静态类型**分派） | 只对成员函数有效；**构造函数不能是 virtual**，`static` 成员不能是 virtual；`inline`/`virtual` 可共存 |
	| `explicit` | 禁止该构造函数/转换运算符参与**隐式转换**与拷贝初始化 | 单参构造函数成为 "converting constructor"，编译器可用它做隐式转换 | 只能写在**类内声明**处，类外定义处不写；C++11 起可用于转换运算符，C++20 起支持 `explicit(bool)` 条件显式 |
	| `override` | 编译器强制校验：基类必须存在**签名完全一致**的虚函数可覆盖，否则直接报错 | 签名写错（参数类型、`const`、`noexcept`、返回类型协变不符）时**不会报错**，而是悄悄"隐藏"成新函数，多态失效 | 只放在派生类声明处；与 `final`（禁止再被覆盖）可叠加 |

	```cpp
	struct A { virtual void f(int); virtual void g() const; };

	struct B : A {
			void f(int) override;        // OK，覆盖
			void f(double) override;     // 编译错误：基类没有 f(double)
			void g() override;           // 编译错误：基类是 g() const，少写 const 就不是覆盖
			void h() override;           // 编译错误：基类没有 h()
	};
	```

	**`explicit` 实例：**

	```cpp
	struct MotorId { MotorId(int id); };            // 不加 explicit
	void use(MotorId);
	use(5);                                          // OK：int 被隐式转成 MotorId

	struct MotorId2 { explicit MotorId2(int id); };
	use(5);                                          // 编译错误
	use(MotorId2{5});                                // OK：直接/列表初始化仍可用
	```
	经验法则（C++ Core Guidelines C.46）：**凡是可单参调用的构造函数，默认加 `explicit`**；拷贝/移动构造不加（加了会破坏按值返回、按值传参）。

	**2）虚函数在内存中的形态**

	编译器为每个**含虚函数的类**生成一张虚表（vtable），本质是一个编译期静态常量数组，通常放在只读数据段，内容大致是：
	- RTTI 信息指针（供 `typeid` / `dynamic_cast` 使用）
	- 该类各虚函数的入口地址，按声明顺序占据固定槽位

	每个对象里被插入一个隐藏成员 **vptr**（虚表指针），通常位于对象内存布局的**最前面**（Itanium ABI / gcc-clang；多重继承时一个对象里会有多个 vptr）。

	对象布局示意：
	```
	DMMotor 对象:
	┌──────────────┬───────────────┬─────────┐
	│ vptr (8 字节) │ position_ ... │ 其他成员 │
	└──────┬───────┴───────────────┴─────────┘
				 │
				 ▼
	DMMotor 的 vtable:
	┌──────────────────────────────────────┐
	│ RTTI 指针                             │
	│ [0] enable      -> DMMotor::enable    │
	│ [1] setPosition -> DMMotor::setPos... │
	│ [2] getPosition -> DMMotor::getPos... │
	│ [3] ~Motor      -> DMMotor::~DMMotor  │
	└──────────────────────────────────────┘
	```
	一次虚调用展开为：取对象首地址 → 读 vptr → 按固定索引取函数地址 → 传入 `this` 间接调用。这就是它无法内联、比普通调用慢的原因。

	派生类会复制基类 vtable 并把自己覆盖了的槽位改写成自己的函数地址；没覆盖的槽位保留基类地址。

	vptr 的初始化发生在**构造函数初始化列表阶段、函数体执行之前**；析构时反向把 vptr 逐层重置回当前类的 vtable——这也解释了为什么在构造/析构函数里调虚函数**不会**有多态效果（此时 vptr 指向的是当前正在构造/析构的那一层）。

	> **补充："虚类" 有两个不同含义，别混**
	> - **抽象类**（含纯虚函数的类）：不能实例化，vtable 中该槽位填的是 `__cxa_pure_virtual` 之类的桩函数，调到就报 "pure virtual function called" 并终止。
	> - **虚基类**（`class D : virtual public B`，解决菱形继承的重复子对象问题）：额外引入虚基类表/偏移量做二次间接寻址，与虚函数表是两套机制。

	**3）虚函数与抽象类的性质**

	虚函数：
	- 只能通过**指针或引用**实现多态；用对象（值）调用永远是静态绑定（还有对象切片问题）
	- 默认参数是**静态绑定**的（按指针的静态类型取），不要在虚函数里改默认参数
	- 访问权限由**静态类型**决定，与动态类型无关
	- 可以是 `inline`、`const`、`noexcept`，可以有函数体
	- 用 `final` 修饰类或虚函数可让编译器做去虚拟化（devirtualization）并内联
	- 性能敏感路径可用 CRTP 做编译期多态替代

	抽象类（含纯虚函数）：
	- **不能实例化**，但可以定义指针/引用指向派生类对象——这是接口的核心用法
	- 派生类必须实现**全部**纯虚函数，否则自己仍是抽象类
	- 可以有构造函数、析构函数、数据成员、非虚成员函数、静态成员
	- 析构函数**仍然必须声明为 virtual**
	- 可以只声明不定义（这里全是 `= 0`，连函数体都不写）

	**4）为什么可以 "赋值 = 0"**

	`= 0` 不是赋值，也不是初始化，它是 C++ 语法里专门的 **pure-specifier（纯说明符）**，语法产生式就是 `pure-specifier: = 0`，只此一种写法，没有别的数值可选。

	之所以选 `0`，是 Bjarne Stroustrup 在《The Design and Evolution of C++》§13.2.3 里说明的：当年引入抽象类时，他判断加新关键字 `pure`/`abstract` 不可能被社区接受，于是沿用 "C/C++ 里用 0 表示『不存在』" 的传统——**在"虚函数集合 = 函数指针数组"的心智模型下，把槽位置 0 就意味着"没有实现"**。这是纯粹的语法符号，与空指针、数值 0 都没有语义关联，只是有个助记作用。实际上多数编译器并不会真的填 NULL，而是填一个会报错终止的桩函数 `__cxa_pure_virtual`。

	**5）除 `= 0` 之外的"初始化"写法**

	注意：`= default` / `= delete` / `{...}` 与 `= 0` 是**互相排斥**的，同一个函数只能选一种。

	| 写法 | 含义 |
	|---|---|
	| `virtual void f() = 0;` | 纯虚：无实现（默认），类变抽象，强制派生类实现 |
	| `virtual void f() = 0;` + 类外 `void Base::f() {...}` | 纯虚**但可以带函数体**！类仍是抽象的，派生类需覆盖，但可在自己的实现里显式 `Base::f()` 复用。常用于"必须重写但提供默认实现"的场景 |
	| `virtual void f() {}` | 空实现的普通虚函数：类**可实例化**，派生类可选覆盖 |
	| `virtual ~Motor() = default;` | 显式要求编译器生成默认实现 |
	| `Motor(const Motor&) = delete;` | 删除该函数，任何调用都编译错误（禁拷贝常用） |
	| `virtual void f() final;` | 虚函数 + 禁止后续派生类再覆盖 |

	用 `= default` 而非手写 `{}` 的好处：编译器生成的实现能正确保留 `noexcept` 推导、trivial 性等属性，且将来类里加了成员也不会漏掉析构逻辑。

	结合例子逐行讲解：
	```cpp
	class Motor {
	public:
		virtual void enable() = 0;
		virtual void setPosition(double position) = 0;
		virtual double getPosition() const = 0;
		virtual ~Motor() = default;
	};
	```

	逐行：

	- **`class Motor {`**
		定义类 `Motor`。这是个**纯接口类**（interface / 纯抽象基类），等价于其他语言里的 `interface`。它规定"所有电机必须支持哪些操作"，但完全不管怎么实现。

	- **`public:`**
		访问说明符。接口成员必须 `public`，否则派生类和外部调用方都访问不到。

	- **`virtual void enable() = 0;`**
		纯虚函数：上电/使能，无参无返回值。
		- `virtual` → 运行期动态分派，`Motor*` 指向 `DMMotor` 时调 `enable()` 会调到 `DMMotor::enable()`
		- `= 0` → `Motor` 不提供实现，`DMMotor` 必须自己写
		- 这一条的存在使 `Motor` 成为抽象类，`Motor m;` 编译错误

	- **`virtual void setPosition(double position) = 0;`**
		纯虚函数：设置目标位置。参数名 `position` 在纯虚声明里只是**文档性质**（可以不写，写上便于阅读和 IDE 提示）。注意这里传的是 `double` 值拷贝，不涉及 const。

	- **`virtual double getPosition() const = 0;`**
		纯虚函数：读取当前位置。
		- `const` 修饰 `this`，承诺不修改对象 → `const Motor&` 也能调用它
		- `const` 是**函数签名的一部分**，派生类覆盖时**必须也带 `const`**，否则变成另一个函数（此时若写了 `override` 就会编译报错，这正是 `override` 的价值）
		- 返回 `double` 是值拷贝，安全；若返回引用则应写 `const double&`

	- **`virtual ~Motor() = default;`**
		**最关键的一行。** 任何作为多态基类使用的类，析构函数都必须是 virtual：
		```cpp
		Motor* m = new DMMotor();
		delete m;   // 若 ~Motor() 非虚 → 只调用 Motor 的析构，DMMotor 部分不被销毁 → 未定义行为/资源泄漏
		```
		`= default` 让编译器生成，比手写 `{}` 更规范（保留 `noexcept` 等属性）。 即使 `Motor` 是抽象类、不能被 `new` 出来，这行也不能省——因为 `delete` 是通过基类指针发生的。
		> 规律：**有虚函数 ⇒ 就应该有虚析构**；反之，不是为继承设计的类就不要加虚析构（会白白引入 vptr 开销）。

	**编译期/运行期实际发生什么：**
	- `Motor` 有自己的 vtable（编译器仍会生成），三个纯虚槽位填 `__cxa_pure_virtual`
	- 任何 `Motor` 对象都不能被创建
	- `DMMotor : public Motor` 实现三个函数后，`DMMotor` 的 vtable 槽位被替换成自己的地址，`DMMotor` 可实例化
	- 继承必须是 `public`：写成 `class DMMotor : Motor` 默认是 `private` 继承，外部无法把 `DMMotor*` 转成 `Motor*`，多态就废了

- *Q:* 一个function用const修饰意味着什么？例如下面的`getPosition`。
	```cpp
	class Motor{
		// ...
		virtual double getPosition() const = 0;
	}

	class DMMotor : public Motor {
		// ...
		double getPosition() const override { return position_; }
	}
	```

	*A:*

	**核心：成员函数末尾的 `const` 修饰的是隐含的 `this` 指针**

	普通成员函数的 `this` 类型是 `Motor* const`；加上尾部 `const` 后变成 `const Motor* const`（指向常量的常指针）。于是：

	1. **编译器强制不能修改对象状态**
		 在函数体内给任何非 `mutable` 成员赋值都会编译错误；也不能调用非 const 成员函数。
		 ```cpp
		 double DMMotor::getPosition() const {
				 position_ = 0;      // 编译错误
				 calibrate();        // 编译错误（calibrate 非 const）
				 return position_;   // OK，只读
		 }
		 ```

	2. **const 对象 / const 引用 / const 指针只能调用 const 成员函数**
		 ```cpp
		 void inspect(const Motor& m) {
				 m.getPosition();   // OK，getPosition 是 const
				 m.setPosition(1);  // 编译错误，setPosition 非 const
		 }
		 ```
		 这就是接口里 `getPosition` 必须带 `const` 的原因——否则所有只读上下文都用不了它。而 `setPosition` 天然不该带 `const`（它就是要改状态）。

	3. **const 是函数签名的一部分，构成重载维度**
		 一个类可以同时有 `double f()` 和 `double f() const`，编译器按调用对象的 const 性选择。

	4. **`override` 在这里的作用（重点）**
		 基类声明是 `virtual double getPosition() const = 0`。派生类如果写成：
		 ```cpp
		 double getPosition() override { ... }   // 少了 const
		 ```
		 编译器会报 **"marked override but does not override any member functions"**——因为 `const` 参与了签名匹配，少了 `const` 就是一个全新的函数，同时还**隐藏**了基类的版本，多态调用会出人意料。加上 `override` 就把这类笔误在编译期抓住。**这是 `override` 最典型的价值场景。**

	5. **语义含义：逻辑常量性（logical constness）**
		 `const` 表达的是"这个操作**对外**不改变对象可见状态"。若内部需要改（缓存、懒计算、加互斥锁），把成员声明为 `mutable` 即可绕开：
		 ```cpp
		 mutable std::mutex mtx_;
		 double getPosition() const { std::lock_guard lk(mtx_); return position_; }
		 ```
		 `mutable` 就是为"物理上改了、逻辑上没改"准备的。`const_cast` 也能绕过，但通常是设计有问题的信号。

	6. **返回值的 const 无关性**
		 这里返回 `double`（值拷贝），带不带 const 成员函数都不影响。若改成返回引用，就必须 `const double& getPosition() const`，否则外部能通过这个引用改内部状态，const 承诺就破了。

	7. **其他细节**
		 - `const` 成员函数里，`this` 已是 `const Motor*`，不能再 `const_cast` 后修改（除非原对象本身非 const，否则 UB）
		 - 类的静态成员函数**不能**加尾部 `const`（没有 `this`）
		 - `const` 还能与 `noexcept`、引用限定符（`&` / `&&`）叠加：`double getPosition() const noexcept &;`

- *Q:* `.hpp`和`.h`有什么功能上的区别？

	*A:*

	**结论：对编译器和预处理器而言，完全没区别。**

	`#include` 是纯文本替换，扩展名不参与语义。区别只在**约定、可读性、工具链识别**：

	| | `.h` | `.hpp`（还有 `.hxx` `.hh` `.h++` `.H`） |
	|---|---|---|
	| 语言归属 | 暧昧：可能是 C，也可能是 C++，也可能混着 | 明确表示"这是 C++ 头文件，C 编译器不该碰" |
	| 典型内容 | 声明为主 | 声明 + 模板/inline 实现（header-only） |
	| 常见出处 | POSIX、系统库、需要 `extern "C"` 的混编接口 | Boost、Eigen、多数现代 C++ 库 |
	| 工具识别 | IDE/linter 常**默认按 C 解析**（这正是 VSCode 高亮出问题的根因） | 默认按 C++ 解析 |

	历史背景：C++ 从 C 继承了 `.h`；1998 年标准把标准库头文件去掉了 `.h`（`<iostream>` 取代 `<iostream.h>`）以区分 C 头，用户自定义头文件的空缺后来由 `.hpp` 这个约定补上。

	**真正影响编译器的是源文件扩展名，不是头文件：**
	- `.c` → gcc 按 C 编译；`.cpp` `.cc` `.cxx` → 按 C++ 编译
	- 头文件扩展名不决定编译模式，只决定 IDE 怎么高亮和索引

	**实践建议：**
	- 项目内**统一**即可：`*.h + *.cpp` 或 `*.hpp + *.cpp`，别混
	- C/C++ 混编时用 `.h` 给 C 接口、`extern "C"` 包起来；C++ 专用用 `.hpp`
	- 模板密集、header-only 库偏向 `.hpp`；模板实现单独抽文件时常用 `.tpp` / `.ipp`（在头文件末尾 `#include "xxx.tpp"`）
	- 无论用哪个，都要配 `#pragma once` 或 include guard

- *Q:* 直接在`class Class{...}`内定义成员，和在外部加前缀`Class::`来定义/重载有什么区别、分别在什么时候用。

	*A:*

	**两者的本质区别：`inline` 与编译/链接单元**

	| | 类内定义 | 类外 `Class::` 定义 |
	|---|---|---|
	| 链接属性 | **隐式 inline**，可以在多个 TU 中出现而不违反 ODR | 非 inline，**整个程序只能有一份定义**，否则链接报 multiple definition |
	| 通常位置 | 头文件（`.h`/`.hpp`） | 若非 inline，必须放 `.cpp` |
	| 改实现的影响 | 所有 include 该头文件的 `.cpp` 全部重编译 | 只需重编译这一个 `.cpp` |
	| 编译依赖 | 实现里用到的类型必须在头文件可见（污染头文件依赖） | 可用前向声明 + Pimpl 隐藏依赖 |
	| 代码膨胀 | 每个 TU 一份内联代码，可能变大 | 只有一份代码 |
	| 运行开销 | 可被内联，无调用开销 | 普通函数调用（LTO 下也可能内联） |

	```cpp
	// Motor.hpp（类内定义）
	class Motor {
	public:
			int id() const { return id_; }   // 隐式 inline，OK
	private:
			int id_;
	};

	// Motor.hpp（只声明）
	class Motor {
	public:
			int id() const;
	private:
			int id_;
	};

	// Motor.cpp（类外定义）
	int Motor::id() const { return id_; }
	```

	**类外定义的语法要点：**
	- 要重复返回类型、类名、`::`、以及 `const` / `noexcept` / 引用限定符等尾部限定
	- **默认实参只在类内声明处写一次**，类外定义处不能重复写
	- `static` 关键字只在类内写，类外定义处不写 `static`
	- `virtual` 关键字只在类内写，类外定义处**不能**写 `virtual`（写了编译错误）
	- `explicit` 同理，只写在类内声明处
	- 但 `override` / `final` 只在类内声明处写（它们本就是声明属性）

	**关于"重载"（overload）：**
	- **新增重载只能在类内声明**。不能在类外凭空 `Class::f(double)` 加一个新签名——类外只能**实现**类内已经声明过的东西。
		```cpp
		class Motor {
		public:
				void set(double);      // 必须先在类内声明
				void set(int);         // 重载也要在类内声明
		};
		void Motor::set(double d) { ... }   // 类外实现
		void Motor::set(int i) { ... }
		void Motor::set(float f) { ... }    // 编译错误：类内没声明过
		```
	- 类外也**不能**覆盖/添加虚函数的新版本。

	**必须用类内（或同头文件内）定义的情况：**
	- **模板**：模板的定义必须在实例化点可见，通常整个写在头文件里（或 `#include "xxx.tpp"`）
	- `constexpr` / `consteval` 函数（需要在编译期看到定义）
	- 静态数据成员：C++17 起可以 `inline static int x = 5;` 在类内定义；C++17 之前类内只是**声明**，必须在某个 `.cpp` 里 `int Class::x = 5;` 定义一次（C++17 后 `inline` 变量放宽了这个限制）

	**什么时候用哪个：**
	- **类内**：一两行的 getter/setter、空实现、模板、需要高频内联的小函数、header-only 库
	- **类外（放 `.cpp`）**：函数体较大、实现依赖很多头文件或第三方库签名、需要隐藏实现细节、希望减少重编译范围
	- 经验：先把函数体放 `.cpp`，只有确认是热路径或必须暴露时再挪进头文件

### CMake

- *Q:* 各命令名称中的`executable`、`library`和`target`指代分别是什么？

	*A:*

	**CMake 最核心的抽象是 target（目标）**——一个构建系统要产出的"工件"，它自带一组属性（源文件、包含目录、编译选项、链接库、依赖关系……）。三类命令围绕它分工：

	| 命令 | 作用 | 名称里的词指什么 |
	|---|---|---|
	| `add_executable(<name> ...)` | **创建**一个可执行程序 target | `executable` = "我要造的 target 的类型是可执行文件" |
	| `add_library(<name> [STATIC\|SHARED\|MODULE\|OBJECT\|INTERFACE] ...)` | **创建**一个库 target | `library` = "我要造的 target 的类型是库" |
	| `target_link_libraries(<target> ...)`、`target_include_directories(<target> ...)`、`target_compile_definitions`、`target_compile_options`、`target_sources`、`target_precompile_headers`、`target_compile_features`、`set_target_properties` … | 往**已存在**的 target 上**挂属性** | 第一个参数 `<target>` 就是之前 `add_executable` / `add_library` 里给的那个 `<name>` |

	也就是说：
	- 命令名里字面出现的 **`executable` / `library`** 是"要造什么**类型**的 target"
	- 命令名里的 **`target`** 是"往哪个**已存在**的 target 上操作"

	例子：
	```cmake
	add_library(robot_core STATIC src/motor.cpp src/dm_motor.cpp)   # 造出 target: robot_core，产出 librobot_core.a
	add_executable(robot_app src/main.cpp)                           # 造出 target: robot_app，产出可执行文件 robot_app

	target_include_directories(robot_core PUBLIC include)            # 给 robot_core 挂包含目录
	target_link_libraries(robot_app PRIVATE robot_core)              # robot_app 链接 robot_core
	```

	**target 名 ≠ 文件名：** target 名是 CMake 层面的逻辑标识；实际产出文件名由 `OUTPUT_NAME`、`CMAKE_STATIC_LIBRARY_PREFIX/SUFFIX` 等决定（Linux 下 `robot_core` → `librobot_core.a`，`robot_app` → `robot_app`）。

	**其他几类 target：**
	- `add_custom_target(<name> ...)`：不产出文件的伪目标（如 `doc`、`format`），总被认为"过期"，每次都执行
	- `add_custom_command`：产出文件的自定义命令，不是 target，但可被 target 依赖
	- **IMPORTED target**：`add_library(xxx SHARED IMPORTED)`，代表一个外部已编译好的库
	- **ALIAS target**：`add_library(robot::core ALIAS robot_core)`，给 target 起带命名空间的别名，推荐在 `target_link_libraries` 里用别名（拼错会报错，拼错原名不会）

	**为什么要用 `target_*` 而不是老的全局命令：**
	- 全局版 `include_directories()` / `link_directories()` / `add_definitions()` 是**目录作用域**的：只影响该 `CMakeLists.txt` 及子目录中**在它之后定义**的 target，顺序敏感、难维护
	- `target_*` 版带 **usage requirements** 传播机制：`PUBLIC` → 既给自己用也传给依赖方；`PRIVATE` → 只给自己用；`INTERFACE` → 只传给依赖方（自己不用，用于 header-only 库）
	- 现代 CMake 风格（"Modern CMake" / Effective CMake）核心原则：**一切围绕 target 和它的 usage requirements**

- *Q:* ```cmake
	add_library(robot_core
		# ...
	)
	```
	和
	```cmake
	add_library(
		robot_core
		# ...
	)
	```
	是否有区别？

	*A:*

	**没有任何语义区别，两种写法完全等价。**

	CMake 语言的规则：命令调用形如 `command_name(arg1 arg2 ...)`，参数之间由**空白字符**（空格、Tab、换行）分隔。**换行符在参数列表中就是普通的分隔空白，等价于一个空格。** 只要命令名紧跟左括号，参数在括号内怎么换行、换行多少都不影响解析。

	同理，下面这些也全都等价：
	```cmake
	add_library(robot_core STATIC a.cpp b.cpp)
	add_library(robot_core
			STATIC
			a.cpp
			b.cpp)
	add_library  (robot_core STATIC a.cpp b.cpp)   # 命令名与括号间有空格：多数版本能解析，但不推荐
	```

	**真正会让换行产生差异的只有两种情况：**
	1. **引号参数内的换行是字面内容**：
		 ```cmake
		 set(MSG "第一行
		 第二行")    # MSG 里真的含有一个 \n
		 ```
	2. **方括号参数 `[[...]]` 内的换行也是字面内容**：
		 ```cmake
		 set(SCRIPT [[
		 echo hello
		 ]])         # SCRIPT 含换行
		 ```

	另外 `#` 到行尾是注释——如果某行被 `#` 注释掉了，那一整行的换行自然也就"消失"了，这是唯一可能因为换行位置而意外合并参数的地方（但这是注释造成的，不是换行本身）。

	**风格建议：** 参数多时，每个源文件/关键字独占一行。理由：
	- 可读性好
	- `git diff` 更干净（增删一个源文件只影响一行）
	- 与 `.clang-format` 对齐后团队一致

- 坑点：在添加dm电机后，别忘了修改`CMakeLists.txt`。

- 【杂项】VSCode C++ & CMake IntelliSense: [docs/cmake-intellisense.md](docs/cmake-intellisense.md)
- 【杂项】图形 / 渲染 / 视频栈速查（OpenGL、Skia、DirectX、EGL、GLFW 都是什么）: [docs/graphics-stack.md](docs/graphics-stack.md)
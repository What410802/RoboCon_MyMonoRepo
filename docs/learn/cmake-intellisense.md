# VSCode C++ & CMake IntelliSense

**先分清两件事——很多人把这两件事混为一谈：**
- **语法高亮**：由文件的"语言模式"决定，语言模式选中某个 TextMate grammar 来着色。语言模式由扩展名 + `files.associations` 决定。**不需要任何编译信息**。
- **IntelliSense（补全/跳转/红色波浪线）**：由语言服务器（微软 C/C++ 扩展的 cpptools，或 clangd）提供，**需要知道真实的编译参数**。

所以"高亮不对"和"全是红色波浪线"是两个问题，要分别处理。

**第 1 步：确认语言模式（解决高亮）**

看 VSCode 右下角状态栏的语言标识。如果打开 `.h` 文件显示的是 `C` 而不是 `C++`，`class` / `template` / `namespace` / `override` 就不会正确着色。

- 临时修：点击状态栏语言标识 → 选 `C++`（只对当前文件当前会话生效）
- 永久修：在 `.vscode/settings.json` 里加文件关联
	```jsonc
	{
		"files.associations": {
			"*.h":   "cpp",
			"*.hpp": "cpp",
			"*.tpp": "cpp",
			"*.ipp": "cpp",
			"*.inc": "cpp",
			"*.tcc": "cpp",
			"CMakeLists.txt": "cmake"
		}
	}
	```
	如果项目里同时有纯 C 的头，用路径限定更精确：
	```jsonc
	"files.associations": {
		"**/include/**/*.h": "cpp",
		"**/src/c_api/*.h":  "c"
	}
	```

**第 2 步：装对扩展（二选一，对应下面的方案 A / 方案 B）**

- **方案 A（推荐给多子项目 / 大项目 / Linux 重度用户）**：`clangd`（`llvm-vs-code-extensions.vscode-clangd`）
	- 需要先装本体：`sudo apt install clangd`（Ubuntu）
	- 再配 `"C_Cpp.intelliSenseEngine": "disabled"` 关掉微软插件的语义引擎，只保留它的调试能力（或改用 CodeLLDB）
- **方案 B（微软官方，生态整合最好）**：`C/C++`（`ms-vscode.cpptools`）+ 可选 `CMake Tools`（`ms-vscode.cmake-tools`）

**两个语言服务器会打架**（两套诊断、两套跳转、互相拖慢），不要同时开着做语义分析。

**第 3 步：生成编译数据库（两个方案共用，最关键）**

CMake 默认不导出，要显式开（`Unix Makefiles` 和 `Ninja` 两种 generator 都支持，不用为此换 Ninja）：
```cmake
# 每个子项目的顶层 CMakeLists.txt 都加
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
```
或每个子项目各自 configure 一次：
```bash
cmake -S projA -B projA/build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake -S projB -B projB/build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```
目标结构（本文后面都以它为例）：
```
workspace/                  ← VSCode 打开这一层（单工作区）
├── projA/{CMakeLists.txt, src/, build/compile_commands.json}
├── projB/{CMakeLists.txt, src/, build/compile_commands.json}
└── projC/{CMakeLists.txt, src/, build/compile_commands.json}
```
这份文件记录每个源文件的真实编译命令（`-I`、`-D`、`-std`），语言服务器读它就能 100% 准确解析。非 CMake 项目用 `bear -- make` 生成。

> **前提约定**：build 目录**必须叫 `build`** 且位于各自子项目根下——clangd 的零配置完全依赖这条约定。若你叫 `out/`、`build-dbg/`，clangd 侧要退回 4A 的 `.clangd` 写法。

---

## 方案 A：clangd —— 零配置（推荐）

**核心认知：clangd 是 per-file 查找，不是 per-workspace。** 每打开一个源文件，它就单独为这个文件找一次数据库：从文件所在目录逐级向上，且**每一级祖先目录里额外检查名为 `build/` 的子目录**。所以 `projA/src/foo.cpp` 命中 `projA/build/`，`projB/src/bar.cpp` 命中 `projB/build/`，天然隔离。

**⚠️ 最容易踩的坑：不要设 `--compile-commands-dir`。** 这个 CLI 参数优先级最高，一旦指定，clangd 就**只**看这一个目录、不再向上查找，多子项目会被全部按同一套 flags 解析，彻底乱套。单项目教程里那行 `"--compile-commands-dir=${workspaceFolder}/build"` 在多子项目下是**有害**的。

所以 `.vscode/settings.json` 只需要这些：

```jsonc
{
	// —— 高亮（第 1 步）——
	"files.associations": {
		"*.h": "cpp", "*.hpp": "cpp", "*.tpp": "cpp", "*.ipp": "cpp",
		"CMakeLists.txt": "cmake"
	},

	// —— clangd ——
	"clangd.arguments": [
		"--background-index",                              // 后台建立全局索引
		"--clang-tidy",                                    // 顺带跑静态检查
		"--completion-style=detailed",
		"--header-insertion=never",                        // 别自动塞 #include
		"--query-driver=/usr/bin/g++,/usr/bin/clang++,/usr/bin/arm-none-eabi-*"  // 交叉编译时必加
	],

	// —— 关掉 cpptools 的语义引擎，只留它的调试功能，避免两套诊断打架 ——
	"C_Cpp.intelliSenseEngine": "disabled",

	// —— 别让插件扫描 build 目录 ——
	"files.watcherExclude": { "**/build/**": true },
	"C_Cpp.files.exclude":  { "**/build/**": true }
}
```

`--query-driver` 是交叉编译（机器人项目常见 `arm-none-eabi-gcc`）下能不能拿到正确内建头路径的关键，不加 clangd 会放弃探测编译器而报一堆标准库找不到。

**只有 build 目录不叫 `build/` 时才需要 `.clangd`**（放在工作区根，用 `---` 分片）：
```yaml
If:
	PathMatch: projA/.*
CompileFlags:
	CompilationDatabase: projA/build-dbg     # 给"目录"，不是文件路径
---
If:
	PathMatch: projB/.*
CompileFlags:
	CompilationDatabase: projB/out
```
也可以**每个子项目各放一份自己的 `.clangd`**（只写 `CompileFlags: CompilationDatabase: build-dbg`，不加 `If`），更内聚。优先级：用户配置 > 内层项目 > 外层项目。

**验证**：
```bash
clangd --check=projA/src/foo.cpp     # 打印它实际采用的编译命令
```
或 Output 面板 → clangd，看 `Loaded compilation database from .../projA/build/compile_commands.json`，再打开 projB 的文件确认它会切换。

---

## 方案 B：微软 cpptools —— 单工作区 + `c_cpp_properties.json` 数组

**和 clangd 的本质差异**：cpptools 是"给整个工作区找**一份**数据库"。它的自动检测语义是找到多份时弹下拉框让你**挑一个**；且当数据库里没有当前文件对应的条目时，会**回退**到 `includePath` + `defines`（状态栏显示 "Configure IntelliSense"）。所以挑了 projA 那份，projB 的文件就全部失效。

**所以必须显式列出全部来源，写在 `.vscode/c_cpp_properties.json` 里**（`compileCommands` 官方支持数组）：

```jsonc
// .vscode/c_cpp_properties.json —— 单工作区 + 多子项目
{
	"configurations": [
		{
			"name": "Multi-Project",
			"compilerPath": "/usr/bin/g++",
			"cStandard":  "c17",
			"cppStandard": "c++17",
			"intelliSenseMode": "linux-gcc-x64",
			// 关键：数组，把每个子项目的编译数据库都列进来
			"compileCommands": [
				"${workspaceFolder}/projA/build/compile_commands.json",
				"${workspaceFolder}/projB/build/compile_commands.json",
				"${workspaceFolder}/projC/build/compile_commands.json"
			]
		}
	],
	"version": 4
}
```

**⚠️ 别搞混两个同名字段**：
| 位置 | 字段 | 能填几个 |
|---|---|---|
| `.vscode/settings.json` | `C_Cpp.default.compileCommands` | **单值字符串**，只能一份 |
| `.vscode/c_cpp_properties.json` | `compileCommands` | **数组**，可以列多份 |

多子项目只能走后者。

**`compilerPath` 建议照写**：它让扩展去查询编译器的系统头路径和预定义宏；`intelliSenseMode` 要与编译器/OS 匹配（Linux 是 `linux-gcc-x64`，只写 `gcc-x64` 属 legacy 但会自动转换）。

**备选写法：一个 configuration 对应一个子项目**
```jsonc
{
	"configurations": [
		{ "name": "projA", "compilerPath": "/usr/bin/g++",
			"compileCommands": "${workspaceFolder}/projA/build/compile_commands.json" },
		{ "name": "projB", "compilerPath": "/usr/bin/g++",
			"compileCommands": "${workspaceFolder}/projB/build/compile_commands.json" }
	],
	"version": 4
}
```
适合子项目之间编译器/标准差异大的情况（比如一个是 C++17 本机、一个是交叉编译），用状态栏的 `C/C++: Select IntelliSense Configuration` 手动切换。

**验证**：`Ctrl+Shift+P` → `C/C++: Log Diagnostics`，看输出的 include 路径是不是来自你预期那个子项目的 `-I`。若看到 `${workspaceFolder}/**` 之类的兜底路径，说明匹配失败、已回退。

---

## 补充方案（按需取用）

**C. 合并成一份数据库**（兼容性最好，clangd / cpptools / clang-tidy / ccls 通吃）

用 `jq` 把所有子项目的数据库合并到工作区根：
```bash
# 一次性
jq -s 'map(.[])' projA/build/compile_commands.json projB/build/compile_commands.json > compile_commands.json
```
我写了带去重、幂等的脚本（`./merge_compile_commands.sh` 自动搜索，或 `-o` 指定输出），也可以挂到 `tasks.json` 的 post-build 里。

合并后两侧都只需指向这一份：
```jsonc
"C_Cpp.default.compileCommands": "${workspaceFolder}/compile_commands.json"
```
**硬伤**：如果子项目用**不同编译器/架构**（本机 x86 g++ vs `arm-none-eabi-gcc`），合并进同一份会让语言服务器用错 driver。这种情况回到方案 A 的 `.clangd` 分片。

**D. 让 CMake Tools 当配置提供者**（最贴合 CMake 工作流）
```jsonc
// .vscode/c_cpp_properties.json
{ "configurations": [{ "name": "Linux", "configurationProvider": "ms-vscode.cmake-tools" }],
	"version": 4 }
```
CMake Tools 支持多项目，状态栏显示 active folder，且**默认会根据当前编辑的文件自动切换 active project**（`cmake.autoSelectActiveFolder` 默认 true）。单根目录下的多个子项目用 `cmake.sourceDirectory` 配多个路径即可。 **注意**：`configurationProvider` 优先级**高于**你手写的 `includePath`/`defines`，配了就别再手工维护两套矛盾的包含路径。

**E. 多根工作区**（`.code-workspace`） 不打开父目录，而是把每个子项目加成一个 workspace folder，各自目录下放自己的 `.vscode/c_cpp_properties.json`。或在 `.code-workspace` 里按 folder 分别给 settings（`${workspaceFolder}` 在此解析为**该 folder 自己的路径**，所以两行字面相同却各指各的）：
```jsonc
{ "folders": [
		{ "path": "projA", "settings": { "C_Cpp.default.compileCommands": "${workspaceFolder}/build/compile_commands.json" } },
		{ "path": "projB", "settings": { "C_Cpp.default.compileCommands": "${workspaceFolder}/build/compile_commands.json" } }
] }
```
这是隔离性最好的结构，代价是 `.code-workspace` 需要维护，且 clangd 仍是一个窗口一份实例。

---

## 通用补充配置

```jsonc
// .vscode/settings.json
{
	// 保存时格式化：项目根放 .clang-format，团队统一风格
	"editor.formatOnSave": true,
	"[cpp]": { "editor.defaultFormatter": "xaver.clang-format" },
	"[c]":   { "editor.defaultFormatter": "xaver.clang-format" },

	// 索引性能：别扫构建产物
	"files.watcherExclude": { "**/build/**": true, "**/.git/**": true },
	"C_Cpp.intelliSenseCacheSize": 2048,      // MB，默认过大时磁盘 I/O 吃紧
	"C_Cpp.workspaceParsingPriority": "low"   // 大项目避免开窗口瞬间抢满 CPU
}
```
```yaml
# .clang-format（项目根）
BasedOnStyle: Google
IndentWidth: 4
ColumnLimit: 100
```
```yaml
# .clang-tidy（项目根，配合 --clang-tidy）
Checks: >-
	-*, bugprone-*, performance-*,
	modernize-use-override, modernize-use-nullptr
WarningsAsErrors: ''
HeaderFilterRegex: '.*'
FormatStyle: file
```

---

**排查清单：**
1. 右下角语言模式是不是 `C++`？→ 配 `files.associations`
2. 每个子项目的 `build/compile_commands.json` 都存在吗？→ 开 `CMAKE_EXPORT_COMPILE_COMMANDS` 并各自 configure
3. 目录**是不是就叫 `build`**？不是 → `.clangd` 或合并方案
4. clangd 侧：确认 `settings.json` 里**没有** `--compile-commands-dir` → 它锁死单一目录，是多子项目失效的头号原因
5. cpptools 侧：确认是写在 `c_cpp_properties.json` 的**数组**里，而不是 settings.json 的单值字段
6. `compile_commands.json` 里 `directory` 字段是不是绝对路径？相对值（如 `"."`）会让 cpptools 拼错 `-I` 路径
7. clangd 和 cpptools 是不是都开着？→ 二选一
8. 换了配置没生效？→ `clangd: Restart language server` / `C/C++: Reset IntelliSense Database`
9. 高亮颜色难看（不是高亮错误）→ 配色主题问题，换主题（Dark+ / One Dark Pro），与 C++ 配置无关
10. 想确认某个 token 被当成什么 → `Developer: Inspect Editor Tokens and Scopes`

**配置分层建议（和项目/机器耦合度相关）：**
- **用户级 settings.json**（`Ctrl+Shift+P` → `Preferences: Open User Settings (JSON)`）：放和这台机器相关的东西，如 `compilerPath`、`clangd.path`
- **工作区 `.vscode/`**：放项目属性，如 `files.associations`、`cppStandard`、`compileCommands` 数组、`.clangd` / `.clang-format` / `.clang-tidy`——跟着项目走，别人 clone 下来直接能用

**一句话结论**：clangd = 什么都不配，只要目录叫 `build/` 且不设 `--compile-commands-dir`；cpptools = 必须在 `c_cpp_properties.json` 里把所有子项目的数据库**列成数组**。

**本项目的实际配置**：conda／pixi 工具链特有的问题（`--query-driver` 要写在工作区文件里、clangd 找不到 conda 头文件）与最终采用的写法，见 [`../pitfalls/environment.md`](../pitfalls/environment.md) 的「C++ 工具链（pixi 提供）与编辑器提示」一节。 C++ 语法侧的问答笔记见 [`cpp-cmake.md`](cpp-cmake.md)；文档索引见 [`../../README.md`](../../README.md)。

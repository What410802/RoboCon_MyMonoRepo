# cpp/：任务 4 的落点

任务 2 的 C++ 版在 [`../cpp_task2/`](../cpp_task2/)（平地场景 + 零力矩静止趴卧、静止判定、与 Python 侧逐项对照的数字）。

本目录留给任务 4：把 [`../python/`](../python/) 的双缓冲结构用 C++ 复刻一遍——物理线程独占 `mjData`、渲染只读快照副本、锁只罩 memcpy，之后再接官方 `Simulate` 界面或自带渲染循环。

目前这里只有本说明；下一步放 `CMakeLists.txt` 与 `src/`，构建目录用 `cpp/build/`（已被 `.gitignore` 忽略）。

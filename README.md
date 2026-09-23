# RoboCon 培训学习仓库

## 任务记录

+ `@20260923_robot_cpp_training`


## 知识点记录

### C++

#### Grammar

- `virtual` & `explicit` & `override`：每个加或不加有什么区别、虚函数（虚类）在内存中是什么形态、虚函数和虚类有哪些性质、为何可以赋值等于0、除赋值0外还有哪些初始化方法。[complete]

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

	[complete]

- 一个function用const修饰意味着什么？例如下面的`getPosition`。
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

	[complete]

- `.hpp`和`.h`有什么功能上的区别？

	[complete]

- 直接在`class Class{...}`内定义成员，和在外部加前缀`Class::`来定义/重载有什么区别、分别在什么时候用。

	[complete]

### CMake

- 各命令名称中的`executable`、`library`和`target`指代分别是什么？

	[complete]

- ```cmake
	add_library(robot_core
		// ...
	)
	```
	和
	```cmake
	add_library(
		robot_core
		// ...
	)
	```
	是否有区别？
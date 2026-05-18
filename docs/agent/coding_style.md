# C++ Coding Style

本文只规定 C++ 命名风格和 clean code 硬约束。

## 1. 命名风格

1. 类名、结构体名、枚举名、类型别名使用大驼峰：`SlamTrackRequest`、`VisualFeatureSet`。
2. 函数名使用大驼峰：`TrackRaw`、`PrepareStereoImagesForTracking`。
3. 命名空间使用大驼峰：`SmartDrone::Core::Ports`。
4. 类成员变量使用 `m_` 前缀加小驼峰：`m_frameId`、`m_leftImage`。
5. 全局变量使用 `g_` 前缀加小驼峰：`g_runtimeMode`。
6. 全局常量使用全大写加下划线：`MAX_FEATURE_COUNT`。
7. 局部变量使用小驼峰：`frameId`、`leftImage`。
8. 函数参数使用小驼峰：`inputFrame`、`trackRequest`。
9. 结构体成员使用小驼峰：`leftKeypoints`、`descriptorScore`。
10. 宏名使用全大写加下划线。
11. 命名必须表达业务含义，避免拼音、无意义缩写和 `tmp1`、`data2` 这类编号式命名。

## 2. Clean Code 要求

1. 不允许自定义编译宏。
2. 单个函数小于 50 行。
3. 单个函数参数不大于 5 个；超过时必须使用参数对象。
4. 一个函数只做一件事。
5. 一个类只承担一个清晰职责。
6. 删除重复代码；相同逻辑出现第二次时应抽取函数或公共模块。
7. 优先使用早返回降低嵌套层级。
8. 嵌套层级不超过 3 层。
9. 注释只解释原因、约束和非显然逻辑，不重复代码本身。
10. 不保留无用代码、失效注释和调试残留。
11. 修改已有文件时保持邻近代码风格一致；新增代码遵守本文规则。

![pioneer_badge](.asset\picture\pioneer_badge.png)

***SCNU-PIONEER (c) 2024-2025***

[TOC]

# 框架配置

## 框架目录

如下为`robot_frame_24`主体框架目录

> [robot_frame_24 coding仓库URL](https://scnu-pioneer.coding.net/p/ec/d/robot_frame_24/git)

```C
─robot_core                                      
    ├─Algorithm        // PIONEER电控常用算法库   
    ├─APP              // 机器人应用顶层
    ├─Base             // 机器人应用底层
    ├─Devices          // 外接机器驱动
    ├─OS               // 操作系统
    ├─tests            // 测试相关
    ├─tools            // 调试工具                                    
    └─drv_conf.h       // 机器人配置文件
```

框架搭配电机库`MOTOR`食用，目录如下

> [MOTOR coding仓库URL](https://scnu-pioneer.coding.net/p/ec/d/MOTOR/git)

```C
─MOTOR                                      
    ├─motor_lib            // 电机库协议层   
    ├─motor_utils          // 电机库相关数学工具
    ├─motor_ctrl.c         // 电机库应用顶层  
    ├─motor_ctrl.h         // 电机库应用顶层
    ├─motor_def.h          // 电机库数据类型定义
    ├─motor_headers.h      // 电机库被引用接口    
    ├─motor_conf_sample.h  // 电机库配置文件的范例                      
    └─motor_conf.h         // 电机库配置文件 (需要用户自己实现)
```

## 框架背景

本框架基于`robot_frame_21` 改造，更名为`robot_frame_24` ，设计原型为**裸机**，在适配越来越丰富的赛场功能的同时，对于模块化处理的概念更加显著，降低调试难度，对于拥有“底盘、云台、发射机构”的机器人在代码层面能够得到高度统一，增强大家的沟通有效性和提升全组的开发效率。

> 使用新框架的机器人：工程、经典步兵、平衡步兵、英雄、哨兵、无人机

值得一提的是，相比于旧版框架，**电机驱动相关底层被单独隔离，并存在于另一个远端仓库中**。

随着队伍使用的电机种类越来越多，旧版的电机驱动相关底层早已无法适配大部分适用情况，而将电机库从框架中隔离开单独开发更新，在所有人都使用正确的“多人协作”方式开发时，使得电控组能够持续不断更新电机的驱动方式，同时新的内容也更加方便同步更新给全组。

框架`robot_frame_24`现已适配三种主控板

> board_c (STM32F407IGHx)
>
> board_dm_mc02 (STM32H723VGTx)
>
> board_TAN (STM32G473CBTx) —— **现已停止维护！**

但本框架并非尽善尽美，并且开发者也承认其许多细节上未经过仔细推敲，在"框架"的概念上与现代化流行方案有弊性质的区别，以下是使用本框架时需要着重注意的点！

1. **本框架以C语言作为主编程语言，但部分实现思想为面向对象的编程思想，整体为面向过程和面向对象混合，且有许多模块并未完全封装，仍有部分变量暴露，未得到高度私有化**
2. **本框架的有限状态机并非属于传统“有限状态机FSM”概念范畴，而更加原始，仅满足战队当下技术水平使用**
3. **本框架沿用战队前辈撰写的****UI****驱动文件，有无法渲染字符的缺陷**
4. **本框架在启用操作系统方面的稳定性需要时间验证**
5. **截止框架定稿，控制主脑仍为****单片机****（嵌入式主控板），未实现上位机全权控制，小电脑仅为"运算单元"**

## 框架技术栈

本框架建立在工程之上，可选两种技术栈进行开发

**若惯用系统为Windows，建议使用方案一，但仍推荐开发使用Linux系统并学习方案二的嵌入式开发技术栈**

一：

**CubeMX** - **Vscode - Keil - FreeMaster** (Windows系统)

CubeMX生成ARM-GCC的工程基础，使用Vscode编写代码，Keil进行编译烧录和少部分调试，FreeMaster进行参数整定和相关调试

点击根目录下的.code-workspace文件即可进入Vscode的工作区

二：

**CubeMX** - **Vscode - Cmake - Ninja - arm-none-eabi-gcc - OpenOCD - FreeMaster**(支持Windows&Linux)

CubeMX生成工程并提供CMakeList，Vscode编写代码，arm-none-eabi-gcc+Cmake+Ninja高速编译，OpenOCD&FreeMaster烧录调试

该方案摒弃Keil，使用更加现代化的开发方案，但也受到OpenOCD只在特定版本支持dap-link的影响

# 框架详解

认识一个框架，需要从两个维度进行

1. 框架如何运行（宏观层面）
2. 框架的具体实现（具体代码层面）

> 看懂框架 ≠ 看懂代码
>
> 看懂代码 ⊇ 看懂框架 

看懂框架可以对框架进行初步使用和调试

看懂代码可以对框架进行开发

对于框架需要先有一个大概的认识，也就是先有一个宏观的认识，来作为完成“看懂代码”这个最终任务的主线

本文档不对行句进行具体解释，只阐释大概含义，省去纷繁解释之余，留下更多开发的构想，**并且以步兵作为主体**

初始化进程存在于`main`函数(主要使能外设)和`robot_init`函数内(主要对用户实现逻辑进行抽象含义的初始化)

1. `main`中的初始化：在初始化进程中配置DWT定时器、开启机器人的定时器外设、启用或关断部分PWM通道
2. `robot_init`中的初始化：在初始化进程中执行主线任务（底盘任务、云台任务、发射机构任务），以及其他必要的任务（IMU解算、异常检测等）对应的初始化函数
3. *若`drv_conf`中的`OS_ENABLE`被置一，则代表使用操作系统，`main`函数将关停任务定时器，并同时执行`task_create` 创建操作系统的各个任务

### **主线**

一台机器人的主线就是完成本职工作，对于经典步兵机器人来说，包括底盘运动、云台运动、正常发射弹丸、视觉模块的使用等

#### 机器人配置 ./drv_conf

机器人配置文件 `drv_conf.h`，本文件不允许调用任何框架内其他模块的头文件以及使用任何变量

机器人配置文件用于设定机器人自身特殊的配置

1. 时钟频率定义
2. 外设句柄定义
3. 是否使用操作系统
4. 各任务的执行频率或是中断优先级
5. 板间通信相关配置
6. 机构相关配置
7. 其他功能性裁剪配置

##### **时钟频率定义**

**这些配置因主控板选择不同和机器人设计策略不同而变更**

```C
#define CORE_FREQ 168           // 主频 Mhz
#define TIM3_IT_FREQ (10.f)     // 10hz(bus_detect定时器)
#define TIM9_IT_FREQ (1250.f)   // 1250Hz IMU特供
#define TIM6_IT_FREQ (200.f)    // 200hz(中速定时器)
#define TIM7_IT_FREQ (1000.f)   // 500hz(高速定时器)

#define M_TIM_FREQ TIM6_IT_FREQ
#define H_TIM_FREQ TIM7_IT_FREQ
#define IMU_TIM_FREQ TIM9_IT_FREQ
#define DETECT_TIM_FREQ TIM3_IT_FREQ
```

开发者需要配置时钟频率，这些宏将会被多处调用解决相关数学问题，在`main` 也会根据这些宏配置定时器的时钟预分频PSC和自动重装载值ARR

```C
// 以下数值是综合考虑对应APB总线频率和ARR、PSC为16位或24位寄存器后的结果
// 若为16位寄存器 取值范围为0-65535(即2^16-1)
M_TIM_HANDLE.Instance->ARR = 1e4f / M_TIM_FREQ - 1; // 200hz
M_TIM_HANDLE.Instance->PSC = 8400 - 1; // 该定时器挂载的APB的频率是多少就是多少乘100-1

H_TIM_HANDLE.Instance->ARR = 1e4f / H_TIM_FREQ - 1; // 500hz
H_TIM_HANDLE.Instance->PSC = 8400 - 1;
IMU_TIM_HANDLE.Instance->ARR = 1e4f / IMU_TIM_FREQ - 1; // 1250hz
IMU_TIM_HANDLE.Instance->PSC = 16800 - 1;
DETECT_TIM_HANDLE.Instance->ARR = 1e4f / DETECT_TIM_FREQ - 1; // 10hz
DETECT_TIM_HANDLE.Instance->PSC = 8400 - 1;
```

低速定时器推荐配置10Hz，主要供给异常检测和调试工具的使用，不太重要的任务尽量都低一些

中速定时器推荐配置200Hz

高速定时器推荐配置500Hz

**对于任何事物，相应频率都是越高越好的（你的电脑就是以GHz为单位的），但是****单片机****有很大的硬件局限性**

1. **主频受限**：执行每一条语句都是需要实现的，特别是计算复杂的数学问题时会消耗更多时间（如高维矩阵运算），目前使用的主频最快的单片机为喵板，H723，主频最高位550MHz
2. **通信负载**：若高频任务下挂载了通信相关的发送操作，可能会引起数据丢失或数据溢出的问题。例如CAN通信中存在协议的物理极限，用**CAN负载上限**来描述这个极限，正是因为这个极限的存在，**一条总线上能搭载的节点数量总是受限的，一条总线上能承载的信息量总是受限的**

> **对于一路经典的****CAN****，最多能接受在1000Hz下挂载6个大疆电机**

##### **外设句柄定义**

```C
#define M_TIM_HANDLE htim6
#define H_TIM_HANDLE htim7
#define IMU_TIM_HANDLE htim9
#define DELAY_TIM_HANDLE htim5
#define DETECT_TIM_HANDLE htim3

#define BEEP_TIM_HANDLE htim4
#define BEEP_TIM_CHANNAL TIM_CHANNEL_3

#define MAGAZINE_TIM_HANDLE htim1
#define MAGAZINE_TIM_CHANNAL TIM_CHANNEL_2

#define WS2812_TIM_HANDLE htim8
#define WS2812_TIM_CHANNAL TIM_CHANNEL_1
```

很明显，像【htim6】【htim7】【htim9】这样的变量会因为主控芯片的选择而不同（或CubeMX中配置不同而不同），在配置文件中统一配置处理，可以使得后续开发的调用中不会受此影响而编译报错

##### **关于操作系统**

```C
///< OS_ENABLE 置1时使用操作系统
#define OS_ENABLE 0 
```

当使用操作系统时，系统在初始化时会创建所需的任务，包括但不限于主线任务、异常检测任务等，具体执行逻辑请看根目录下`OS/robot_task` 

本框架使用的操作系统为FreeRTOS，且多为动态内存规划，推荐在熟悉FreeRTOS相关使用方法后再启用，否则容易出现`HardFault`

**本框架主要使用到操作系统的****任务时间片调度、抢占式调度、互斥锁****等功能**

时间片调度和抢占式调度主要代替裸机下硬件定时器的功能

互斥锁主要解决任务抢占中出现的内存访问冲突的问题（不论是裸机还是操作系统，都尽可能使对全局变量的操作为“原子操作”！！！）

是否启用操作系统主要考虑以下几点

1. 若有很多需要定时完成而硬件定时器不够用，或有很多需要调度器辅助完成的任务以节省带宽（如UI系统），建议启用
2. 若有任务的触发频率很接近甚至超过1000Hz，则不建议启用，FreeRTOS的最大调度频率只有1000Hz

> **PS**：**使用操作系统，是嵌入式开发中一个优雅的动作**，但并不是在任何情况下都需要启用的，对于需要调度执行的项目，启用操作系统确有一定的优势，但在一台标准的步兵机器人中（至少对于现在的框架来说），操作系统只是一个软件定时器，而当机器人需要用到超过1kHz运行的进程时，操作系统将无能为力。**但诚然，操作系统有无限潜力，虽本框架的默认版本并没有使用，但不代表它不好，使用操作系统必定是行业内嵌入式开发的基本要求，期待未来的队员能够继续在此基础有所耕耘**

##### **裸机需注意**

**在裸机的版本中，使用硬件定时器全权代理所有任务的定时执行，但同时也受主控硬件定时器的数量限制**

选择两个硬件定时器作为定时执行控制任务，在C板中使用TIM6和TIM7，分别作为中速定时器和高速定时器，具体执行任务在后续说明

另外还需要选择一个定时器用于异常检测，一个定时器用于做IMU解算，一个定时器用于实现阻塞式延时

```C
#define M_TIM_FREQ TIM6_IT_FREQ
#define H_TIM_FREQ TIM7_IT_FREQ

#define IMU_TIM_FREQ TIM9_IT_FREQ
#define DETECT_TIM_FREQ TIM3_IT_FREQ

#define DELAY_TIM_HANDLE htim5
```

##### **板间通信相关配置**

使用本框架的机器人支持双板通信和单板控制

当使用单板控制时，`BOARDS_MODE`置0

当使用双板通信时，`BOARDS_MODE`置1，同时需要配置当前板的类型

```C
// 如果为上板（主机）
#define MACHINE_TYPE GIMBAL_MASTER
// 如果为下板（从机）
#define MACHINE_TYPE CHASSIS_SLAVE
```

除此之外，还需配置当前板的通信总线端口、通信频率通信数据包的基础帧头

```C
#if MACHINE_TYPE == GIMBAL_MASTER
    // 配置上下板对应的通信端口
    // 配置上下板的通信频率上限
    #define BOARDS_INTERACT_CAN CAN1
    #define BOARDS_INTERACT_CAN_HANDLE hcan1
    #define INTERACT_TIM_FREQ G2C_INTERACT_TIM_FREQ
#else
    #define BOARDS_INTERACT_CAN CAN2
    #define BOARDS_INTERACT_CAN_HANDLE hcan2
    #define INTERACT_TIM_FREQ C2G_INTERACT_TIM_FREQ
#endif
// 配置基础帧头
// 实际包帧头为基础帧头 + 包id
#define G2C_CAN_BASE_ID 0x010
#define C2G_CAN_BASE_ID 0x020
```

##### **机构相关配置**

配置文件中需要对机构的控制频率进行配置，意在统一后续在具体机构处理时的时间变量

```C
// 使用了上文提到的中速定时器和高速定时器
#define CHASSIS_TIM_FREQ M_TIM_FREQ
#define GIMBAL_TIM_FREQ H_TIM_FREQ  // 云台需要更高精度  默认放置在高速loop中
#define SHOOTER_TIM_FREQ M_TIM_FREQ

// 不同的底盘构型标签
#define MECANUM 1
#define OMNI 2
#define WHEELLEGGED 3
#define SWERVE 4

#define CHASSIS_TYPE OMNI

#define GENERAL_NUM_OF_WHEEL 4
#define BALANCED_NUM_OF_WHEEL 2
```

不同的底盘构型决定了在底盘底层中的解算逻辑，例如麦轮底盘和全向轮底盘，底层结算逻辑肯定是不一样的，这里的配置会影响`base_chassis` 中的解算方法选择

##### **其他功能性裁剪配置**

配置文件中有一些宏用于相关功能的快速开启和关断，

在日常开发和调试中，时常需要选择性关闭一些功能，此处已提前设计好一些常接触功能的关断，避免使用者执行同样操作时在具体实现中乱作修改而出现更多bug

```C
#define USE_REFEREE 0           ///< 是否使用裁判系统
#define READY_FOR_BATTLE 0      ///< 为0不会触发阵亡检测
#define USE_DT7_WHEEL 1         ///< 如果DT7遥控器拨轮坏了就置零不启用
#define USE_TEST_FILE 0         ///< 测试功能启用文件test.c 
#define USE_GIMBAL_RESET 1      ///< 使能云台缓启动
#define USE_CHASSIS_RESET 1     ///< 使能底盘缓启动
#define USE_CENTER_FIRE 0       ///< 使能中心火控
#define SPIN_DEBUG 0            ///< 给视觉调试的时候置1得到匀速小陀螺

#define USE_ASR 1               ///< 是否使用ASR播报功能(需要接入喇叭)
#define USE_WS2812 1            ///< 是否使用WS2812显示状态
```

#### 机器人任务 ./APP/robot

机器人执行总览文件`robot.c` 及其头文件`robot.h` 

头文件中包含了绝大部分驱动文件和算法文件，驱动文件如电机库、IMU、串口、CAN等，算法文件如pid、filter等，因为均为模块化开发，并遵循层级设计的逻辑，所以不会出现头文件互相包含的错误而导致编译失败

**注**：其实一个头文件包含大量头文件是一个不太优雅的做法，实际上会有很多冗余的包含关系，主要是笔者比较懒，这样做方便笔者在另一个文件直接包含`robot.h` 就能获得大量信息

##### 机器人初始化

```C
void robot_init(void)
```

在本函数中执行了框架相关的初始化进程，除串口外，不包括直接的外设配置相关

**初始化进程是一项不希望被任何事件打断的进程**，**故在初始化的开头会关闭所有的中断，禁止当前进程被抢占（即进入“临界区”），相应的，在初始化最后会解除这种禁制（即退出“临界区”）**

那么初始化到底初始化了什么？

在并不熟悉具体运行逻辑和机器人所被要求的所有动作之前，硬生生理解所有的初始化是一件困难的事情

在宏观意义上，对其进行阐释如下：

遥控器初始化：打开用于接收无线遥控信息的串口，配置相关传输策略

裁判系统初始化：裁判系统同样使用串口接收，故同上

状态机初始化：对初始状态进行装载

底盘机构初始化：注册底盘机构上的电机（何为**“注册”**，在下文关于电机库的说明会提到，可以先简单理解为“构造函数”），对底盘的状态量进行初始化

云台机构初始化：同上

发射机构初始化：同上

测试文件初始化：在本框架定稿时，测试文件主要完成功能为【测试电机】，故也需要对电机对象进行注册

双板通信初始化：注册双板通信相关的数据包（注册含义同电机的）

UI初始化：在旧版UI实现中为创建静态图像，实现UI数据包的预装载（UI的实现是把数据通过特定协议发给裁判系统，裁判系统再将这些数据解码并通过图传模块显示）

WS2812初始化：WS2812是灯珠，在框架中把每一颗灯珠都抽象成对象，采用面向对象的形式控制，因此在初始化中注册所有对象（注册含义同）

ASR初始化：**ASR 意为** **Automatic Speech Recognition** **，****自动语音识别****。**做这个东西的本意是为了**提高调试效率，让机器人能够把调试信息“说出来”**，甚至能够识别环境的声音甚至是人类的自然语言，来作出一定的反应，例如让自然语言完全代替遥控器，实现**“人喊车动”**的功能。这个模块仍在施工中，处于方案探索的阶段（目前的方案为采用Twen的ASR_PRO芯片，coding仓库中有对应实现）

IMU初始化：**IMU 意为 Inertial Measurement Unit ，惯性测量单元。**我们在C板、喵板、TAN上搭载的IMU都为BMI088，这是一个六轴传感器，在初始化程序中需要通过SPI协议对芯片进行初始化。

##### 任务执行封装

```C
void robot_imu_tim_cycle(void)
void robot_state_update(void)
void robot_bus_detect(void)
void robot_gimbal_ctrl(void)
void robot_chassis_ctrl(void)
void robot_shooter_ctrl(void)
void robot_main_loop(void)
void robot_test_loop(void)

void robot_delay_us_tim(uint32_t period)
void robot_delay_us_systick(uint32_t period)
```

光看函数名和代码里的注释其实就能知道这些函数是用来干嘛的了，这些封装函数将会被相应的操作系统任务函数或定时器任务函数调用，将整个系统进行任务上的细分，独立或递进执行，会让调试工作变得更加条理清晰

在部分任务的开头有相关代码：

```C
// 这里用测试文件的举例子
test_dt = DWT_GetDeltaT(&test_dwt_cnt);
if(!test_dt) test_dt = 1.f/TEST_TIM_FREQ;
```

`dt` 就是时间间隔的意思，是程序上次一次经过这里至本次经过这里的间隔时间，单位为秒

实现方法为利用STM32中的隐藏定时器**“DWT”**来实现，原本是供给给烧录器辅助使用的，框架封装了相关API，来源于王工，详细可见`Base/bsp_dwt` 和上网查阅

状态机任务持续**不断接收来自“上层”的控制信息**（【上层】有可能指代**DT7遥控器**，也有可能指代**相机图传**，也有可能指代**导航上位机**，或是别的什么在未来会研发的控制源），并将其转换为更加简单的数字信号，改变机器人的状态量，控制状态之间的切换逻辑，详细可见`APP/behaviour` 中描述和实现了各个机器人状态量的作用

> PS：其实这一版实现的状态机并不是严格意义上的状态机，只是面向RM赛场的控制信息处理中心，对于标准意义状态机中的“状态切换处理”，受限于单片机的体量，可能并没有体现得那么完善。
>
> （笔者眼中的状态机是Unity Editor引擎中的状态机系统，感兴趣可以去试试开发Unity，C#语言开发，做游戏挺好玩的✩~）

机器人结构运动任务（底盘任务、云台任务、发射机构任务）执行后，会直接进入到其顶层，后续会继续讲解

异常检测任务会持续检测诸如电机是否掉线、其他各类机构在线异常等，并驱动相应的调试工具输出信息告知开发者

IMU解算任务会持续将IMU的原始数据转化为可用的加速度、角速度、欧拉角等信息，同时本任务会将这些数据上传到上位机以供自瞄系统计算其他机器人相对机体的位置信息

```C
now_timestamp += 1000.f / (uint32_t)IMU_TIM_FREQ; // 单位ms
```

机器人下位机会持续积分时间戳，得到已经过的时间

```C
void diag_slow_time_cycle(void) // 低速任务
void robot_m_tim_cycle(void) // 中速任务
void robot_h_tim_cycle(void) // 高速任务
void robot_imu_tim_cycle(void) // IMU解算任务
```

若使用裸机，`robot` 中还实现了提供给有限定时器的任务封装函数

毕竟受到定时器的数量限制，所以相应任务的控制频率受限也会比较多，比如底盘控制和发射机构控制都在中速定时器中实现，那么两者的控制频率都为`M_TIM_FREQ`

这些函数会被定时器回调函数`void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)`调用

回调函数的重写位于`main`中

```C
// C板！！！
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  #if OS_ENABLE == 0
  if (htim == &htim3) {
   diag_slow_time_cycle();
  }
  if (htim == &htim9) {
   robot_imu_tim_cycle();
  }
  if (htim == &htim6){
   robot_m_tim_cycle();
  }
  if (htim == &htim7) {
   robot_h_tim_cycle();
  }
  #endif
  /* USER CODE END Callback 1 */
}
```

#### 状态机 ./APP/behaviour

##### robot_t

状态机头文件`behaviour.h` 定义了大部分机器人的状态量

```C
robot_basic_mode_t; //基本模式
robot_rc_t;         //控制源

chassis_tag_t;      // 底盘姿态
gimbal_tag_t;       // 云台姿态
shooter_tag_t;      // 发射模式

robot_state_t;      // 机器人运动状态量
imu_data_t;         // 机器人姿态

robot_t;            // 机器人状态量
```

代码中已经通过注释对含义解释详尽，这里就不再过多列举说明

主要关注到robot_t的定义

```C
typedef struct {
    // 一级状态
    bool robot_control_flag  ;    // 机器人泄力状态
    bool robot_survival_flag ;    // 机器人存活状态

    // Master控制状态
    bool use_supercap_flag   ;    // 超电期望启用使能标志
    bool use_radar_flag      ;    // 雷达期望启用使能标志
    bool use_fric_flag       ;    // 摩擦轮期望启用使能标志
    bool use_vision_flag     ;    // 挂期望启用使能标志

    bool fire_flag           ;    // !!!开火!!!
    bool rush_flag           ;    // !!!开创!!!  

    chassis_tag_t chassis_tag,chassis_tag_last    ;   // 底盘控制模式
    gimbal_tag_t gimbal_tag,gimbal_tag_last       ;   // 云台控制模式
    shooter_tag_t shooter_tag,shooter_tag_last    ;   // 发射控制模式

    // 被动更新二级状态
    bool reset_flag          ;    // 相应机构没有缓启动完成不会主动改变任何tag
    bool keyboard_flag       ;    // 键盘启用使能标志
    bool charge_request_flag ;

    // 其他 (定义为在双板中不重要的状态量)
    enum _magazine_switch_flag{
      MAG_OFF = 0u,
      MAG_ON,
      UNCONTROL,
    }magazine_switch_flag;                  // 弹仓补充机构开关

    robot_basic_mode_t robot_basic_mode;    // 机器人基础控制模式
    robot_rc_t rc_sc;                       // 机器人控制源

    robot_state_t *expected_state, *current_state;
    
    // INS
    imu_data_t *imu_data;
    imu_data_t *ext_imu;
    imu_data_t *chassis_imu;
}robot_t;
```

1. **一级状态**：**表示这些状态拥有最高权限**，可以“一票否决”相关行为，从字面上看其实很好理解，机器人泄力状态量`robot_control_flag`是决定机器人受不受控的依据，**当此标志被置零时，机器人将不会受到任何形式的控制，“泄力”一词意味着此时所有动力系统都将取消输出**，**保证安全**；机器人存活状态量`robot_survival_flag` 在比赛中会影响上述的泄力状态，在比赛中机器人被击败后应直接“趴窝”，不应输出任何力，**否则会被视为严重违规而直接罚下**
2. **Master控制状态**（主动更新二级状态）：**Master意为主机，这是一个拥有主从概念的框架**，从机全权受控于主机，当控制链路中只存在上下双板时，一般上板（位于云台上的主控板）为主机，下板为从机。在设计中，上板和下板都使用本框架，需要修改地方仅为`drv_conf` 中的双板通信配置项**（详见上文）**,Master控制状态由主机控制，从机只读。详见注释和数据结构内容能够了解这些量的具体含义
3. **被动更新二级状态**：该项状态量不由控制源主动修改，通常经过一些条件检测后被动更新

机器人基础控制模式`robot_basic_mode_t`：普通模式（经典遥控作战）、自动模式（未开发，为以后做半自动预留）、检录模式（方便比赛前检录各项的模式，此模式和普通模式的遥控器映射不同）

机器人控制源`robot_rc_t`：包括遥控器DT7、图传VT、主机BOARDS、雷达RADAR，通过特定机制切换控制源（机制这块可以再琢磨琢磨）

##### 状态机进程

```C
void behaviour_init(robot_t* robot)
```

初始化函数在机器人初始化阶段执行，对各个状态量进行初始化，赋上默认值，并且确保初始化时为“泄力”状态保证安全

```C
void behaviour_ctrl_loop(rc_ctrl_t *rc_recv, robot_t* robot)
```

状态机更新函数入口参数为遥控命令结构体指针和机器人结构体指针

遥控命令来源于`Base/rc` 中的解算，遥控有两种，一种是图传链路，一种是DT7遥控器

目前暂不支持雷达作为控制源（还没开发出半自动步兵呢）

函数执行步骤如下：

1. **选择控制源**，现版本中默认尝试优先选择DT7遥控器，若判定DT7遥控器离线，则切换为图传链路
2. **阵亡检测**，实时检测自身血量是否归零（血量数据来源于裁判系统），归零则判定为死亡，将`robot_survival_flag` 置零
3. **控制机器人上下控**，检测遥控命令中对应`robot_control_flag` 的映射，同时也受到阵亡检测的影响，`robot_control_flag` 置一称为上控，`robot_control_flag` 置零称为下控或断控
4. **选择使用键鼠映射还是DT7遥控映射**，DT7遥控器除了支持实物遥控操控，还支持连接电脑使用键鼠操控，两者对应的映射关系不一样，这一点通过`keyboard_flag` 来实现映射方法的改变。（若使用图传链路控制，默认为键鼠映射，毕竟使用图传链路就是基于电脑上的客户端的，肯定只能用键鼠）
5. **若上控，则根据控制源对二级控制量进行更新，否则重置状态量**

**如果上一状态为断控，本次状态为上控，那么将会执行“缓启动”程序（在后面会详细讲到缓启动）**

缓启动结束后，会根据控制源对二级控制量进行更新

**若控制源为主机BOARDS（即当前板为从机板）**，则不对任何Master控制状态量进行更新，因为这些都来源于主机，从机只读

**若当前板就是主机，将从键鼠和遥控之间选择**（遥控器还是图传链路在之前选择控制源的时候已经完成选择了，对应的方法映射已完成，只需将数据结构体传入）

```C
if(robot->reset_flag == FINISH){ 
    if(robot->rc_sc == DT7_CONTROLLER || robot->rc_sc == VT_CONTROLLER){
        if(robot->keyboard_flag == ENABLE) //使用键鼠
            update_robot_status_for_keyboard(rc_recv,robot);
        else/*(robot->keyboard_flag == DISABLE*/ //使用遥控器
            update_robot_status_for_remoteControl(rc_recv,robot);
    }  
}
```

##### 缓启动

**机器人需要缓启动，主要是出于安全考虑**。当机器人不存在缓启动，直接上控，那么很有可能会闭环当前姿态而导致剧烈运动，虽然可以通过闭环回到正常，但在此期间运动幅度大、速度快、力饱和等都会产生安全隐患

另外对于平衡步兵来说，当底盘失去缓启动时，若此时机体处在墙边且向墙倾倒，则很有可能在向墙体俯冲尝试起身时卡墙，导致最终无法成功起身

因此缓启动的设置是有必要的

在代码注释中有如下说明

> *云台与底盘的握手机制 云台先缓启动 然后轮到底盘缓启动 都缓启动完成后才会传输完成标志位* 

值得一提的是，本框架的主体是经典步兵，**故在底盘缓启动执行期间不会有任何动作，并默认为底盘缓启动已完成**；另外在云台的缓启动中有一系列较为重要的机制，联系到两个关键的表达：“**云台真缓启动**”、“**云台缓启动**”。

这将会在下文关于云台的部分中详述

##### 泄力

机器人泄力的本质就是电机不输出任何力矩，恒定输出为零

```C
void motor_switch(uint8_t switch_flag)
```

框架使用本函数控制机器人泄力，函数被实现在`behaviour` 中，被调用在低速定时器（或异常检测任务）

Q：为什么不用在每个机构任务的结尾？

A：机构任务的执行频率高，低速定时器推荐为10Hz，考虑到CAN负载，所以实现了对全部电机进行泄力的此函数在低速定时器中影响全局。而在实际测试中，频率降到了10Hz也完全够用，对安全保障并无太大影响

#### 底盘顶层 ./APP/chassis

##### 底盘初始化

```C
void chassis_init(void)
```

函数负责对底盘电机进行注册，注册信息包括**电机的种类**、**工作模式**、**所挂载的****CAN****端口**、**ID**、**减速比**、**输出低通滤波器**（本电机库会给电机默认需求一个低通滤波器作为输出滤波）、**父机构**

**另外函数装载底盘任务所需的所有PID对象**

##### 底盘控制顶层

```C
void chassis_ctrl_loop(float dt, robot_t *robot)
```

入口参数为此函数的实际运行间隔和机器人结构体指针

前者由`robot` 中算出并通过任务封装传值进来

后者来自`behaviour` 处理出所有的状态信息，显然，底盘受控于`behaviour` ，而`behaviour` 信息来自于再上层的控制源（遥控器、图传链路、雷达等），控制链路是一级一级往下传递而不出现耦合的，上一级的变量不会被下一级篡改，避免越级导致的代码屎山

该函数的内部处理逻辑即其目的大致为：**装载当前值（cur），根据状态机设置期望值（exp）**

#### 底盘底层 ./Base/base_chassis

底盘底层中实现各种类型底盘的底层解算，即描述了**机体的质点运动****到底盘****各电机的期望力矩****输出的****映射**

用麦克纳姆轮举例子，在代码文件一开始就实现了正解和逆解：

```C
/*麦克纳姆轮运动学正向解算，即从四轮轮速换算到整车速度 轮速单位:rpm*/
void mecanum_forward(general_wheels_state_t *cur_wheel, robot_state_t *cur_state){
  cur_state->v_x = C_WHEEL * (cur_wheel->M_LF - cur_wheel->M_RF + cur_wheel->M_LB - cur_wheel->M_RB) / 4 / 60;
  cur_state->v_y = C_WHEEL * (KXY_BACK * cur_wheel->M_RF + KXY_BACK * cur_wheel->M_LF - KXY_FRONT * cur_wheel->M_LB - KXY_FRONT * cur_wheel->M_RB) / (2 * (KXY_BACK + KXY_FRONT)) / 60;
  cur_state->w_z = (D_WHEEL / 2) * (cur_wheel->M_RF + cur_wheel->M_LF + cur_wheel->M_LB + cur_wheel->M_RB) / (2 * (KXY_BACK + KXY_FRONT));
}
/*麦克纳姆轮运动学逆向解算，即从整车速度换算到四轮轮速 轮速单位:rpm*/
void mecanum_reverse(robot_state_t *exp_state, general_wheels_state_t *exp_wheel){
  exp_wheel->M_RF = (60 / C_WHEEL) * (-exp_state->v_x + exp_state->v_y) + KXY_FRONT / (D_WHEEL / 2) * exp_state->w_z;
  exp_wheel->M_LF = (60 / C_WHEEL) * (exp_state->v_x + exp_state->v_y) + KXY_FRONT / (D_WHEEL / 2) * exp_state->w_z;
  exp_wheel->M_LB = (60 / C_WHEEL) * (exp_state->v_x - exp_state->v_y) + KXY_BACK / (D_WHEEL / 2) * exp_state->w_z;
  exp_wheel->M_RB = (60 / C_WHEEL) * (-exp_state->v_x - exp_state->v_y) + KXY_BACK / (D_WHEEL / 2) * exp_state->w_z;
}
```

##### 撰写脉络

**底盘底层的实现因机器人的不同构型而有所区别，具体实现需要根据实际情况而定，但大致的脉络是不变的**

1. **对传递进来的当前值做数据处理**
2. **对传递进来的期望值进行到电机的映射**
3. **实现对上述过程的闭环**
4. **基于对功率限制而对最终输出限幅**

在本框架默认的四轮步兵底层结算中（麦克纳姆轮、全向轮）

基本按照上述脉络进行编写

**对于四轮步兵的底层结算：**

**理论上来说，只需要将机体期望速度通过解算映射到各个轮子，并与轮子当前的转速做增量式的pid运算，从而计算出轮电机需输出的力矩，也就是使用四组pid即可完成**

**但是实际代码中在做解算映射之前，使用额外三组pid对机体期望速度做增量式pid运算，先计算出最终期望速度，再进行到轮的解算，再用四组pid完成轮力矩的计算**

**实际的实现其实相当于串联了pid进去，会比单环控制更好一些，但相对的因为pid结构数量的增多，需要整定的参数也会变多，这是一种关于取舍的选择**

#### 功率限制 ./Base/power_ctrl

截至本文档撰写完成，框架使用的功率限制方案为2024赛季电控组曾昭庆师兄的方案

以下摘自`power_ctrl.h`

> - *功率限制原理:*
> - *参考:**https://www.bilibili.com/video/BV1tu41177wm,**来自西交利物浦的分享~*
> - *电机输入功率满足P=Kn\*omega\*Iq + ESR\*Iq^2 + MLC\*omega^2,*
> - *其中Kn、ESR、MLC均为常值, 所以上述关系可以简化为P=F(omega,Iq)*
> - *应用时首先可以根据expt_omega与expt_Iq得到expt_P,又已知咱们需要限制的最大功率max_P,*
> - *那就可以按照各电机的expt_P之比分配max_P得到set_P*
> - *接下来就是需要将set_P转为set_Iq了,观察功率模型,可以发现以Iq为未知量时式子其实就是一个一元二次方程,*
> - *omega为当前角速度,这样就可以得到每个电机在有最大功率限制时的每个时刻应该有的输出了*

功率限制会对机器人加速度造成较大影响，特别是在小陀螺开启下的运动

在当前研发版本中，功率限制对平衡机器人影响较大，容易出现超功率死亡的问题，更需要保证平衡步兵拥有更加稳定的控制能力，在开发中实现多种应对不同工况的处理手段

#### 超级电容 ./Base/super_cap

超级电容的相关实现与功率限制部分强相关

主要处理从超级电容控制板发送给当前主控的信息（本版本为CAN通讯）

得到车载超级电容的实时信息

1. 供给功率限制使用，实时动态更新策略
2. 让操作手能够观测机器人的“能量”使用情况，主动避免超功率

#### 云台顶层 ./APP/gimbal

##### 云台初始化

与底盘初始化极为相似，不再赘述

注意一些专属云台机构的初始化变量即可

如初始化函数中出现的leso和kalman，意味着云台并非使用简单的双环pid控制，下文相应地方会讲解

##### 云台控制顶层

###### **计算云台当前位置**

> **首先需要了解一个关于坐标系的概念，何为机体系？何为世界系？**
>
> **这是一个三维运动学概念的问题，两者最重要的区别是选取什么作为参考系**
>
> **如果我们的研究对象是云台，并且IMU放在云台上**
>
> **那么机体系的参考系为底盘，世界系的参考系为大地**

```C
pitch_rad = motors->pitch.real.abs_angle / PITCH_RR;
yaw_rad = motors->yaw.real.abs_angle / YAW_RR;
pitch_rad = range_map(pitch_rad, 0, 2 * PI);
if(pitch_rad < 0)pitch_rad += 2*PI;
yaw_rad = range_map(yaw_rad, 0, 2 * PI);
if(yaw_rad < 0)yaw_rad += 2*PI;
```

这里是电机位置到云台机体系位置的映射，其中abs_angle为电机的多圈值，考虑云台机械上的减速结构（如传动带、传动杆等，都可能具有不等于1的减速比），最终对结果进行周期的映射处理，结果即为云台机体系的位置

**对应世界系，可以直接读取IMU的欧拉角**

###### **选择基于哪个坐标系控制云台**

```C
gimbal_ctrl_type_t select_gimbal_ctrl_type(robot_t *robot, gimbal_reseting_state_t reset_state)
```

传入的第二个参数为云台的缓启动标志

**若缓启动已完成，则使用世界系进行控制**

**若缓启动未完成，则使用机体系进行控制**

> **为什么要设计两种坐标系控制方式？**
>
> **考虑机器人在未受控制的一般情况（如战死），重新受控的第一个行为，应该是将头部摆正，该过程底盘不运动（即底盘不动，云台相对于底盘动，运动到一个相对于底盘的指定位置，该位置指向底盘正前方），当然也可以选择头部不动而相对运动底盘，但是显然前者更加安全！**
>
> **要实现这样一个运动，机体系是必要的，因为该行为的期望是相对机体而言，而非世界。**
>
> **而在缓启动完成后（即平时的默认运行方式），则使用世界系以便于外界交互**

###### **关于云台缓启动**

云台缓启动分为两种：**真缓启动**，**缓启动**

两者都属于云台缓启动，用机体坐标系进行控制，但是**前者量纲为电机编码**，**后者量纲为弧度**

根据前文所述，后者是基于多圈值abs_angle计算得到的，然而abs_angle的零点需要先确定，才能积分出这个多圈值，因此想要使用后者，就必须先使用前者进行零点标定！

> **也许会有人提出疑问：那每一次云台缓启动都使用真缓启动不就好了，为何要多此一举？**
>
> **这个问题关乎到机械结构，若云台的转动需要减速机构（传动带）的参与，且该减速比大于1，则使用真缓启动会出现位置错误**
>
> **举个栗子：yaw轴存在减速比为2的传动带，yaw轴电机转两圈相当于云台转动一圈，则机体中出现一前一后两处电机零点，根据取劣弧算法，在缓启动时云台会选择离当前位置最近的零点为目标移动，所以会有可能出现云台缓启动完成后的位置与预期位置相反，这并非我们所期望的**
>
> **故只要真缓启动成功且正确，定义机体系的真实零点，归一化整个圆域，那么之后的所有缓启动不论初始在何处都可以缓启动到正确的位置上**

需要注意的是，第一次缓启动即**真缓启动**仍然受传动带减速比诟病的影响，故仍然需要在**真缓启动**前摆正方向，保证第一次缓启动到正确位置上

该部分逻辑存在于

```C
/**
 * @brief 云台缓启动解算函数
 * @attention 复位的第一次缓启动结束的位置决定了机器人0点 故要保证摆好方向再上电启动
 * @note is_confirm_0degeree 决定了缓启动是以电机中心编码值作为参考 还是以云台rad中心作为参考 会被need_change_yaw_center抢占
 * @param exp_state 机器人期望状态指针
 * @param pmotors 电机集指针
 * @retval gimbal_reset_state 返回缓启动状态标识
 */
gimbal_reseting_state_t gimbal_smooth_reseting(robot_state_t *exp_state,motors_t *pmotors)
```

考虑到现实的诸多不确定性，框架设计了主动进行一次**真缓启动**的接口`need_change_yaw_center`，允许操作手在局中通过操作触发该标志以随机应变

###### **计算限幅角度**

实际上，云台的俯仰角只能在一定的范围运动，所以云台在运行过程中，常常需要进行限位

限位分为两种：机械限位（硬限位）、电控限位（软限位）

若机器人只存在机械限位，控制俯仰角的电机会经常在限位处堵转，这会对电机造成不可逆的损伤，因此进行电控限位是必要的

**因步兵机器人具有能够360°转动的云台，所以本框架中只对pitch轴做限制，不对yaw轴做限制**

**若要对yaw轴进行限制可以参考2025赛季无人机代码**

**实现电控限位即计算出限位角度范围，在给定角度期望超过该范围时则限制该期望在该范围内**

需注意其中一些量会因为电机反装与否而导致极性错误（这也是框架写的不够好的一个地方，未能做到适配）

###### **处理并传递实际值和期望值**

在实现上与底盘控制顶层的脉络极为相似，期望值根据云台tag的不同而不同，另外如果启用了自瞄并触发时，期望值为自瞄系统计算出的角度而非遥控输入

注意要对不做限制的自由度期望进行周期上的处理

```C
exp_state->yaw_ang = get_minor_arc(exp_state->yaw_ang, 0, IMU_RANGE); // 使电机响应不过来的时候不会误判(比如快速掉头)
```

**传递到云台底层的实际值和期望值一定为相同量纲！**

若为机体系控制，且未确认零点，则量纲为电机的编码

若为机体系控制，且已确认零点（即完成过真缓启动后），则量纲为弧度制

若为世界系控制，则量纲为IMU编码（框架中为8192）

#### 云台底层 ./Base/base_gimbal

云台底层实现**机体的期望位置变换****到底盘****各电机的期望****输出的****映射**

该部分与底盘底层极为相似

需要注意的是，底层运算策略会根据电机期望输出的物理含义而改变

若电机输出量纲为**力矩**，则通过**简单双环PID**对**角度差**和**转速**闭环计算输出力矩

若电机输出量纲为**电压**，则通过**融合LESO的双环PID**对**角度差**和**转速**闭环计算输出电压（使用GM6020电机作为云台电机的机器人多用这种方式）

若电机输出量纲为**弧度**，则根据**电机当前角度**和**期望角度差**计算出本次的目标位置（使用DM4310可用）

#### IMU解算 ./APP/imu_bmi088

队伍机器人使用BMI088作为惯性测量单元（IMU）

本框架中集成了BMI088的驱动，在开发中一般不需要关注这些底层文件

本文件为IMU的上层，过程函数实现对BMI088的**原始数据获取**和**数据处理**，并且在执行每个过程之前都会检查BMI088芯片的工作情况

原始数据为三轴的角速度和加速度，环境温度，请查阅数据手册了解量纲

数据处理过程包括：**对数据进行修正和补偿**（包括IMU不在机体中心的相关修正）、**对数据进行滤波**、**基于四元数的过程计算欧拉角**

IMU的相关数据在关于状态机中提及，原始数据、滤波后数据、欧拉角、温度都定义在结构体`imu_data_t`中

```C
typedef struct{
  imu_data_raw_t raw_data; // 单位 m/s^2  rad/s
  imu_data_fp_t fp_data; // 单位 m/s^2  rad/s
  eular_t eular; // 单位 rad
  float temperature; // 单位 ℃
}imu_data_t;
```

阅读分支board_c分支下imu_bmi088.c的内容，可以发现数据优化的算法是卡尔曼滤波

在分支board_dm_mc02分支下，imu_bmi088.c则提供了两种数据优化的算法：互补滤波，卡尔曼滤波

#### 自瞄解算 ./APP/vision

##### **与上位机通信**

下位机和上位机通过USB_DEVICE进行通信

下位机发送的数据包定义和接收的数据包都在头文件中有定义，根据上下位机工作的行为会有不一样的数据传输情况（如导航和自瞄，而自瞄中又分为瞄能量机关和瞄装甲板）

下位机发送的数据包大多为比赛实时信息（如自身颜色、ID等）、在世界系的欧拉角（为上位机提供欧拉角进行位置解算）

##### **自瞄**

车载的MINIPC向下位机发送数据包，数据包中包括敌方机器人在世界系下的运动情况，以及被视觉系统识别到的装甲板的相关参数等

利用这些参数计算出当前云台应设置什么样的位置作为期望角，使得弹丸能够落在敌方装甲板上

自瞄的算法根据视觉给出的方案而定

计算出的角度通过函数`vision_ctrl` 通过指针传值

```C
uint8_t vision_ctrl(float *pitch_ang, float *yaw_ang)
```

#### 发射机构 ./APP/shooter17(shooter42)

2025赛季后的地面兵种取消实体补弹，中供弹或下供弹成为了硬性要求

若机器人使用中供弹，则发射机构（对于步兵而言为两个摩擦动力电机和一个拨弹电机）全部存在于云台，全部由上板控制

若机器人使用下供弹，则拨弹电机存在于底盘，由下板控制

因发射机构的具体实现因机器人发射弹丸口径不同而不同，故此处不再详述

##### **发射机构初始化**

与云台和底盘初始化极为相似

##### 发射机构控制

与云台和底盘很相似

1. 获取实际状态量
2. 根据Tag设定期望(设定)状态量
3. 根据裁判系统反馈的热量信息限制拨弹电机的输出
4. 设计防拨弹电机堵转

###### 防超热量

机器人每发射一颗子弹，裁判系统就会累计“热量”，并随着时间“冷却”即逐渐降低该值，“热量”存在上限，超过该值会被视为违规！

旧比赛规则中机器人超热量会直接扣血，多发射几颗子弹就会暴毙，新规中超热量会直接被锁定发射机构，整句无法发射弹丸，因为超热量而被处罚是非常亏的，因此宁愿留有热量的盈余，不要冒险。

```C
int16_t remaining_heat = game_robot_status.shooter_barrel_heat_limit -
                             power_heat_data.shooter_id1_17mm_cooling_heat;
    if (remaining_heat > MIN_SHOOT_HEAT ||
        game_robot_status.shooter_barrel_heat_limit == 65535 ||
        game_robot_status.shooter_barrel_heat_limit == 0)
      heat_enable_flag = ENABLE;
    else
      heat_enable_flag = DISABLE;
```

代码中使用`heat_enable_flag ` 作为当前热量情况是否允许发射的标志位

###### 防堵转

```C
void anti_stuck_output(shooter_tag_t tag, float curr, float cur_rpm, float exp_rpm)
```

防堵转函数会持续检测拨弹电机使用情况，当发生堵转时会尝试往反方向转动一段距离，再恢复原来的方向继续工作

#### 火力限制 ./APP/fire_ctrl

比赛中机器人携带的弹丸量有限，同时考虑到存在“热量上限”，因此设定一些条件以控制机器人的发弹是很有必要的，为了保证机器人有较高的有效输出（即弹丸的命中率），“火控”算法会根据视觉系统传递的相关信息计算在什么时候开火为最佳，标志位为`suggest_fire` 发射机构会联合此标志位进行相关决策

#### 双板通信 ./APP/boards_interact

该版本双板通信使用CAN协议

头文件中定义了上下板需传递的数据及其载体

源文件中实现其发送和解析，接收及解析函数被CAN接收回调函数调用

同时也实现了每个数据包的检测，以此来判断上下板是否在线或通信有效

#### UI顶层 ./APP/graphic_draw

UI的实现关乎官方裁判系统串口协议，裁判系统串口协议文件非常重要，需仔细翻阅

目前已实现通信底层文件，默认使用旧版drv_uiOld.c

新版尚有缺陷，且未经过稳定性测试，不建议使用

**包含drv_ui.h后，调用drv_uiOld.c中的函数实现****UI****的绘制**

#### PID与滤波参数 ./APP/pid_lpf_param

这里存放机器人的所有PID和LPF（除电机的专用LPF）相关参数

### **电机库**

**仓库链接已在本文档开头给出，请在框架目录./Devices/下拉取，拉取之前请保证不存在名为"MOTOR"的文件夹，或同级的"MOTOR"文件夹内为空**

↓**请翻阅源文件查看相关注释↓**

#### 电机协议层 ./Devices/MOTOR/motor_lib/*

 该部分实现各种类型电机的用户协议即封装好的通信函数

#### 电机数据定义 ./Devices/MOTOR/motor_def

#### 电机库函数 ./Devices/MOTOR/motor_ctrl

```C
HAL_StatusTypeDef set_all_motor_output(motors_type_t motors_type)
```

实现逻辑：根据协议类型选择通道->挂载相应数据总线->往数据总线装载数据->将总线数据发出

#### 电机库配置 ./Devices/MOTOR/motor_conf

此处定义机器人所有存在的电机

注意，新拉下来的电机库是不包含这个文件的，需要用户根据实际情况参照"motor_conf_example.h"的内容，重写一份名为"motor_conf.h"的文件

### **调试**

#### 异常检测 ./APP/detect

```C
void detect_loop(float dt)
```

该函数被robot.c中的`void robot_bus_detect(void)` 挂载，为异常检测的主体，主要检测各机构的（电机）的使用情况、是否掉线，视觉系统运行是否正常，遥控器、裁判系统通信是否正常等

目前有效的表示异常手段是灯珠ws2812

#### 状态指示灯 ./Tools/ws2812

采用PWM-DMA的方案驱动ws2812 灯珠

基于面向对象将挂载的ws2812对象化，方便异常检测用于表示相关信息

除供给异常检测外，也可将灯珠置于图传下方，用于提示某些信息给操作手

源文件中有各种颜色的预设，所有灯珠对象的degree_percent应尽可能小，否则灯珠会非常亮，晃眼睛

头文件已提供接口，示例可见异常检测中的调用

#### 异常播报 ./Tools/asr

该版本未有较为完善且实用的方案，未来可以考虑发展

#### 测试文件 ./Tests/test

当前预设好的test文件功能为测试电机，在drv_conf配置文件中启用 USE_TEST_FILE即可启用

注意启用测试文件时，会失能所有机构，不会注册除测试电机以外的所有电机

### **其他**

**此处多为使用HAL库实现调用外设信息接口的内容，属于HAL库基础知识范畴，不再赘述**

#### 非阻塞计时 ./Base/bsp_dwt

#### CAN协议层 ./Base/drv_can

#### UART驱动层 ./Base/drv_uart

#### UI底层 ./Base/drv_ui(drv_uiOld)

#### 遥控指令驱动层 ./Base/rc

#### 裁判系统驱动层 ./Base/referee

# 食用方法

## 使用事项

### **部署流程**

#### 修改机器人配置文件

根目录下路径\USERCODE\robot_core\drv_conf.h

如上文所说，drv_conf中有许多关于机器人功能的裁切和配置

一般在不部署一台新机器人时需要注意按照实际情况修改以下

1. 主频，定时器句柄宏定义、各定时器的频率，任务的频率
2. 是否启用操作系统
3. 通信端口(UART、CAN)的内容裁切(如遥控器、图传链路、裁判系统等)、通信端口宏定义

```C
#define USE_RC_UART 1
#define USE_VT_RC_UART 1
#define USE_REFEREE_UART 1
#define USE_EXT_CAN_IMU 1
#define USE_EXT_IMU_UART 0
.....//还有其他
```

1. 双板通信的配置，决定使用单板还是双板，机器类型是从机还是主机，以及双板通信的频率和端口
2. 选择底盘类型，全向轮底盘，麦克纳姆轮底盘，亦或是特殊底盘，这将会影响底盘底层的解算策略
3. 其他功能的裁切

```C
#define USE_REFEREE 0       ///< 是否使用裁判系统
#define READY_FOR_BATTLE 0  ///< 为0不会触发阵亡检测
#define USE_DT7_WHEEL 1     ///< 如果DT7遥控器拨轮坏了就置零不启用
#define USE_TEST_FILE 0     ///< 测试功能启用文件test.c
#define USE_GIMBAL_RESET 1  ///< 使能云台缓启动
#define USE_CHASSIS_RESET 1 ///< 使能底盘缓启动
.....// 还有其他
```

PS：若使用Linux系统开发，可以配合同级目录下的drv_conf.cmake文件(需要根据需要编写)，使用ccmake开关这些配置

#### 部署电机库

电机库需要在仓库进行拉取

根目录下路径\USERCODE\robot_core\Devices

使用git工具拉取coding仓库https://scnu-pioneer.coding.net/p/ec/d/MOTOR/git/releases

拉取最新的稳定版本

#### **修改电机库配置文件**

在注册电机之前，需要先对电机库进行用户层面的配置

在./Devices/MOTOR中，注意到文件**motor_conf_example.h**

这是一个已经写好的示例文件，请根据机器人的实际情况，自行照着写一份**motor_conf.h**在同级目录中

在  `motor_conf.h` 中，你需要修改motors_t结构体的定义，使其符合当前机器人上电机的实际情况，并根据结构体中的内容，相应地修改机构的电机INDEX和电机NUMBER，若设置有误将带来隐患！

#### 注册电机

在每个机构的初始化函数，需要对电机进行注册，注意电机的品牌类型，不同厂家的电机基本都需要使用不同的注册函数

在需要注册电机库的地方（如机构应用层chassis.h）调用电机库

```C
#include "Devices/MOTOR/motor_headers.h"
```

根据机器人上所需要驱动的电机，对各个电机进行注册

调用电机库暴露出的register函数，填入对象电机对应的信息

```C
// 这是对右前轮毂电机的注册操作，指明其为M3508，以及对应的通信端口和其他信息
RM_QUAD_motor_register(&motors->RF_wheel, motor_model_RM_QUAD_M3508,
                         QUAD_CURR, COM_CAN, (uint32_t *)&CHASSIS_MOTORS_HCAN,
                         1, RR_M3508, &RF_wheel_lpf_init_val, M_CHASSIS);
```

请按照以下规范注册的格式：

```C
// 如在底盘初始化函数中
void chassis_init(void){
  //  CAN1
  //  ID  rxId    txId
  //  1   0x201   0x200
  RM_QUAD_motor_register(&motors->RF_wheel, motor_model_RM_QUAD_M3508,
                         QUAD_CURR, COM_CAN, (uint32_t *)&CHASSIS_MOTORS_HCAN,
                         1, RR_M3508, &RF_wheel_lpf_init_val, M_CHASSIS);
  //  2   0x202   0x200
  RM_QUAD_motor_register(&motors->LF_wheel, motor_model_RM_QUAD_M3508,
                         QUAD_CURR, COM_CAN, (uint32_t *)&CHASSIS_MOTORS_HCAN,
                         2, RR_M3508, &LF_wheel_lpf_init_val, M_CHASSIS);
  //  3   0x203   0x200
  RM_QUAD_motor_register(&motors->LB_wheel, motor_model_RM_QUAD_M3508,
                         QUAD_CURR, COM_CAN, (uint32_t *)&CHASSIS_MOTORS_HCAN,
                         3, RR_M3508, &LB_wheel_lpf_init_val, M_CHASSIS);
  //  4   0x204   0x200
  RM_QUAD_motor_register(&motors->RB_wheel, motor_model_RM_QUAD_M3508,
                         QUAD_CURR, COM_CAN, (uint32_t *)&CHASSIS_MOTORS_HCAN,
                         4, RR_M3508, &RB_wheel_lpf_init_val, M_CHASSIS);
}
```

想了解注册的详细含义，请跳转到注册函数的定义自行查看

另外，还需要配置CAN过滤器，过滤标准为只接受制定帧头的CAN数据包进入CAN的FIFO缓冲区

```C
  can_user_init(&CHASSIS_MOTORS_HCAN,
                (uint32_t[]){rx_stdid_of(motors->RF_wheel),
                             rx_stdid_of(motors->LF_wheel),
                             rx_stdid_of(motors->LB_wheel),
                             rx_stdid_of(motors->RB_wheel)},
                0);
```

注意CAN句柄要与注册信息对应

#### 定时任务设置

在main.c文件下有关于定时器使能失能的操作

另外在main函数下方有定时器回调函数

```C
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
```

在这里加入所需要执行的定时任务

值得一提的是，如果启用操作系统，那么以上使用STM32硬件定时器的操作将会失效（硬件定时器必须失能防止抢占），转而使用\USERCODE\robot_core\OS\robot_task.c 中的操作系统任务调度执行定时任务

### **调试流程**

#### 修正IMU方向

关注到根目录下\USERCODE\robot_core\APP\imu_bmi088.c

在300~400行之间找到原始数据的方向变换

```C
      /**
       * @attention 框架欧拉角规定 抬头pitch减小 顺时针yaw减小
       * 车头向前顺时针roll增大
       */
      imu_raw.ax = -bmi08x_accel.x;
      imu_raw.ay = -bmi08x_accel.y;
      imu_raw.az = bmi08x_accel.z;
      imu_raw.gx = -bmi08x_gyro.x;
      imu_raw.gy = -bmi08x_gyro.y;
      imu_raw.gz = bmi08x_gyro.z;
```

C板中的IMU遵循右手坐标系，R标正向自己，右手坐标系下，拇指朝向为Z轴，食指方向为X轴，中指方向为Y轴，要根据C板的实际安装位置来修正方向，保证前进方向为X轴，并符合框架欧拉角规定

对于其他带IMU的主控板或独立IMU，请查阅其用户手册

#### 设置云台减速比

关注到根目录下\USERCODE\robot_core\APP\gimbal.h

```C
// 根据机械给出的减速比填入即可
#define PITCH_RR 1.f
#define YAW_RR 1.f
```

#### 设置云台真缓启动零点

关注到根目录下\USERCODE\robot_core\APP\gimbal.h

确认上电安全，并且电机接收数据无误后，手动摆正pitch电机和yaw电机到正确的位置上

1. yaw轴摆到机器人云台（枪口）指向正前方
2. pitch轴摆到机器人视线水平正视前方

在变量监测窗口中监测变量`motor` ，观察pitch和yaw电机对象的raw_scale，将此值填入

```C
#define GIMBAL_PITCH_MOTOR_CENTER 5325
#define GIMBAL_YAW_MOTOR_CENTER 1697
```

#### **调试云台电机系**

关注到根目录下\USERCODE\robot_core\APP\gimbal.c

首先需要确认物理量的极性，这将是后续对云台调试的绝对基础

> ***调云台前须知（断控时，可以用手摆动云台，检测变量）:***
>
> - ***要求yaw轴顺时针转时 yaw_rad逐渐减小***
> - ***要求pitch轴顺时针转时(即低头时) pitch_rad逐渐增大***
> - ***因此反装电机时计算rad需要给abs_angle加负号 否则将影响底盘换系! pitch同理***
> - ***电机正反装会影响get_minor_arc的结果***
> - ***这里要求减正数得到小角度,减负数得到大角度,反了的话就改get_minor_arc两角顺序***
> - ***框架欧拉角规定 抬头pitch减小  顺时针yaw减小 车头向前顺时针roll增大***

如遇电机反装的情况，还要关注云台底层中电机输出方向是否与人在世界系下观察的期望方向保持一致，若不一致会导致PID正反馈闭环输出

在云台选择控制坐标系函数`select_gimbal_ctrl_type` 中，让电机系控制变为常驻效果

```C
#if 0
    cur_state->pitch_ang = robot->imu_data->eular.pitch;
    cur_state->roll_ang = robot->imu_data->eular.roll;
    cur_state->yaw_ang = robot->imu_data->eular.yaw;
    return USE_IMU;
#else
    // 测试电机系常驻效果
    cur_state->pitch_ang = motors->pitch.real.abs_angle / PITCH_RR;
    cur_state->roll_ang = robot->imu_data->eular.roll / IMU_RANGE * 2 * PI;
    cur_state->yaw_ang = motors->yaw.real.abs_angle / YAW_RR;
    return USE_MOTOR;
#endif
```

在上控前，务必确认极性已正常，否则必须将`pid_lpf_param` 中与电机系有关的PID参数设小，在调试窗口中缓慢改变值，可再次确认极性是否正确

以上步骤确认完毕后可正常整定PID参数

#### **调试云台IMU系**

调试云台IMU系的方式与电机系相当，在调试之前，请确保机器人欧拉角符合欧拉角规定 在云台选择控制坐标系函数`select_gimbal_ctrl_type` 中，取消电机系控制为常驻效果

```C
#if 1 // <----------------
    cur_state->pitch_ang = robot->imu_data->eular.pitch;
    cur_state->roll_ang = robot->imu_data->eular.roll;
    cur_state->yaw_ang = robot->imu_data->eular.yaw;
    return USE_IMU;
#else
    // 测试电机系常驻效果
    cur_state->pitch_ang = motors->pitch.real.abs_angle / PITCH_RR;
    cur_state->roll_ang = robot->imu_data->eular.roll / IMU_RANGE * 2 * PI;
    cur_state->yaw_ang = motors->yaw.real.abs_angle / YAW_RR;
    return USE_MOTOR;
#endif
```

在上控调试前，必须将`pid_lpf_param` 中与IMU系有关的PID参数设小，在调试窗口中缓慢改变值

#### **云台俯仰角限幅**

对云台俯仰角限幅的前提是已确认俯仰自由度的相关极性正确

1. 上电完成真缓启动（需要电机系调试完成）
2. 断控情况下手动摆动俯仰角到预定限位，监测pitch_rad
3. 将值填入gimbal.h中的*PITCH_REL_RAD_MAX/MIN*

> *PITCH_REL_RAD_MAX/MIN 真缓启动结束后 读机械极限角度时pitch_rad，低头角度为MAX 抬头角度为MIN*

1. 调整PITCH_RES_CPST

> *// 本质是一个死区，用于补偿pitch响应不完美导致的卡顿*
>
> *// 全名pitch_response_compensation 与pitch响应 pitch前馈 imu稳定性有关、*

#### 调试底盘

1. 先关闭底盘跟随的PID，前往`pid_lpf_param` 将`chassis_follow_pid_init_val`和`chassis_vision_follow_pid_init_val`的out_max参数设置为0，

在FreeMaster（或其他曲线检测软件如vofa）检测轮速实际值的跟随情况，适当整定参数

```C
// 监测这两个结构体的内容曲线
general_wheels_state_t cur_wheels;
general_wheels_state_t exp_wheels; 
```

1. 监测`cur_state`和`exp_state`的跟随情况，这两个变量实际为`robot.expected_state`和`robot.current_state` ，整定PID参数
2. 开启底盘跟随（把out_max改回原来的值），整定PID参数

#### 调试发射机构

关注到\USERCODE\robot_core\APP\shooter.h 

这里的对象为步兵，发射17mm小弹丸

`DIAL_RPM` 代表拨弹盘转速，决定了发射弹频，结合`C_PUSHER_DIAL`简单计算可以得到映射关系，但最好是通过裁判系统读值是否超过规则规定的弹频

`FRIC_RPM` 代表摩擦轮转速，决定了弹速，结合`C_FRIC_WHEEL`简单计算可以得到映射关系，但最好是通过裁判系统读值是否超过规则固定的弹速，最好比规定的弹速上限要小两到三个单位

`ONE_SHOOT_ANG` 检测拨弹盘电机的abs_angle，用手拨动拨弹盘转过发射一颗弹丸所需的角度，该值即为此角度值（带单位）

`MIN_SHOOT_HEAT` 允许的最小热量余量，当机器人检测到裁判系统当前热量距离热量上限之间，已不足该值，则不允许发弹

 另外还要关注堵转的相关参数

```C
#define STUCK_MAX_TIME_CNT 10 // 600    // 最大允许堵转时间
#define STUCK_SPEED 8                   // 转速小于该值判断为堵转
#define STUCK_RESTORE_RPM -81           // 堵转后尝试反转的速度
#define STUCK_REVERSE_TIME_CNT 30       // 堵转后尝试反转的时间
#define STUCK_CURRENT 2.0f // 安培      // 电流大于该值为堵转
#define STUCK_FORWARD -1                // 表示正转的方向
#define STUCK_REVERSE 1                 // 表示反转的方向
```

#### 视控联调

1. 视控静态调整（真值检验）

**该步骤用于检验电控在自瞄系统中解算出的角度是否有误**

使用水平仪观察云台俯仰角

在debug窗口中查看IMU的欧拉角数据，上控推动拨杆使得imu->pitch的值尽量向0靠拢，并观察水平仪是否水平

若不水平则需要在./Algorithm/imu_dcm.c中修改变量`pitch_revise` 的值，直至水平仪水平

然后在./APP/vision.c的`vision_ctrl()`函数中，将target的x0和z0改为现实中装甲板中心至云台中心的距离（单位：米，可使用米尺测量）

开火，观察是否击中装甲板中心位置

1. IMU零飘调整

上电后保持机器人云台静止不动，在debug窗口中观测./APP/imu_bmi088.c 中`imu_gyro_cal_t gyro_cali`，等到数据稳定或一直在一定范围内波动时,那就代表数据稳定了，对单项数据,发现它会上升那就手动给它加点值,直到它开减为止,反之同理

1. 弹速稳定调整

在FreeMaster中观察裁判系统反馈的弹速曲线，精调摩擦轮PID参数，令机器人在持续开火时，尽量使得弹速稳定在±0.5以内

值得注意的是，若机器人尚未接入裁判系统，是无法通过测速模块获得精确的弹速的，此时自瞄算法默认会将shooter.h中的宏`DEFAULT_SHOOT_SPEED` 作为弹丸出膛速度（此为期望发射速度，只能作为实际发射速度的粗略替代品）

1. 视控动态调整（响应调整和补偿，**视觉那边的参数修改后，动态调整都必须重做一遍****！**）

**该步骤用于修正信号传输延迟和电机响应带来的误差**

需要注意的是，在自瞄过程中，**弹速需要稳定**，**且云台响应必须足够迅速且准确**，微小的期望也能够使其顺利响应，可通过在FreeMaster软件中观察云台的期望值和实际值，精调PID参数

询问视觉是否已保证在运动时，整车模型预测正确

在保证云台响应足够后，同样在FreeMaster监测目标运动速度(来源于上位机发送来的视觉数据)，在目标匀速时开火，在./APP/vision.c 中调整参数`const_dl` ，观察击打位置，直到击打位置在装甲板水平中心

令目标在原地小陀螺，在./APP/fire_ctrl.c中调整参数`yaw_motor_res_speed`以补偿yaw轴电机响应，目标装甲板运动到“合适位置”(即将消失时)，机器人能够放弃瞄准该装甲板，并迅速瞄准下一块即将到来的装甲板，该“合适位置”由此变量控制

## 未来开发事项

施工中...

这里写下未来可能可以再开发的内容，综合上述详解，针对框架现有缺点，希望在未来能得到解决

1. **实现全车导航，为框架编入导航半自动控制模块**
2. **全权转化为面向对象的编程思想，使用C++进行开发，高度封装各个模块，数据抽象和数据封装**
3. **采用节点、订阅、发布等机制重整框架，实现类ROS的应用层模式（可参考湖大开源****），该操作可为移植到更强力的算力设备上并使用ROS2进行控制打下基础（使用ROS2意味着更强大更丰富的开发资源）**
4. **面向对象+操作系统的****UI****框架，优化数据结构，最大化利用信道负载的同时提高库调用的便利性**
#include "ControllerWindow.h"
#include "ChassisControl.h"
#include "RemoteControl.h"
#include "robot-config.h"
#include "RobotControl.h"
#include "units.h"
#include "auto.h"
#include "vex.h"

BaseWindow *InitWindow = new BaseWindow("初始化", nullptr, InitWindowFun);
BaseWindow *RemoteControlWindow = new BaseWindow("遥控选择", InitWindow, RemoteControlWindowFun);
BaseWindow *ModWindow = new BaseWindow("模式选择", RemoteControlWindow, ModWindowFun);

BaseWindow *RemoteControlWindow_Mod = new BaseWindow("模式-遥控", ModWindow, RemoteControlWindow_ModFun);             //  模式-遥控
BaseWindow *RemoteControlWindow_Compete = new BaseWindow("比赛-遥控模式", ModWindow, RemoteControlWindow_CompeteFun); //  比赛-遥控模式
BaseWindow *AutoWindow = new BaseWindow("自动测试模式", ModWindow, AutoWindowFun);                                    //  自动测试模式
BaseWindow *TestWindow = new BaseWindow("测试模式", ModWindow, TestWindowFun);                                        //  测试模式
BaseWindow *MemoryWindow = new BaseWindow("记忆模式", ModWindow, MemoryWindowFun);                                    //  记忆模式

BaseWindow *AutoChoiceWindow_Compete = new BaseWindow("比赛-自动选择", ModWindow, AutoChoiceWindow_CompeteFun);  // 比赛-自动选择
BaseWindow *RemoteControlWindow_Auto = new BaseWindow("自动-遥控选择", AutoWindow, RemoteControlWindow_AutoFun); // 自动-遥控选择

BaseWindow *CompeteConfirmWindow = new BaseWindow("比赛-确认选择", AutoChoiceWindow_Compete, CompeteConfirmWindowFun); // 比赛-确认选择
BaseWindow *CompeteWindow = new BaseWindow("比赛-连接场控", CompeteConfirmWindow, CompetitionlWindowFun);              // 比赛连接场控

BaseWindow *TestMotorChoiceWindow = new BaseWindow("电机测试 选择", TestWindow, TestMotorChoiceWindowFun);             // 电机测试（选择）
BaseWindow *TestAirpoweredChoiceWindow = new BaseWindow("活塞测试 选择", TestWindow, TestAirpoweredChoiceWindowFun);   // 活塞测试（选择）
BaseWindow *TestInertialWindow = new BaseWindow("陀螺仪测试", TestWindow, TestInertialWindowFun);                      // 陀螺仪测试（选择）
BaseWindow *TestDistanceChoiceWindow = new BaseWindow("距离传感器测试 选择", TestWindow, TestDistanceChoiceWindowFun); // 距离传感器测试（选择）
BaseWindow *TestDecenteringWindow = new BaseWindow("脱轴检测", TestWindow, TestDecenteringFun);                        // 电机脱轴测试

BaseWindow *TestMotorWindow = new BaseWindow("电机测试", TestMotorChoiceWindow, TestMotorWindowFun);                // 电机测试
BaseWindow *TestAirpoweredWindow = new BaseWindow("活塞测试", TestAirpoweredChoiceWindow, TestAirpoweredWindowFun); // 活塞测试
BaseWindow *TestDistanceWindow = new BaseWindow("距离测试", TestDistanceChoiceWindow, TestDistanceWindowFun);
; // 距离传感器测试
BaseWindow *TestRotationWindow = new BaseWindow("定位轮测试", TestMotorChoiceWindow, TestRotationWindowFun);
; // 定位轮测试

// 各个函数实现------------------------------------------

// 从选项中选择一个选项
int ChoiceIndex(std::vector<char *> options)
{
    int index = 0;
    int n = options.size();
    bool print_flag = true;

    bool ButtonRightFlag = false;
    bool ButtonLeftFlag = false;

    while (true)
    {
        if (print_flag)
        {
            PrintInLine(2);

            char formattedString[50]; // 创建字符数组来存储格式化字符串
            // 使用 snprintf 生成格式化字符串
            snprintf(formattedString, 50, "-> %s <-", options[index]);
            // 使字符串居中
            CenterStr(formattedString);

            Controller.Screen.print(formattedString);
        }

        print_flag = false;

        // 选项加一
        if ((!Controller.ButtonRight.pressing() && ButtonRightFlag) || Controller.Axis4.position() > 50 || Controller.Axis1.position() > 50)
        {
            index = (index + 1) % n;
            print_flag = true;
        }

        // 选项减一
        if ((!Controller.ButtonLeft.pressing() && ButtonLeftFlag) || Controller.Axis4.position() < -50 || Controller.Axis1.position() < -50)
        {
            index = index - 1;
            if (index == -1)
            {
                index = n - 1;
            }
            print_flag = true;
        }

        ButtonRightFlag = Controller.ButtonRight.pressing();
        ButtonLeftFlag = Controller.ButtonLeft.pressing();

        // 选项确认
        if (Controller.ButtonA.pressing())
        {
            return index;
        }

        // 退出选择
        if (Controller.ButtonB.pressing())
        {
            return -1;
        }

        wait(100, msec);
    }
}

// 运行自动函数并且结束后更新时间到遥控器
int RunAutoFunAndUpdataToControl()
{
    // 运行自动程序
    ChassisControlSTS.ResetPPS();

    // 线程开启
    double start_time = GetNowTime();

    RunningAutoFun = &AutoFunctions[auto_index];
    RunningAutoFun->start_time = start_time;
    RunningAutoFun->Over = false;
    vex::thread Task = thread(AutoControl);
    double now_time = GetNowTime();

    while (now_time - start_time < RunningAutoFun->out_time && !RunningAutoFun->IsOver())
    {
        now_time = GetNowTime(); // 运动开始时间
        if (Controller.ButtonX.pressing() && Controller.ButtonA.pressing())
        {
            Controller.rumble(".");
            break;
        }
        wait(10, msec);
    }
    // ChassisControlSTS.StopMoving();
    Task.interrupt(); // 中止线程
    if (ChassisControlSTS.IsMoving())
    {
        // ChassisControlSTS.StopMoving();
        ChassisControlSTS.Moveing = false;
    }
    double end_time = GetNowTime();
    ChassisControlSTS.ChassisStoppingCoast(); // 底盘停止抱死
    RestoreInitial();
    PrintInLine(1);
    Controller.Screen.print("运行完成");
    PrintInLine(2);
    Controller.Screen.print("运行时间： %.2f", end_time - start_time);
    // wait(2000, msec);
    return 0;
}

// 初始化函数
BaseWindow *InitWindowFun()
{
    // 站位选择
    // 当前用不到 已屏蔽 ////////////////////////////////////////

    // PrintInLine(1);
    // Controller.Screen.print("选择你的站位");
    // PrintInLine(2);
    // Controller.Screen.print("      蓝方(L1+L2)   ");
    // PrintInLine(3);
    // Controller.Screen.print("      红方(R1+R2)   ");

    // while (true)
    // {
    //     // 选项确认
    //     if (Controller.ButtonR1.pressing() && Controller.ButtonR2.pressing())
    //     {
    //         stance = 'r';
    //         break;
    //     }

    //     if (Controller.ButtonL2.pressing() && Controller.ButtonL1.pressing())
    //     {
    //         stance = 'b';
    //         break;
    //     }

    //     wait(50, msec);
    // }

    // 如果遥控已经确定，则跳过遥控选择 进行模式选择

    return RemoteControlWindow;
}

// 遥控模式选择
BaseWindow *RemoteControlWindowFun()
{
    int index = -1;
    std::vector<char *> options;

    for (ChassisRomoteControlParams Params : ChassisRomoteControlList)
    {
        options.push_back(Params.name);
    }

    PrintInLine(1);
    Controller.Screen.print("遥控>遥控选择");
    PrintInLine(3);
    Controller.Screen.print("      确定(A)   返回(B)");

    if (ChassisRomoteControlList.size() != 1)
    {
        index = ChoiceIndex(options); // 获取用户操作
    }
    else
    {
        index = 0;
    }

    if (index == -1)
    {
        return RemoteControlWindow->Back();
    }

    // 更改遥控模式
    ChassisRomoteControlSTS.name = ChassisRomoteControlList[index].name;
    ChassisRomoteControlSTS.func = ChassisRomoteControlList[index].Work;

    // 一小段遥控，允许用户进行简单操作
    PrintInLine(2);
    Controller.Screen.print("<<临时遥控>>");
    RemoteControlUnit(false, false, 5, "", 999999999999);
    Controller.rumble("..");
    while (Controller.ButtonB.pressing())
    {
        wait(10, msec);
    }

    std::cout << options[index] << std::endl;

    return ModWindow;
}

// 模式选择
BaseWindow *ModWindowFun()
{

    int index = -1;
    std::vector<char *> options = {"遥控模式", "自动模式", "测试模式", "记忆模式", "比赛模式"};
    PrintInLine(1);
    Controller.Screen.print("选择你的操作");
    PrintInLine(3);
    Controller.Screen.print("      确定(A)   返回(B)");

    index = ChoiceIndex(options); // 获取用户操作
    if (index == -1)
    {
        // 如果遥控已经确定，则跳过遥控选择 进行站位选择
        if (ChassisRomoteControlList.size() == 1)
        {
            return ModWindow->Back()->Back();
        }
        else
        {
            return ModWindow->Back();
        }
    }

    std::cout << options[index] << std::endl;

    std::vector<BaseWindow *> ChildWindow{RemoteControlWindow_Mod, AutoWindow, TestWindow, MemoryWindow, AutoChoiceWindow_Compete};

    return ChildWindow[index];
}

//  模式-遥控
BaseWindow *RemoteControlWindow_ModFun()
{
    // int index = -1;
    // std::vector<char*> options = {"双摇杆", "左单摇杆", "右单摇杆", "坦克操控"};
    // PrintInLine(1);
    // Controller.Screen.print("遥控>遥控选择");
    // PrintInLine(3);
    // Controller.Screen.print("      确定(A)   返回(B)");

    // index = ChoiceIndex(options);  // 获取用户操作
    // RemoteControlMod = index + 1;  // 更改遥控选择

    // if (index == -1){
    //   return RemoteControlWindow_Mod->Back();
    // }

    // std::cout << options[index] << std::endl;
    // Controller.Screen.clearScreen();  // 清除屏幕
    PrintInLine(2);
    RemoteControl();

    return ModWindow;
}

//  比赛-遥控模式
BaseWindow *RemoteControlWindow_CompeteFun()
{
    int index = -1;
    std::vector<char *> options;

    for (ChassisRomoteControlParams Params : ChassisRomoteControlList)
    {
        options.push_back(Params.name);
    }

    PrintInLine(1);
    Controller.Screen.print("遥控>遥控选择");
    PrintInLine(3);
    Controller.Screen.print("      确定(A)   返回(B)");

    index = ChoiceIndex(options); // 获取用户操作

    if (index == -1)
    {
        return RemoteControlWindow->Back();
    }

    // 更改遥控模式
    ChassisRomoteControlSTS.name = ChassisRomoteControlList[index].name;
    ChassisRomoteControlSTS.func = ChassisRomoteControlList[index].Work;

    std::cout << options[index] << std::endl;

    return AutoChoiceWindow_Compete;
}

//  自动测试模式
BaseWindow *AutoWindowFun()
{
    WaitInertialCalibrating();

    int index = -1;
    // 获取各个自动程序的名称和内容
    std::vector<char *> options;

    for (AutoFun auto_fun : AutoFunctions)
    {
        options.push_back(auto_fun.name);
    }
    PrintInLine(1);
    Controller.Screen.print("自动>自动选择");
    PrintInLine(3);
    Controller.Screen.print("      确定(A)   返回(B)");
    ChassisControlSTS.Reset(); // 初始化

    // 获取用户操作
    int n = options.size();
    bool print_flag = true;
    int index_i = 0;

    bool ButtonRightFlag = false;
    bool ButtonLeftFlag = false;

    while (true)
    {
        PrintInLine(1);
        Controller.Screen.print(ChassisInertial.rotation());

        if (print_flag)
        {
            PrintInLine(2);

            char formattedString[50]; // 创建字符数组来存储格式化字符串
            // 使用 snprintf 生成格式化字符串
            snprintf(formattedString, 50, "-> %s <-", options[index_i]);
            // 使字符串居中
            CenterStr(formattedString);

            Controller.Screen.print(formattedString);
        }

        print_flag = false;

        // 选项加一
        if ((!Controller.ButtonRight.pressing() && ButtonRightFlag) || Controller.Axis4.position() > 50 || Controller.Axis1.position() > 50)
        {
            index_i = (index_i + 1) % n;
            print_flag = true;
        }

        // 选项减一
        if ((!Controller.ButtonLeft.pressing() && ButtonLeftFlag) || Controller.Axis4.position() < -50 || Controller.Axis1.position() < -50)
        {
            index_i = index_i - 1;
            if (index_i == -1)
            {
                index_i = n - 1;
            }
            print_flag = true;
        }

        // 底盘重新初始化
        if (Controller.ButtonX.pressing())
        {
            PrintInLine(1);
            Controller.Screen.print("自动>初始化成功！");
            ChassisControlSTS.Reset(); // 初始化
            wait(500, msec);
            PrintInLine(1);
            Controller.Screen.print("自动>自动选择");
        }

        ButtonRightFlag = Controller.ButtonRight.pressing();
        ButtonLeftFlag = Controller.ButtonLeft.pressing();

        // 选项确认
        if (Controller.ButtonA.pressing())
        {
            index = index_i;
            break;
        }

        // 退出选择
        if (Controller.ButtonB.pressing())
        {
            index = -1;
            break;
        }

        // 自动选择更换
        auto_index = index_i;

        wait(100, msec);
    }

    if (index == -1)
    {
        return RemoteControlWindow_Mod->Back();
    }

    std::cout << options[index] << std::endl;

    // 运行自动程序
    ChassisControlSTS.ResetPPS();
    auto_index = index;

    RunAutoFunAndUpdataToControl(); // 运行自动程序

    PrintInLine(3);
    Controller.Screen.print("      遥控(A)   重试(B)");

    // 等待下一步操作
    bool last_A = Controller.ButtonA.pressing();
    bool last_B = Controller.ButtonB.pressing();
    while (true)
    {
        // 选项确认
        if (Controller.ButtonA.pressing() && !last_A)
        {
            return RemoteControlWindow_Auto;
        }

        // 退出选择
        if (Controller.ButtonB.pressing() && !last_B)
        {
            return AutoWindow;
        }
        last_A = Controller.ButtonA.pressing();
        last_B = Controller.ButtonB.pressing();
        wait(10, msec);
    }

    return ModWindow;
}

//  测试模式
BaseWindow *TestWindowFun()
{
    int index = -1;
    std::vector<char *> options = {"脱轴检测", "电机测试", "陀螺仪测试", "距离测试", "活塞测试", "定位轮测试"};
    PrintInLine(1);
    Controller.Screen.print("测试>测试选择");
    PrintInLine(3);
    Controller.Screen.print("      确定(A)   返回(B)");

    index = ChoiceIndex(options); // 获取用户操作

    // 选项确认
    if (index == 0)
    {
        return TestDecenteringWindow;
    }

    if (index == 1)
    {
        return TestMotorChoiceWindow;
    }

    if (index == 2)
    {
        return TestInertialWindow;
    }

    if (index == 3)
    {
        return TestDistanceChoiceWindow;
    }

    if (index == 4)
    {
        return TestAirpoweredChoiceWindow;
    }

    if (index == 5)
    {
        return TestRotationWindow;
    }

    if (index == 6)
    {
        return TestRotationWindow;
    }

    // 退出选择
    if (index == -1)
    {
        return ModWindow;
    }

    return ModWindow;
}

// 记忆模式
BaseWindow *MemoryWindowFun()
{
    int index = -1;
    std::vector<char *> options = {"5s", "15s", "60s"};
    PrintInLine(1);
    Controller.Screen.print("记忆>时间选择");
    PrintInLine(3);
    Controller.Screen.print("      确定(A)   返回(B)");

    index = ChoiceIndex(options); // 获取用户操作

    int timeout = 0;

    // 选项确认
    if (index == 0)
    {
        timeout = 5;
    }

    if (index == 1)
    {
        timeout = 15;
    }

    if (index == 2)
    {
        timeout = 60;
    }

    RemoteControlUnit(true, false, 50, FileName, timeout);

    // 退出选择
    if (index == -1)
    {
        return ModWindow;
    }

    return ModWindow;
}

//  比赛-自动选择
BaseWindow *AutoChoiceWindow_CompeteFun()
{
    WaitInertialCalibrating();

    int index = -1;
    // 获取各个自动程序的名称和内容
    std::vector<char *> options;

    for (AutoFun auto_fun : AutoFunctions)
    {
        options.push_back(auto_fun.name);
    }
    PrintInLine(1);
    Controller.Screen.print("比赛>自动选择");
    PrintInLine(3);
    Controller.Screen.print("   选定(A) 运行(R1+R2)");
    ChassisControlSTS.Reset(); // 初始化

    // 获取用户操作
    int n = options.size();
    bool print_flag = true;
    int index_i = 0;

    bool ButtonRightFlag = false;
    bool ButtonLeftFlag = false;

    while (true)
    {
        PrintInLine(1);
        Controller.Screen.print(ChassisInertial.rotation());

        if (print_flag)
        {
            PrintInLine(2);

            char formattedString[50]; // 创建字符数组来存储格式化字符串
            // 使用 snprintf 生成格式化字符串
            snprintf(formattedString, 50, "-> %s <-", options[index_i]);
            // 使字符串居中
            CenterStr(formattedString);

            Controller.Screen.print(formattedString);
        }

        print_flag = false;

        // 选项加一
        if ((!Controller.ButtonRight.pressing() && ButtonRightFlag) || Controller.Axis4.position() > 50 || Controller.Axis1.position() > 50)
        {
            index_i = (index_i + 1) % n;
            print_flag = true;
        }

        // 选项减一
        if ((!Controller.ButtonLeft.pressing() && ButtonLeftFlag) || Controller.Axis4.position() < -50 || Controller.Axis1.position() < -50)
        {
            index_i = index_i - 1;
            if (index_i == -1)
            {
                index_i = n - 1;
            }
            print_flag = true;
        }

        // 底盘重新初始化
        if (Controller.ButtonX.pressing())
        {
            PrintInLine(1);
            Controller.Screen.print("自动>初始化成功！");
            ChassisControlSTS.Reset(); // 初始化
            wait(500, msec);
            PrintInLine(1);
            Controller.Screen.print("自动>自动选择");
        }

        ButtonRightFlag = Controller.ButtonRight.pressing();
        ButtonLeftFlag = Controller.ButtonLeft.pressing();

        // 选项确认
        if (Controller.ButtonA.pressing())
        {
            index = index_i;
            auto_index = index;
            return CompeteConfirmWindow;
        }

        // 运行自动
        if (Controller.ButtonR1.pressing() && Controller.ButtonR2.pressing())
        {
            index = index_i;
            break;
        }

        // 退出选择
        if (Controller.ButtonB.pressing())
        {
            index = -1;
            break;
        }

        // 自动选项更换
        auto_index = index_i;
        RunningAutoFun = &AutoFunctions[auto_index];
        RunningAutoFun->Over = false;

        wait(100, msec);
    }

    if (index == -1)
    {
        return RemoteControlWindow_Mod->Back();
    }

    std::cout << options[index] << std::endl;

    // 运行自动程序
    ChassisControlSTS.ResetPPS();
    auto_index = index;

    RunAutoFunAndUpdataToControl(); // 运行自动程序

    PrintInLine(3);
    Controller.Screen.print("      遥控(A)   重试(B)");

    // 等待下一步操作
    bool last_A = Controller.ButtonA.pressing();
    bool last_B = Controller.ButtonB.pressing();
    while (true)
    {
        // 选项确认
        if (Controller.ButtonA.pressing() && !last_A)
        {
            return RemoteControlWindow_Mod;
        }

        // 退出选择
        if (Controller.ButtonB.pressing() && !last_B)
        {
            return AutoChoiceWindow_Compete;
        }
        last_A = Controller.ButtonA.pressing();
        last_B = Controller.ButtonB.pressing();
        wait(10, msec);
    }

    return ModWindow;
}

// 自动-遥控选择
BaseWindow *RemoteControlWindow_AutoFun()
{
    // int index = -1;
    // std::vector<char*> options = {"左单摇杆", "右单摇杆", "双摇杆", "坦克操控"};
    // PrintInLine(1);
    // Controller.Screen.print("自动结束>遥控选择");
    // PrintInLine(3);
    // Controller.Screen.print("      确定(A)   返回(B)");

    // index = ChoiceIndex(options);  // 获取用户操作
    // RemoteControlMod = index + 1;  // 更改遥控选择

    // if (index == -1){
    //   return RemoteControlWindow_Auto->Back();
    // }

    // std::cout << options[index] << std::endl;
    // Controller.Screen.clearScreen();  // 清除屏幕
    RemoteControlUnit(false, false, 2, "", 999999999999);

    return RemoteControlWindow_Auto->Back();
    ;
}

// 比赛确认选择
BaseWindow *CompeteConfirmWindowFun()
{

    PrintInLine(1);
    Controller.Screen.print("自动：%s", AutoFunctions[auto_index].name);

    PrintInLine(2);
    Controller.Screen.print("遥控：%s", ChassisRomoteControlSTS.name);

    PrintInLine(3);
    Controller.Screen.print("      确定(X)   返回(B)");

    while (true)
    {
        if (Competition.isFieldControl())
        {
            PrintInLine(3);
            Controller.Screen.print("你没有按X确认");
            wait(900, msec);

            PrintInLine(3);
            Controller.Screen.print("如果有误请重启所有设备");
            wait(900, msec);

            PrintInLine(3);
            Controller.rumble(".");
            PrintInLine(1);
            Controller.Screen.print("自动：%s", AutoFunctions[auto_index].name);
            PrintInLine(2);
            Controller.Screen.print("遥控：%s", ChassisRomoteControlSTS.name);
            PrintInLine(3);
            Controller.Screen.print("即将自动确认选项（3）");
            wait(900, msec);

            PrintInLine(3);
            Controller.rumble(".");
            Controller.Screen.print("即将自动确认选项（2）");
            wait(900, msec);

            PrintInLine(3);
            Controller.rumble(".");
            Controller.Screen.print("即将自动确认选项（1）");
            wait(900, msec);

            return CompeteWindow;
        }

        if (Controller.ButtonX.pressing())
        {
            return CompeteWindow;
        }
        if (Controller.ButtonB.pressing())
        {
            return CompeteConfirmWindow->Back();
        }
        wait(10, msec);
    }
}

// 已经选完了，连接接场控
BaseWindow *CompetitionlWindowFun()
{

    PrintInLine(1);
    Controller.Screen.print("比赛-已选择完成");
    PrintInLine(2);
    Controller.Screen.print("请连接场控");

    // 等待场控连接
    double start_time = GetNowTime();
    double now_time = GetNowTime();

    while (!Competition.isFieldControl())
    {

        now_time = GetNowTime();

        if (std::abs(Controller.Axis2.position()) > 50 || std::abs(Controller.Axis3.position()) > 50)
        {
            RemoteControl();
        }

        if (Controller.ButtonB.pressing())
        {
            return CompeteWindow->Back();
        }

        wait(100, msec);
    }

    Controller.rumble("-");
    PrintInLine(1);
    Controller.Screen.print("场控已连接");
    wait(500, msec);

    PrintInLine(2);
    Controller.Screen.print("自动已加入");
    wait(500, msec);

    Competition.drivercontrol(RemoteControl); // 遥控程序加入
    Competition.autonomous(AutoControl);      // 自动程序加入

    while (true)
    {
        wait(180000, msec);
    }
}

// 电机测试（选择）
BaseWindow *TestMotorChoiceWindowFun()
{
    int index = -1;
    std::vector<char *> options = MotorNames;
    PrintInLine(1);
    Controller.Screen.print("测试>选择电机");
    PrintInLine(3);
    Controller.Screen.print("      确定(A)   返回(B)");

    index = ChoiceIndex(options); // 获取用户操作

    if (index == -1)
    {
        return TestMotorChoiceWindow->Back();
    }

    std::cout << options[index] << std::endl;
    TestMotorWindow->num = index;
    return TestMotorWindow;
}

// 活塞测试（选择）
BaseWindow *TestAirpoweredChoiceWindowFun()
{
    int index = -1;
    std::vector<char *> options = generateAlphabetStrings('A', 'H');
    PrintInLine(1);
    Controller.Screen.print("测试>选择活塞端口");
    PrintInLine(3);
    Controller.Screen.print("      确定(A)   返回(B)");

    index = ChoiceIndex(options); // 获取用户操作

    if (index == -1)
    {
        return TestAirpoweredChoiceWindow->Back();
    }

    std::cout << options[index] << std::endl;
    TestAirpoweredWindow->num = index;
    return TestAirpoweredWindow;
}

// 陀螺仪测试（选择）
BaseWindow *TestInertialChoiceWindowFun()
{
    int index = -1;
    std::vector<char *> options = generateNumberStrings(1, 20);
    PrintInLine(1);
    Controller.Screen.print("测试>选择陀螺仪端口");
    PrintInLine(3);
    Controller.Screen.print("      确定(A)   返回(B)");

    index = ChoiceIndex(options); // 获取用户操作

    if (index == -1)
    {
        return TestInertialChoiceWindow->Back();
    }

    std::cout << options[index] << std::endl;
    TestInertialWindow->num = index;
    return TestInertialWindow;
}

// 距离传感器测试（选择）
BaseWindow *TestDistanceChoiceWindowFun()
{
    int index = -1;
    std::vector<char *> options = DistanceNames;
    PrintInLine(1);
    Controller.Screen.print("测试>选择记录传感器");
    PrintInLine(3);
    Controller.Screen.print("      确定(A)   返回(B)");

    index = ChoiceIndex(options); // 获取用户操作

    if (index == -1)
    {
        return TestDistanceChoiceWindow->Back();
    }

    std::cout << options[index] << std::endl;
    TestDistanceWindow->num = index;
    return TestDistanceWindow;
}

// 脱轴检测（已修改为机器自检）
BaseWindow *TestDecenteringFun()
{
    motor motor_new = Motors[0];
    int motor_last = 0, motor_now = 0;

    PrintInLine(1);
    Controller.Screen.print("                ");
    PrintInLine(2);
    Controller.Screen.print("测试中....");
    PrintInLine(3);
    Controller.Screen.print("                ");

    // 脱轴检测
    ChassisControlSTS.ChassisRsetPosition(); // 设置电机归位

    ChassisControlSTS.ChassisLeftControl(5);
    ChassisControlSTS.ChassisRightControl(5);
    wait(1000, msec);
    ChassisControlSTS.ChassisLeftControl(0);
    ChassisControlSTS.ChassisRightControl(0);

    PrintInLine(1);
    Controller.Screen.print("L1<%d>  R1<%d>", (int)LeftMotor1.position(vex::rotationUnits::deg), (int)RightMotor1.position(vex::rotationUnits::deg));
    PrintInLine(2);
    Controller.Screen.print("L2<%d>  R2<%d>", (int)LeftMotor2.position(vex::rotationUnits::deg), (int)RightMotor2.position(vex::rotationUnits::deg));
    PrintInLine(3);
    Controller.Screen.print("L3<%d>  R3<%d>", (int)LeftMotor3.position(vex::rotationUnits::deg), (int)RightMotor3.position(vex::rotationUnits::deg));

    while (true)
    {
        if (Controller.ButtonB.pressing())
        {
            return TestDecenteringWindow->Back();
        }
        wait(10, msec);
    }
}

// 电机测试
BaseWindow *TestMotorWindowFun()
{
    motor motor_new = Motors[TestMotorWindow->num];

    PrintInLine(3);
    Controller.Screen.print("                返回(B)");

    while (true)
    {
        PrintInLine(1);
        Controller.Screen.print("位置：%f           ", motor_new.position(deg));
        PrintInLine(2);
        Controller.Screen.print("速度：%f           ", motor_new.velocity(dps));

        motor_new.spin(fwd, Controller.Axis3.position() * 0.1, voltageUnits::volt);

        if (Controller.ButtonB.pressing())
        {
            return TestMotorWindow->Back();
        }

        wait(10, msec);
    }
}

// 活塞测试
BaseWindow *TestAirpoweredWindowFun()
{
    std::vector<vex::triport::port> PORT_vector = {Brain.ThreeWirePort.A, Brain.ThreeWirePort.B, Brain.ThreeWirePort.C,
                                                   Brain.ThreeWirePort.D, Brain.ThreeWirePort.E, Brain.ThreeWirePort.F,
                                                   Brain.ThreeWirePort.G, Brain.ThreeWirePort.H};

    digital_out airpowered_new = digital_out(PORT_vector[TestAirpoweredWindow->num]);
    PrintInLine(1);
    Controller.Screen.print("测试>活塞测试");
    PrintInLine(2);
    Controller.Screen.print("状态： ");
    PrintInLine(3);
    Controller.Screen.print("                返回(B)");

    while (true)
    {
        if (Controller.ButtonLeft.pressing())
        {
            PrintInLine(2);
            Controller.Screen.print("状态： 开");
            airpowered_new.set(1);
        }

        if (Controller.ButtonRight.pressing())
        {
            PrintInLine(2);
            Controller.Screen.print("状态： 关");
            airpowered_new.set(0);
        }

        if (Controller.ButtonB.pressing())
        {
            return TestAirpoweredWindow->Back();
        }

        wait(10, msec);
    }
}

// 陀螺仪测试
BaseWindow *TestInertialWindowFun()
{

    inertial Gyro_new = ChassisInertial;

    PrintInLine(1);
    Controller.Screen.print("测试>陀螺仪测试");
    PrintInLine(3);
    Controller.Screen.print("       重置(X)   返回(B)");

    while (true)
    {
        PrintInLine(2);
        Controller.Screen.print("位置：%f           ", Gyro_new.rotation());

        if (Controller.ButtonX.pressing())
        {
            Gyro_new.resetRotation();
            PrintInLine(2);
            Controller.Screen.print("位置： 重置中");
            wait(500, msec);
        }

        if (Controller.ButtonB.pressing())
        {
            return TestInertialWindow->Back();
        }

        wait(10, msec);
    }
}

// 距离传感器测试
BaseWindow *TestDistanceWindowFun()
{
    distance distance_new = Distances[TestDistanceWindow->num];

    PrintInLine(1);
    Controller.Screen.print("距离传感器测试");
    PrintInLine(3);
    Controller.Screen.print("                返回(B)");

    while (true)
    {
        PrintInLine(2);
        Controller.Screen.print("速度：%f           ", distance_new.objectDistance(mm));
        if (Controller.ButtonB.pressing())
        {
            return TestMotorWindow->Back();
        }

        wait(10, msec);
    }
}

// 定位轮测试
BaseWindow *TestRotationWindowFun()
{

    PrintInLine(1);
    Controller.Screen.print("定位轮测试");
    PrintInLine(3);
    Controller.Screen.print("       重置(X)   返回(B)");

    while (true)
    {
        PrintInLine(2);
        Controller.Screen.print("位置：%f           ", Rotation_x.position(deg));

        if (Controller.ButtonX.pressing())
        {
            Rotation_x.resetPosition();
            PrintInLine(2);
            Controller.Screen.print("位置： 重置中");
            wait(500, msec);
        }

        if (Controller.ButtonB.pressing())
        {
            return TestMotorWindow->Back();
        }

        wait(10, msec);
    }
}

// 主窗口类
void MainWindow::Run()
{
    BaseWindow *NowWindow;

    NowWindow = InitWindow;
    while (true)
    {

        NowWindow = NowWindow->Tick();

        wait(10, msec);
    }
}
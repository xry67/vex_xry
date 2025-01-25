#include "vex.h"


// 使字符串居中
void CenterStr(char* str){
    char str_copy[50]; // 目标字符数组，确保足够大以容纳源字符串  

    // 使用 strcpy 进行拷贝  
    strcpy(str_copy, str);  

    int Len = Brain.Screen.getStringWidth(str);
    // 使字符串居中
    snprintf(str, 50, "%*s%s", (ControllerScreenWidth - Len) / 2 / 10 - 1, " ", str_copy); 
}

// 遥控器输出重定义到某一行
void PrintInLine(int line){
      Controller.Screen.clearLine(line);  // 首先清除这一行
      Controller.Screen.setCursor(line, 0);
  }

// 从选项中选择一个选项
int ChoiceIndex(std::vector<char*> options){
    int index = 0;
    int n   = options.size();
    bool print_flag = true;

    while (true){
        if (print_flag){
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
        if (Controller.ButtonRight.pressing()  || Controller.Axis4.position() > 50  || Controller.Axis1.position() > 50){
            index = (index + 1) % n;
            print_flag = true;
        }

        // 选项减一
        if (Controller.ButtonLeft.pressing()  || Controller.Axis4.position() < -50  || Controller.Axis1.position() < -50){
            index = index - 1;
            if (index == -1){
                index = n - 1;
            }
            print_flag = true;
        }

        // 选项确认 
        if (Controller.ButtonA.pressing()){
            return index;
        }

        // 退出选择
        if (Controller.ButtonB.pressing()){
          return -1;
        }

        wait(100, msec);
    }
}



BaseWindow* InitWindow = new BaseWindow("初始化", nullptr, InitWindowFun);
BaseWindow* ModWindow  = new BaseWindow("模式选择", nullptr, ModWindowFun);

BaseWindow* RemoteControlWindow_Mod     = new BaseWindow("模式-遥控", ModWindow, RemoteControlWindow_ModFun);     //  模式-遥控
BaseWindow* RemoteControlWindow_Compete = new BaseWindow("比赛-遥控模式", ModWindow, RemoteControlWindow_CompeteFun); //  比赛-遥控模式
BaseWindow* AutoWindow                  = new BaseWindow("自动测试模式", ModWindow, AutoWindowFun);  //  自动测试模式
BaseWindow* TestWindow                  = new BaseWindow("测试模式", ModWindow, TestWindowFun);      //  测试模式

BaseWindow* AutoChoiceWindow_Compete    = new BaseWindow("比赛-自动选择", RemoteControlWindow_Compete, AutoChoiceWindow_CompeteFun);  //  比赛-自动选择
BaseWindow* RemoteControlWindow_Auto    = new BaseWindow("自动-遥控选择", AutoWindow, RemoteControlWindow_AutoFun);  // 自动-遥控选择


// 各个函数实现------------------------------------------

// 初始化函数
BaseWindow* InitWindowFun(){

    return ModWindow;
}

// 模式选择
BaseWindow* ModWindowFun(){

    int index = -1;
    std::vector<char*> options = {"遥控模式", "比赛模式", "自动模式", "测试模式"};
    PrintInLine(1);
    Controller.Screen.print("选择你的操作");
    PrintInLine(3);
    Controller.Screen.print("                 确定(A)");

    while (index == -1){
        index = ChoiceIndex(options);
    }

    std::cout << options[index] << std::endl;

    std::vector<BaseWindow*> ChildWindow{RemoteControlWindow_Mod, RemoteControlWindow_Compete, AutoWindow, TestWindow};


    return ChildWindow[index];
}

//  模式-遥控
BaseWindow* RemoteControlWindow_ModFun(){
    int index = -1;
    std::vector<char*> options = {"左单摇杆", "右单摇杆", "双摇杆", "坦克操控"};
    PrintInLine(1);
    Controller.Screen.print("选择你的遥控方式");
    PrintInLine(3);
    Controller.Screen.print("      确定(A)   返回(B)");

    index = ChoiceIndex(options);  // 获取用户操作

    if (index == -1){
      return RemoteControlWindow_Mod->Back();
    }

    std::cout << options[index] << std::endl;

    return ModWindow;
}

//  比赛-遥控模式
BaseWindow* RemoteControlWindow_CompeteFun(){
    int index = -1;
    std::vector<char*> options = {"左单摇杆", "右单摇杆", "双摇杆", "坦克操控"};
    PrintInLine(1);
    Controller.Screen.print("选择你的遥控方式");
    PrintInLine(3);
    Controller.Screen.print("      确定(A)   返回(B)");

    index = ChoiceIndex(options);  // 获取用户操作

    if (index == -1){
      return RemoteControlWindow_Mod->Back();
    }

    std::cout << options[index] << std::endl;

    return AutoChoiceWindow_Compete;
}

//  自动测试模式
BaseWindow* AutoWindowFun(){
    int index = -1;
    // 获取各个自动程序的名称和内容
    std::vector<char*> options;

    for (AutoFun auto_fun: AutoFunctions){
        options.push_back(auto_fun.name);
    }
    PrintInLine(1);
    Controller.Screen.print("选择你要运行的程序");
    PrintInLine(3);
    Controller.Screen.print("      确定(A)   返回(B)");

    index = ChoiceIndex(options);  // 获取用户操作

    if (index == -1){
      return RemoteControlWindow_Mod->Back();
    }

    std::cout << options[index] << std::endl;

    // 运行自动程序
    double start_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    AutoFunctions[index].fun();
    double end_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();

    PrintInLine(1);
    Controller.Screen.print("运行完成");
    PrintInLine(2);
    Controller.Screen.print("运行时间： %.2f", end_time - start_time);
    PrintInLine(3);
    Controller.Screen.print("      遥控(A)   重试(B)");

    while (true){
        // 选项确认 
        if (Controller.ButtonA.pressing()){
            return RemoteControlWindow_Auto;
        }

        // 退出选择
        if (Controller.ButtonB.pressing()){
          return AutoWindow;
        }
    }

    return ModWindow;
}

//  测试模式
BaseWindow* TestWindowFun(){
    return ModWindow;
}

//  比赛-自动选择
BaseWindow* AutoChoiceWindow_CompeteFun(){
    int index = -1;
    // 获取各个自动程序的名称和内容
    std::vector<char*> options;

    for (AutoFun auto_fun: AutoFunctions){
        options.push_back(auto_fun.name);
    }
    PrintInLine(1);
    Controller.Screen.print("选择你要运行的程序");
    PrintInLine(3);
    Controller.Screen.print("      确定(A)   返回(B)");

    index = ChoiceIndex(options);  // 获取用户操作

    if (index == -1){
      return RemoteControlWindow_Mod->Back();
    }

    
    return ModWindow;
}     

// 自动-遥控选择
BaseWindow* RemoteControlWindow_AutoFun(){
    int index = -1;
    std::vector<char*> options = {"左单摇杆", "右单摇杆", "双摇杆", "坦克操控"};
    PrintInLine(1);
    Controller.Screen.print("选择你的遥控方式");
    PrintInLine(3);
    Controller.Screen.print("      确定(A)   返回(B)");

    index = ChoiceIndex(options);  // 获取用户操作

    if (index == -1){
      return RemoteControlWindow_Auto->Back();
    }

    std::cout << options[index] << std::endl;

    return ModWindow;
}


// 主窗口类
void MainWindow::Run(){
    BaseWindow* NowWindow;

    NowWindow = InitWindow;
    while (true){

        NowWindow = NowWindow->Tick();

        wait(10, msec);
    }
}
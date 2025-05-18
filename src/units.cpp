#include "vex.h"
#include "robot-config.h"
#include "units.h"

// 使字符串居中
void CenterStr(char* str){
    char str_copy[50]; // 目标字符数组，确保足够大以容纳源字符串  

    // 使用 strcpy 进行拷贝  
    strcpy(str_copy, str);  

    int Len = Brain.Screen.getStringWidth(str);
    // 使字符串居中
    snprintf(str, 50, "%*s%s", (ControllerScreenWidth - Len) / 2 / 10 - 1, " ", str_copy); 
}

// 获得数字序列字符串
std::vector<char*> generateNumberStrings(int start, int end) {  
    std::vector<char*> vec;  

    for (int i = start; i <= end; i++) {  
        // 为每个字符数组分配内存，2个数字加一个结束符  
        char* str = new char[3]; // 3个字符 (00\0)  
        snprintf(str, 3, "%02d", i); // 使用snprintf生成带前导零的字符串  
        vec.push_back(str); // 将字符数组指针添加到vector中  
    }  

    return vec; // 返回含有char*的vector  
} 

// 遥控器输出重定义到某一行
void PrintInLine(int line){
      Controller.Screen.clearLine(line);  // 首先清除这一行
      Controller.Screen.setCursor(line, 0);
}

// 输出到某一行并且打印文字
void PrintAt(int line, char* str){
    PrintInLine(line);
    Controller.Screen.print(str);
}

// 输出到某一行并且打印文字
void PrintAt(int line, char* str, int num){
    PrintInLine(line);
    Controller.Screen.print(str, num);
}

// 输出到某一行并且打印文字
void PrintAt(int line, char* str, float num){
    PrintInLine(line);
    Controller.Screen.print(str, num);
}

// 输出到某一行并且打印文字
void PrintAt(int line, char* str, double num){
    PrintInLine(line);
    Controller.Screen.print(str, num);
}

// 获得字母序列
std::vector<char*> generateAlphabetStrings(char start, char end) {  
    std::vector<char*> vec;  

    for (char c = start; c <= end; ++c) {  
        // 为每个字符数组分配内存，1个字符加一个结束符  
        char* str = new char[2]; // 2个字符 (C\0)  
        str[0] = c; // 填充字符  
        str[1] = '\0'; // 添加结束符  
        vec.push_back(str); // 将字符数组指针添加到vector中  
    }  

    return vec; // 返回含有char*的vector  
} 

// 将角度规范到-180到180
double NormsAngle(double val){
  return val - ((int)val / 360) * 360;
}
#ifndef UNITS_H  
#define UNITS_H 

#include <vector>
#include "vex.h"
#include "robot-config.h"

// 遥控器输出重定义到某一行
void PrintInLine(int line);

// 使字符串居中
void CenterStr(char* str);

// 获得数字序列字符串
std::vector<char*> generateNumberStrings(int start, int end);

// 获得字母序列
std::vector<char*> generateAlphabetStrings(char start, char end);

// 输出到某一行并且打印文字
void PrintAt(int line, char* str);

// 输出到某一行并且打印文字
void PrintAt(int line, char* str, int num);

// 输出到某一行并且打印文字
void PrintAt(int line, char* str, float num);

// 输出到某一行并且打印文字
void PrintAt(int line, char* str, double num);

// 将角度规范到-180到180
double NormsAngle(double val);

// 等待转圈的字符
class WaitStr
{
    private:
        std::vector<char*> str = {"-", "/", "|", "\\"};
        int index = 0;

    public:
        char* GetNextStr(){
            index = (index + 1) % str.size();
            return str[index];
        }

};

// 将数字规范到这个区间
double NumInterval(double num, double mininum, double maxinum);

#endif
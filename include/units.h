#ifndef UNITS_H
#define UNITS_H

#include <vector>
#include "vex.h"
#include "robot-config.h"

// 遥控器输出重定义到某一行
void PrintInLine(int line);

// 使字符串居中
void CenterStr(char *str);

// 获得数字序列字符串
std::vector<char *> generateNumberStrings(int start, int end);

// 获得字母序列
std::vector<char *> generateAlphabetStrings(char start, char end);

// 输出到某一行并且打印文字
void PrintAt(int line, char *str);

// 输出到某一行并且打印文字
void PrintAt(int line, char *str, int num);

// 输出到某一行并且打印文字
void PrintAt(int line, char *str, float num);

// 输出到某一行并且打印文字
void PrintAt(int line, char *str, double num);

// 将角度规范到-180到180
double NormsAngle(double val);

// 将角度规范到0到360
double NormsAngle_0_360(double val);

// 等待转圈的字符
class WaitStr
{
private:
    std::vector<char *> str = {"-", "/", "|", "\\"};
    int index = 0;

public:
    char *GetNextStr()
    {
        index = (index + 1) % str.size();
        return str[index];
    }
};

// 颜色传感器有一个红色物体？
bool OpticalHaveRed(vex::optical Optical);

// 颜色传感器有一个蓝色物体？
bool OpticalHaveBlue(vex::optical Optical);

// 颜色传感器有一个红色物体？
bool OpticalHaveRed_notNear(vex::optical Optical);

// 颜色传感器有一个蓝色物体？
bool OpticalHaveBlue_notNear(vex::optical Optical);

// 方向向量转换为角度
double calculateAngle(int x, int y);

double angleDifference(double current_angle, double target_angle);

// 将数字规范到这个区间
double NumInterval(double num, double mininum, double maxinum);

// 三个数字的中值
double medianOfThree(double a, double b, double c);

// 获取时间（秒）
double GetNowTime();

#endif
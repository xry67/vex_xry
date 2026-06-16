#include "vex.h"
#include "robot-config.h"
#include "units.h"

// 使字符串居中
void CenterStr(char *str)
{
    char str_copy[50]; // 目标字符数组，确保足够大以容纳源字符串

    // 使用 strcpy 进行拷贝
    strcpy(str_copy, str);

    int Len = Brain.Screen.getStringWidth(str);
    // 使字符串居中
    snprintf(str, 50, "%*s%s", (ControllerScreenWidth - Len) / 2 / 10 - 1, " ", str_copy);
}

// 获得数字序列字符串
std::vector<char *> generateNumberStrings(int start, int end)
{
    std::vector<char *> vec;

    for (int i = start; i <= end; i++)
    {
        // 为每个字符数组分配内存，2个数字加一个结束符
        char *str = new char[3];     // 3个字符 (00\0)
        snprintf(str, 3, "%02d", i); // 使用snprintf生成带前导零的字符串
        vec.push_back(str);          // 将字符数组指针添加到vector中
    }

    return vec; // 返回含有char*的vector
}

// 遥控器输出重定义到某一行
void PrintInLine(int line)
{
    Controller.Screen.clearLine(line); // 首先清除这一行
    Controller.Screen.setCursor(line, 0);
}

// 输出到某一行并且打印文字
void PrintAt(int line, char *str)
{
    PrintInLine(line);
    Controller.Screen.print(str);
}

// 输出到某一行并且打印文字
void PrintAt(int line, char *str, int num)
{
    PrintInLine(line);
    Controller.Screen.print(str, num);
}

// 输出到某一行并且打印文字
void PrintAt(int line, char *str, float num)
{
    PrintInLine(line);
    Controller.Screen.print(str, num);
}

// 输出到某一行并且打印文字
void PrintAt(int line, char *str, double num)
{
    PrintInLine(line);
    Controller.Screen.print(str, num);
}

// 获得字母序列
std::vector<char *> generateAlphabetStrings(char start, char end)
{
    std::vector<char *> vec;

    for (char c = start; c <= end; ++c)
    {
        // 为每个字符数组分配内存，1个字符加一个结束符
        char *str = new char[2]; // 2个字符 (C\0)
        str[0] = c;              // 填充字符
        str[1] = '\0';           // 添加结束符
        vec.push_back(str);      // 将字符数组指针添加到vector中
    }

    return vec; // 返回含有char*的vector
}

// 将角度规范到-180到180
double NormsAngle(double val)
{
    return val - ((int)val / 360) * 360;
}

// 将角度规范到0到360
double NormsAngle_0_360(double val)
{
    val = NormsAngle(val);

    if (val < 0)
    {
        val = 360 + val;
    }
    return val;
}

// 颜色传感器有一个红色物体？
bool OpticalHaveRed(vex::optical Optical)
{
    double H = Optical.hue();
    if (Optical.isNearObject() && ((H > 0 && H < 70) || (H > 300 && H < 360)))
    {
        return true;
    }
    return false;
}

// 颜色传感器有一个蓝色物体？
bool OpticalHaveBlue(vex::optical Optical)
{
    double H = Optical.hue();
    if (Optical.isNearObject() && ((H > 100) && (H < 270)))
    {
        return true;
    }
    return false;
}

// 颜色传感器有一个红色物体？
bool OpticalHaveRed_notNear(vex::optical Optical)
{
    double H = Optical.hue();
    if (((H > 0 && H < 70) || (H > 300 && H < 360)))
    {
        return true;
    }
    return false;
}

// 颜色传感器有一个蓝色物体？
bool OpticalHaveBlue_notNear(vex::optical Optical)
{
    double H = Optical.hue();
    if (((H > 100) && (H < 270)))
    {
        return true;
    }
    return false;
}

// 向量转换为角度
double calculateAngle(int x, int y)
{
    // 计算弧度
    double radians = atan2(x, y);
    // 转换为度数
    double degrees = radians * (180.0 / M_PI);
    return degrees;
}

// 计算角度差值的函数
double angleDifference(double current_angle, double target_angle)
{
    double difference = fmod(target_angle - current_angle + 180, 360) - 180;

    // 根据差值的方向调整正负
    if (difference > 0)
    {
        // 如果差值为正，表示向右旋转
        return difference; // 保持正数
    }
    else
    {
        // 如果差值为负，表示向左旋转
        return difference; // 保持负数
    }
}

// 将数字规范到这个区间
double NumInterval(double num, double mininum, double maxinum)
{
    if (num < 0)
    {
        // 控制小车运动
        if (num > -1 * mininum)
        {
            return -1 * mininum;
        }
        else
        {
            return std::max(-1 * maxinum, num);
        }
    }
    else
    {
        // 控制小车运动
        if (num < mininum)
        {
            return mininum;
        }
        else
        {
            return std::min(maxinum, num);
        }
    }
}

// 三个数字的中值
double medianOfThree(double a, double b, double c)
{
    // 将三个数放入数组中
    double arr[] = {a, b, c};
    // 对数组进行排序
    std::sort(arr, arr + 3);
    // 返回中间值
    double ans = arr[1];
    return ans;
}

// 获取时间（秒）
double GetNowTime()
{
    return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); // 运动开始时间
}

// 获取时间（毫秒）
double GetNowTime_ms()
{
    return Brain.Timer.time();
}
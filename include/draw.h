#ifndef DRAW_H
#define DRAW_H

#include "vex.h"
#include "robot-config.h"
#include "units.h"
#include <vector>

// 绘制电量
class DrawBattery
{
public:
    color Color;
    int x = 447, y = 16, w = 15, h = 212;
    double last_battery = 0;

    void Update()
    {
        // 获取电池电量百分比
        double battery_capacity = Brain.Battery.capacity(pct);
        battery_capacity = std::max(0.0, std::min(100.0, battery_capacity));

        if (std::abs(battery_capacity - last_battery) < 10)
        {
            return;
        }
        else
        {
            last_battery = battery_capacity;
        }

        Brain.Screen.setPenColor(color(0, 255, 0));

        // 绘制固定的 10 个长方形
        for (int i = 0; i < 10; ++i)
        {
            int rect_y = y + (i * (h / 10)); // 计算每个长方形的 y 坐标

            // 根据电量决定颜色
            if ((10 - i - 1) < battery_capacity / 10)
            {
                Color.rgb(0, 255, 0); // 电量大于 0，绿色
            }
            else
            {
                Color.rgb(0, 0, 0); // 电量为 0，黑色
            }
            Brain.Screen.drawRectangle(x, rect_y, w, h / 10 - 5, Color); // 绘制长方形
        }
    }
};

// 绘制站位
class DrawStance
{
public:
    color Color;
    int x = 0, y = 0, w = 480, h = 240, BorderWidth = 5;
    char last_stance = 'i';

    void Updata()
    {

        if (last_stance == stance)
        {
            return;
        }
        else
        {
            last_stance = stance;
        }

        if (stance == 'r')
        {
            Color.rgb(255, 0, 0);
        }
        else
        {
            Color.rgb(0, 0, 255);
        }
        Brain.Screen.setPenColor(Color);
        // 绘制四个粗边
        // 上边
        Brain.Screen.drawRectangle(x, y, w, BorderWidth, Color);
        // 右边
        Brain.Screen.drawRectangle(x + w - BorderWidth, y, BorderWidth, h, Color);
        // 下边
        Brain.Screen.drawRectangle(x, y + h - BorderWidth, w, BorderWidth, Color);
        // 左边
        Brain.Screen.drawRectangle(x, y, BorderWidth, h, Color);
    }
};

// 绘制马达
class DrawMotor
{
public:
    color Color;
    int x = 0, y = 0, w = 0, h = 0;
    double last_temperature = -100;

    DrawMotor(int x_, int y_, int w_, int h_)
        : x(x_), y(y_), w(w_), h(h_)
    {
    }

    void Draw(double temperature)
    {

        if (temperature <= 1)
        {
            Brain.Screen.setPenColor(color(255, 255, 255));
            Color.rgb(0, 0, 0);
            Brain.Screen.drawRectangle(x, y, w, h, Color);
            return;
        }
        // 每几度替换一下
        if (std::abs(last_temperature - temperature) < 5)
        {
            return;
        }
        else
        {
            last_temperature = temperature;
        }

        Brain.Screen.setPenColor(color(255, 255, 255));
        Color.hsv(GetTemperatureColor(temperature), 1.0, 1.0);
        Brain.Screen.drawRectangle(x, y, w, h, Color);
    }

    int GetTemperatureColor(int temperature)
    {
        // 120 -0: 绿色到红色
        if (temperature < 30)
        {
            return 120;
        }

        return -2 * temperature * temperature / 15 + 8 * temperature;
    }
};

// 绘制底盘
class DrawChassis
{
public:
    color Color = color(100, 100, 100);
    int x = 51, y = 25, w = 57, h = 82;

    DrawMotor DrawMotorL1 = DrawMotor(42, 31, 15, 19);
    DrawMotor DrawMotorL2 = DrawMotor(42, 80, 15, 19);
    DrawMotor DrawMotorR1 = DrawMotor(101, 31, 15, 19);
    DrawMotor DrawMotorR2 = DrawMotor(101, 80, 15, 19);

    void StartInit()
    {
        Brain.Screen.setPenColor(color(255, 255, 255));
        Brain.Screen.drawRectangle(x, y, w, h, Color);
        Updata();
    }

    void Updata()
    {
        DrawMotorL1.Draw(LeftMotor1.temperature(celsius));
        DrawMotorL2.Draw(LeftMotor2.temperature(celsius));
        DrawMotorR1.Draw(RightMotor1.temperature(celsius));
        DrawMotorR2.Draw(RightMotor2.temperature(celsius));
    }
};

// 颜色传感器
class DrawColor
{
public:
    color Color = color(100, 100, 100);
    int x = 77, y = 171, r = 38;
    int last_color = 0;

    void Updata()
    {
        int now_color = 0;
        if (OpticalHaveRed(Optical1))
        {
            Color.rgb(255, 0, 0);
            now_color = 1;
        }
        else if (OpticalHaveBlue(Optical1))
        {
            Color.rgb(0, 0, 255);
            now_color = 2;
        }
        else
        {
            Color.rgb(0, 0, 0);
            now_color = 3;
        }

        if (now_color == last_color)
        {
            return;
        }
        last_color = now_color;

        Brain.Screen.setPenColor(color(255, 255, 255));
        Brain.Screen.drawCircle(x, y, r, Color);
    }
};

// 绘制各类设备状态

class DrawEquipmentMonitor
{
public:
    DrawChassis Chassis = DrawChassis();
    DrawStance Stance = DrawStance();
    DrawBattery Battery = DrawBattery();
    DrawColor OptColor = DrawColor();

    DrawMotor ToolM1 = DrawMotor(171, 31, 12, 70);
    DrawMotor ToolM2 = DrawMotor(187, 31, 12, 70);
    DrawMotor ToolM3 = DrawMotor(152, 82, 13, 19);
    DrawMotor ToolM4 = DrawMotor(204, 31, 33, 14);
    // DrawMotor ToolM5 = DrawMotor(224, 49, 13, 12);
    DrawMotor ToolM6 = DrawMotor(241, 49, 14, 28);

    DrawEquipmentMonitor()
    {
    }

    void StartInit()
    {
        Stance.Updata();
        Battery.Update();

        Brain.Screen.setPenColor(color(255, 255, 255));
        Brain.Screen.drawRectangle(17, 16, 423, 100, color(0, 0, 0));
        Brain.Screen.drawRectangle(17, 123, 423, 100, color(0, 0, 0));

        OptColor.Updata();
        Chassis.StartInit();
        ToolM1.Draw(ToolMotor1.temperature());
        ToolM2.Draw(ToolMotor2.temperature());
        ToolM3.Draw(ToolMotor3.temperature());
        ToolM4.Draw(ToolMotor4.temperature());
        // ToolM5.Draw(ToolMotor5.temperature());
        ToolM6.Draw(ToolMotor6.temperature());
    }

    void Updata()
    {

        Stance.Updata();
        Battery.Update();
        OptColor.Updata();
        Chassis.Updata();
        ToolM1.Draw(ToolMotor1.temperature(celsius));
        ToolM2.Draw(ToolMotor2.temperature(celsius));
        ToolM3.Draw(ToolMotor3.temperature(celsius));
        ToolM4.Draw(ToolMotor4.temperature(celsius));
        // ToolM5.Draw(ToolMotor5.temperature(celsius));
        ToolM6.Draw(ToolMotor6.temperature(celsius));
        // std::cout << ToolMotor1.temperature(celsius) << " " << ToolMotor2.temperature(celsius) << " " << ToolMotor3.temperature(celsius) << std::endl;
    }
};

#endif
#ifndef PPS_H
#define PPS_H

#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm> // 包含 min 和 max 函数

// P (Proportional Gain)
// 比例增益。

// Path Split Percentage
// 简写：pathSplitPct 或 splitPercentage
// 说明：表示路径分割的百分比。

// Speed Acceleration Factor
// 简写：speedAccelFactor 或 accelMultiplier
// 说明：表示路径分割时的速度加速倍数。

// PPS算法类
class PPS
{
public:
    PPS(){}

    PPS(double kp1, double kp2, double ks)
        : kp1(kp1), kp2(kp2), ks(ks), prev_error(0.0), integral(0.0) {}

    void setSetpoint(double val)
    {
        setpoint = val;
    }

    void setStartpoint(double val){
        startpoint = val;
    }

    void reset(double kp1_, double kp2_, double ks_)
    {
        kp1 = kp1_;
        kp2 = kp2_;
        ks = ks_;
        prev_error = 0.0;
        integral = 0.0;
    }

    double getsetpoint()
    {
        return setpoint;
    }

    double calculate(double measured, double dt)
    {
        double error = setpoint - measured;

        double output = kp1 * error;

        // 分段pps测试
        double k = kp2;
        double Completeness = 1 - std::abs(measured - setpoint) / std::abs(startpoint - setpoint); // 完成度

        if (Completeness < k && k >= 0.001)
        {
            output *= ks * (1 - Completeness);
        }

        return output;
    }
    
    // 及时计算PPS输出值
    // static 静态方法，不需要实例化该类
    static double calculate_now(double kp1_input, double kp2_input, double ks_input, double target_val, double measured)
    {
        double error = target_val - measured;
        double output = kp1_input * error;
        double Completeness = 1 - std::abs(measured - target_val) / std::abs(target_val); // 完成度

        if (Completeness < kp2_input && kp2_input >= 0.001)
        {
            output *= ks_input * (1 - Completeness);
        }

        return output;
    }

private:
    double kp1;
    double kp2;
    double ks;
    double setpoint;
    double startpoint = 0;
    double prev_error;
    double integral;
};

#endif
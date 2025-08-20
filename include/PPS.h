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
class PPS {  
public:  
    PPS(double kp1, double kp2, double ks)  
        : kp1(kp1), kp2(kp2), ks(ks), prev_error(0.0), integral(0.0) {}  

    void setSetpoint(double val){
      setpoint = val;
    }

    void reset(double kp1_, double kp2_, double ks_){
        kp1 = kp1_;
        kp2 = kp2_;
        ks = ks_;
        prev_error = 0.0;
        integral   = 0.0;
    }

    double getsetpoint(){
        return setpoint;
    }

    double calculate(double measured, double dt) {  
        double error = setpoint - measured;  
        
        double output = kp1 * error;  

        // 分段pps测试
        double k = kp2;
        double Completeness = 1 - std::abs(measured - setpoint) / std::abs(setpoint);  // 完成度
        // if (std::abs(measured - setpoint) < 30){
        //     output = output;
        // }
        if (Completeness < k && k >= 0.001){
            output *= ks * (1 - Completeness );
        }

        return output;  
    }  

private:  
    double kp1;  
    double kp2;  
    double ks; 
    double setpoint;
    double prev_error;  
    double integral;  
};  


#endif
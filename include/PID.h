#ifndef PID_H  
#define PID_H 

#include <iostream>  
#include <chrono>  
#include <thread>  

// PID算法类
class PID {  
public:  
    PID(double kp, double ki, double kd)  
        : kp(kp), ki(ki), kd(kd), prev_error(0.0), integral(0.0) {}  

    void setSetpoint(double val){
      setpoint = val;
    }

    void reset(double kp_, double ki_, double kd_){
        kp = kp_;
        ki = ki_;
        kd = kd_;
        prev_error = 0.0;
        integral   = 0.0;
    }

    double getsetpoint(){
        return setpoint;
    }

    double calculate(double measured, double dt) {  
        double error = setpoint - measured;  
        integral += error * dt;  
        double derivative = (error - prev_error) / dt;  
        
        double output = kp * error + ki * integral + kd * derivative;  
        // std::cout << "PID: " << error << " " << integral << " " << derivative << std::endl;
        prev_error = error;  
        return output;  
    }  

private:  
    double kp;  
    double ki;  
    double kd; 
    double setpoint;
    double prev_error;  
    double integral;  
};  


#endif
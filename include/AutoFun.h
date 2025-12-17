#ifndef AUTOFUN_H  
#define AUTOFUN_H 

class AutoFun{
    public:
    char* name;
    void (*fun)();  
    double start_time = 0;   // 记录开始时间
    double out_time   = 15;  // 自动最大允许允许运行时间,默认15秒
    bool Over = false;  // 记录自动是否结束

    AutoFun(char* name_, void (*fun_)()){
        name = name_;
        fun  = fun_;
    }

    AutoFun(char* name_, void (*fun_)(), double out_time_){
        name = name_;
        fun  = fun_;
        out_time = out_time_;
    }

    // 获取自动运行时间
    double GetAutoRunTime(){
        return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count() - start_time;
    }

    // 获取自动是否结束
    bool IsOver(){
        return Over;
    }

    // 运行自动函数
    void RunAutoFun(){
        start_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
        Over = false; 
        fun();
        Over = true; 
    }


};

#endif
#ifndef AUTOFUN_H  
#define AUTOFUN_H 

class AutoFun{
    public:
    char* name;
    void (*fun)();

    AutoFun(char* name_, void (*fun_)()){
        name = name_;
        fun  = fun_;
    }
};

#endif
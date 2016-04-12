[mw_shl_code=cpp,true]#ifndef  MOTOR1_H_
#define MOTOR1_H_   //防止重复包含
class motor1
{
 public:                      //共有部分
      motor1();
      void run(char pin1,char pin2,char pwmpin,char state,int val);     //假如motor1()或run()里定义了变量，就得写出私有部分                                
};                                                                                                     //private:相应的变量。所谓私有，就是不能给用户调用的。
#endif[/mw_shl_code]
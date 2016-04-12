#include <WProgram.h>   
#include "motor1.h"    //include后面加空格（注意这些小细节）

motor1::motor1()
{
}                            //建一个构造函数，当然也可以带参数
void motor1::run(char pin1,char pin2,char pwmpin,char state,int val)    //建一个带参数的子函数 pin1 pin2输入高低电平引脚
{                                                                                          //pwmpin为PWM输入引脚,state为正反转，va为l输入调速值0~255
  pinMode(pin1,1);
  pinMode(pin2,1);
   pinMode(pwmpin,1); 
 if(state)
 {  
  analogWrite(pwmpin,val);
  digitalWrite(pin1,1);
  digitalWrite(pin2,0);
 }
 else
 {  
  analogWrite(pwmpin,val);
  digitalWrite(pin1,0);
  digitalWrite(pin2,1);
 }
}
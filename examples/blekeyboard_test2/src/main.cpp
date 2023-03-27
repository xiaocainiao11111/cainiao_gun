#include <Arduino.h>

#include <BleKeyboard.h>



BleKeyboard blekeyboard("ESP32kEYBOARD","Espressif",100);    
                        // 设备名，制造商，电量
const int OutPIN = 12;    // 定义按键端口
 
void setup() 
{
  Serial.begin(115200);
  blekeyboard.begin();    
  Serial.println("1234");
}
 
void loop() 
{
  if(blekeyboard.isConnected())    // 判断连接是否成功
  {
    static int a = 1;    // 第一次连接上打印连接成功
    if(a == 1)
    {
      Serial.println("111");
      a++;
    }
 
    if(!digitalRead(OutPIN))    // 按键被按下
    {
      delay(50);
      if(!digitalRead(OutPIN))    // 消抖，也不知道ESP32上用不用消抖
      {
        Serial.println("输出");    
        
        blekeyboard.println("Don't say anything that is not conducive to unity");    // 打印字符
        delay(100);
        blekeyboard.write(KEY_NUM_ENTER);    // 按键按下enter
      }
    }
  }
  else    // 没有连接成功
  {
    Serial.println("222");    // 循环打印蓝牙未连接
    delay(1000);
  }
}

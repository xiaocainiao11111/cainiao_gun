//已验证，2023/3/12

#include <Arduino.h>
#include <BleKeyboard.h>



BleKeyboard blekeyboard("cainiao111","Espressif",100);    
                        // 设备名，制造商，电量

static bool ctrl_c()
{
    static int press_cnt1 = 0;
    bool ret_cnt1 = false;
    if(touchRead(12) < 15){
        press_cnt1 ++;
        if (press_cnt1 > 4) {
            if( touchRead(12) < 15){
                ret_cnt1 = true;
                press_cnt1 = -8;
            }
        }
    }else{
        press_cnt1 = 0;
    }    
    return ret_cnt1;
}



static bool ctrl_v()
{
    static int press_cnt2 = 0;
    bool ret_cnt2 = false;
    if(touchRead(14) < 15){
        press_cnt2 ++;
        if (press_cnt2 > 4) {
            if( touchRead(14) < 15){
                ret_cnt2 = true;
                press_cnt2 = -8;
            }
        }
    }else{
        press_cnt2 = 0;
    }    
    return ret_cnt2;
}


 
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

    Serial.println("111");

    // static int a = 1;    // 第一次连接上打印连接成功
    // if(a == 1)
    // {
    //   Serial.println("111");
    //   a++;
    // }
 
  }


  else    // 没有连接成功
  {
    Serial.println("222");    // 循环打印蓝牙未连接
    delay(1000);
  }
  if(ctrl_c()){
    blekeyboard.press(KEY_LEFT_CTRL);
    blekeyboard.press(99);
    delay(50);
    blekeyboard.releaseAll();
    Serial.println("ctrl+c");
  }
  if(ctrl_v()){
    blekeyboard.press(KEY_LEFT_CTRL);
    blekeyboard.press(118);
    delay(50);
    blekeyboard.releaseAll();
    Serial.println("ctrl+v");
  }

}



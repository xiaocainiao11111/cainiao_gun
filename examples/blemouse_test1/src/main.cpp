//已验证，blemouse
#include <Arduino.h>

#include <BleMouse.h>

BleMouse bleMouse("ESP32蓝牙鼠标","Espressif",100);

static bool event1()
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



static bool event2()
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


void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE work!");
  bleMouse.begin();
}


void loop() {
    // Serial.println("左键点击");
    // bleMouse.click(MOUSE_LEFT);
    // delay(500);
 
    // Serial.println("右键单击");
    // bleMouse.click(MOUSE_RIGHT);
    // delay(500);
 
    // Serial.println("滚轮点击");
    // bleMouse.click(MOUSE_MIDDLE);
    // delay(500);
 
    // Serial.println("后退按钮点击");
    // bleMouse.click(MOUSE_BACK);
    // delay(500);
 
    // Serial.println("前进按钮点击");
    // bleMouse.click(MOUSE_FORWARD);
    // delay(500);
 
    // Serial.println("同时单击鼠标左键和鼠标右键");
    // bleMouse.click(MOUSE_LEFT | MOUSE_RIGHT);
    // delay(500);
 
    // Serial.println("同时单击鼠标左键+鼠标右键和滚轮");
    // bleMouse.click(MOUSE_LEFT | MOUSE_RIGHT | MOUSE_MIDDLE);
    // delay(500);

  if(event1()){
    Serial.println("left");
    bleMouse.move(0,1);

    delay(2);
  }
  if(event2()){
    Serial.println("right");
    // bleMouse.click(MOUSE_RIGHT);
    bleMouse.move(1,0);
    delay(2);


  }

}
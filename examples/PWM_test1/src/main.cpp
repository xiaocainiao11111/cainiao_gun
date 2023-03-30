#include <Arduino.h>

// GPIO_14 输出PWM
// GPIO_12 读取IO14输出的信号


void pwm_init(int a){
  for(int c=0;c<8;c++){
  digitalWrite(14, HIGH);
  delay(a);
  digitalWrite(14, LOW);
  delay(a);}

}

void setup()
{
  Serial.begin(115200);
  Serial.println("1112");

  // pinMode(14,OUTPUT);
  // digitalWrite(14, HIGH);

  // ledcSetup(8, 1, 10);  //设置LEDC通道8频率为1，分辨率为10位，即占空比可选0~1023
  // ledcAttachPin(14, 8); //设置LEDC通道8在IO14上输出

  // pinMode(12, INPUT_PULLDOWN);   //引脚12，下拉输入

  // for (int i = 0; i < 5; i++)
  // {
  //   ledcWrite(8, 250 * i); //设置输出PWM
  //   for (int j = 0; j < 100; j++)
  //   {
  //     delay(100);
  //     Serial.println(digitalRead(12));  //读取引脚12的电平并输出
  //   }
  // }
}

void loop()
{
  // for(int b=800;b>=100;b=b-50){
  // pwm_init(b);
  // }
  Serial.println(millis());
  delay(100);


}

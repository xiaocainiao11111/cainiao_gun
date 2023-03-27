#include <Arduino.h>

#include "combo/BleCombo.h"


#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


int16_t ax, ay, az, gx, gy, gz;

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



const int x = 0;
const int y = 4;
const int sw = 23;

uint8_t moveX=0,moveY=0,IO5=0,IO18=0,IO16=0,IO17=0,pat=0;
int8_t mouseX=0,mouseY=0;
uint8_t cnt1=1,cnt2=1,cnt3=1,cnt4=1;

//触摸消抖
static bool T32()
{
    static int press_cnt1 = 0;
    bool ret_cnt1 = false;
    if(touchRead(32) < 10){
        press_cnt1 ++;
        if (press_cnt1 > 6) {
            if( touchRead(32) < 10){
                ret_cnt1 = true;
                press_cnt1 = -8;
            }
        }
    }else{
        press_cnt1 = 0;
    }    
    return ret_cnt1;
}



static bool T33()
{
    static int press_cnt2 = 0;
    bool ret_cnt2 = false;
    if(touchRead(33) < 15){
        press_cnt2 ++;
        if (press_cnt2 > 4) {
            if( touchRead(33) < 15){
                ret_cnt2 = true;
                press_cnt2 = -8;
            }
        }
    }else{
        press_cnt2 = 0;
    }    
    return ret_cnt2;
}


//mpu6050_DMP滤波
void mpu_DMP(){
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            // Serial.print("ypr\t");
            // Serial.print(ypr[0] * 180/M_PI/15);
            // Serial.print("\t");
            // Serial.print(ypr[1] * 180/M_PI/15);
            // Serial.print("\t");
            // Serial.println(ypr[2] * 180/M_PI);

            // Serial.print("ypr\t");
            // Serial.print(gy/150);
            // Serial.print("\t");
            // Serial.print(ypr[2] * 180/M_PI);
            // Serial.print("\t");
            // Serial.println(gz/150);
            // delay(100);

            // mouseX=-gz/150;
            // mouseY=-gy/150;

        #endif

    }
}

//模式切换
void pattern_ctrl(){
    if(digitalRead(5)){pat=1;}
    else{pat=0;}
}


//模式功能
void pattern(){
    if(pat==0){
        mouseX=ypr[0] * 180/M_PI/15;
        mouseY=ypr[1] * 180/M_PI/15;
    }
    else{mouseX=-gz/150;mouseY=-gy/150;}
}

//摇杆移动
void encoder(){
    int x_value = analogRead(x),y_value = analogRead(y);

    if(x_value>3500){
        if(cnt3){Keyboard.press(119);moveX=1;cnt3=0;}
    }
    else if(x_value<500){
        if(cnt3){Keyboard.press(115);moveX=-1;cnt3=0;}
    }
    else{
        if(cnt3==0){Keyboard.release(119);Keyboard.release(115);cnt3=1;}
    }
    if(y_value>3800){
        if(cnt4){Keyboard.press(100);moveY=1;cnt4=0;}
    }
    else if(y_value<500){
        if(cnt4){Keyboard.press(97);moveY=-1;cnt4=0;}
    }
    else{
        if(cnt4==0){Keyboard.release(100);Keyboard.release(97);cnt4=1;}
    }}


//鼠标移动
void mouse_Move(){
//   if(Keyboard.isConnected()) {
    // if(mouseX!=0|mouseY!=0){
    Mouse.move(mouseX,-mouseY);
    delay(4);
    // }
//   }
}

//鼠标右键
void mouse_Right(){
    if(digitalRead(16)){
        if(cnt1){Mouse.press(MOUSE_RIGHT);cnt1=0;Serial.println("111111111111111");}
    }
    else{
        if(cnt1==0){Mouse.release(MOUSE_RIGHT);cnt1=1;}
    }
}

//鼠标左键
void mouse_Left(){
    if(digitalRead(17)){
        if(cnt2){Mouse.press(MOUSE_LEFT);cnt2=0;}
    }
    else{
        if(cnt2==0){Mouse.release(MOUSE_LEFT);cnt2=1;}
    }
}


//参数打印
void _print(){
    Serial.println(
        "moveX:"+String(moveX)+
        "  moveY:"+String(moveY)+
        "  mouseX:"+String(mouseX)+
        "  mouseY:"+String(mouseY)+
        "  IO5:"+String(digitalRead(5))+
        "  IO18:"+String(digitalRead(18))+
        "  IO16:"+String(digitalRead(16))+
        "  IO17:"+String(digitalRead(17))+
        "  pat:"+String(pat)
    );
}

void setup() {
    Serial.begin(115200);
    Serial.println("Starting work!");
    Keyboard.begin();
    Mouse.begin();
    pinMode(sw,INPUT_PULLUP);
    pinMode(5,INPUT_PULLDOWN);
    pinMode(18,INPUT_PULLDOWN);
    pinMode(16,INPUT_PULLDOWN);
    pinMode(17,INPUT_PULLDOWN);

    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    while (!Serial); // wait for Leonardo enumeration, others continue immediately


    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // // wait for ready
    // Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    // while (Serial.available() && Serial.read()); // empty buffer
    // while (!Serial.available());                 // wait for data
    // while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

}

void loop() {
    pattern_ctrl();
    pattern();
    mpu_DMP();
    mouse_Move();
    mouse_Right();
    mouse_Left();
    encoder();
    _print();
    moveX=0,moveY=0,mouseX=0,mouseY=0,IO5=0,IO18=0,IO16=0,IO17=0;
    delay(1);

  //摇杆：已验证
//   int x_value = analogRead(x);
//   int y_value = analogRead(y);
//   Serial.print("x: ");
//   Serial.print(x_value);
//   Serial.print("\t");
//   Serial.print("y: ");
//   Serial.print(y_value);
//   Serial.print("\t");
//   Serial.print("sw: ");
//   Serial.print(digitalRead(sw));
//   Serial.println("\t");
//   delay(200);
  //..................


  // if(Keyboard.isConnected()) {
  //   Serial.println("Sending 'Hello world'");
  //   // Keyboard.println("Hello World");

  //   delay(1000);
  //   Serial.println("Sending Enter key...");

  //   if(digitalRead(34) == 1){Serial.println(1);}
  //   if(digitalRead(34) == 0){Serial.println(0);}
    //.................
    // Keyboard.write(KEY_RETURN);
    // Keyboard.press(99);
    // delay(50);
    // Keyboard.releaseAll();
    //......................


    // delay(1000);
  
    // Serial.println("Sending Play/Pause media key...");
    // Keyboard.write(KEY_MEDIA_PLAY_PAUSE);

    // delay(1000);

    // Serial.println("Sending Ctrl+Alt+Delete...");
    // Keyboard.press(KEY_LEFT_CTRL);
    // Keyboard.press(KEY_LEFT_ALT);
    // Keyboard.press(KEY_DELETE);
    // delay(100);
    // Keyboard.releaseAll();

    //.........................
    // unsigned long startTime;

    // Serial.println("Move mouse pointer up");
    // startTime = millis();
    // while(millis()<startTime+1000) {
    //   Mouse.move(0,-1);
    //   delay(5);
    // }
    //............................

    // Serial.println("Move mouse pointer left");
    // startTime = millis();
    // while(millis()<startTime+1000) {
    //   Mouse.move(-1,0);
    //   delay(5);
    // }

    // Serial.println("Move mouse pointer down");
    // startTime = millis();
    // while(millis()<startTime+1000) {
    //   Mouse.move(0,1);
    //   delay(5);
    // }

    // Serial.println("Move mouse pointer right");
    // startTime = millis();
    // while(millis()<startTime+1000) {
    //   Mouse.move(1,0);
    //   delay(5);
    // }
    
    // Serial.println("Scroll Down");
    // Mouse.move(0,0,-1);

    // Serial.println("Left click");
    // Mouse.click(MOUSE_LEFT);
    // delay(500);

    // Serial.println("Right click");
    // Mouse.click(MOUSE_RIGHT);
    // delay(500);

    // Serial.println("Scroll wheel click");
    // Mouse.click(MOUSE_MIDDLE);
    // delay(500);

    // Serial.println("Back button click");
    // Mouse.click(MOUSE_BACK);
    // delay(500);

    // Serial.println("Forward button click");
    // Mouse.click(MOUSE_FORWARD);
    // delay(500);

    // Serial.println("Click left+right mouse button at the same time");
    // Mouse.click(MOUSE_LEFT | MOUSE_RIGHT);
    // delay(500);

    // Serial.println("Click left+right mouse button and scroll wheel at the same time");
    // Mouse.click(MOUSE_LEFT | MOUSE_RIGHT | MOUSE_MIDDLE);
    // delay(500);


  // }
  
  // Serial.println("Waiting 2 seconds...");
  // delay(2000);
}

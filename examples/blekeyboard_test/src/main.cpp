#include <Arduino.h>
#include <BleKeyboard.h>


/*
  BleKeyboard(std::string deviceName = "ESP32 Keyboard", std::string deviceManufacturer = "Espressif", uint8_t batteryLevel = 100);
  void begin(void);
  void end(void);
  void sendReport(KeyReport* keys);
  void sendReport(MediaKeyReport* keys);
  size_t press(uint8_t k);
  size_t press(const MediaKeyReport k);
  size_t release(uint8_t k);
  size_t release(const MediaKeyReport k);
  size_t write(uint8_t c);
  size_t write(const MediaKeyReport c);
  size_t write(const uint8_t *buffer, size_t size);
  void releaseAll(void);
  bool isConnected(void);
  void setBatteryLevel(uint8_t level);
  void setName(std::string deviceName);  
  void setDelay(uint32_t ms);

  void set_vendor_id(uint16_t vid);
  void set_product_id(uint16_t pid);
  void set_version(uint16_t version);
protected:
  virtual void onStarted(BLEServer *pServer) { };
  virtual void onConnect(BLEServer* pServer) override;
  virtual void onDisconnect(BLEServer* pServer) override;
  virtual void onWrite(BLECharacteristic* me) override;
*/

BleKeyboard bleKeyboard("tst","cainaio",100);

static bool touch_pad_press_12()
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


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Starting BLE work!");
  bleKeyboard.begin();


}

void loop() {
  // put your main code here, to run repeatedly:
    if(bleKeyboard.isConnected()) {
      if(touch_pad_press_12()){Serial.println("111");

    delay(100);
    }

    }

  Serial.println("Waiting 5 seconds...");
  delay(5000);

}
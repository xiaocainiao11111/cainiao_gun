#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

/*函数注释
  void begin(void);//把对应pin设置为输出
  void show(void);//运用数据
  void setPin(int16_t p);//设置或改变输出pin
  void setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b);//单独设置某一灯珠的数据，n:灯号，r，g，b
  void setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b, uint8_t w);//w:亮度
  void setPixelColor(uint16_t n, uint32_t c);//c:32位颜色值
  void fill(uint32_t c = 0, uint16_t first = 0, uint16_t count = 0);//颜色填充
  void setBrightness(uint8_t);//设置亮度
  void clear(void);//关闭所有灯
  void updateLength(uint16_t n);//改变长度
  void updateType(neoPixelType t);//改变像素模式，比如下面的NEO_GRB + NEO_KHZ800，建议跟clear使用

  //至于更高级的效果基本都是根据这些函数手撸的，更多可参考例程
*/

#define WS2812_PIN          19           //定义选择引脚
#define WS2812_NUM          4            //定义led个数，决定数组长度

Adafruit_NeoPixel strip = Adafruit_NeoPixel(WS2812_NUM, WS2812_PIN, NEO_GRB + NEO_KHZ800);

void colorWipe(uint32_t color, int wait);
void theaterChase(uint32_t color, int wait);
void rainbow(int wait);
void theaterChaseRainbow(int wait);


unsigned long aaa=0;
int r1,r2,r3;
int _millis;
int RGB_time=0;

void mode1(){
  if(RGB_time==1){
    strip.clear();
    strip.setPixelColor(1, 100, 255, 100);
    strip.show();
  }
  if(RGB_time==100){
    strip.clear();
    strip.setPixelColor(2, 100, 255, 100);
    strip.show();
  }
  else if(RGB_time==200){
    strip.clear();
    strip.setPixelColor(3, 100, 255, 100);
    strip.show();
  }
  else if(RGB_time==300){
    strip.clear();
    RGB_time=0;
  }
}

void RGB_mode(bool mode){
  if(mode){

  }
  else{

  }
}

void setup() {
  Serial.begin(115200);
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)把对应pin设置为输出
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(30); // Set BRIGHTNESS to about 1/5 (max = 255)，设置亮度

}


void loop() {
  r1=random(255);
  r2=random(255);
  r3=random(255);
  RGB_time++;
  mode1();
  delay(5);

  // Fill along the length of the strip in various colors...
  // colorWipe(strip.Color(255,   0,   0), 50); // Red
  // colorWipe(strip.Color(  0, 255,   0), 50); // Green
  // colorWipe(strip.Color(  0,   0, 255), 50); // Blue

  // // Do a theater marquee effect in various colors...
  // theaterChase(strip.Color(127, 127, 127), 50); // White, half brightness
  // theaterChase(strip.Color(127,   0,   0), 50); // Red, half brightness
  // theaterChase(strip.Color(  0,   0, 127), 50); // Blue, half brightness

  // rainbow(0);             // Flowing rainbow cycle along the whole strip
  // theaterChaseRainbow(50); // Rainbow-enhanced theaterChase variant

  // Serial.println(r1);
  // delay(100);
  // Serial.println(r2);
  // delay(100);
  // Serial.println(r3);
  // delay(100);

}


// Some functions of our own for creating animated effects -----------------

// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.
void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}

// Theater-marquee-style chasing lights. Pass in a color (32-bit value,
// a la strip.Color(r,g,b) as mentioned above), and a delay time (in ms)
// between frames.
void theaterChase(uint32_t color, int wait) {
  for(int a=0; a<10; a++) {  // Repeat 10 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in steps of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show(); // Update strip with new contents
      delay(wait);  // Pause for a moment
    }
  }
}

void rainbow(int wait) {
  for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256) {

    strip.rainbow(firstPixelHue);
    // Above line is equivalent to:
    // strip.rainbow(firstPixelHue, 1, 255, 255, true);
    strip.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
  }
}

// Rainbow-enhanced theater marquee. Pass delay time (in ms) between frames.
void theaterChaseRainbow(int wait) {
  int firstPixelHue = 0;     // First pixel starts at red (hue 0)
  for(int a=0; a<30; a++) {  // Repeat 30 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in increments of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        // hue of pixel 'c' is offset by an amount to make one full
        // revolution of the color wheel (range 65536) along the length
        // of the strip (strip.numPixels() steps):
        int      hue   = firstPixelHue + c * 65536L / strip.numPixels();
        uint32_t color = strip.gamma32(strip.ColorHSV(hue)); // hue -> RGB
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show();                // Update strip with new contents
      delay(wait); // Pause for a moment
      firstPixelHue += 65536 / 90; // One cycle of color wheel over 90 frames
    }
  }
}

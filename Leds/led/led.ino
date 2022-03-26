#include <FastLED.h>

#define NUM_LEDS 48
#define NUM_STRIP 2
#define BRIGHTNESS 64
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
#define RIGHT_CLIMBER_PIN 6
#define LEFT_CLIMBER_PIN 7
#define COLOR_BLUE 0
#define COLOR_RED 1
#define RIGHT_CLIMBER 0
#define LEFT_CLIMBER 1
CRGB leds[NUM_STRIP][NUM_LEDS];
int animationSelection=0;

void setup()
{
    FastLED.addLeds<LED_TYPE, RIGHT_CLIMBER_PIN, GRB>(leds[RIGHT_CLIMBER], NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.addLeds<LED_TYPE, LEFT_CLIMBER_PIN, GRB>(leds[LEFT_CLIMBER], NUM_LEDS).setCorrection(TypicalLEDStrip);

    Serial.begin(9600);
}
void loop(){
    switch (animationSelection){
      case 0:
        setAll(0,255,0);
        break;
      case 1:
        Fire(55,120,30, COLOR_BLUE);
        break;
      case 2:
        Fire(55,120,30, COLOR_RED);
        break;
    }
}

void serialEvent() {
  byte ser = Serial.read();
  Serial.write(ser);
  if (ser == 'b') {
    animationSelection = 1;
  } else if (ser == 'r'){
    animationSelection = 2;
  } else if (ser == 'n') {
    animationSelection = 0;
  }
}

void CylonSequence() {
  for(int i=0;i<=1;i++){
    if (i==0) {
      CylonBounceReverse(0xff,0, 0, 6, 0, 0, RIGHT_CLIMBER);
      CylonBounceReverse(0,0, 0xff, 6, 0, 0, LEFT_CLIMBER);
    } else {
      CylonBounceReverse(0,0, 0xff, 6, 0, 0, RIGHT_CLIMBER);
      CylonBounceReverse(0xff,0, 0, 6, 0, 0, LEFT_CLIMBER);
    }
  }
}

void setPixel(int Pixel, byte red, byte green, byte blue) {
  for (int i = 0; i<2; i++) {
   setPixelStrip(i,Pixel,red,green,blue);
  }
}

void setPixelStrip(int strip, int Pixel, byte red, byte green, byte blue) {
   leds[strip][Pixel].r = red;
   leds[strip][Pixel].g = green;
   leds[strip][Pixel].b = blue;
}

void setAll(byte red, byte green, byte blue) {
  for(int i = 0; i < NUM_LEDS; i++ ) {
    setPixel(i, red, green, blue);
  }
  showStrip();
}

void showStrip() {
   FastLED.show();
}

void Fire(int Cooling, int Sparking, int SpeedDelay, bool color) {
  static byte heat[NUM_LEDS];
  int cooldown;
 
  // Step 1.  Cool down every cell a little
  for( int i = 0; i < NUM_LEDS; i++) {
    cooldown = random(0, ((Cooling * 10) / NUM_LEDS) + 2);
   
    if(cooldown>heat[i]) {
      heat[i]=0;
    } else {
      heat[i]=heat[i]-cooldown;
    }
  }
 
  // Step 2.  Heat from each cell drifts 'up' and diffuses a little
  for( int k= NUM_LEDS - 1; k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
  }
   
  // Step 3.  Randomly ignite new 'sparks' near the bottom
  if( random(255) < Sparking ) {
    int y = random(7);
    heat[y] = heat[y] + random(160,255);
    //heat[y] = random(160,255);
  }

  // Step 4.  Convert heat to LED colors
  for( int j = 0; j < NUM_LEDS; j++) {
    setPixelHeatColor(j, heat[j], color);
  }

  showStrip();
  delay(SpeedDelay);
}

void setPixelHeatColor (int Pixel, byte temperature, bool color) {
  // Scale 'heat' down from 0-255 to 0-191
  byte t192 = round((temperature/255.0)*191);
 
  // calculate ramp up from
  byte heatramp = t192 & 0x3F; // 0..63
  heatramp <<= 2; // scale up to 0..252
 
  // figure out which third of the spectrum we're in:
  if (color == COLOR_BLUE) {
    if( t192 > 0x80) {                     // hottest
      setPixel(Pixel, heatramp, 255, 255);
    } else if( t192 > 0x40 ) {             // middle
      setPixel(Pixel, 0, heatramp, 255);
    } else {                               // coolest
      setPixel(Pixel, 0, 0, heatramp);
    }
  } else {
    if( t192 > 0x80) {                     // hottest
      setPixel(Pixel, 255, 255, heatramp);
    } else if( t192 > 0x40 ) {             // middle
      setPixel(Pixel, 255, heatramp, 0);
    } else {                               // coolest
      setPixel(Pixel, heatramp, 0, 0);
    }
  }
}

void CylonBounceReverse(byte red, byte green, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay, int strip){
  for(int i = NUM_LEDS-EyeSize; i > 0; i--) {
    setAll(0,0,0);
    setPixelStrip(strip,i, red/10, green/10, blue/10);
    for(int j = 1; j <= EyeSize; j++) {
      setPixelStrip(strip,i+j, blue,  green, red);
    }
    setPixelStrip(strip,i+EyeSize+1, red/10, green/10, blue/10);
    showStrip();
    delay(SpeedDelay);
  }
  delay(ReturnDelay);
  for(int i = 0; i < NUM_LEDS-EyeSize; i++) {
    setAll(0,0,0);
    setPixelStrip(strip,i, red/10, green/10, blue/10);
    for(int j = 1; j <= EyeSize; j++) {
      setPixelStrip(strip,i+j, blue, green, red);
    }
    setPixelStrip(strip,i+EyeSize+1, red/10, green/10, blue/10);
    showStrip();
    delay(SpeedDelay);
  }
}

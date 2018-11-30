#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

#define NeoPIN 9 // default is pin 10
#define PIXELS_PER_RING 24
#define DEFAULT_EYE_BRIGHTNESS 64 //127 // med brightness
const uint32_t BLACK = 0;

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel strip = Adafruit_NeoPixel((PIXELS_PER_RING * 2), NeoPIN, NEO_GRB + NEO_KHZ800);

typedef struct 
 {
     uint8_t r;
     uint8_t g;
     uint8_t b;
 } eyeColor_t;

eyeColor_t eyeColor = {0, 0, 0};

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.

////////////////////////////////////////////////////////////////////////////////
void setup() {
  // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
#if defined (__AVR_ATtiny85__)
  if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
#endif
  // End of trinket special code

  pinMode(LED_BUILTIN, OUTPUT); // heartbeat LED
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  randomSeed(analogRead(0));

  eyeColor = {0, 0, DEFAULT_EYE_BRIGHTNESS}; // default to blue
  // eyeColor = {0,  DEFAULT_EYE_BRIGHTNESS, 0}; // default to green
  // eyeColor = {0,  DEFAULT_EYE_BRIGHTNESS, DEFAULT_EYE_BRIGHTNESS}; // aqua
  // eyeColor = {DEFAULT_EYE_BRIGHTNESS, DEFAULT_EYE_BRIGHTNESS, DEFAULT_EYE_BRIGHTNESS}; // default to white

  // indicate startup
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on
    delay(300);
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off
    delay(300);
  }

  EyesOn(strip.Color(eyeColor.r, eyeColor.g, eyeColor.b) );
}


////////////////////////////////////////////////////////////////////////////////
void loop() {

  EyesBlink(strip.Color(eyeColor.r, eyeColor.g, eyeColor.b), 5); // Blue, delay

  // Blink at random times
  uint16_t randTime = random(1000, 6000);
  delay(randTime);
}


////////////////////////////////////////////////////////////////////////////////
// Subroutines

// EYES ON
// Quickly fill all the dots with a color.  Is there a better function for this?
void EyesOn(uint32_t c) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
  }
  strip.show(); // move to end of loop?
}


// EYES OFF
// Quickly fill all the dots with a color.  Is there a better function for this?
void EyesOff() {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0));
  }
  strip.show(); // move to end of loop?
}

// WAKE UP
void WakeUp(uint8_t wait) {

  for (uint16_t i = 0; i < 255; i++) {
    uint16_t LeftEyeRightSide = i;
    uint16_t LeftEyeLeftSide = (PIXELS_PER_RING - 1) - i;
    uint16_t RightEyeRightSide = PIXELS_PER_RING + i;
    uint16_t RightEyeLeftSide = ((PIXELS_PER_RING * 2) - 1) - i;

    strip.setPixelColor(LeftEyeRightSide, BLACK);
    strip.setPixelColor(LeftEyeLeftSide, BLACK );
    strip.setPixelColor(RightEyeRightSide, BLACK);
    strip.setPixelColor(RightEyeLeftSide, BLACK );
    strip.show();
    delay(wait);
  }
}

////////////////////////////////////////////////////////////
// BLINK EYES!
// Assumes eyes are on.  Blink Off then back On
// 24 LEDs per ring, 12 each side, 2 rings
const int LeftRingStart = 0;
const int RightRingStart = PIXELS_PER_RING;
const int HALF_RING = PIXELS_PER_RING / 2;

void EyesBlink(uint32_t c, uint8_t wait) {

  // Turn LEDs Off
  // for(int16_t i=(HALF_RING-1); i>=0; i--) {
  for (uint16_t i = 0; i < HALF_RING; i++) {
    uint16_t LeftEyeRightSide = i;
    uint16_t LeftEyeLeftSide = (PIXELS_PER_RING - 1) - i;
    uint16_t RightEyeRightSide = PIXELS_PER_RING + i;
    uint16_t RightEyeLeftSide = ((PIXELS_PER_RING * 2) - 1) - i;

    strip.setPixelColor(LeftEyeRightSide, BLACK);
    strip.setPixelColor(LeftEyeLeftSide, BLACK );
    strip.setPixelColor(RightEyeRightSide, BLACK);
    strip.setPixelColor(RightEyeLeftSide, BLACK );
    strip.show();
    delay(wait);
  }
  delay(100);

  // Turn LEDs Back ON
  for (int16_t i = (HALF_RING - 1); i >= 0; i--) {
    //  for(uint16_t i=0; i<HALF_RING; i++) {
    uint16_t LeftEyeRightSide = i;
    uint16_t LeftEyeLeftSide = (PIXELS_PER_RING - 1) - i;
    uint16_t RightEyeRightSide = PIXELS_PER_RING + i;
    uint16_t RightEyeLeftSide = ((PIXELS_PER_RING * 2) - 1) - i;

    strip.setPixelColor(LeftEyeRightSide, c);
    strip.setPixelColor(LeftEyeLeftSide, c );
    strip.setPixelColor(RightEyeRightSide, c);
    strip.setPixelColor(RightEyeLeftSide, c );
    strip.show();
    delay(wait);
  }
}


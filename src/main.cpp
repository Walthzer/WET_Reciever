#include <Arduino.h>

#define DEBUG

#define ARM_MATH_CM7
#include <arm_math.h>
#include <VGA_t4.h>

static VGA_T4 vga;
static int fb_width, fb_height;

//Parameters
#define SCREEN_W 640 // Set according to line 21!
#define SCREEN_H 480 // .
#define MAX_FFT 200
#define BOX_HEIGHTS 176

//UI DEFINES
#define INSET 31
#define BLUE       VGA_RGB(0, 0, 170)
#define GREEN      VGA_RGB(0, 191, 0)
#define WHITE      VGA_RGB(255, 255, 255)
#define GREY       VGA_RGB(128, 128, 128)
#define BLACK      VGA_RGB(0, 0, 0)
#define LIGHT_BLUE VGA_RGB(0, 136, 255)

#define UIBkg BLUE
#define UIFrg GREEN

const float PROGMEM SPECTRUM_MIN_DB = 30.0;
const float PROGMEM SPECTRUM_MAX_DB = 60.0;

const int PROGMEM FFT_SIZE  = 256;
const int PROGMEM SAMPLE_RATE_HZ = 9387;

const int PROGMEM AUDIO_INPUT_PIN = 14;        // Input ADC pin for audio data.
const int PROGMEM ANALOG_READ_RESOLUTION = 10; // Bits of resolution for the ADC.
const int PROGMEM ANALOG_READ_AVERAGING = 16;  // Number of samples to average with each ADC reading

const int PROGMEM FFT_WINDOW[4] = {31, 31, FFT_SIZE, MAX_FFT};
const int PROGMEM CLK_WINDOW[4] = {(SCREEN_W / 2) + 47, (SCREEN_H / 8), FFT_SIZE + 1 - INSET, (SCREEN_H / 8)};
int BIN_WINDOW[6] = {31, (SCREEN_H / 2) + 31, FFT_SIZE, BOX_HEIGHTS + 1, 0, 0};
int UTF_WINDOW[6] = {(SCREEN_W / 2) + 31, (SCREEN_H / 2) + 31, FFT_SIZE, BOX_HEIGHTS + 1, 0, 0};

int Spectogram[MAX_FFT][FFT_SIZE/2] = {0};
int Roll_In_Pl[FFT_SIZE/2] = {0};

//CLOCK
const int TRIGGER_PIN = 22;
char clock[10] = "00:00:000";
volatile bool clock_on = false;
elapsedMillis ms;

//Output Boxes
#define BINARY 0
#define TEXT 1

#define CHAR_W 8
#define CHAR_H 9
#define CHAR_OFFSET 3

char BIN_BUF[2];
int BIN_WINDOW_STAT[2] = {0, 0}; // #char on current line, #current line
int BIN_TEXT_STAT[2] = {0, 0}; // #char on current line, #current line

//FFT
IntervalTimer samplingTimer;
float samples[FFT_SIZE*2];
float magnitudes[FFT_SIZE];
int sampleCounter = 0;

//440 Detection
const int PROGMEM TARGET_BIN = 12;

 
////////////////////////////////////////////////////////////////////////////////
// SAMPLING FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

void samplingCallback() {
  // Read from the ADC and store the sample data
  samples[sampleCounter] = (float32_t)analogRead(AUDIO_INPUT_PIN);
  // Complex FFT functions require a coefficient for the imaginary part of the input.
  // Since we only have real data, set this coefficient to zero.
  samples[sampleCounter+1] = 0.0;
  // Update sample buffer position and stop after the buffer is filled
  sampleCounter += 2;
  if (sampleCounter >= FFT_SIZE*2) {
    samplingTimer.end();
  }
}

void samplingBegin() {
  // Reset sample buffer position and start callback at necessary rate.
  sampleCounter = 0;
  samplingTimer.begin(samplingCallback, 1000000/SAMPLE_RATE_HZ);
}

boolean samplingIsDone() {
  return sampleCounter >= FFT_SIZE*2;
}

vga_pixel pixelHSVtoRGBColor(float hue, float saturation, float value) {
  // Implemented from algorithm at http://en.wikipedia.org/wiki/HSL_and_HSV#From_HSV
  float chroma = value * saturation;
  float h1 = float(hue)/60.0;
  float x = chroma*(1.0-fabs(fmod(h1, 2.0)-1.0));
  float r = 0;
  float g = 0;
  float b = 0;
  if (h1 < 1.0) {
    r = chroma;
    g = x;
  }
  else if (h1 < 2.0) {
    r = x;
    g = chroma;
  }
  else if (h1 < 3.0) {
    g = chroma;
    b = x;
  }
  else if (h1 < 4.0) {
    g = x;
    b = chroma;
  }
  else if (h1 < 5.0) {
    r = x;
    b = chroma;
  }
  else // h1 <= 6.0
  {
    r = chroma;
    b = x;
  }
  float m = value - chroma;
  r += m;
  g += m;
  b += m;
  return VGA_RGB(int(255*r), int(255*g), int(255*b));
}

vga_pixel magnitude_rgb (float mag) {
    mag = 20.0*log10(mag);
    // Scale the intensity and clamp between 0 and 1.0.
    mag -= SPECTRUM_MIN_DB;
    mag = mag < 0.0 ? 0.0 : mag;
    mag /= (SPECTRUM_MAX_DB-SPECTRUM_MIN_DB);
    mag = mag > 1.0 ? 1.0 : mag;
    return pixelHSVtoRGBColor(360*mag, 1.0, mag);
}

void scroll_spectrum() {
    for(int i = 0; i < MAX_FFT; i++) {
        for(int k = 0; k < FFT_SIZE/2; k++) {
            int K_VAL = Spectogram[i][k];
            Spectogram[i][k] = Roll_In_Pl[k];
            Roll_In_Pl[k] = K_VAL;
        }
    }
}

void roll_in() {
    for(int k = 1; k < FFT_SIZE/2; k++) {
        Roll_In_Pl[k] = magnitude_rgb(magnitudes[k]);
    }
}

void drawSpectrogram() {
    for(int i = 0; i < MAX_FFT; i++) {
        for(int k = 0; k < FFT_SIZE/2; k++) {
            vga.drawPixel(k + FFT_WINDOW[0] + 1, i + FFT_WINDOW[1] + 1, Spectogram[i][k]);
        }
        vga.drawPixel(5 + 140 + FFT_WINDOW[0] + 1, i + FFT_WINDOW[1] + 1, Spectogram[i][12]);
    }
}

void enable_clock() {
    ms = 0;
    clock_on = true;
    detachInterrupt(digitalPinToInterrupt(TRIGGER_PIN));
}

void update_clock(int ms) {
    int msl = ms;

    int m = msl / (1000 * 60);
    msl -= m * (1000 * 60);

    int s = msl / 1000;
    msl -= s * 1000;

    snprintf(clock, 10, "%i:%i:%i", m, s, msl);
}

void window_print(const char* value, int* WINDOW) {
    vga.drawText(WINDOW[0] + CHAR_OFFSET + (CHAR_W * WINDOW[4]), WINDOW[1] + CHAR_OFFSET + (CHAR_H * WINDOW[5]), value, UIFrg, BLACK, false);
}

void detect_440() {

}

void setup() {

      // Set up serial port.
    Serial.begin(38400);
    
    samplingTimer.priority(255);

    //Setup Clock Trigger.
    pinMode(TRIGGER_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(TRIGGER_PIN), enable_clock, RISING);

    // Set up ADC and audio input.
    pinMode(AUDIO_INPUT_PIN, INPUT);
    analogReadResolution(ANALOG_READ_RESOLUTION);
    analogReadAveraging(ANALOG_READ_AVERAGING);

    vga_error_t err = vga.begin(VGA_MODE_640x480);
    if(err != 0)
    {
        Serial.println("fatal error");
        while(1);
    }
  
    vga.clear(UIBkg);
    vga.get_frame_buffer_size(&fb_width, &fb_height);

    //Setup Partion Rectangels

    vga.drawText(55, FFT_WINDOW[1] - 20, "Spectogram", UIFrg, UIBkg, true); //FFT
    vga.drawRect(FFT_WINDOW[0], FFT_WINDOW[1], FFT_WINDOW[2]/2 + 2, FFT_WINDOW[3] + 2, UIFrg); //FFT

    vga.drawText(5 + 195, FFT_WINDOW[1] - 20, "440hz", UIFrg, UIBkg, true); //FFT
    vga.drawRect(5 + FFT_WINDOW[2]/2 + FFT_WINDOW[0] + 1, FFT_WINDOW[1], FFT_WINDOW[2]/2 + 1 - 5, FFT_WINDOW[3] + 2, UIFrg); //FFT
    vga.drawRect(5 + FFT_WINDOW[2]/2 + FFT_WINDOW[0] + 2, FFT_WINDOW[1] + 1, FFT_WINDOW[2]/2 - 1 - 5, FFT_WINDOW[3], BLACK); //FFT

    vga.drawText((SCREEN_W / 2) + 140, CLK_WINDOW[1] - 20, "Timer", UIFrg, UIBkg, true); //Clock
    vga.drawRect(CLK_WINDOW[0], CLK_WINDOW[1], CLK_WINDOW[2] + 2, CLK_WINDOW[3] + 2, UIFrg); //Clock
    vga.drawRect(CLK_WINDOW[0] + 1, CLK_WINDOW[1] + 1, CLK_WINDOW[2], CLK_WINDOW[3], BLACK); //Clock
    vga.drawText(CLK_WINDOW[2]/2 + CLK_WINDOW[0] - 30,CLK_WINDOW[3]/2 + CLK_WINDOW[1] - 7, "0:0:000", UIFrg, BLACK, true); //Clock-TEXT

    vga.drawText(130, BIN_WINDOW[1] - 20, "Binairy", UIFrg, UIBkg, true); //Binary
    vga.drawRect(BIN_WINDOW[0], BIN_WINDOW[1], BIN_WINDOW[2] + 4, BIN_WINDOW[3] + 5, UIFrg); //Binary
    vga.drawRect(BIN_WINDOW[0] + 1, BIN_WINDOW[1] + 1, BIN_WINDOW[2] + 2, BIN_WINDOW[3] + 3, BLACK); //Binary

    vga.drawText((SCREEN_W / 2) + 130, UTF_WINDOW[1] - 20, "Text", UIFrg, UIBkg, true); //UTF
    vga.drawRect(UTF_WINDOW[0], UTF_WINDOW[1], UTF_WINDOW[2] + 4, UTF_WINDOW[3] + 5, UIFrg); //UTF
    vga.drawRect(UTF_WINDOW[0] + 1, UTF_WINDOW[1] + 1, UTF_WINDOW[2] + 2, UTF_WINDOW[3] + 3, BLACK); //UTF

    window_print("125", BIN_WINDOW);
    window_print("125", UTF_WINDOW);

    // Begin sampling audio
    samplingBegin();
}

void loop() {

    if (clock_on) {
        update_clock(ms);
        //vga.waitSync();
        vga.drawText(CLK_WINDOW[2]/2 + CLK_WINDOW[0] - 30,CLK_WINDOW[3]/2 + CLK_WINDOW[1] - 7, clock, UIFrg, BLACK, true); //Clock-TEXT
    }

    if (samplingIsDone()) {
        // Run FFT on sample data.
        arm_cfft_radix4_instance_f32 fft_inst;
        arm_cfft_radix4_init_f32(&fft_inst, FFT_SIZE, 0, 1);
        arm_cfft_radix4_f32(&fft_inst, samples);
        // Calculate magnitude of complex numbers output by the FFT.
        arm_cmplx_mag_f32(samples, magnitudes, FFT_SIZE);

        roll_in();
        scroll_spectrum();

        // Restart audio sampling.
        samplingBegin();
    }

    vga.waitSync();
    drawSpectrogram();
}
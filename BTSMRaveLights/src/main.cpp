#include <Arduino.h>
#include "fix_fft.h"
#include <driver/i2s.h>
#include "FastLED.h"

#define NUM_SAMPLES 256
#define FFT_SIZE 512
#define POW 8

#define DIVIDOR 512

// you shouldn't need to change these settings
#define SAMPLE_BUFFER_SIZE NUM_SAMPLES
#define SAMPLE_RATE 24000
// most microphones will probably default to left channel but you may need to tie the L/R pin low
#define I2S_MIC_CHANNEL I2S_CHANNEL_FMT_ONLY_LEFT
// either wire your microphone to the same pins or change these to match your wiring
#define I2S_MIC_SERIAL_CLOCK GPIO_NUM_35
#define I2S_MIC_LEFT_RIGHT_CLOCK GPIO_NUM_36
#define I2S_MIC_SERIAL_DATA GPIO_NUM_37
#define I2S_MIC_SERIAL_DATA_CS GPIO_NUM_7

SemaphoreHandle_t fft_mic_data_mtx = NULL;
char mic_data_fft_copy[FFT_SIZE], im[FFT_SIZE];  // FFT Array Variables
char fft_inplace_buffer[FFT_SIZE];
char data_out[FFT_SIZE];

SemaphoreHandle_t fft_animation_copy_mtx = NULL;
uint8_t fft_data_animation_copy[FFT_SIZE];
#define LED_MATRIX_GPIO GPIO_NUM_6

// Which pin on the Arduino is connected to the NeoPixels?
#define PIN        6 // On Trinket or Gemma, suggest changing this to 1

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 144 // Popular NeoPixel ring size


void led_matrix_thread(void *parameters);
void fft_thread(void *parameters);
void mic_thread(void *parameters);

// don't mess around with this
i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 1024,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0};

// and don't mess around with this
i2s_pin_config_t i2s_mic_pins = {
    .bck_io_num = I2S_MIC_SERIAL_CLOCK,
    .ws_io_num = I2S_MIC_LEFT_RIGHT_CLOCK,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_MIC_SERIAL_DATA};

int32_t raw_samples[SAMPLE_BUFFER_SIZE];

CRGB leds[NUMPIXELS];


void setup() {
  // we need serial output for the plotter
  Serial.begin();
  // start up the I2S peripheral

  pinMode(I2S_MIC_SERIAL_DATA_CS, OUTPUT);
  digitalWrite(I2S_MIC_SERIAL_DATA_CS, 0);

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &i2s_mic_pins);

  Serial.println("Ready to print stuff!");

  fft_mic_data_mtx = xSemaphoreCreateMutex();
  fft_animation_copy_mtx = xSemaphoreCreateMutex();

  // Start the microphone thread
  xTaskCreate(mic_thread, 
    "Microphone Thread", 
    4096, 
    NULL, 
    0, 
    NULL
  );

  // Start FFT processing Thread
  xTaskCreate(fft_thread, 
    "FFT Processing Thread",
    4096, 
    NULL,
    0, 
    NULL);

  // Start LED Matrix Thread
  xTaskCreate(led_matrix_thread, 
    "led_matrix_thread", 
    4096, 
    NULL, 
    0, 
    NULL);
}

void fft_thread(void *parameters){
  
  for(;;){

    xSemaphoreTake(fft_mic_data_mtx, portMAX_DELAY);
    for(int i = 0; i < NUM_SAMPLES; i++){
      fft_inplace_buffer[i] = mic_data_fft_copy[i];
    }
    xSemaphoreGive(fft_mic_data_mtx);

    fix_fft((int8_t*)fft_inplace_buffer, (int8_t*)im, 8, 0);

    for(int i = 0; i < NUM_SAMPLES; i++){
      data_out[i] = fft_inplace_buffer[i];
    }

    xSemaphoreTake(fft_animation_copy_mtx, portMAX_DELAY);
    for (int i = 0; i < NUM_SAMPLES; i++) // only need N/2 elements of the array
    {
      fft_data_animation_copy[i] = abs(data_out[i]);
    }
    xSemaphoreGive(fft_animation_copy_mtx);
  }
}

void led_matrix_thread(void *parameters){
  FastLED.addLeds<NEOPIXEL, PIN>(leds, NUMPIXELS);  // GRB ordering is assumed
  FastLED.setBrightness(90);
  for(;;){
    xSemaphoreTake(fft_animation_copy_mtx, portMAX_DELAY);
    for(int n = 0; n < NUM_SAMPLES; n++){
      leds[n].setHSV(fft_data_animation_copy[n], 255,  fft_data_animation_copy[n]);
    }
    xSemaphoreGive(fft_animation_copy_mtx);

    FastLED.show();

    delay(30);
  }
}

void mic_thread(void *parameters){

  for(;;){
    // read from the I2S device
    size_t bytes_read = 0;
    // BLOCKING READ to get all data
    i2s_read(I2S_NUM_0, raw_samples, sizeof(int32_t) * SAMPLE_BUFFER_SIZE, &bytes_read, portMAX_DELAY);
    int samples_read = bytes_read / sizeof(int32_t);

    for(int i = 0; i < NUM_SAMPLES; i++){
      raw_samples[i] = raw_samples[i] / DIVIDOR;
      if(raw_samples[i] > 127){
        //raw_samples[i] = 127;
      }

      if(raw_samples[i] < -127){
        //raw_samples[i] = -127;
      }
      Serial.println(raw_samples[i]);
    }

    xSemaphoreTake(fft_mic_data_mtx, portMAX_DELAY);
    for(int i = 0; i < NUM_SAMPLES; i++){
      mic_data_fft_copy[i] = (char)raw_samples[i];
    }
    xSemaphoreGive(fft_mic_data_mtx);

    delay(30);
    }
}

void loop() {
  vTaskDelete(NULL);
}  
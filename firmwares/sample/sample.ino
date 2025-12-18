
// ==============================================================================================================================================
// PROTO-SYNTH V2 - SAMPLE - GC Lab Chile
// ==============================================================================================================================================

// ==============================================================================================================================================
// HARDWARE
// ==============================================================================================================================================
// - Microcontrolador ESP32 DevKit
// - Sensor de movimiento IMU MPU6050 (acelerómetro/giroscopio I2C) |VCC -> 3.3V, GND -> GND, SCL -> PIN 22, SDA -> PIN 21| 
// - 4 Botones con pull-up |1 -> PIN 18, 2 -> PIN 4, 3 -> PIN 15, 4 -> PIN 19|
// - 4 LEDs indicadores |1 -> PIN 23, 2 -> PIN 32, 3 -> PIN 5, 4 -> PIN 2|
// - 4 Potenciómetros analógicos |1 -> PIN 13, 2 -> PIN 14, 3 -> PIN 12, 4 -> PIN 27|
// - Salida MIDI (Serial Hardware, 31250 baudio) |Pin TX0| 
// - Sensor de luz LDR |Pin 26|
// - Jack de audio DAC |Pin 25|
// - Micrófono |Pin 33|
// - 2 Headers para conexiones adicionales |1 -> PIN 34, 2 -> PIN 35|
// ==============================================================================================================================================

// ==============================================================================================================================================
// DESCRIPCIÓN
// ==============================================================================================================================================
// Sampler básico con grabación y reproducción en bucle, control de pitch y efecto de reversa.
// ==============================================================================================================================================

// ==============================================================================================================================================
// FUNCIONAMIENTO
// ==============================================================================================================================================
// CONTROLES DE EXPRESIÓN:
// - Potenciómetro 1: No se usa
// - Potenciómetro 2: No se usa
// - Potenciómetro 3: Pitch 
// - Potenciómetro 4: No se usa
// - Botón 1: Grabar
// - Botón 2: Play/Stop
// - Botón 3: Play Once
// - Botón 4: Efecto reverse
// - LED 1: Grabación
// - LED 2: Reproducción
// - LED 3: No se usa
// - LED 4: No se usa
// - IMU: No se usa
// - LDR: No se usa
// - Micrófono: Grabadora
// - Header 1: No se usa
// - Header 2: No se usa
// - Salida MIDI: No se usa
//
// MODO DE USO:
// 1. Graba pulsando el boton 1
// 2. Reproduce en bucle con el boton 2
// 3. Reproduce una vez con el boton 3
// 4. Activa/desactiva el efecto reverse con el boton 4
// 5. Ajusta el pitch con el potenciómetro 3
//
// INFORMACIÓN DEL CODIGO:
// - El sample que se graba no puede ser muy largo debido a limitaciones de memoria.
// ==============================================================================================================================================

// ==============================================================================================================================================
// COMENTARIOS
// ==============================================================================================================================================
// - Para subir código exitosamente, asegúrate de que el Potenciómetro 3 esté girado al máximo.
// - Los Pines 2,4,12,13,14,15,25,26,27 no van a funcionar si el Bluetooth está activado ya que están conectados al ADC2 del ESP32.
// ==============================================================================================================================================

// ==============================================================================================================================================
// INCLUSIÓN DE LIBRERÍAS
// ==============================================================================================================================================
#include <driver/adc.h>
#include <driver/dac.h>
#include <Arduino.h>

// ==============================================================================================================================================
// CONFIGURACIÓN DE HARDWARE - PINES
// ==============================================================================================================================================
const int MIC_PIN = 33;          // Micrófono en pin 33
const int AUDIO_OUT_PIN = 25;    // Salida DAC
const int REC_BTN = 18;          // Botón grabar (Pin 18)
const int PLAY_STOP_BTN = 4;     // Botón play/stop (Pin 4) 
const int PLAY_ONCE_BTN = 15;    // Botón play once (Pin 15)
const int EFFECT_BTN = 19;       // Botón efecto (Pin 19)
const int PITCH_POT = 12;        // Pot pitch (Pin 12)
const int LED_REC = 23;          // LED grabación
const int LED_PLAY = 32;         // LED reproducción

// ==============================================================================================================================================
// PROGRAMA
// ==============================================================================================================================================
const int MAX_SAMPLES = 24000;
const int SEQUENCE_STEP_MS = 500;
const int NUM_STEPS = 8;

uint16_t audioBuffer[MAX_SAMPLES];
volatile int sampleCount = 0;
volatile int playIndex = 0;
volatile bool hasRecording = false;
volatile bool isPlaying = false;
volatile bool isRunning = false;
bool loopMode = false;
unsigned long lastStepTime = 0;
int currentStep = 0;
bool effectActive = false;

void startPlayback(bool loop) {
  playIndex = 0;
  currentStep = 0;
  lastStepTime = millis();
  isPlaying = true;
  loopMode = loop;
}

void playSample() {
   if (!isPlaying || !hasRecording) return;
   
   unsigned long currentTime = millis();
   if (currentTime - lastStepTime >= SEQUENCE_STEP_MS) {
       lastStepTime = currentTime;
       playIndex = 0;
       
       if (!loopMode) {
           isPlaying = false;
           return;
       }
       
       currentStep = (currentStep + 1) % NUM_STEPS;
   }
   
   int potValue = analogRead(PITCH_POT);
   int playbackDelay = map(potValue, 0, 4095, 5, 250);
   
   if (playIndex < sampleCount) {
       uint16_t sample;
       if (effectActive) {
           sample = audioBuffer[sampleCount - 1 - playIndex]; // Reverse
       } else {
           sample = audioBuffer[playIndex]; // Normal
       }
       
       dac_output_voltage(DAC_CHANNEL_1, sample >> 4);
       delayMicroseconds(playbackDelay);
       playIndex++;
   }
}

void setup() {
  // Deshabilitar logging para evitar interferencias
  esp_log_level_set("*", ESP_LOG_NONE);
  
  pinMode(LED_REC, OUTPUT);
  pinMode(LED_PLAY, OUTPUT);
  pinMode(REC_BTN, INPUT_PULLUP);
  pinMode(PLAY_STOP_BTN, INPUT_PULLUP);
  pinMode(PLAY_ONCE_BTN, INPUT_PULLUP);
  pinMode(EFFECT_BTN, INPUT_PULLUP);
  
  digitalWrite(LED_PLAY, LOW);
  
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  analogSetWidth(12);
  
  dac_output_enable(DAC_CHANNEL_1);
}

void loop() {
  if (digitalRead(REC_BTN) == LOW) {
      isPlaying = false;
      sampleCount = 0;
      digitalWrite(LED_REC, HIGH);
      
      while(digitalRead(REC_BTN) == LOW && sampleCount < MAX_SAMPLES) {
          audioBuffer[sampleCount++] = analogRead(MIC_PIN);
          delayMicroseconds(62);
      }
      
      digitalWrite(LED_REC, LOW);
      hasRecording = true;
  }

  if (digitalRead(PLAY_STOP_BTN) == LOW) {
      delay(50);
      if (digitalRead(PLAY_STOP_BTN) == LOW) {
          isRunning = !isRunning;
          if (isRunning) {
              startPlayback(true);
          } else {
              isPlaying = false;
          }
      }
  }
  
  if (digitalRead(PLAY_ONCE_BTN) == LOW) {
      startPlayback(false);
  }
  
  if (digitalRead(EFFECT_BTN) == LOW) {
   delay(50);
   if (digitalRead(EFFECT_BTN) == LOW) {
       effectActive = !effectActive;
       while(digitalRead(EFFECT_BTN) == LOW) {
           delay(1);
       }
       delay(100);
   }
}
  
  if (isPlaying) {
      playSample();
      digitalWrite(LED_PLAY, HIGH);
  } else {
      digitalWrite(LED_PLAY, LOW);
  }
}
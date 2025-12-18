
// ==============================================================================================================================================
// PROTO-SYNTH V2 - ARPEGGIATOR ENGINE CON SATURACIÓN - GC Lab Chile
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
// Arpegeador de trance optimizado con saturación y control de filtro LPF mediante IMU.
// ==============================================================================================================================================

// ==============================================================================================================================================
// FUNCIONAMIENTO
// ==============================================================================================================================================
// CONTROLES DE EXPRESIÓN:
// - Potenciómetro 1: TEMPO (60-180 BPM)
// - Potenciómetro 2: VOLUMEN (0-100%)
// - Potenciómetro 3: SATURACIÓN/DRIVE (1x-10x) 
// - Potenciómetro 4: PITCH GLOBAL (-60 a +12 semitonos)
// - Botón 1: Grabar/Parar grabación
// - Botón 2: Play/Stop secuenciador
// - Botón 3: Cambiar patrón de trance (16 patrones)
// - Botón 4: Cambiar longitud de secuencia (4/8/16 steps)
// - LED 1: Indicador de sequencia
// - LED 2: Indicador de sequencia
// - LED 3: Indicador de sequencia
// - LED 4: Indicador de sequencia
// - IMU: Control de filtro LPF
// - LDR: No se usa
// - Micrófono: No se usa
// - Header 1: No se usa
// - Header 2: No se usa
// - Salida MIDI: No se usa
//
// MODO DE USO:
// 1. Grabar un sample pulsando el Botón 1.
// 2. Ajustar los controles de expresión según preferencia.
// 3. Pulsar el Botón 2 para iniciar el arpegiador.
// 4. Cambiar patrón y longitud con los Botones 3 y 4 según necesidad.
//
// INFORMACIÓN DEL CÓDIGO:
// - ESCALA FIJA: Menor Natural (octava normal)
// - Delay post-grabación configurable (POST_RECORDING_DELAY = 800ms)
// - Evita ruido del botón en el sample grabado
// - Auto-start después del delay
// - Rango ajustable: 50ms, 100ms, 150ms, 200ms o más según necesidad
// ==============================================================================================================================================

// ==============================================================================================================================================
// COMENTARIOS
// ==============================================================================================================================================
// - Los potenciometros están invertidos.
// - Para subir código exitosamente, asegúrate de que el Potenciómetro 3 esté girado al máximo.
// - Los Pines 2,4,12,13,14,15,25,26,27 no van a funcionar si el Bluetooth está activado ya que están conectados al ADC2 del ESP32.
// ==============================================================================================================================================

// ==============================================================================================================================================
// INCLUSIÓN DE LIBRERÍAS
// ==============================================================================================================================================
#include <driver/adc.h>
#include <driver/dac.h>
#include <Arduino.h>
#include <Wire.h>

// MPU6050 I2C
#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B

// ==============================================================================================================================================
// CONFIGURACIÓN DE HARDWARE - PINES
// ==============================================================================================================================================
// Pines Hardware
const int MIC_PIN = 33;
const int REC_BTN = 18;
const int PLAY_BTN = 4;
const int PATTERN_BTN = 15;
const int LENGTH_BTN = 19;
const int TEMPO_POT = 13;
const int VOLUME_POT = 14;
const int SATURATION_POT = 12;   // Control de saturación
const int PITCH_POT = 27;
const int LED1 = 23;
const int LED2 = 32;
const int LED3 = 5;
const int LED4 = 2;

// ==============================================================================================================================================
// PROGRAMA
// ==============================================================================================================================================
// Configuración de audio
const int SAMPLE_RATE = 22000;
const int MAX_SAMPLES = 33000;
const int BASE_FREQ = 55;

// Variables del filtro LPF
float filter_x1 = 0, filter_x2 = 0;
float filter_y1 = 0, filter_y2 = 0;
float filter_a0, filter_a1, filter_a2;
float filter_b1, filter_b2;

// Variables del IMU
int16_t accelX = 0, accelY = 0;
float filtered_x = 0.0, filtered_y = 0.0;
const float IMU_FILTER_ALPHA = 0.1;
unsigned long lastIMURead = 0;
const int IMU_READ_INTERVAL = 10;

// Escala fija: Menor Natural (octava normal)
const int DARK_SCALE[] = {0, 2, 3, 5, 7, 8, 10};
const int SCALE_SIZE = 7;

// Variables de control
float globalVolume = 1.0;
float globalSaturation = 1.0;

// Variable para delay post-grabación (evitar ruido del botón)
const unsigned long POST_RECORDING_DELAY = 800;  // 100ms por defecto, ajustable hasta 200ms o más
unsigned long recordingFinishTime = 0;
bool waitingToStart = false;

// Patrones de trance (16 patrones auténticos)
const int trancePatterns[][16] = {
  {1, 0, 3, 0, 5, 0, 3, 0, 1, 0, 3, 0, 5, 0, 7, 0}, // Trance básico
  {1, 0, 0, 0, 5, 0, 0, 0, 3, 0, 0, 0, 7, 0, 0, 0}, // Espaciado
  {1, 3, 5, 7, 5, 3, 1, 0, 1, 3, 5, 7, 5, 3, 1, 0}, // Ascendente
  {7, 5, 3, 1, 3, 5, 7, 0, 7, 5, 3, 1, 3, 5, 7, 0}, // Descendente
  {1, 1, 5, 5, 3, 3, 7, 0, 1, 1, 5, 5, 3, 3, 7, 0}, // Repetitivo
  {1, 0, 5, 3, 0, 7, 1, 0, 5, 0, 3, 7, 0, 1, 5, 0}, // Sincopado
  {1, 3, 0, 5, 7, 0, 3, 1, 5, 7, 0, 1, 3, 0, 5, 7}, // Complejo
  {1, 0, 0, 5, 0, 0, 1, 5, 0, 3, 0, 7, 0, 1, 0, 0}, // Breakbeat
  {7, 7, 5, 5, 3, 3, 1, 1, 7, 7, 5, 5, 3, 3, 1, 1}, // Dobles
  {1, 5, 1, 7, 1, 3, 1, 5, 1, 7, 1, 3, 1, 5, 1, 0}, // Bajo constante
  {0, 1, 0, 3, 0, 5, 0, 7, 0, 5, 0, 3, 0, 1, 0, 0}, // Off-beat
  {1, 3, 5, 1, 7, 5, 3, 7, 1, 3, 5, 1, 7, 5, 3, 0}, // Arpegio
  {5, 0, 5, 0, 1, 0, 1, 0, 3, 0, 3, 0, 7, 0, 7, 0}, // Alternado
  {1, 7, 1, 5, 1, 3, 1, 7, 5, 7, 5, 3, 5, 1, 5, 0}, // Saltos
  {3, 3, 3, 5, 5, 5, 1, 0, 7, 7, 7, 5, 5, 5, 3, 0}, // Grupos
  {1, 0, 7, 0, 5, 0, 3, 0, 5, 0, 7, 0, 1, 0, 0, 0}  // Clásico
};
const int NUM_PATTERNS = 16;

// Variables del sampler
uint16_t* audioBuffer;
volatile int totalSamples = 0;
volatile bool hasRecording = false;
volatile bool isRecording = false;

// Variables del secuenciador
volatile bool isPlaying = false;
int currentPattern = 0;
int baseTempo = 120;
float globalPitchBend = 0.0;
int currentStep = 0;
int sequenceLength = 16;
unsigned long lastStepTime = 0;
unsigned long stepInterval = 125;

// Variables para reproducción de audio no bloqueante
struct AudioPlayback {
  bool isActive;
  float samplePosition;
  float pitchRatio;
  unsigned long startTime;
  unsigned long duration;
  float volume;
};
AudioPlayback currentPlayback = {false, 0.0, 1.0, 0, 0, 1.0};

// Variables para botones
bool lastRecState = HIGH;
bool lastPlayState = HIGH;
bool lastPatternState = HIGH;
bool lastLengthState = HIGH;
unsigned long lastRecPress = 0;
unsigned long lastPlayPress = 0;
unsigned long lastPatternPress = 0;
unsigned long lastLengthPress = 0;
const unsigned long debounceDelay = 200;

// Variables para LEDs
bool isNotePlaying = false;

// FUNCIONES DEL IMU
void initMPU6050() {
  Wire.begin();
  Wire.setClock(400000);
  
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission(true);
  
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true);
  
  delay(100);
}

void readMPU6050() {
  unsigned long currentTime = millis();
  if (currentTime - lastIMURead < IMU_READ_INTERVAL) return;
  lastIMURead = currentTime;
  
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 4, true);
  
  if (Wire.available() >= 4) {
    accelX = (Wire.read() << 8) | Wire.read();
    accelY = (Wire.read() << 8) | Wire.read();
    
    float imu_accel_x = accelX / 16384.0;
    float imu_accel_y = accelY / 16384.0;
    
    filtered_x = filtered_x * (1.0 - IMU_FILTER_ALPHA) + imu_accel_x * IMU_FILTER_ALPHA;
    filtered_y = filtered_y * (1.0 - IMU_FILTER_ALPHA) + imu_accel_y * IMU_FILTER_ALPHA;
  }
}

void calculateLPFCoefficients() {
  // EJE X para frecuencia (igual que Y para resonancia)
  float x_abs = abs(filtered_x);
  x_abs = constrain(x_abs, 0.0, 1.0);
  float cutoff_freq = 50.0 + x_abs * 7950.0;  // 50Hz a 8000Hz
  
  // EJE Y para resonancia
  float y_abs = abs(filtered_y);
  y_abs = constrain(y_abs, 0.0, 1.0);
  float Q = 0.5 + y_abs * 25.0;
  
  float omega = 2.0 * PI * cutoff_freq / SAMPLE_RATE;
  float sin_omega = sin(omega);
  float cos_omega = cos(omega);
  float alpha = sin_omega / (2.0 * Q);
  
  float b0 = (1.0 - cos_omega) / 2.0;
  float b1 = 1.0 - cos_omega;
  float b2 = (1.0 - cos_omega) / 2.0;
  float a0_temp = 1.0 + alpha;
  float a1_temp = -2.0 * cos_omega;
  float a2_temp = 1.0 - alpha;
  
  filter_a0 = b0 / a0_temp;
  filter_a1 = b1 / a0_temp;
  filter_a2 = b2 / a0_temp;
  filter_b1 = a1_temp / a0_temp;
  filter_b2 = a2_temp / a0_temp;
}

float applyLPF(float input) {
  // APLICAR SATURACIÓN ANTES DEL FILTRO (como en tu código de ejemplo)
  float driven_input = input * globalSaturation;
  
  // Saturación tipo tanh para más carácter (copiado de tu ejemplo)
  if (driven_input > 1.5) {
    driven_input = 0.9; // Clip duro
  } else if (driven_input < -1.5) {
    driven_input = -0.9; // Clip duro
  } else if (driven_input > 0.8) {
    driven_input = 0.8 + (driven_input - 0.8) * 0.2; // Saturación suave
  } else if (driven_input < -0.8) {
    driven_input = -0.8 + (driven_input + 0.8) * 0.2; // Saturación suave
  }
  
  // Aplicar filtro LPF
  float output = filter_a0 * driven_input + filter_a1 * filter_x1 + filter_a2 * filter_x2
                 - filter_b1 * filter_y1 - filter_b2 * filter_y2;
  
  filter_x2 = filter_x1;
  filter_x1 = driven_input;
  filter_y2 = filter_y1;
  filter_y1 = output;
  
  return output;
}

void processFilterControl() {
  readMPU6050();
  calculateLPFCoefficients();
}

// FUNCIONES DE AUDIO
float calculateNoteFreq(int scaleIndex) {
  if (scaleIndex < 0 || scaleIndex >= SCALE_SIZE) {
    return BASE_FREQ;
  }
  
  int semitones = DARK_SCALE[scaleIndex];
  
  float totalSemitones = semitones + globalPitchBend;
  return BASE_FREQ * pow(2.0, totalSemitones / 12.0);
}

// SISTEMA DE AUDIO NO BLOQUEANTE
void startSamplePlayback(float targetFreq, unsigned long duration) {
  if (!hasRecording || targetFreq <= 0) return;
  
  currentPlayback.isActive = false;
  
  currentPlayback.isActive = true;
  currentPlayback.samplePosition = 0.0;
  currentPlayback.pitchRatio = targetFreq / BASE_FREQ;
  currentPlayback.startTime = millis();
  currentPlayback.duration = duration;
  currentPlayback.volume = globalVolume;
  
  isNotePlaying = true;
}

void processAudioPlayback() {
  if (!currentPlayback.isActive) {
    if (isNotePlaying) {
      isNotePlaying = false;
      dac_output_voltage(DAC_CHANNEL_1, 128);
    }
    return;
  }
  
  unsigned long currentTime = millis();
  
  if ((currentTime - currentPlayback.startTime) >= currentPlayback.duration ||
      currentPlayback.samplePosition >= totalSamples - 1) {
    currentPlayback.isActive = false;
    isNotePlaying = false;
    dac_output_voltage(DAC_CHANNEL_1, 128);
    return;
  }
  
  int currentSample = (int)currentPlayback.samplePosition;
  float fraction = currentPlayback.samplePosition - currentSample;
  
  uint16_t rawSample;
  if (currentSample < totalSamples - 1) {
    uint16_t sample1 = audioBuffer[currentSample];
    uint16_t sample2 = audioBuffer[currentSample + 1];
    rawSample = sample1 + (uint16_t)(fraction * (sample2 - sample1));
  } else {
    rawSample = audioBuffer[currentSample];
  }
  
  float floatSample = (rawSample - 2048.0) / 2048.0;
  floatSample = applyLPF(floatSample);
  floatSample *= currentPlayback.volume;
  floatSample = constrain(floatSample, -1.0, 1.0);
  uint16_t finalSample = (uint16_t)((floatSample + 1.0) * 2048.0);
  
  if (finalSample > 4095) finalSample = 4095;
  
  dac_output_voltage(DAC_CHANNEL_1, finalSample >> 4);
  currentPlayback.samplePosition += currentPlayback.pitchRatio;
}

// SECUENCIADOR
void playSequencerStep() {
  if (!isPlaying || !hasRecording) return;
  
  unsigned long currentTime = millis();
  
  if (currentTime - lastStepTime >= stepInterval) {
    
    int noteValue = trancePatterns[currentPattern][currentStep];
    
    if (noteValue > 0) {
      int scaleIndex = (noteValue - 1) % SCALE_SIZE;
      
      float noteFreq = calculateNoteFreq(scaleIndex);
      unsigned long noteDuration = stepInterval * 0.9;
      
      startSamplePlayback(noteFreq, noteDuration);
    }
    
    currentStep = (currentStep + 1) % sequenceLength;
    lastStepTime = currentTime;
    updateLEDs();
  }
}

// FUNCIONES DE CONTROL
void readControls() {
  int tempoPot = analogRead(TEMPO_POT);
  int volumePot = analogRead(VOLUME_POT);
  int saturationPot = analogRead(SATURATION_POT);
  int pitchPot = analogRead(PITCH_POT);
  
  baseTempo = map(tempoPot, 0, 4095, 60, 180);
  
  // Control de volumen (0-100%)
  globalVolume = volumePot / 4095.0;
  
  // Control de saturación (1x a 10x) - COMO EN TU EJEMPLO
  float saturation_normalized = saturationPot / 4095.0;
  globalSaturation = 1.0 + saturation_normalized * 9.0; // Drive de 1x a 10x
  
  globalPitchBend = map(pitchPot, 0, 4095, -60, 12);
  
  stepInterval = 60000 / (baseTempo * 4);
}

void updateLEDs() {
  digitalWrite(LED1, isNotePlaying ? HIGH : LOW);
  digitalWrite(LED2, isPlaying ? HIGH : LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, isRecording ? HIGH : LOW);
}

void showRecordingAnimation() {
  static unsigned long lastBlink = 0;
  static bool blinkState = false;
  
  if (millis() - lastBlink > 150) {
    blinkState = !blinkState;
    lastBlink = millis();
    
    digitalWrite(LED1, blinkState);
    digitalWrite(LED2, blinkState);
    digitalWrite(LED3, blinkState);
    digitalWrite(LED4, HIGH);
  }
}

void startRecording() {
  isRecording = true;
  isPlaying = false;
  totalSamples = 0;
  hasRecording = false;
  currentPlayback.isActive = false;
  isNotePlaying = false;
}

void stopRecording() {
  isRecording = false;
  hasRecording = (totalSamples > 0);
  
  // NUEVO: Configurar delay post-grabación
  if (hasRecording) {
    recordingFinishTime = millis();
    waitingToStart = true;
  }
}

void startArpeggiator() {
  if (!hasRecording) return;
  
  // NUEVO: Verificar si aún estamos en delay post-grabación
  if (waitingToStart) {
    unsigned long currentTime = millis();
    if (currentTime - recordingFinishTime < POST_RECORDING_DELAY) {
      return; // Aún en período de espera
    }
    waitingToStart = false; // Ya pasó el delay
  }
  
  isPlaying = true;
  currentStep = 0;
  lastStepTime = millis();
  currentPlayback.isActive = false;
  isNotePlaying = false;
}

void startLedEffect(int effectType) {
  switch (effectType) {
    case 1: // Efecto patrón
      for (int i = 0; i < 4; i++) {
        digitalWrite(LED1, i == 0 ? HIGH : LOW);
        digitalWrite(LED2, i == 1 ? HIGH : LOW);
        digitalWrite(LED3, i == 2 ? HIGH : LOW);
        digitalWrite(LED4, i == 3 ? HIGH : LOW);
        delay(30);
      }
      for (int i = 0; i < 4; i++) {
        digitalWrite(LED1 + i, LOW);
      }
      break;
      
    case 2: // Efecto longitud
      {
        int ledsToShow = (sequenceLength == 4) ? 1 : ((sequenceLength == 8) ? 2 : 4);
        for (int i = 0; i < 4; i++) {
          digitalWrite(LED1 + i, (i < ledsToShow) ? HIGH : LOW);
        }
        delay(300);
        for (int i = 0; i < 4; i++) {
          digitalWrite(LED1 + i, LOW);
        }
      }
      break;
  }
}

void handleButtons() {
  unsigned long currentTime = millis();
  
  // Botón grabar
  bool currentRecState = digitalRead(REC_BTN);
  if (currentRecState == LOW && lastRecState == HIGH && 
      (currentTime - lastRecPress) > debounceDelay) {
    if (!isRecording) {
      startRecording();
    } else {
      stopRecording();
    }
    lastRecPress = currentTime;
  }
  lastRecState = currentRecState;
  
  // Botón play/stop
  bool currentPlayState = digitalRead(PLAY_BTN);
  if (currentPlayState == LOW && lastPlayState == HIGH && 
      (currentTime - lastPlayPress) > debounceDelay) {
    if (!isPlaying) {
      startArpeggiator();
    } else {
      isPlaying = false;
      currentPlayback.isActive = false;
      isNotePlaying = false;
    }
    lastPlayPress = currentTime;
  }
  lastPlayState = currentPlayState;
  
  // Botón patrón
  bool currentPatternState = digitalRead(PATTERN_BTN);
  if (currentPatternState == LOW && lastPatternState == HIGH && 
      (currentTime - lastPatternPress) > debounceDelay) {
    currentPattern = (currentPattern + 1) % NUM_PATTERNS;
    currentStep = 0;
    if (isPlaying) {
      lastStepTime = currentTime;
    }
    startLedEffect(1);
    lastPatternPress = currentTime;
  }
  lastPatternState = currentPatternState;
  
  // Botón longitud
  bool currentLengthState = digitalRead(LENGTH_BTN);
  if (currentLengthState == LOW && lastLengthState == HIGH && 
      (currentTime - lastLengthPress) > debounceDelay) {
    sequenceLength = (sequenceLength == 4) ? 8 : ((sequenceLength == 8) ? 16 : 4);
    
    if (currentStep >= sequenceLength) {
      currentStep = 0;
    }
    startLedEffect(2);
    lastLengthPress = currentTime;
  }
  lastLengthState = currentLengthState;
}

void setup() {
  audioBuffer = (uint16_t*)malloc(MAX_SAMPLES * sizeof(uint16_t));
  if (!audioBuffer) {
    while(1) delay(1000);
  }
  
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  
  pinMode(REC_BTN, INPUT_PULLUP);
  pinMode(PLAY_BTN, INPUT_PULLUP);
  pinMode(PATTERN_BTN, INPUT_PULLUP);
  pinMode(LENGTH_BTN, INPUT_PULLUP);
  
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  analogSetWidth(12);
  dac_output_enable(DAC_CHANNEL_1);
  dac_output_voltage(DAC_CHANNEL_1, 128);
  
  randomSeed(analogRead(0));
  
  initMPU6050();
  
  delay(100);
  readMPU6050();
  calculateLPFCoefficients();
  
  // Test de LEDs
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, HIGH);
    digitalWrite(LED3, HIGH);
    digitalWrite(LED4, HIGH);
    delay(150);
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);
    digitalWrite(LED4, LOW);
    delay(150);
  }
}

void loop() {
  readControls();
  handleButtons();
  processFilterControl();
  
  if (isRecording) {
    showRecordingAnimation();
    
    if (totalSamples < MAX_SAMPLES) {
      uint16_t sample = analogRead(MIC_PIN);
      audioBuffer[totalSamples++] = sample;
      delayMicroseconds(45);
    } else {
      stopRecording();
    }
  } else {
    // NUEVO: Auto-start con delay post-grabación
    if (hasRecording && !isPlaying && waitingToStart) {
      unsigned long currentTime = millis();
      if (currentTime - recordingFinishTime >= POST_RECORDING_DELAY) {
        startArpeggiator(); // Iniciar automáticamente después del delay
      }
    }
    
    if (isPlaying) {
      playSequencerStep();
    }
    
    processAudioPlayback();
    
    if (!isPlaying) {
      updateLEDs();
    }
  }
  
  delayMicroseconds(5);
}
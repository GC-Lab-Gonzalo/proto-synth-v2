
// ==============================================================================================================================================
// PROTO-SYNTH V2 -  Secuenciador de Trance Electrónico - GC Lab Chile
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
// Sequenciador de música trance electrónico controlado por IMU.
// ==============================================================================================================================================

// ==============================================================================================================================================
// FUNCIONAMIENTO
// ==============================================================================================================================================
// CONTROLES DE EXPRESIÓN:
// - Potenciómetro 1: Control de Ataque
// - Potenciómetro 2: Volumen General (0-100%)
// - Potenciómetro 3: Tempo (40-300 BPM)
// - Potenciómetro 4: Decay/Release
// - Botón 1: Play/Stop
// - Botón 2: Cambiar longitud de secuencia
// - Botón 3: Cambiar escala
// - Botón 4: Cambiar patrón
// - LED 1: Indicador de sequencia
// - LED 2: Indicador de sequencia
// - LED 3: Indicador de sequencia
// - LED 4: Indicador de sequencia
// - IMU: Control de filtro LPF
// - LDR: No se usa
// - Micrófono:
// - Header 1: No se usa
// - Header 2: No se usa
// - Salida MIDI: No se usa
//
// MODO DE USO:
// 1. Inicia o detén la reproducción con el botón 1.
// 2. Ajusta el tempo con el potenciómetro 3.
// 3. Cambia la escala musical con el botón 3.
// 4. Cambia el patrón de secuencia con el botón 4.
// 5. Ajusta el volumen general con el potenciómetro 2.
// 6. Ajusta el ataque de las notas con el potenciómetro 1.
// 7. Ajusta el decay/release con el potenciómetro 4.
// 8. Controla el filtro en tiempo real moviendo el IMU.
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
#include "driver/dac.h"
#include "math.h"
#include "Wire.h"

// ==============================================================================================================================================
// CONFIGURACIÓN DE HARDWARE - PINES
// ==============================================================================================================================================
const int ATTACK_POT_PIN = 13;  // Nuevo: Control de ataque
const int VOLUME_POT_PIN = 14;  // Control de volumen general
const int TEMPO_POT_PIN = 12;
const int DECAY_POT_PIN = 27;

// Pines I2C para IMU
const int SDA_PIN = 21;
const int SCL_PIN = 22;

// Dirección del IMU (MPU6050 típicamente 0x68)
const int IMU_ADDRESS = 0x68;

const int SCALE_BTN_PIN = 15;
const int PATTERN_BTN_PIN = 4;
const int PLAY_BTN_PIN = 18;
const int LENGTH_BTN_PIN = 19;

const int LED_PINS[4] = {23, 32, 5, 2};

// ==============================================================================================================================================
// PROGRAMA
// ==============================================================================================================================================
// Configuración de audio
const int SAMPLE_RATE = 8000;
const int MAX_AMPLITUDE = 90;  // Amplitud máxima
float currentVolume = 1.0;     // Variable para volumen actual (0.0-1.0)

// Variables del filtro controladas por IMU
float filter_x1 = 0, filter_x2 = 0;
float filter_y1 = 0, filter_y2 = 0;
float filter_a0, filter_a1, filter_a2;
float filter_b1, filter_b2;

// Variables del IMU
float imu_accel_x = 0.0;
float imu_accel_y = 0.0;
float imu_accel_z = 0.0;
unsigned long lastIMURead = 0;
const unsigned long IMU_READ_INTERVAL = 10; // Leer IMU cada 10ms

// Filtros para suavizar lecturas del IMU
float filtered_x = 0.0;
float filtered_y = 0.0;
const float IMU_FILTER_ALPHA = 0.1; // Factor de suavizado (0.0-1.0)

// Variables del secuenciador
bool isPlaying = false;
unsigned long lastStepTime = 0;
int currentStep = 0;
int sequenceLength = 4;
int currentPattern = 0;
int currentScale = 0;
float baseFreq = 130.81; // C3 - frecuencia base

// Variables para control de envelope
float decayFactor = 0.5;
float attackFactor = 0.1;  // Nueva variable para controlar el ataque

// Variables para eliminar chasquidos entre notas
float lastSample = 128.0;
bool noteActive = false;

// Escalas musicales (intervalos en semitonos)
const int scales[][8] = {
  {0, 2, 4, 5, 7, 9, 11, 12},  // Mayor
  {0, 2, 3, 5, 7, 8, 10, 12},  // Menor natural
  {0, 2, 3, 5, 7, 8, 11, 12},  // Menor armónica
  {0, 1, 3, 4, 6, 8, 10, 12},  // Árabe
  {0, 2, 4, 6, 7, 9, 11, 12},  // Lidia
  {0, 1, 4, 5, 7, 8, 11, 12}   // Frigia
};
const char* scaleNames[] = {"Mayor", "Menor", "Armonica", "Arabe", "Lidia", "Frigia"};
const int numScales = 6;

// Patrones de secuencia (16 patrones diferentes)
const int patterns[][16] = {
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
const int numPatterns = 16;

// Variables para botones (sin librería Bounce)
bool lastScaleState = HIGH;
bool lastPatternState = HIGH;
bool lastPlayState = HIGH;
bool lastLengthState = HIGH;

// Variables para anti-rebote
unsigned long lastScalePress = 0;
unsigned long lastPatternPress = 0;
unsigned long lastPlayPress = 0;
unsigned long lastLengthPress = 0;
const unsigned long debounceDelay = 200;

// Variables para efectos visuales sin delay
unsigned long ledEffectStartTime = 0;
int ledEffectType = 0; // 0=ninguno, 1=escala, 2=patrón, 3=longitud
int ledEffectStep = 0;
bool ledEffectActive = false;

// FUNCIÓN PARA ACTUALIZAR ATAQUE (INVERTIDA)
void updateAttack() {
  int attackPot = analogRead(ATTACK_POT_PIN);
  // Invertir la lectura: 4095 - valor original
  attackPot = 4095 - attackPot;
  
  // Mapear de 0-4095 a rango de ataque
  // Valores pequeños = ataque rápido, valores grandes = ataque lento
  // Rango de 0.01 (muy rápido) a 2.0 (muy lento)
  attackFactor = 0.01 + (attackPot / 4095.0) * 1.99;
}

// FUNCIÓN PARA ACTUALIZAR VOLUMEN (INVERTIDA)
void updateVolume() {
  int volumePot = analogRead(VOLUME_POT_PIN);
  // Invertir la lectura: 4095 - valor original
  volumePot = 4095 - volumePot;
  
  // Mapear de 0-4095 a 0.0-1.0 con curva logarítmica para mejor control
  float linearVolume = volumePot / 4095.0;
  
  // Aplicar curva logarítmica para control más natural del volumen
  // Esto hace que los cambios pequeños en volumen bajo sean más perceptibles
  if (linearVolume > 0.0) {
    currentVolume = pow(linearVolume, 2.0); // Curva cuadrática
  } else {
    currentVolume = 0.0;
  }
  
  // Opcional: agregar un umbral mínimo para evitar volumen completamente silencioso
  // cuando el potenciómetro no está completamente en 0
  if (currentVolume < 0.02 && linearVolume > 0.01) {
    currentVolume = 0.02;
  }
}

// FUNCIÓN PARA INICIALIZAR EL IMU
void initIMU() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); // 400kHz I2C
  
  // Despertar el MPU6050
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // Despertar el dispositivo
  Wire.endTransmission(true);
  
  // Configurar el acelerómetro (±2g)
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(0x1C); // ACCEL_CONFIG register
  Wire.write(0x00); // ±2g
  Wire.endTransmission(true);
  
  delay(100); // Esperar estabilización
}

// FUNCIÓN PARA LEER EL IMU
void readIMU() {
  unsigned long currentTime = millis();
  if (currentTime - lastIMURead < IMU_READ_INTERVAL) {
    return; // No leer tan frecuentemente
  }
  lastIMURead = currentTime;
  
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(0x3B); // Starting register for accelerometer
  Wire.endTransmission(false);
  Wire.requestFrom(IMU_ADDRESS, 6, true);
  
  if (Wire.available() >= 6) {
    // Leer aceleración X
    int16_t raw_x = Wire.read() << 8 | Wire.read();
    // Leer aceleración Y  
    int16_t raw_y = Wire.read() << 8 | Wire.read();
    // Leer aceleración Z
    int16_t raw_z = Wire.read() << 8 | Wire.read();
    
    // Convertir a valores en g (-1.0 a +1.0)
    imu_accel_x = raw_x / 16384.0;
    imu_accel_y = raw_y / 16384.0;
    imu_accel_z = raw_z / 16384.0;
    
    // Aplicar filtro de suavizado
    filtered_x = filtered_x * (1.0 - IMU_FILTER_ALPHA) + imu_accel_x * IMU_FILTER_ALPHA;
    filtered_y = filtered_y * (1.0 - IMU_FILTER_ALPHA) + imu_accel_y * IMU_FILTER_ALPHA;
  }
}

// FUNCIÓN PARA ACTUALIZAR DECAY (INVERTIDA)
void updateDecay() {
  int decayPot = analogRead(DECAY_POT_PIN);
  // Invertir la lectura: 4095 - valor original
  decayPot = 4095 - decayPot;
  
  // Rango de decay: de 0.05 (súper corto) a 5.0 (súper largo)
  decayFactor = 0.05 + (decayPot / 4095.0) * 4.95;
}

// Función para calcular coeficientes del filtro usando IMU
void updateFilterCoefficients() {
  // Leer datos del IMU
  readIMU();
  
  // Convertir aceleración X a frecuencia de corte (200Hz - 3000Hz)
  // Usar valor absoluto y mapear de 0.0-1.0 a rango de frecuencias
  float x_abs = abs(filtered_x);
  x_abs = constrain(x_abs, 0.0, 1.0);
  float cutoff_freq = 200.0 + x_abs * 2800.0;
  
  // Convertir aceleración Y a resonancia (Q factor)
  // Usar valor absoluto y mapear de 0.0-1.0 a rango de Q
  float y_abs = abs(filtered_y);
  y_abs = constrain(y_abs, 0.0, 1.0);
  float Q = 0.7 + y_abs * 15.0;
  
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

// Función para aplicar el filtro
float applyFilter(float input) {
  float output = filter_a0 * input + filter_a1 * filter_x1 + filter_a2 * filter_x2
                 - filter_b1 * filter_y1 - filter_b2 * filter_y2;
  
  filter_x2 = filter_x1;
  filter_x1 = input;
  filter_y2 = filter_y1;
  filter_y1 = output;
  
  return output;
}

// Función para iniciar efecto visual sin bloquear
void startLedEffect(int effectType) {
  ledEffectType = effectType;
  ledEffectActive = true;
  ledEffectStartTime = millis();
  ledEffectStep = 0;
}

// Función para manejar efectos visuales sin delay
void updateLedEffects() {
  if (!ledEffectActive) return;
  
  unsigned long currentTime = millis();
  unsigned long elapsed = currentTime - ledEffectStartTime;
  
  switch (ledEffectType) {
    case 1: // Efecto escala - todos los LEDs parpadean
      if (elapsed < 50) {
        for (int i = 0; i < 4; i++) {
          digitalWrite(LED_PINS[i], HIGH);
        }
      } else if (elapsed < 100) {
        for (int i = 0; i < 4; i++) {
          digitalWrite(LED_PINS[i], LOW);
        }
      } else {
        ledEffectActive = false;
      }
      break;
      
    case 2: // Efecto patrón - secuencial
      {
        int stepDuration = 30;
        int currentLed = elapsed / stepDuration;
        
        if (currentLed < 4) {
          for (int i = 0; i < 4; i++) {
            digitalWrite(LED_PINS[i], (i == currentLed) ? HIGH : LOW);
          }
        } else if (elapsed < 150) {
          for (int i = 0; i < 4; i++) {
            digitalWrite(LED_PINS[i], LOW);
          }
        } else {
          ledEffectActive = false;
        }
      }
      break;
      
    case 3: // Efecto longitud
      if (elapsed < 100) {
        for (int i = 0; i < 4; i++) {
          digitalWrite(LED_PINS[i], LOW);
        }
      } else if (elapsed < 400) {
        int ledsToShow = (sequenceLength == 4) ? 1 : ((sequenceLength == 8) ? 2 : 4);
        for (int i = 0; i < 4; i++) {
          digitalWrite(LED_PINS[i], (i < ledsToShow) ? HIGH : LOW);
        }
      } else {
        for (int i = 0; i < 4; i++) {
          digitalWrite(LED_PINS[i], LOW);
        }
        ledEffectActive = false;
      }
      break;
  }
}

// Función para tocar una nota con ataque y decay controlables
void playNote(float frequency, int duration_ms) {
  // Actualizar controles al inicio de cada nota
  updateVolume();
  updateAttack();
  
  if (frequency == 0) {
    // Silencio con fade-out suave
    int silence_samples = (duration_ms * SAMPLE_RATE) / 1000;
    int fadeout_samples = min(200, silence_samples);
    
    for (int i = 0; i < silence_samples && isPlaying; i++) {
      float sample_value = 128.0;
      
      if (i < fadeout_samples && noteActive) {
        float fade_factor = 1.0 - ((float)i / fadeout_samples);
        sample_value = 128.0 + (lastSample - 128.0) * fade_factor;
      }
      
      dac_output_voltage(DAC_CHANNEL_1, (int)sample_value);
      lastSample = sample_value;
      delayMicroseconds(125);
    }
    noteActive = false;
    return;
  }
  
  updateDecay();
  
  float samples_per_cycle = SAMPLE_RATE / frequency;
  int total_samples = (duration_ms * SAMPLE_RATE) / 1000;
  int filter_update_rate = SAMPLE_RATE / 200;
  
  // Calcular duración del ataque basado en attackFactor
  // attackFactor va de 0.01 (rápido) a 2.0 (lento)
  // Convertir a número de muestras
  int attack_samples = (int)(attackFactor * SAMPLE_RATE * 0.1); // Max 0.2 segundos de ataque
  attack_samples = constrain(attack_samples, 50, total_samples / 2); // Límites razonables
  
  // Calcular amplitud actual basada en el volumen
  int currentAmplitude = (int)(MAX_AMPLITUDE * currentVolume);
  
  for (int i = 0; i < total_samples && isPlaying; i++) {
    // Actualizar controles ocasionalmente durante la nota
    if (i % (SAMPLE_RATE / 50) == 0) { // Actualizar 50 veces por segundo
      updateVolume();
      updateAttack();
      currentAmplitude = (int)(MAX_AMPLITUDE * currentVolume);
    }
    
    if (i % filter_update_rate == 0) {
      updateFilterCoefficients();
    }
    
    // Onda diente de sierra
    float cycle_position = fmod(i, samples_per_cycle) / samples_per_cycle;
    float raw_sample = 2 * cycle_position - 1;
    
    // Envelope con ataque y decay controlables
    float envelope = 1.0;
    
    // Fase de ataque controlable
    if (i < attack_samples) {
      float attack_progress = (float)i / attack_samples;
      // Curva exponencial para ataque más natural
      envelope = 1.0 - exp(-attack_progress * 5.0);
      
      // Crossfade suave con la nota anterior si existe
      if (noteActive && i < 50) {
        float crossfade = (float)i / 50.0;
        float prev_contribution = (lastSample - 128.0) / (float)currentAmplitude * (1.0 - crossfade);
        raw_sample = raw_sample * crossfade + prev_contribution * (1.0 - crossfade);
      }
    }
    // Fase de sustain y decay
    else {
      // Decay/Release controlable
      float decay_progress = (float)(i - attack_samples) / (total_samples - attack_samples);
      envelope = exp(-decay_progress * (6.0 / decayFactor));
    }
    
    raw_sample *= envelope;
    
    float filtered_sample = applyFilter(raw_sample);
    
    // Aplicar volumen
    int sample = 128 + (int)(currentAmplitude * filtered_sample);
    
    sample = constrain(sample, 0, 255);
    
    dac_output_voltage(DAC_CHANNEL_1, sample);
    lastSample = (float)sample;
    delayMicroseconds(125);
  }
  noteActive = true;
}

// Obtener frecuencia de la nota
float getNoteFrequency(int scaleStep) {
  if (scaleStep == 0) return 0; // Silencio
  
  int semitone = scales[currentScale][scaleStep - 1];
  float frequency = baseFreq * pow(2.0, semitone / 12.0);
  
  return frequency;
}

void setup() {
  // Deshabilitar logging para evitar interferencias
  esp_log_level_set("*", ESP_LOG_NONE);
  
  // Configurar pines de potenciómetros
  pinMode(ATTACK_POT_PIN, INPUT);  // Nuevo: pin de ataque
  pinMode(VOLUME_POT_PIN, INPUT);  // Pin de volumen
  pinMode(TEMPO_POT_PIN, INPUT);
  pinMode(DECAY_POT_PIN, INPUT);
  
  // Inicializar IMU
  initIMU();
  
  // Configurar botones con pull-up interno
  pinMode(SCALE_BTN_PIN, INPUT_PULLUP);
  pinMode(PATTERN_BTN_PIN, INPUT_PULLUP);
  pinMode(PLAY_BTN_PIN, INPUT_PULLUP);
  pinMode(LENGTH_BTN_PIN, INPUT_PULLUP);
  
  // Configurar LEDs
  for (int i = 0; i < 4; i++) {
    pinMode(LED_PINS[i], OUTPUT);
    digitalWrite(LED_PINS[i], LOW);
  }
  
  // Configurar DAC
  dac_output_enable(DAC_CHANNEL_1);
  dac_output_voltage(DAC_CHANNEL_1, 128); // Silencio inicial
  
  // Leer controles iniciales
  updateVolume();
  updateAttack();
  
  // Leer IMU inicial para inicializar filtros
  delay(100);
  readIMU();
  updateFilterCoefficients();
  
  // Secuencia de inicio con LEDs (sin delays bloqueantes)
  // Esta parte solo se ejecuta una vez al inicio, así que los delays están bien aquí
  for (int i = 0; i < 4; i++) {
    digitalWrite(LED_PINS[i], HIGH);
    delay(150);
    digitalWrite(LED_PINS[i], LOW);
    delay(50);
  }
  
  // Parpadeo final para indicar listo
  for (int j = 0; j < 3; j++) {
    for (int i = 0; i < 4; i++) {
      digitalWrite(LED_PINS[i], HIGH);
    }
    delay(100);
    for (int i = 0; i < 4; i++) {
      digitalWrite(LED_PINS[i], LOW);
    }
    delay(100);
  }
}

void loop() {
  // Manejo de botones sin interrumpir la secuencia
  unsigned long currentTime = millis();
  
  // Botón Play/Stop (Pin 15)
  bool currentPlayState = digitalRead(PLAY_BTN_PIN);
  if (currentPlayState == LOW && lastPlayState == HIGH && 
      (currentTime - lastPlayPress) > debounceDelay) {
    isPlaying = !isPlaying;
    
    if (isPlaying) {
      currentStep = 0;
      lastStepTime = currentTime;
    } else {
      dac_output_voltage(DAC_CHANNEL_1, 128); // Silencio
      for (int i = 0; i < 4; i++) {
        digitalWrite(LED_PINS[i], LOW);
      }
    }
    lastPlayPress = currentTime;
  }
  lastPlayState = currentPlayState;
  
  // Botón de escala (Pin 18)
  bool currentScaleState = digitalRead(SCALE_BTN_PIN);
  if (currentScaleState == LOW && lastScaleState == HIGH && 
      (currentTime - lastScalePress) > debounceDelay) {
    currentScale = (currentScale + 1) % numScales;
    lastScalePress = currentTime;
    startLedEffect(1); // Iniciar efecto visual para escala
  }
  lastScaleState = currentScaleState;
  
  // Botón de patrón (Pin 4)
  bool currentPatternState = digitalRead(PATTERN_BTN_PIN);
  if (currentPatternState == LOW && lastPatternState == HIGH && 
      (currentTime - lastPatternPress) > debounceDelay) {
    currentPattern = (currentPattern + 1) % numPatterns;
    lastPatternPress = currentTime;
    startLedEffect(2); // Iniciar efecto visual para patrón
  }
  lastPatternState = currentPatternState;
  
  // Botón de longitud (Pin 19)
  bool currentLengthState = digitalRead(LENGTH_BTN_PIN);
  if (currentLengthState == LOW && lastLengthState == HIGH && 
      (currentTime - lastLengthPress) > debounceDelay) {
    sequenceLength = (sequenceLength == 4) ? 8 : ((sequenceLength == 8) ? 16 : 4);
    
    if (currentStep >= sequenceLength) {
      currentStep = 0;
    }
    lastLengthPress = currentTime;
    startLedEffect(3); // Iniciar efecto visual para longitud
  }
  lastLengthState = currentLengthState;
  
  // Actualizar efectos visuales sin bloquear
  updateLedEffects();
  
  // Secuenciador principal
  if (isPlaying) {
    // Solo actualizar LEDs si no hay efecto activo
    if (!ledEffectActive) {
      for (int i = 0; i < 4; i++) {
        digitalWrite(LED_PINS[i], (i == (currentStep % 4)) ? HIGH : LOW);
      }
    }
    
    int tempoPot = analogRead(TEMPO_POT_PIN);
    // Invertir la lectura del tempo: 4095 - valor original
    tempoPot = 4095 - tempoPot;
    
    int bpm = 40 + (tempoPot / 4095.0) * 260; // 40-300 BPM
    unsigned long stepDuration = 60000 / (bpm * 4); // 16th notes
    
    if (currentTime - lastStepTime >= stepDuration) {
      int noteValue = patterns[currentPattern][currentStep];
      float frequency = getNoteFrequency(noteValue);
      
      int noteDuration = stepDuration * 0.9;
      playNote(frequency, noteDuration);
      
      currentStep = (currentStep + 1) % sequenceLength;
      lastStepTime = currentTime;
    }
  }
}

// ==============================================================================================================================================
// PROTO-SYNTH V2 - Secuenciador de Trance Electrónico - GC Lab Chile
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
// Sequenciador de trance electrónico con filtro controlado por LDR.
// ==============================================================================================================================================

// ==============================================================================================================================================
// FUNCIONAMIENTO
// ==============================================================================================================================================
// CONTROLES DE EXPRESIÓN:
// - Potenciómetro 1: Control de Volumen
// - Potenciómetro 2: Resonancia del filtro
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
// - IMU: No se usa
// - LDR: Control de filtro LPF
// - Micrófono: No se usa
// - Header 1: No se usa
// - Header 2: No se usa
// - Salida MIDI: No se usa
//
// MODO DE USO:
// 1. Ajusta el Potenciómetro 1 para controlar el volumen de salida.
// 2. Usa el Potenciómetro 2 para ajustar la resonancia del filtro.
// 3. Usa el Potenciómetro 3 para ajustar el tempo de la secuencia.
// 4. Usa el Potenciómetro 4 para ajustar el decay/release de las notas.
// 5. Presiona el Botón 1 para iniciar o detener la secuencia.
// 6. Usa el Botón 2 para cambiar la longitud de la secuencia (4, 8, 16 pasos).
// 7. Usa el Botón 3 para cambiar entre diferentes escalas musicales.
// 8. Usa el Botón 4 para cambiar entre diferentes patrones de secuencia.
// 9. Ajusta la luz ambiental para modificar la frecuencia de corte del filtro LPF mediante el LDR.
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

// ==============================================================================================================================================
// CONFIGURACIÓN DE HARDWARE - PINES
// ==============================================================================================================================================
const int VOLUME_POT_PIN = 13;      // Control de volumen
const int CUTOFF_LDR_PIN = 26;      // LDR para frecuencia de corte del filtro
const int RESONANCE_POT_PIN = 14;
const int TEMPO_POT_PIN = 12;
const int DECAY_POT_PIN = 27;

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
const int MAX_AMPLITUDE = 90;  // CAMBIADO: Ahora es amplitud máxima
int currentAmplitude = 90;     // NUEVO: Amplitud actual controlada por volumen

// Variables del filtro
float filter_x1 = 0, filter_x2 = 0;
float filter_y1 = 0, filter_y2 = 0;
float filter_a0, filter_a1, filter_a2;
float filter_b1, filter_b2;

// Variables del secuenciador
bool isPlaying = false;
unsigned long lastStepTime = 0;
int currentStep = 0;
int sequenceLength = 4;
int currentPattern = 0;
int currentScale = 0;
float baseFreq = 130.81; // C3 - frecuencia base

// Variables para control de decay
float decayFactor = 0.5;

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

// NUEVA FUNCIÓN PARA ACTUALIZAR VOLUMEN
void updateVolume() {
  int volumePot = analogRead(VOLUME_POT_PIN);
  // INVERTIR: Potenciómetro soldado al revés
  volumePot = 4095 - volumePot;
  // Mapear el potenciómetro a un rango de amplitud de 0 a MAX_AMPLITUDE
  // Con una curva logarítmica para mejor control de volumen
  float volumePercent = volumePot / 4095.0;
  currentAmplitude = (int)(MAX_AMPLITUDE * volumePercent * volumePercent);
  
  // Asegurar un mínimo volumen cuando no esté completamente a 0
  if (volumePercent > 0.02 && currentAmplitude < 5) {
    currentAmplitude = 5;
  }
}

// FUNCIÓN PARA ACTUALIZAR DECAY
void updateDecay() {
  int decayPot = analogRead(DECAY_POT_PIN);
  // INVERTIR: Potenciómetro soldado al revés
  decayPot = 4095 - decayPot;
  // Rango de decay: de 0.05 (súper corto) a 5.0 (súper largo)
  decayFactor = 0.05 + (decayPot / 4095.0) * 4.95;
}

// Función para calcular coeficientes del filtro
void updateFilterCoefficients() {
  int cutoff_raw = analogRead(CUTOFF_LDR_PIN);  // Leer LDR
  int resonance_raw = analogRead(RESONANCE_POT_PIN);
  // INVERTIR: Potenciómetro de resonancia soldado al revés
  resonance_raw = 4095 - resonance_raw;
  
  // Para el LDR: valores más altos de luz = frecuencias más altas
  // Invertir la lectura para que más luz = más brillo en el filtro
  float cutoff_freq = 200.0 + (cutoff_raw / 4095.0) * 2800.0;
  float Q = 0.7 + (resonance_raw / 4095.0) * 15.0;
  
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

// Función para tocar una nota con decay controlable
void playNote(float frequency, int duration_ms) {
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
  updateVolume();  // NUEVO: Actualizar volumen durante la reproducción
  
  float samples_per_cycle = SAMPLE_RATE / frequency;
  int total_samples = (duration_ms * SAMPLE_RATE) / 1000;
  int filter_update_rate = SAMPLE_RATE / 200;
  int fadein_samples = min(100, total_samples / 4);
  
  for (int i = 0; i < total_samples && isPlaying; i++) {
    if (i % filter_update_rate == 0) {
      updateFilterCoefficients();
    }
    
    // Onda diente de sierra
    float cycle_position = fmod(i, samples_per_cycle) / samples_per_cycle;
    float raw_sample = 2 * cycle_position - 1;
    
    // Envelope con fade-in/out
    float envelope = 1.0;
    float progress = (float)i / total_samples;
    
    // Fade-in suave
    if (i < fadein_samples) {
      float fadein_factor = (float)i / fadein_samples;
      envelope *= fadein_factor;
      
      if (noteActive && i < 50) {
        float crossfade = (float)i / 50.0;
        float prev_contribution = (lastSample - 128.0) / (float)currentAmplitude * (1.0 - crossfade);
        raw_sample = raw_sample * crossfade + prev_contribution * (1.0 - crossfade);
      }
    }
    
    // Decay/Release controlable
    if (i >= fadein_samples) {
      float decay_progress = (float)(i - fadein_samples) / (total_samples - fadein_samples);
      envelope *= exp(-decay_progress * (6.0 / decayFactor));
    }
    
    raw_sample *= envelope;
    
    float filtered_sample = applyFilter(raw_sample);
    int sample = 128 + (int)(currentAmplitude * filtered_sample);  // CAMBIADO: Usar currentAmplitude
    
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
  
  // Configurar pines de potenciómetros y LDR
  pinMode(VOLUME_POT_PIN, INPUT);      // Pin de volumen
  pinMode(CUTOFF_LDR_PIN, INPUT);      // LDR para cutoff del filtro
  pinMode(RESONANCE_POT_PIN, INPUT);
  pinMode(TEMPO_POT_PIN, INPUT);
  pinMode(DECAY_POT_PIN, INPUT);
  
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
  updateFilterCoefficients();
  updateVolume();  // NUEVO: Inicializar volumen
  
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
    // INVERTIR: Potenciómetro de tempo soldado al revés
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
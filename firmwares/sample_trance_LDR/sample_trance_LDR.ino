
// ==============================================================================================================================================
// PROTO-SYNTH V2 - AUDIO LIMPIO CON FILTRO LDR AJUSTADO - GC Lab Chile
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
// Sampler de audio limpio con secuenciador de patrones trance.
// ==============================================================================================================================================

// ==============================================================================================================================================
// FUNCIONAMIENTO
// ==============================================================================================================================================
// CONTROLES DE EXPRESIÓN:
// - Potenciómetro 1: TEMPO (40-220 BPM) - INVERTIDO
// - Potenciómetro 2: VOLUMEN (0-100%) - INVERTIDO
// - Potenciómetro 3: ATAQUE/ATTACK (0-50ms) - INVERTIDO 
// - Potenciómetro 4: PITCH GLOBAL (-60 a +12 semitonos) - INVERTIDO
// - Botón 1: Grabar/Parar grabación
// - Botón 2: Play/Stop secuenciador
// - Botón 3: Cambiar patrón de trance (16 patrones)
// - Botón 4: Cambiar longitud de secuencia (4/8/16 steps)
// - LED 1: Indicador de secuencia
// - LED 2: Indicador de secuencia
// - LED 3: Indicador de secuencia
// - LED 4: Indicador de secuencia
// - IMU: No se usa
// - LDR: Control de filtro LPF (500Hz-6000Hz, Q=3.5)
// - Micrófono: Grabadora
// - Header 1: No se usa
// - Header 2: No se usa
// - Salida MIDI: No se usa
//
// MODO DE USO:
// 1. Presiona el Botón 1 para iniciar la grabación.
// 2. Habla o reproduce un sonido cerca del micrófono para grabar.
// 3. Presiona el Botón 1 nuevamente para detener la grabación.
// 4. Ajusta el Potenciómetro 1 (TEMPO), Potenciómetro 2 (VOLUMEN), Potenciómetro 3 (ATAQUE) y Potenciómetro 4 (PITCH) según tu preferencia.
// 5. Presiona el Botón 2 para iniciar el secuenciador.
// 6. Usa el Botón 3 para cambiar entre los 16 patrones de trance.
// 7. Usa el Botón 4 para cambiar la longitud de la secuencia entre 4, 8 y 16 pasos.
// 8. Ajusta la LDR para modificar el filtro LPF en tiempo real mientras se reproduce la secuencia.
//
// INFORMACIÓN DEL CODIGO:
// - ESCALA FIJA: Menor Natural (octava normal)
// ==============================================================================================================================================

// ==============================================================================================================================================
// COMENTARIOS
// ==============================================================================================================================================
// - Todos los potenciómetros invertidos (4095-valor)
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
const int MIC_PIN = 33;
const int REC_BTN = 18;
const int PLAY_BTN = 4;
const int PATTERN_BTN = 15;
const int LENGTH_BTN = 19;
const int TEMPO_POT = 13;
const int VOLUME_POT = 14;
const int ATTACK_POT = 12;
const int PITCH_POT = 27;
const int LDR_PIN = 26;           // NUEVO: LDR para control de filtro
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

// Escala fija: Menor Natural
const int DARK_SCALE[] = {0, 2, 3, 5, 7, 8, 10};
const int SCALE_SIZE = 7;

// Variables de control (solo las esenciales)
float globalVolume = 1.0;
float globalAttack = 0.0;

// Variables del filtro simple controlado por LDR
float filter_x1 = 0, filter_x2 = 0;
float filter_y1 = 0, filter_y2 = 0;
float filter_a0 = 1.0, filter_a1 = 0.0, filter_a2 = 0.0;  // Inicializar con valores seguros
float filter_b1 = 0.0, filter_b2 = 0.0;

// Patrones de trance
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
  float attackSamples;
  bool inAttackPhase;
};
AudioPlayback currentPlayback = {false, 0.0, 1.0, 0, 0, 1.0, 0.0, false};

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

// FUNCIONES DEL FILTRO SIMPLE CON LDR (CORREGIDO)
void calculateSimpleLPF() {
  // Leer LDR para frecuencia de corte
  int ldrValue = analogRead(LDR_PIN);
  float ldr_normalized = constrain(ldrValue / 4095.0, 0.0, 1.0);
  
  // Mapear LDR a frecuencia de corte (500Hz - 6000Hz) - Rango más seguro
  float cutoff_freq = 500.0 + ldr_normalized * 5500.0;
  
  // AUMENTADO: Resonancia más alta para notar mejor el cambio
  float Q = 3.5;  // Resonancia más alta (era 1.2)
  
  // Normalizar frecuencia de corte
  float norm_freq = cutoff_freq / (SAMPLE_RATE / 2.0);
  norm_freq = constrain(norm_freq, 0.02, 0.9);  // Límites seguros
  
  float omega = PI * norm_freq;
  float sin_omega = sin(omega);
  float cos_omega = cos(omega);
  float alpha = sin_omega / (2.0 * Q);
  
  float b0 = (1.0 - cos_omega) / 2.0;
  float b1 = 1.0 - cos_omega;
  float b2 = (1.0 - cos_omega) / 2.0;
  float a0_temp = 1.0 + alpha;
  float a1_temp = -2.0 * cos_omega;
  float a2_temp = 1.0 - alpha;
  
  // Verificar que los coeficientes no sean cero o inválidos
  if (a0_temp != 0) {
    filter_a0 = b0 / a0_temp;
    filter_a1 = b1 / a0_temp;
    filter_a2 = b2 / a0_temp;
    filter_b1 = a1_temp / a0_temp;
    filter_b2 = a2_temp / a0_temp;
  }
}

float applySimpleLPF(float input) {
  float output = filter_a0 * input + filter_a1 * filter_x1 + filter_a2 * filter_x2
                 - filter_b1 * filter_y1 - filter_b2 * filter_y2;
  
  // Verificar que la salida no sea NaN o infinita
  if (isnan(output) || isinf(output)) {
    output = input; // Si hay problema, pasar señal sin filtrar
  }
  
  filter_x2 = filter_x1;
  filter_x1 = input;
  filter_y2 = filter_y1;
  filter_y1 = output;
  
  return output;
}
// FUNCIONES DE AUDIO (LIMPIAS)
float calculateNoteFreq(int scaleIndex) {
  if (scaleIndex < 0 || scaleIndex >= SCALE_SIZE) {
    return BASE_FREQ;
  }
  
  int semitones = DARK_SCALE[scaleIndex];
  float totalSemitones = semitones + globalPitchBend;
  return BASE_FREQ * pow(2.0, totalSemitones / 12.0);
}

void startSamplePlayback(float targetFreq, unsigned long duration) {
  if (!hasRecording || targetFreq <= 0) return;
  
  currentPlayback.isActive = false;
  
  currentPlayback.isActive = true;
  currentPlayback.samplePosition = 0.0;
  currentPlayback.pitchRatio = targetFreq / BASE_FREQ;
  currentPlayback.startTime = millis();
  currentPlayback.duration = duration;
  currentPlayback.volume = globalVolume;
  
  // Configurar ataque
  currentPlayback.attackSamples = (globalAttack / 1000.0) * SAMPLE_RATE;
  currentPlayback.inAttackPhase = (currentPlayback.attackSamples > 0);
  
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
  
  if (currentSample >= totalSamples) {
    currentPlayback.isActive = false;
    isNotePlaying = false;
    dac_output_voltage(DAC_CHANNEL_1, 128);
    return;
  }
  
  float fraction = currentPlayback.samplePosition - currentSample;
  
  uint16_t rawSample;
  if (currentSample < totalSamples - 1) {
    uint16_t sample1 = audioBuffer[currentSample];
    uint16_t sample2 = audioBuffer[currentSample + 1];
    rawSample = sample1 + (uint16_t)(fraction * (sample2 - sample1));
  } else {
    rawSample = audioBuffer[currentSample];
  }
  
  // CONVERSIÓN LIMPIA
  float floatSample = (rawSample - 2048.0) / 2048.0;
  
  // Aplicar envelope de ataque si está configurado
  float attackMultiplier = 1.0;
  if (currentPlayback.inAttackPhase && currentPlayback.attackSamples > 0) {
    float attackProgress = currentPlayback.samplePosition / currentPlayback.attackSamples;
    if (attackProgress < 1.0) {
      attackMultiplier = attackProgress * attackProgress; // Curva suave
    } else {
      currentPlayback.inAttackPhase = false;
    }
  }
  
  // NUEVO: Aplicar filtro simple controlado por LDR
  floatSample = applySimpleLPF(floatSample);
  
  // Aplicar volumen y ataque
  floatSample *= currentPlayback.volume * attackMultiplier;
  
  // Clipping suave
  floatSample = constrain(floatSample, -1.0, 1.0);
  
  // Conversión DAC
  uint16_t finalSample = (uint16_t)((floatSample + 1.0) * 2048.0);
  finalSample = constrain(finalSample, 0, 4095);
  
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
  int attackPot = analogRead(ATTACK_POT);
  int pitchPot = analogRead(PITCH_POT);
  
  // INVERTIR TODOS LOS POTENCIÓMETROS (cableado al revés)
  tempoPot = 4095 - tempoPot;
  volumePot = 4095 - volumePot;
  attackPot = 4095 - attackPot;
  pitchPot = 4095 - pitchPot;
  
  // CAMBIADO: Tempo desde 40 BPM mínimo
  baseTempo = map(tempoPot, 0, 4095, 40, 220);
  
  globalVolume = volumePot / 4095.0;
  
  // REDUCIDO: Ataque hasta 50ms máximo (era 80ms)
  globalAttack = (attackPot / 4095.0) * 50.0;  // 0 a 50ms
  
  globalPitchBend = map(pitchPot, 0, 4095, -60, 12);
  
  stepInterval = 60000 / (baseTempo * 4);
  
  // Actualizar filtro LDR cada vez que se leen controles
  calculateSimpleLPF();
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
}

void startArpeggiator() {
  if (!hasRecording) return;
  
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
  
  // NUEVO: Inicializar filtro con valores seguros
  calculateSimpleLPF();
  
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
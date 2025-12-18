
// ==============================================================================================================================================
// PROTO-SYNTH V2 - MELODIC SAMPLER - GC Lab Chile
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
// Sampler melódico que permite grabar un sample base y reproducirlo siguiendo la melodía de Super Mario Bros y otras melodias populares.
// ==============================================================================================================================================

// ==============================================================================================================================================
// FUNCIONAMIENTO
// ==============================================================================================================================================
// CONTROLES DE EXPRESIÓN:
// - Potenciómetro 1: Control de PITCH (+/- 24 semitonos)
// - Potenciómetro 2: Control de VOLUMEN MASTER (0-100%)
// - Potenciómetro 3: Control de TEMPO (60-200 BPM) 
// - Potenciómetro 4: No se usa
// - Botón 1: Grabación del sample base
// - Botón 2: Reproducción/Detención de la melodía
// - Botón 3: Activa/desactiva modo REVERSA del sample
// - Botón 4: Selector de melodia (No funciona - solo hay una melodía disponible)
// - LED 1: Parpadea con cada nota (beat indicator)
// - LED 2: Encendido durante reproducción
// - LED 3: Indica melodía actual (no usado con una sola melodía)
// - LED 4: Encendido durante grabación
// - IMU: No se usa
// - LDR: No se usa
// - Micrófono: Grabadora
// - Header 1: No se usa
// - Header 2: No se usa
// - Salida MIDI: No se usa
//
// MODO DE USO:
// 1. Presiona el BOTÓN 1 para iniciar la grabación del sample base.
// 2. Graba tu sample (máximo 1.5 segundos). El LED 4 se encenderá fijo y los otros LEDs parpadearán.
// 3. Presiona el BOTÓN 1 nuevamente para detener la grabación.
// 4. Presiona el BOTÓN 2 para reproducir la melodía usando tu sample grabado.
// 5. Ajusta el Potenciómetro 1 para cambiar el pitch.
// 6. Ajusta el Potenciómetro 2 para cambiar el volumen master.
// 7. Ajusta el Potenciómetro 3 para cambiar el tempo.
// 8. Presiona el BOTÓN 3 para activar/desactivar el modo reversa del sample.
// 9. Presiona el BOTÓN 2 nuevamente para detener la reproducción.
//
// INFORMACIÓN DEL CODIGO:
// - Melodias Incluidas:
//   1. Super Mario Bros - Overworld Theme (completa)
//   2. Twinkle Twinkle Little Star (No implementada)
//   3. Happy Birthday (No implementada)
//   4. Ode to Joy Beethoven (No implementada)
//   5. Mary Had a Little Lamb (No implementada)
// - NOTAS TÉCNICAS:
//   - Sample Rate: 22kHz
//   - Resolución: 12 bits
//   - Duración máxima de grabación: 1.5 segundos
//   - La melodía base está 4 octavas abajo del original
//   - El control de pitch permite subir/bajar 2 octavas adicionales  
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
// Pines Proto-Synth V2.0
const int MIC_PIN = 33;          // Micrófono
const int AUDIO_OUT_PIN = 25;    // Salida DAC

// Botones
const int REC_BTN = 18;          // Botón grabar
const int PLAY_BTN = 4;          // Botón play/stop
const int REVERSE_BTN = 15;      // Botón reversa sample
const int CYCLE_BTN = 19;        // Cambio de melodías

// Potenciómetros
const int TEMPO_POT = 12;        // Control de tempo
const int PITCH_POT = 13;        // Control de pitch (+/- 24 semitonos)
const int VOLUME_POT = 14;       // Control de volumen master
const int EXTRA_POT = 27;        // Extra control (por definir)

// LEDs para feedback visual
const int LED1 = 23;  // Beat indicator
const int LED2 = 32;  // Playing indicator
const int LED3 = 2;   // Melody indicator
const int LED4 = 5;   // Recording indicator

// ==============================================================================================================================================
// PROGRAMA
// ==============================================================================================================================================
// Configuración de audio
const int SAMPLE_RATE = 22000;   
const int MAX_SAMPLES = 33000;   // 1.5 segundos a 22kHz
const int REFERENCE_FREQ = 110;  // LA2 = 110Hz

// Definiciones de notas (frecuencias en Hz)
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define REST     0

// Estructura para una nota musical
struct MelodyNote {
  int frequency;  // Frecuencia de la nota
  int duration;   // Duración (4=quarter, 8=eighth, etc., negativo=dotted)
};

// Super Mario Bros melody - extraída del código proporcionado
const MelodyNote MARIO_MELODY[] = {
  {NOTE_E5,8}, {NOTE_E5,8}, {REST,8}, {NOTE_E5,8}, {REST,8}, {NOTE_C5,8}, {NOTE_E5,8},
  {NOTE_G5,4}, {REST,4}, {NOTE_G4,8}, {REST,4}, 
  {NOTE_C5,-4}, {NOTE_G4,8}, {REST,4}, {NOTE_E4,-4},
  {NOTE_A4,4}, {NOTE_B4,4}, {NOTE_AS4,8}, {NOTE_A4,4},
  {NOTE_G4,-8}, {NOTE_E5,-8}, {NOTE_G5,-8}, {NOTE_A5,4}, {NOTE_F5,8}, {NOTE_G5,8},
  {REST,8}, {NOTE_E5,4}, {NOTE_C5,8}, {NOTE_D5,8}, {NOTE_B4,-4},
  {NOTE_C5,-4}, {NOTE_G4,8}, {REST,4}, {NOTE_E4,-4},
  {NOTE_A4,4}, {NOTE_B4,4}, {NOTE_AS4,8}, {NOTE_A4,4},
  {NOTE_G4,-8}, {NOTE_E5,-8}, {NOTE_G5,-8}, {NOTE_A5,4}, {NOTE_F5,8}, {NOTE_G5,8},
  {REST,8}, {NOTE_E5,4}, {NOTE_C5,8}, {NOTE_D5,8}, {NOTE_B4,-4},
  
  {REST,4}, {NOTE_G5,8}, {NOTE_FS5,8}, {NOTE_F5,8}, {NOTE_DS5,4}, {NOTE_E5,8},
  {REST,8}, {NOTE_C5,8}, {NOTE_A4,8}, {NOTE_C5,8}, {REST,8}, {NOTE_A4,8}, {NOTE_C5,8}, {NOTE_D5,8},
  {REST,4}, {NOTE_DS5,4}, {REST,8}, {NOTE_D5,-4},
  {NOTE_C5,2}, {REST,2},

  {REST,4}, {NOTE_G5,8}, {NOTE_FS5,8}, {NOTE_F5,8}, {NOTE_DS5,4}, {NOTE_E5,8},
  {REST,8}, {NOTE_C5,8}, {NOTE_A4,8}, {NOTE_C5,8}, {REST,8}, {NOTE_A4,8}, {NOTE_C5,8}, {NOTE_D5,8},
  {REST,4}, {NOTE_DS5,4}, {REST,8}, {NOTE_D5,-4},
  {NOTE_C5,2}, {REST,2}
};

// Twinkle Twinkle Little Star
const MelodyNote TWINKLE_MELODY[] = {
  {NOTE_C4,4}, {NOTE_C4,4}, {NOTE_G4,4}, {NOTE_G4,4},
  {NOTE_A4,4}, {NOTE_A4,4}, {NOTE_G4,2},
  {NOTE_F4,4}, {NOTE_F4,4}, {NOTE_E4,4}, {NOTE_E4,4},
  {NOTE_D4,4}, {NOTE_D4,4}, {NOTE_C4,2}
};

// Happy Birthday
const MelodyNote BIRTHDAY_MELODY[] = {
  {NOTE_C4,8}, {NOTE_C4,8}, {NOTE_D4,4}, {NOTE_C4,4}, {NOTE_F4,4}, {NOTE_E4,2},
  {NOTE_C4,8}, {NOTE_C4,8}, {NOTE_D4,4}, {NOTE_C4,4}, {NOTE_G4,4}, {NOTE_F4,2},
  {NOTE_C4,8}, {NOTE_C4,8}, {NOTE_C5,4}, {NOTE_A4,4}, {NOTE_F4,4}, {NOTE_E4,4}, {NOTE_D4,4}
};

// Ode to Joy (Beethoven)
const MelodyNote ODE_MELODY[] = {
  {NOTE_E4,4}, {NOTE_E4,4}, {NOTE_F4,4}, {NOTE_G4,4},
  {NOTE_G4,4}, {NOTE_F4,4}, {NOTE_E4,4}, {NOTE_D4,4},
  {NOTE_C4,4}, {NOTE_C4,4}, {NOTE_D4,4}, {NOTE_E4,4},
  {NOTE_E4,4}, {NOTE_D4,8}, {NOTE_D4,2}
};

// Mary Had a Little Lamb
const MelodyNote MARY_MELODY[] = {
  {NOTE_E4,4}, {NOTE_D4,4}, {NOTE_C4,4}, {NOTE_D4,4},
  {NOTE_E4,4}, {NOTE_E4,4}, {NOTE_E4,2},
  {NOTE_D4,4}, {NOTE_D4,4}, {NOTE_D4,2},
  {NOTE_E4,4}, {NOTE_G4,4}, {NOTE_G4,2}
};

// Solo una melodía: Super Mario Bros completa
const MelodyNote* MELODIES[] = {
  MARIO_MELODY
};

const int MELODY_SIZES[] = {
  sizeof(MARIO_MELODY) / sizeof(MelodyNote)
};

const char* MELODY_NAMES[] = {
  "Super Mario Bros - Overworld Theme"
};

const int NUM_MELODIES = 1;

// Variables del sampler
uint16_t* audioBuffer;
volatile int totalSamples = 0;
volatile bool hasRecording = false;
volatile bool isRecording = false;

// Variables del secuenciador melódico
volatile bool isPlaying = false;
volatile bool reverseMode = false;
int currentMelody = 0;
int currentNote = 0;
int baseTempo = 120; // BPM base
float masterVolume = 1.0;
int pitchBend = 0; // semitonos +/- 24 (centro en 0)

// Timing
unsigned long lastNoteTime = 0;
unsigned long noteDuration = 0;
unsigned long lastButtonTime = 0;
const int DEBOUNCE_MS = 100;

// LEDs
unsigned long lastLedUpdate = 0;
bool isNotePlaying = false;

float calculatePitchRatio(int noteFreq) {
  if (noteFreq == REST) return 1.0;
  
  // Bajar toda la melodía 4 octavas por defecto (-48 semitonos)
  // Luego aplicar el pitch bend del potenciómetro (+/- 24 semitonos)
  float totalPitch = -48.0 + pitchBend; // Base -4 octavas + control del pot
  float adjustedFreq = noteFreq * pow(2.0, totalPitch / 12.0);
  
  return adjustedFreq / REFERENCE_FREQ;
}

void updateLEDs() {
  digitalWrite(LED1, isNotePlaying ? HIGH : LOW);
  digitalWrite(LED2, isPlaying ? HIGH : LOW);
  digitalWrite(LED3, (currentMelody % 2) ? HIGH : LOW);
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
  Serial.println("Grabando en LA2 (110Hz)...");
}

void stopRecording() {
  isRecording = false;
  hasRecording = (totalSamples > 0);
  Serial.print("Grabación terminada. Samples: ");
  Serial.println(totalSamples);
}

void startMelody() {
  if (!hasRecording) return;
  
  isPlaying = true;
  currentNote = 0;
  lastNoteTime = millis();
  
  Serial.print("Reproduciendo: ");
  Serial.println(MELODY_NAMES[currentMelody]);
}

void playMelodyStep() {
  if (!isPlaying || !hasRecording) return;
  
  unsigned long currentTime = millis();
  
  // Si es momento de la siguiente nota
  if (currentTime - lastNoteTime >= noteDuration) {
    
    // Si llegamos al final de la melodía
    if (currentNote >= MELODY_SIZES[currentMelody]) {
      currentNote = 0; // Loop de la melodía
    }
    
    MelodyNote note = MELODIES[currentMelody][currentNote];
    
    // Calcular duración de la nota basada en tempo (como en el código original)
    int wholenote = (60000 * 4) / baseTempo;
    int divider = note.duration;
    
    if (divider > 0) {
      // Nota regular
      noteDuration = wholenote / divider;
    } else if (divider < 0) {
      // Notas con puntillo (dotted notes)
      noteDuration = wholenote / abs(divider);
      noteDuration *= 1.5; // Aumenta duración 50% para notas con puntillo
    }
    
    isNotePlaying = (note.frequency != REST);
    
    if (note.frequency != REST) {
      // Calcular pitch ratio para esta nota
      float pitchRatio = calculatePitchRatio(note.frequency);
      
      // Reproducir sample con el pitch de la nota
      playSampleAtPitch(pitchRatio, noteDuration * 0.9); // 90% de duración
    }
    
    lastNoteTime = currentTime;
    currentNote++;
    
    updateLEDs();
  }
}

void playSampleAtPitch(float pitchRatio, unsigned long maxDuration) {
  if (!hasRecording) return;
  
  unsigned long startTime = millis();
  float samplePosition = 0.0;
  
  while (samplePosition < totalSamples - 1 && 
         (millis() - startTime) < maxDuration) {
    
    if (!isPlaying) break;
    
    int currentSample = (int)samplePosition;
    float fraction = samplePosition - currentSample;
    
    uint16_t sample;
    
    if (reverseMode) {
      int reverseIndex = totalSamples - 1 - currentSample;
      if (reverseIndex > 0) {
        uint16_t sample1 = audioBuffer[reverseIndex];
        uint16_t sample2 = audioBuffer[reverseIndex - 1];
        sample = sample1 + (uint16_t)(fraction * (sample2 - sample1));
      } else {
        sample = audioBuffer[0];
      }
    } else {
      if (currentSample < totalSamples - 1) {
        uint16_t sample1 = audioBuffer[currentSample];
        uint16_t sample2 = audioBuffer[currentSample + 1];
        sample = sample1 + (uint16_t)(fraction * (sample2 - sample1));
      } else {
        sample = audioBuffer[currentSample];
      }
    }
    
    // Aplicar volumen master
    sample = (uint16_t)(sample * masterVolume);
    if (sample > 4095) sample = 4095;
    
    // Output al DAC
    dac_output_voltage(DAC_CHANNEL_1, sample >> 4);
    
    // Avanzar posición con pitch ratio
    samplePosition += pitchRatio;
    
    // Delay fijo para mantener calidad
    delayMicroseconds(45);
  }
}

void readControls() {
  // Leer potenciómetros
  int tempoPot = analogRead(TEMPO_POT);
  int pitchPot = analogRead(PITCH_POT);
  int volumePot = analogRead(VOLUME_POT);
  
  // Mapear valores
  baseTempo = map(tempoPot, 0, 4095, 60, 200); // 60-200 BPM
  pitchBend = map(pitchPot, 0, 4095, -24, 24); // -24 a +24 semitonos (4 octavas rango)
  masterVolume = map(volumePot, 0, 4095, 0, 100) / 100.0;
}

void handleButtons() {
  unsigned long currentTime = millis();
  if (currentTime - lastButtonTime < DEBOUNCE_MS) return;
  
  // Botón grabar
  static bool lastRecState = HIGH;
  bool recState = digitalRead(REC_BTN);
  if (recState == LOW && lastRecState == HIGH) {
    if (!isRecording) {
      startRecording();
    } else {
      stopRecording();
    }
    lastButtonTime = currentTime;
  }
  lastRecState = recState;
  
  // Botón play/stop
  static bool lastPlayState = HIGH;
  bool playState = digitalRead(PLAY_BTN);
  if (playState == LOW && lastPlayState == HIGH) {
    if (!isPlaying) {
      startMelody();
    } else {
      isPlaying = false;
      isNotePlaying = false;
      Serial.println("Melodía detenida");
    }
    lastButtonTime = currentTime;
  }
  lastPlayState = playState;
  
  // Botón reversa
  static bool lastReverseState = HIGH;
  bool reverseState = digitalRead(REVERSE_BTN);
  if (reverseState == LOW && lastReverseState == HIGH) {
    reverseMode = !reverseMode;
    Serial.print("Modo reversa: ");
    Serial.println(reverseMode ? "ON" : "OFF");
    lastButtonTime = currentTime;
  }
  lastReverseState = reverseState;
  
  // Botón cambio de melodía (deshabilitado ya que solo hay una)
  // Ya no es necesario cambiar melodías
}

void setup() {
  Serial.begin(115200);
  Serial.println("🎵 MELODIC SAMPLER v2.0");
  
  // Allocar buffer de audio
  audioBuffer = (uint16_t*)malloc(MAX_SAMPLES * sizeof(uint16_t));
  if (!audioBuffer) {
    Serial.println("ERROR: No se pudo allocar memoria!");
    while(1) delay(1000);
  }
  Serial.println("✓ Buffer de audio allocado");
  
  // Configurar pines
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  
  pinMode(REC_BTN, INPUT_PULLUP);
  pinMode(PLAY_BTN, INPUT_PULLUP);
  pinMode(REVERSE_BTN, INPUT_PULLUP);
  pinMode(CYCLE_BTN, INPUT_PULLUP);
  
  // Configurar ADC y DAC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  analogSetWidth(12);
  dac_output_enable(DAC_CHANNEL_1);
  dac_output_voltage(DAC_CHANNEL_1, 128);
  
  // Test de LEDs
  for (int i = 0; i < 2; i++) {
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, HIGH);
    digitalWrite(LED3, HIGH);
    digitalWrite(LED4, HIGH);
    delay(200);
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);
    digitalWrite(LED4, LOW);
    delay(200);
  }
  
  Serial.println("🎼 MELODÍA: Super Mario Bros - Overworld Theme completa");
  
  Serial.println("🎛️ CONTROLES:");
  Serial.println("  REC(18): Grabar sample base");
  Serial.println("  PLAY(4): Reproducir melodía");
  Serial.println("  REVERSE(15): Toggle reversa sample");
  Serial.println("  CYCLE(19): (No usado - solo hay una melodía)");
  Serial.println("  TEMPO(12): 60-200 BPM");
  Serial.println("  PITCH(13): +/- 24 semitonos desde base (-4 octavas)");
  Serial.println("  VOLUME(14): 0-100%");
  Serial.println("🚀 ¡READY TO MAKE MUSIC!");
}

void loop() {
  readControls();
  handleButtons();
  
  if (isRecording) {
    showRecordingAnimation();
    
    // Grabar con sampling rate apropiado
    if (totalSamples < MAX_SAMPLES) {
      uint16_t sample = analogRead(MIC_PIN);
      audioBuffer[totalSamples++] = sample;
      delayMicroseconds(45); // ~22kHz sampling
    } else {
      stopRecording();
    }
  } else if (isPlaying) {
    playMelodyStep();
  } else {
    updateLEDs();
  }
  
  delayMicroseconds(10);
}
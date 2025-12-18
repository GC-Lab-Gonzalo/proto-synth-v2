
// ==============================================================================================================================================
// PROTO-SYNTH V2 - Controlador MIDI con Secuenciador Optimizado - GC Lab Chile
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
// Controlador MIDI con secuenciador de 4 pasos optimizado para performance en tiempo real.
// ==============================================================================================================================================

// ==============================================================================================================================================
// FUNCIONAMIENTO
// ==============================================================================================================================================
// CONTROLES DE EXPRESIÓN:
// - Potenciómetro 1: Paso 1
// - Potenciómetro 2: Paso 2
// - Potenciómetro 3: Paso 3 
// - Potenciómetro 4: Paso 4
// - Botón 1: Aumentar velocidad
// - Botón 2: Disminuir velocidad
// - Botón 3: Cambio de escala
// - Botón 4: Cambio de octava
// - LED 1: Paso 1
// - LED 2: Paso 2
// - LED 3: Paso 3
// - LED 4: Paso 4
// - IMU: Control MIDI
// - LDR: No se usa
// - Micrófono: No se usa
// - Header 1: No se usa
// - Header 2: No se usa
// - Salida MIDI: Salida de sequencias y control
//
// MODO DE USO:
// 1. Conectar Proto-Synth a DAW/sintetizador via MIDI (o por USB MIDI serial utilizando Hairless MIDI y cambiando el baudio de 31250 a 115200)
// 2. Ajustar velocidad con botones 1 y 2 (LEDs parpadean al cambiar)
// 3. Seleccionar escala con botón 3 (LEDs parpadean al cambiar)
// 4. Seleccionar octava con botón 4 (LEDs parpadean al cambiar)
// 5. Ajustar valores de cada paso con potenciómetros 1-4
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
#include <Bounce2.h>
#include <Wire.h>

// ==============================================================================================================================================
// CONFIGURACIÓN DE HARDWARE - PINES
// ==============================================================================================================================================
#define MPU6050_ADDR 0x68 // Dirección I2C del MPU6050

const int POT1 = 13;    // Potenciómetro 1 - Paso 1
const int POT2 = 14;    // Potenciómetro 2 - Paso 2
const int POT3 = 12;    // Potenciómetro 3 - Paso 3  
const int POT4 = 27;    // Potenciómetro 4 - Paso 4

const int BOTON1 = 18;  // Botón 1 para aumentar velocidad
const int BOTON2 = 4;   // Botón 2 para disminuir velocidad
const int BOTON3 = 15;  // Botón 3 para cambio de escala
const int BOTON4 = 19;  // Botón 4 para cambio de octava

const int LED1 = 23;    // LED 1 - Paso 1
const int LED2 = 32;    // LED 2 - Paso 2
const int LED3 = 5;     // LED 3 - Paso 3
const int LED4 = 2;     // LED 4 - Paso 4

// ==============================================================================================================================================
// PROGRAMA
// ==============================================================================================================================================
// Variables para velocidad
int speedStep = 5;
const int speedLevels[] = {1000, 800, 600, 500, 400, 300, 250, 200, 150, 100, 50};
const int maxSpeedStep = 10;

// Variables para octavas
int currentOctave = 0;
const int octaveBases[] = {36, 48, 60};

// Variables para modos y controles
int currentStep = 0;
unsigned long lastStepTime = 0;
int interval = 500;

// Variables para efectos visuales no bloqueantes
enum EffectType {
  NO_EFFECT,
  SPEED_EFFECT,
  SCALE_EFFECT,
  OCTAVE_EFFECT
};

EffectType currentEffect = NO_EFFECT;
unsigned long effectStartTime = 0;
int effectStep = 0;
int effectCycles = 0;

// Escalas extendidas
const int JONICA[] = {0, 2, 4, 5, 7, 9, 11, 12, 14, 16, 17, 19, 21, 23, 24, 26, 28, 29, 31, 33, 35};
const int DORICA[] = {0, 2, 3, 5, 7, 9, 10, 12, 14, 15, 17, 19, 21, 22, 24, 26, 27, 29, 31, 33, 34};
const int FRIGIA[] = {0, 1, 3, 5, 7, 8, 10, 12, 13, 15, 17, 19, 20, 22, 24, 25, 27, 29, 31, 32, 34};
const int LIDIA[] = {0, 2, 4, 6, 7, 9, 11, 12, 14, 16, 18, 19, 21, 23, 24, 26, 28, 30, 31, 33, 35};
const int MIXOLIDIA[] = {0, 2, 4, 5, 7, 9, 10, 12, 14, 16, 17, 19, 21, 22, 24, 26, 28, 29, 31, 33, 34};
const int EOLICA[] = {0, 2, 3, 5, 7, 8, 10, 12, 14, 15, 17, 19, 20, 22, 24, 26, 27, 29, 31, 32, 34};
const int LOCRIA[] = {0, 1, 3, 5, 6, 8, 10, 12, 13, 15, 17, 18, 20, 22, 24, 25, 27, 29, 30, 32, 34};
const int PENTATONICA[] = {0, 2, 4, 7, 9, 12, 14, 16, 19, 21, 24, 26, 28, 31, 33};
const int FLAMENCA[] = {0, 1, 4, 5, 7, 8, 10, 12, 13, 16, 17, 19, 20, 22, 24, 25, 28, 29, 31, 32, 34};
const int NEOCLASICA[] = {0, 1, 3, 5, 7, 8, 11, 12, 13, 15, 17, 19, 20, 23, 24, 25, 27, 29, 31, 32, 35};

const int* scales[] = {JONICA, DORICA, FRIGIA, LIDIA, MIXOLIDIA, EOLICA, LOCRIA, PENTATONICA, FLAMENCA, NEOCLASICA};
const int scaleLengths[] = {21, 20, 20, 21, 20, 20, 20, 15, 21, 21};
const char* scaleNames[] = {"Jónica", "Dórica", "Frigia", "Lidia", "Mixolidia", "Eólica", "Locria", "Pentatónica", "Flamenca", "Neoclásica"};
int currentScale = 0;

// Variables para la gestión de notas MIDI
int lastNotePlayed = 0;

// Variables optimizadas para MPU6050
int16_t accelX, accelY, accelZ;
float angleY = 0;
unsigned long lastMPURead = 0;
const unsigned long MPU_READ_INTERVAL = 10; // Leer cada 10ms para mayor resolución
int lastCutoffValue = 64; // Valor inicial centrado
int smoothCutoffValue = 64; // Valor suavizado
const int CUTOFF_CC = 1;

// Variables para suavizado y optimización del IMU
float angleYFiltered = 0;
const float FILTER_ALPHA = 0.3; // Factor de suavizado (0.0 = muy suave, 1.0 = sin filtro)
const int MIN_CC_CHANGE = 1; // Cambio mínimo para enviar CC
const int NOISE_THRESHOLD = 100; // Umbral para ruido del acelerómetro
unsigned long lastCCSent = 0;
const unsigned long MIN_CC_INTERVAL = 5; // Mínimo 5ms entre envíos de CC

// Variables para el estado de los botones con Bounce2
Bounce buttonSpeedUp = Bounce();
Bounce buttonSpeedDown = Bounce();
Bounce buttonScale = Bounce();
Bounce buttonOctave = Bounce();

// Buffer para optimizar envío MIDI
struct MidiMessage {
  byte status;
  byte data1;
  byte data2;
};

// Funciones MIDI optimizadas
inline void sendMidiNoteOn(byte note, byte velocity, byte channel) {
  Serial.write(0x90 + (channel - 1));
  Serial.write(note);
  Serial.write(velocity);
}

inline void sendMidiNoteOff(byte note, byte velocity, byte channel) {
  Serial.write(0x80 + (channel - 1));
  Serial.write(note);
  Serial.write(velocity);
}

inline void sendMidiCC(byte controller, byte value, byte channel) {
  Serial.write(0xB0 + (channel - 1));
  Serial.write(controller);
  Serial.write(value);
}

void setup() {
  // Deshabilitar logging y configurar Serial para MIDI
  esp_log_level_set("*", ESP_LOG_NONE);
  Serial.end();
  delay(100);
  
  Serial.begin(31250, SERIAL_8N1, 3, 1); // RX=3, TX=1 (TX0)
  
  // Inicializar I2C con velocidad optimizada
  Wire.begin();
  Wire.setClock(400000); // 400kHz para I2C rápido
  initMPU6050();
  
  // Configuración de pines
  pinMode(BOTON1, INPUT_PULLUP);
  pinMode(BOTON2, INPUT_PULLUP);
  pinMode(BOTON3, INPUT_PULLUP);
  pinMode(BOTON4, INPUT_PULLUP);
  
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  
  // Configuración de los debouncers con intervalo más rápido
  buttonSpeedDown.attach(BOTON1);
  buttonSpeedDown.interval(15);
  
  buttonSpeedUp.attach(BOTON2);
  buttonSpeedUp.interval(15);
  
  buttonScale.attach(BOTON3);
  buttonScale.interval(15);
  
  buttonOctave.attach(BOTON4);
  buttonOctave.interval(15);
  
  interval = speedLevels[speedStep];
  
  // Lectura inicial del MPU para estabilizar
  for(int i = 0; i < 10; i++) {
    readMPU6050();
    delay(10);
  }
  
  startupSequence();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Prioridad 1: Actualizar IMU (más frecuente)
  updateCutoffFromMPU(currentTime);
  
  // Prioridad 2: Control de secuencia temporal
  if (currentTime - lastStepTime >= interval) {
    lastStepTime = currentTime;
    
    currentStep = (currentStep + 1) % 4;
    
    // Actualizar LEDs solo si no hay efecto activo
    if (currentEffect == NO_EFFECT) {
      updateStepLEDs();
    }
    
    // Reproducir nota del paso actual
    playNoteFromCurrentStep();
  }
  
  // Prioridad 3: Botones (menos crítico)
  updateButtons();
  
  // Prioridad 4: Efectos visuales (menos crítico)
  handleVisualEffects();
}

void updateCutoffFromMPU(unsigned long currentTime) {
  // Leer MPU de forma más eficiente
  if (currentTime - lastMPURead >= MPU_READ_INTERVAL) {
    lastMPURead = currentTime;
    
    // Lectura optimizada del MPU6050
    readMPU6050Fast();
    
    // Filtrar ruido del acelerómetro
    if (abs(accelY) > NOISE_THRESHOLD && abs(accelZ) > NOISE_THRESHOLD) {
      // Calcular ángulo de inclinación
      float accelY_g = accelY / 16384.0;
      float accelZ_g = accelZ / 16384.0;
      
      angleY = atan2(accelY_g, accelZ_g) * 180.0 / PI;
      
      // Aplicar filtro pasa-bajos para suavizar
      angleYFiltered = (FILTER_ALPHA * angleY) + ((1.0 - FILTER_ALPHA) * angleYFiltered);
      
      // Mapear a valor MIDI CC con rango optimizado
      float clampedAngle = constrain(angleYFiltered, -30.0, 30.0);
      int targetCutoffValue = map(clampedAngle * 100, -3000, 3000, 0, 127);
      targetCutoffValue = constrain(targetCutoffValue, 0, 127);
      
      // Suavizado adicional del valor final
      smoothCutoffValue = (smoothCutoffValue * 3 + targetCutoffValue) / 4;
      
      // Enviar CC solo si hay cambio suficiente y ha pasado tiempo mínimo
      if (abs(smoothCutoffValue - lastCutoffValue) >= MIN_CC_CHANGE && 
          currentTime - lastCCSent >= MIN_CC_INTERVAL) {
        
        sendMidiCC(CUTOFF_CC, smoothCutoffValue, 1);
        lastCutoffValue = smoothCutoffValue;
        lastCCSent = currentTime;
      }
    }
  }
}

void readMPU6050Fast() {
  // Lectura optimizada - solo acelerómetro Y y Z
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3D); // ACCEL_YOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 4, true); // Solo 4 bytes
  
  accelY = Wire.read() << 8 | Wire.read();
  accelZ = Wire.read() << 8 | Wire.read();
}

void updateButtons() {
  buttonSpeedUp.update();
  buttonSpeedDown.update();
  buttonScale.update();
  buttonOctave.update();
  
  if (buttonSpeedUp.fell()) {
    if (speedStep < maxSpeedStep) {
      speedStep++;
      interval = speedLevels[speedStep];
      startEffect(SPEED_EFFECT);
    }
  }
  
  if (buttonSpeedDown.fell()) {
    if (speedStep > 0) {
      speedStep--;
      interval = speedLevels[speedStep];
      startEffect(SPEED_EFFECT);
    }
  }
  
  if (buttonScale.fell()) {
    currentScale = (currentScale + 1) % (sizeof(scales) / sizeof(scales[0]));
    startEffect(SCALE_EFFECT);
  }
  
  if (buttonOctave.fell()) {
    currentOctave = (currentOctave + 1) % 3;
    startEffect(OCTAVE_EFFECT);
  }
}

void playNoteFromCurrentStep() {
  // Detener nota anterior de forma más eficiente
  if (lastNotePlayed != 0) {
    sendMidiNoteOff(lastNotePlayed, 0, 1);
    lastNotePlayed = 0;
  }
  
  // Obtener valor del potenciómetro
  int potPin = getPotForStep(currentStep);
  int potValue = analogRead(potPin);
  
  // Umbral de silencio ajustado
  if (potValue <= 150) { // Aumentado de 50 a 150
    return;
  }
  
  // Mapear a escala
  int index = map(potValue, 150, 4095, 0, scaleLengths[currentScale] - 1);
  index = constrain(index, 0, scaleLengths[currentScale] - 1);
  
  int note = octaveBases[currentOctave] + scales[currentScale][index];
  note = constrain(note, 0, 127); // Asegurar rango MIDI válido
  
  // Enviar nota
  sendMidiNoteOn(note, 127, 1);
  lastNotePlayed = note;
}

int getPotForStep(int step) {
  switch (step) {
    case 0: return POT1;
    case 1: return POT2;
    case 2: return POT3;
    case 3: return POT4;
    default: return POT1;
  }
}

void updateStepLEDs() {
  // Apagar todos los LEDs
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);
  
  // Encender LED actual
  switch (currentStep) {
    case 0: digitalWrite(LED1, HIGH); break;
    case 1: digitalWrite(LED2, HIGH); break;
    case 2: digitalWrite(LED3, HIGH); break;
    case 3: digitalWrite(LED4, HIGH); break;
  }
}

void startEffect(EffectType effect) {
  currentEffect = effect;
  effectStartTime = millis();
  effectStep = 0;
  effectCycles = 0;
}

void handleVisualEffects() {
  if (currentEffect == NO_EFFECT) return;
  
  unsigned long currentTime = millis();
  
  switch (currentEffect) {
    case SPEED_EFFECT:
      handleSpeedEffect(currentTime);
      break;
    case SCALE_EFFECT:
      handleScaleEffect(currentTime);
      break;
    case OCTAVE_EFFECT:
      handleOctaveEffect(currentTime);
      break;
  }
}

void handleSpeedEffect(unsigned long currentTime) {
  if (currentTime - effectStartTime >= 80) { // Más rápido
    effectStartTime = currentTime;
    
    if (effectStep == 0) {
      digitalWrite(LED1, HIGH);
      digitalWrite(LED2, HIGH);
      digitalWrite(LED3, HIGH);
      digitalWrite(LED4, HIGH);
      effectStep = 1;
    } else {
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, LOW);
      digitalWrite(LED3, LOW);
      digitalWrite(LED4, LOW);
      effectStep = 0;
      effectCycles++;
      
      if (effectCycles >= 2) {
        currentEffect = NO_EFFECT;
        updateStepLEDs();
      }
    }
  }
}

void handleScaleEffect(unsigned long currentTime) {
  if (currentTime - effectStartTime >= 80) { // Más rápido
    effectStartTime = currentTime;
    
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);
    digitalWrite(LED4, LOW);
    
    switch (effectStep) {
      case 0: digitalWrite(LED1, HIGH); break;
      case 1: digitalWrite(LED2, HIGH); break;
      case 2: digitalWrite(LED3, HIGH); break;
      case 3: digitalWrite(LED4, HIGH); break;
    }
    
    effectStep++;
    
    if (effectStep >= 4) {
      effectStep = 0;
      effectCycles++;
      
      if (effectCycles >= 2) {
        currentEffect = NO_EFFECT;
        updateStepLEDs();
      }
    }
  }
}

void handleOctaveEffect(unsigned long currentTime) {
  if (currentTime - effectStartTime >= 800) { // Más corto
    currentEffect = NO_EFFECT;
    updateStepLEDs();
    return;
  }
  
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);
  
  for(int i = 0; i <= currentOctave; i++) {
    switch(i) {
      case 0: digitalWrite(LED1, HIGH); break;
      case 1: digitalWrite(LED2, HIGH); break;
      case 2: digitalWrite(LED3, HIGH); break;
    }
  }
}

void startupSequence() {
  for(int i = 0; i < 2; i++) {
    digitalWrite(LED1, HIGH);
    delay(100);
    digitalWrite(LED2, HIGH);
    delay(100);
    digitalWrite(LED3, HIGH);
    delay(100);
    digitalWrite(LED4, HIGH);
    delay(100);
    
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);
    digitalWrite(LED4, LOW);
    delay(150);
  }
}

void initMPU6050() {
  // Despertar MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  
  // Configurar filtro pasa-bajos interno del MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1A); // CONFIG
  Wire.write(0x03); // DLPF_CFG = 3 (44Hz)
  Wire.endTransmission(true);
  
  // Configurar rango del acelerómetro a ±2g
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true);
  
  // Configurar sample rate a 100Hz
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x19); // SMPLRT_DIV
  Wire.write(0x07); // Sample Rate = 1kHz/(1+7) = 125Hz
  Wire.endTransmission(true);
}

void readMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  
  accelX = Wire.read() << 8 | Wire.read();
  accelY = Wire.read() << 8 | Wire.read();
  accelZ = Wire.read() << 8 | Wire.read();
  
  // Saltar temperatura
  Wire.read();
  Wire.read();
  
  // Leer giroscopio (por si se necesita después)
  int16_t gyroX = Wire.read() << 8 | Wire.read();
  int16_t gyroY = Wire.read() << 8 | Wire.read();
  int16_t gyroZ = Wire.read() << 8 | Wire.read();
}
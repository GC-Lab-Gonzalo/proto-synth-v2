
// ==============================================================================================================================================
// PROTO-SYNTH V2 - EmotionTrigger - GC Lab Chile
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
// Controlador MIDI gestual que utiliza el acelerómetro MPU6050 para disparar 
// emociones musicales mediante golpes e impactos bruscos. Incluye 4 escalas modales 
// emotivas con control completo de expresión musical a través de potenciómetros.
// ==============================================================================================================================================

// ==============================================================================================================================================
// FUNCIONAMIENTO
// ==============================================================================================================================================
// CONTROLES DE EXPRESIÓN:
// - Potenciómetro 1: Volumen/velocidad MIDI (0-127)
// - Potenciómetro 2: Aleatoriedad de notas (0-100%)
// - Potenciómetro 3: Rango inferior - nota más grave (C1-C5)
// - Potenciómetro 4: Rango superior - nota más aguda (C4-Bb6)
// - Boton 1: Cambio a escala frigia (Melancólica, misteriosa, español)
// - Boton 2: Cambio a escala mixolidia (Alegre, rockera, festiva)
// - Boton 3: Cambio a escala lidia (Mágica, etérea, luminosa)
// - Boton 4: Cambio a escala eolica (Triste, emotiva, nostálgica)
// - LED 1: Indica escala frigia activa
// - LED 2: Indica escala mixolidia activa
// - LED 3: Indica escala lidia activa
// - LED 4: Indica escala eolica activa
// - IMU: Detecta impactos bruscos para disparar notas MIDI
// - LDR: No se usa
// - Micrófono: No se usa
// - Header 1: No se usa
// - Header 2: No se usa
// - Salida MIDI: Envía notas MIDI trigger según impactos y configuración
//
// MODO DE USO:
// 1. Conectar Proto-Synth a DAW/sintetizador via MIDI (o por USB MIDI serial utilizando Hairless MIDI y cambiando el baudio de 31250 a 115200)
// 2. Seleccionar escala emocional con botones (LED indica modo activo)
// 3. Ajustar volumen con potenciómetro 1
// 4. Configurar rango de notas con potenciómetros 3 y 4
// 5. Ajustar aleatoriedad con potenciómetro 2
// 6. Realizar golpes/impactos con la placa para disparar emociones musicales
//
// INFORMACIÓN DEL CODIGO:
// - La intensidad del impacto afecta la velocidad y/o volumen de las notas.
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
#include <Wire.h>

// ==============================================================================================================================================
// CONFIGURACIÓN DE HARDWARE - PINES
// ==============================================================================================================================================
#define MPU6050_ADDR 0x68 // Dirección I2C del MPU6050

#define BUTTON1_PIN 18    // Botón 1
#define BUTTON2_PIN 4     // Botón 2 
#define BUTTON3_PIN 15    // Botón 3
#define BUTTON4_PIN 19    // Botón 4

#define LED1_PIN 23       // LED 1
#define LED2_PIN 32       // LED 2
#define LED3_PIN 5        // LED 3
#define LED4_PIN 2        // LED 4

#define POT1_PIN 13       // Potenciómetro 1
#define POT2_PIN 14       // Potenciómetro 2
#define POT3_PIN 12       // Potenciómetro 3
#define POT4_PIN 27       // Potenciómetro 4

#define MOVEMENT_THRESHOLD 15000 // Umbral para detectar impactos
#define NOTE_DURATION 150  // Duración de notas trigger

// ==============================================================================================================================================
// PROGRAMA
// ==============================================================================================================================================

// ESCALAS EMOCIONALES - 4 Modos diferentes
enum ScaleMode {
  PHRYGIAN_MODE = 0,    // Frigio
  MIXOLYDIAN_MODE,      // Mixolidio
  LYDIAN_MODE,          // Lidio
  AEOLIAN_MODE          // Eólico (Menor Natural)
};

ScaleMode currentMode = PHRYGIAN_MODE;

// Escalas definidas con características emotivas - RANGO COMPLETO MIDI
// Modo Frigio
int phrygianNotes[] = {
  // Octava 1-6: 1, b2, b3, 4, 5, b6, b7
  24, 25, 27, 29, 31, 32, 34,  // C1, Db1, Eb1, F1, G1, Ab1, Bb1
  36, 37, 39, 41, 43, 44, 46,  // C2, Db2, Eb2, F2, G2, Ab2, Bb2
  48, 49, 51, 53, 55, 56, 58,  // C3, Db3, Eb3, F3, G3, Ab3, Bb3
  60, 61, 63, 65, 67, 68, 70,  // C4, Db4, Eb4, F4, G4, Ab4, Bb4
  72, 73, 75, 77, 79, 80, 82,  // C5, Db5, Eb5, F5, G5, Ab5, Bb5
  84, 85, 87, 89, 91, 92, 94   // C6, Db6, Eb6, F6, G6, Ab6, Bb6
};

// Modo Mixolidio
int mixolydianNotes[] = {
  // Octava 1-6: 1, 2, 3, 4, 5, 6, b7
  24, 26, 28, 29, 31, 33, 34,  // C1, D1, E1, F1, G1, A1, Bb1
  36, 38, 40, 41, 43, 45, 46,  // C2, D2, E2, F2, G2, A2, Bb2
  48, 50, 52, 53, 55, 57, 58,  // C3, D3, E3, F3, G3, A3, Bb3
  60, 62, 64, 65, 67, 69, 70,  // C4, D4, E4, F4, G4, A4, Bb4
  72, 74, 76, 77, 79, 81, 82,  // C5, D5, E5, F5, G5, A5, Bb5
  84, 86, 88, 89, 91, 93, 94   // C6, D6, E6, F6, G6, A6, Bb6
};

// Modo Lidio
int lydianNotes[] = {
  // Octava 1-6: 1, 2, 3, #4, 5, 6, 7
  24, 26, 28, 30, 31, 33, 35,  // C1, D1, E1, F#1, G1, A1, B1
  36, 38, 40, 42, 43, 45, 47,  // C2, D2, E2, F#2, G2, A2, B2
  48, 50, 52, 54, 55, 57, 59,  // C3, D3, E3, F#3, G3, A3, B3
  60, 62, 64, 66, 67, 69, 71,  // C4, D4, E4, F#4, G4, A4, B4
  72, 74, 76, 78, 79, 81, 83,  // C5, D5, E5, F#5, G5, A5, B5
  84, 86, 88, 90, 91, 93, 95   // C6, D6, E6, F#6, G6, A6, B6
};

// Modo Eólico
int aeolianNotes[] = {
  // Octava 1-6: 1, 2, b3, 4, 5, b6, b7
  24, 26, 27, 29, 31, 32, 34,  // C1, D1, Eb1, F1, G1, Ab1, Bb1
  36, 38, 39, 41, 43, 44, 46,  // C2, D2, Eb2, F2, G2, Ab2, Bb2
  48, 50, 51, 53, 55, 56, 58,  // C3, D3, Eb3, F3, G3, Ab3, Bb3
  60, 62, 63, 65, 67, 68, 70,  // C4, D4, Eb4, F4, G4, Ab4, Bb4
  72, 74, 75, 77, 79, 80, 82,  // C5, D5, Eb5, F5, G5, Ab5, Bb5
  84, 86, 87, 89, 91, 92, 94   // C6, D6, Eb6, F6, G6, A6, Bb6
};

// Variables para lectura del MPU6050
int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;
float totalAccel = 0;
float prevTotalAccel = 0;
float accelDiff = 0;

// Variables para control de notas
unsigned long lastNoteTime = 0;
unsigned long minNoteDuration = 50;
bool canPlayNote = true;

// Variables para potenciómetros (valores cacheados para evitar ruido)
int volumeValue = 64;         // Volumen (0-127)
int randomnessValue = 50;     // Aleatoriedad (0-100%)
int lowRangeValue = 24;       // Nota más grave (C1=24)
int highRangeValue = 94;      // Nota más aguda (Bb6=94)
unsigned long lastPotRead = 0;
#define POT_READ_INTERVAL 50  // Leer potenciómetros cada 50ms

// Variables para manejo de Note Off automático
struct ActiveNote {
  int note;
  unsigned long noteOffTime;
  bool active;
};

#define MAX_ACTIVE_NOTES 8
ActiveNote activeNotes[MAX_ACTIVE_NOTES];

// Variables para botones (debounce)
unsigned long lastButtonPress = 0;
bool buttonPressed[4] = {false, false, false, false};

void setup() {
  Serial.begin(31250);  // Velocidad estándar para MIDI
  
  // Configurar pines de entrada (botones con pull-up interno)
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(BUTTON3_PIN, INPUT_PULLUP);
  pinMode(BUTTON4_PIN, INPUT_PULLUP);
  
  // Configurar pines de salida (LEDs)
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(LED4_PIN, OUTPUT);
  
  // Inicializar I2C
  Wire.begin(21, 22);
  Wire.setClock(400000);
  
  // Inicializar MPU6050
  initMPU6050();
  
  // Inicializar generador de números aleatorios
  randomSeed(analogRead(0));
  
  // Inicializar array de notas activas
  for (int i = 0; i < MAX_ACTIVE_NOTES; i++) {
    activeNotes[i].active = false;
  }
  
  // Mostrar modo inicial
  updateLEDs();
  
  // Enviar All Notes Off al inicio
  sendAllNotesOff();
  
  // Mensaje de inicio con información de modos
  delay(1000);
  printModeInfo();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Leer potenciómetros periódicamente para reducir ruido
  if (currentTime - lastPotRead >= POT_READ_INTERVAL) {
    readPotentiometers();
    lastPotRead = currentTime;
  }
  
  // Verificar botones para cambio de modo
  checkButtons(currentTime);
  
  // Verificar y enviar Note Off para notas que han expirado
  checkAndSendNoteOff(currentTime);
  
  // Leer datos del MPU6050
  readMPU6050();
  
  // Calcular magnitud total de aceleración
  totalAccel = sqrt((accelX * accelX) + (accelY * accelY) + (accelZ * accelZ));
  
  // Calcular diferencia con la lectura anterior
  accelDiff = abs(totalAccel - prevTotalAccel);
  
  // Detectar impacto brusco y verificar si puede tocar nota
  if (accelDiff > MOVEMENT_THRESHOLD && canPlayNote) {
    if (currentTime - lastNoteTime >= 20) {  // Mínimo 20ms entre notas
      lastNoteTime = currentTime;
      
      // Obtener nota según el modo actual y configuración de potenciómetros
      int note = getRandomNoteFromCurrentMode();
      
      // Aplicar volumen del potenciómetro 1
      int baseVelocity = map(constrain(accelDiff, MOVEMENT_THRESHOLD, MOVEMENT_THRESHOLD * 3), 
                        MOVEMENT_THRESHOLD, MOVEMENT_THRESHOLD * 3, 40, 127);
      
      // Modular con el volumen del potenciómetro (volumeValue ya está en 0-127)
      int velocity = map(baseVelocity, 40, 127, 
                        volumeValue * 40 / 127,  // Mínimo proporcional
                        volumeValue);            // Máximo = volumeValue
      
      velocity = constrain(velocity, 1, 127);  // Asegurar rango MIDI válido
      
      // Enviar nota MIDI con Note Off programado
      sendMidiNoteTrigger(note, velocity, currentTime);
      
      canPlayNote = false;
    }
  }
  
  // Permitir nueva nota cuando el impacto se calme
  if (accelDiff < MOVEMENT_THRESHOLD * 0.3) {
    canPlayNote = true;
  }
  
  prevTotalAccel = totalAccel;
}

void readPotentiometers() {
  // Leer potenciómetros (invertidos: 4095 a 0)
  int pot1Raw = 4095 - analogRead(POT1_PIN);  // Volumen
  int pot2Raw = 4095 - analogRead(POT2_PIN);  // Aleatoriedad  
  int pot3Raw = 4095 - analogRead(POT3_PIN);  // Rango inferior
  int pot4Raw = 4095 - analogRead(POT4_PIN);  // Rango superior
  
  // Potenciómetro 1: Volumen (0-127)
  volumeValue = map(pot1Raw, 0, 4095, 0, 127);
  
  // Potenciómetro 2: Aleatoriedad (0-100%)
  randomnessValue = map(pot2Raw, 0, 4095, 0, 100);
  
  // Potenciómetro 3: Rango inferior (C1=24 a C5=72)
  lowRangeValue = map(pot3Raw, 0, 4095, 24, 72);
  
  // Potenciómetro 4: Rango superior (C4=60 a Bb6=94)
  highRangeValue = map(pot4Raw, 0, 4095, 60, 94);
  
  // Asegurar que el rango superior sea mayor que el inferior
  if (highRangeValue <= lowRangeValue) {
    highRangeValue = lowRangeValue + 12;  // Al menos una octava de diferencia
    if (highRangeValue > 94) {
      highRangeValue = 94;
      lowRangeValue = 82;  // Bb5
    }
  }
}

void checkButtons(unsigned long currentTime) {
  // Debounce de 200ms
  if (currentTime - lastButtonPress < 200) return;
  
  bool button1 = !digitalRead(BUTTON1_PIN);
  bool button2 = !digitalRead(BUTTON2_PIN);
  bool button3 = !digitalRead(BUTTON3_PIN);
  bool button4 = !digitalRead(BUTTON4_PIN);
  
  ScaleMode newMode = currentMode;
  
  if (button1 && !buttonPressed[0]) {
    newMode = PHRYGIAN_MODE;
    buttonPressed[0] = true;
  } else if (button2 && !buttonPressed[1]) {
    newMode = MIXOLYDIAN_MODE;
    buttonPressed[1] = true;
  } else if (button3 && !buttonPressed[2]) {
    newMode = LYDIAN_MODE;
    buttonPressed[2] = true;
  } else if (button4 && !buttonPressed[3]) {
    newMode = AEOLIAN_MODE;
    buttonPressed[3] = true;
  }
  
  // Actualizar estado de botones
  buttonPressed[0] = button1;
  buttonPressed[1] = button2;
  buttonPressed[2] = button3;
  buttonPressed[3] = button4;
  
  if (newMode != currentMode) {
    currentMode = newMode;
    lastButtonPress = currentTime;
    updateLEDs();
    sendAllNotesOff();  // Limpiar notas al cambiar modo
    printModeInfo();
  }
}

void updateLEDs() {
  // Apagar todos los LEDs
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  digitalWrite(LED3_PIN, LOW);
  digitalWrite(LED4_PIN, LOW);
  
  // Encender LED según modo actual
  switch (currentMode) {
    case PHRYGIAN_MODE:
      digitalWrite(LED1_PIN, HIGH);
      break;
    case MIXOLYDIAN_MODE:
      digitalWrite(LED2_PIN, HIGH);
      break;
    case LYDIAN_MODE:
      digitalWrite(LED3_PIN, HIGH);
      break;
    case AEOLIAN_MODE:
      digitalWrite(LED4_PIN, HIGH);
      break;
  }
}

int getRandomNoteFromCurrentMode() {
  int* currentScale;
  int scaleSize;
  
  switch (currentMode) {
    case PHRYGIAN_MODE:
      currentScale = phrygianNotes;
      scaleSize = sizeof(phrygianNotes) / sizeof(phrygianNotes[0]);
      break;
    case MIXOLYDIAN_MODE:
      currentScale = mixolydianNotes;
      scaleSize = sizeof(mixolydianNotes) / sizeof(mixolydianNotes[0]);
      break;
    case LYDIAN_MODE:
      currentScale = lydianNotes;
      scaleSize = sizeof(lydianNotes) / sizeof(lydianNotes[0]);
      break;
    case AEOLIAN_MODE:
      currentScale = aeolianNotes;
      scaleSize = sizeof(aeolianNotes) / sizeof(aeolianNotes[0]);
      break;
  }
  
  // Crear array de notas filtradas por rango
  int validNotes[42];  // Máximo posible (6 octavas x 7 notas)
  int validCount = 0;
  
  for (int i = 0; i < scaleSize; i++) {
    if (currentScale[i] >= lowRangeValue && currentScale[i] <= highRangeValue) {
      validNotes[validCount] = currentScale[i];
      validCount++;
    }
  }
  
  // Si no hay notas válidas, usar toda la escala
  if (validCount == 0) {
    return currentScale[random(0, scaleSize)];
  }
  
  // Aplicar aleatoriedad basada en potenciómetro 2
  int selectedNote;
  
  if (randomnessValue >= 95) {
    // Muy aleatorio: cualquier nota del rango
    selectedNote = validNotes[random(0, validCount)];
  } else if (randomnessValue >= 70) {
    // Bastante aleatorio: favorece ligeramente notas centrales
    int center = validCount / 2;
    int range = max(1, validCount / 3);
    int index = constrain(center + random(-range, range + 1), 0, validCount - 1);
    selectedNote = validNotes[index];
  } else if (randomnessValue >= 30) {
    // Moderadamente aleatorio: favorece notas medias del rango
    int center = validCount / 2;
    int range = max(1, validCount / 4);
    int index = constrain(center + random(-range, range + 1), 0, validCount - 1);
    selectedNote = validNotes[index];
  } else {
    // Poco aleatorio: favorece mucho las notas centrales del rango
    int center = validCount / 2;
    int range = max(1, validCount / 6);
    int index = constrain(center + random(-range, range + 1), 0, validCount - 1);
    selectedNote = validNotes[index];
  }
  
  return selectedNote;
}

void printModeInfo() {
  Serial.println();
  switch (currentMode) {
    case PHRYGIAN_MODE:
      Serial.println("MODO 1: FRIGIO - Melancólico, Misterioso, Español");
      break;
    case MIXOLYDIAN_MODE:
      Serial.println("MODO 2: MIXOLIDIO - Alegre, Rockero, Festivo");
      break;
    case LYDIAN_MODE:
      Serial.println("MODO 3: LIDIO - Mágico, Etéreo, Luminoso");
      break;
    case AEOLIAN_MODE:
      Serial.println("MODO 4: EÓLICO - Triste, Emotivo, Nostálgico");
      break;
  }
}

void initMPU6050() {
  // Despertar el MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);  // Registro PWR_MGMT_1
  Wire.write(0);     // Despertar dispositivo
  Wire.endTransmission();
  
  // Configurar rango del acelerómetro (±2g)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);  // Registro ACCEL_CONFIG
  Wire.write(0x00);  // ±2g
  Wire.endTransmission();
  
  // Configurar rango del giroscopio (±250°/s)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);  // Registro GYRO_CONFIG
  Wire.write(0x00);  // ±250°/s
  Wire.endTransmission();
  
  delay(100);  // Tiempo de estabilización
}

void readMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);  // Registro de inicio de acelerómetro
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);  // Leer 14 bytes
  
  // Leer acelerómetro (6 bytes)
  accelX = Wire.read() << 8 | Wire.read();
  accelY = Wire.read() << 8 | Wire.read();
  accelZ = Wire.read() << 8 | Wire.read();
  
  // Saltar temperatura (2 bytes)
  Wire.read();
  Wire.read();
  
  // Leer giroscopio (6 bytes)
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
}

void sendMidiNoteTrigger(int note, int velocity, unsigned long currentTime) {
  // Buscar un slot libre para la nueva nota
  int freeSlot = -1;
  for (int i = 0; i < MAX_ACTIVE_NOTES; i++) {
    if (!activeNotes[i].active) {
      freeSlot = i;
      break;
    }
  }
  
  // Si no hay slots libres, usar el más antiguo
  if (freeSlot == -1) {
    freeSlot = 0;
    unsigned long oldestTime = activeNotes[0].noteOffTime;
    for (int i = 1; i < MAX_ACTIVE_NOTES; i++) {
      if (activeNotes[i].noteOffTime < oldestTime) {
        oldestTime = activeNotes[i].noteOffTime;
        freeSlot = i;
      }
    }
    // Enviar Note Off de la nota que vamos a reemplazar
    if (activeNotes[freeSlot].active) {
      sendNoteOff(activeNotes[freeSlot].note);
    }
  }
  
  // Enviar Note On
  Serial.write(0x90);     // Note On, canal 1
  Serial.write(note);     // Nota
  Serial.write(velocity); // Velocidad
  
  // Programar el Note Off
  activeNotes[freeSlot].note = note;
  activeNotes[freeSlot].noteOffTime = currentTime + NOTE_DURATION;
  activeNotes[freeSlot].active = true;
}

void checkAndSendNoteOff(unsigned long currentTime) {
  for (int i = 0; i < MAX_ACTIVE_NOTES; i++) {
    if (activeNotes[i].active && currentTime >= activeNotes[i].noteOffTime) {
      // Tiempo de enviar Note Off
      sendNoteOff(activeNotes[i].note);
      activeNotes[i].active = false;
    }
  }
}

void sendNoteOff(int note) {
  Serial.write(0x80);  // Note Off, canal 1
  Serial.write(note);  // Nota
  Serial.write(0);     // Velocidad 0
}

void sendAllNotesOff() {
  // Enviar All Notes Off (CC 123)
  Serial.write(0xB0);  // Control Change, canal 1
  Serial.write(123);   // All Notes Off
  Serial.write(0);     // Valor
  
  delay(10);
  
  // También enviar All Sound Off por seguridad
  Serial.write(0xB0);  // Control Change, canal 1
  Serial.write(120);   // All Sound Off
  Serial.write(0);     // Valor
}
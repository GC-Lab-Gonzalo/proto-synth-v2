
// ==============================================================================================================================================
// PROTO-SYNTH V2 - MONITOREO - GC Lab Chile
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
// Este código implementa un sistema de monitoreo para el Proto-Synth V2. 
// Permite interactuar con botones, potenciómetros, un sensor de luz, un micrófono y un sensor de movimiento MPU6050. 
// Los datos recopilados se envían al monitor serial para visualización en tiempo real. 
// ==============================================================================================================================================

// ==============================================================================================================================================
// FUNCIONAMIENTO
// ==============================================================================================================================================
// CONTROLES DE EXPRESIÓN:
// - Potenciómetro 1: Solo Diagnostico
// - Potenciómetro 2: Solo Diagnostico
// - Potenciómetro 3: Solo Diagnostico 
// - Potenciómetro 4: Solo Diagnostico
// - Botón 1: Solo Diagnostico
// - Botón 2: Solo Diagnostico
// - Botón 3: Solo Diagnostico
// - Botón 4: Solo Diagnostico
// - LED 1: Solo Diagnostico
// - LED 2: Solo Diagnostico
// - LED 3: Solo Diagnostico
// - LED 4: Solo Diagnostico
// - IMU: Solo Diagnostico
// - LDR: Solo Diagnostico
// - Micrófono: Solo Diagnostico
// - Header 1: No se usa
// - Header 2: No se usa
// - Salida MIDI: No se usa
//
// MODO DE USO:
// 1. Carga el codigo
// 2. Abre el monitor serial o monitor plotter
// 3. Mueve los potenciometros y presiona los botones
// 4. Observa como reacciona el monitor serial
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
#define BOTON_1 18
#define BOTON_2 4
#define BOTON_3 15
#define BOTON_4 19

#define POT_1 13    // Pin analógico
#define POT_2 14    // Pin analógico
#define POT_3 12    // Pin analógico
#define POT_4 27    // Pin analógico

#define LDR_PIN 26  // Pin analógico
#define MIC_PIN 33  // Pin analógico

#define LED_1 23
#define LED_2 32
#define LED_3 5
#define LED_4 2

// Dirección I2C del MPU6050
#define MPU6050_ADDR 0x68

// ==============================================================================================================================================
// PROGRAMA
// ==============================================================================================================================================
// Variables para almacenar lecturas
int boton1, boton2, boton3, boton4;
int pot1, pot2, pot3, pot4;
int ldrValue, micValue;
int16_t ax, ay, az, gx, gy, gz, temp;

// Variables para el micrófono
int micMin = 4095, micMax = 0;
int micBaseline = 2048;
bool micCalibrated = false;

// Variables para diagnóstico
bool mpuConnected = false;
unsigned long lastDiagnostic = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== INICIANDO PROTO-SYNTH MONITOR ===");
  
  // Configurar pines de botones con pull-up interno
  pinMode(BOTON_1, INPUT_PULLUP);
  pinMode(BOTON_2, INPUT_PULLUP);
  pinMode(BOTON_3, INPUT_PULLUP);
  pinMode(BOTON_4, INPUT_PULLUP);
  
  // Configurar LEDs como salida
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);
  pinMode(LED_4, OUTPUT);
  
  // Configurar resolución ADC a 12 bits
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db); // Para rango completo de voltaje
  
  // Inicializar I2C
  Wire.begin();
  Wire.setClock(400000); // Velocidad rápida I2C
  
  // Configurar MPU6050
  setupMPU6050();
  
  // Calibrar micrófono
  calibrateMicrophone();
  
  Serial.println("Configuración completada. Iniciando monitoreo...");
  Serial.println("Btn1,Btn2,Btn3,Btn4,Pot1,Pot2,Pot3,Pot4,LDR,MicLevel,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ");
  
  delay(500);
}

void loop() {
  // Leer botones (invertir lógica porque usan pull-up)
  boton1 = !digitalRead(BOTON_1) ? 1000 : 0;
  boton2 = !digitalRead(BOTON_2) ? 1000 : 0;
  boton3 = !digitalRead(BOTON_3) ? 1000 : 0;
  boton4 = !digitalRead(BOTON_4) ? 1000 : 0;
  
  // Leer potenciómetros
  pot1 = analogRead(POT_1);
  pot2 = analogRead(POT_2);
  pot3 = analogRead(POT_3);
  pot4 = analogRead(POT_4);
  
  // Leer LDR con múltiples muestras para estabilidad
  ldrValue = readStableAnalog(LDR_PIN, 5);
  
  // Leer micrófono y calcular nivel de audio
  micValue = readMicrophoneLevel();
  
  // Leer datos del MPU6050 si está conectado
  if (mpuConnected) {
    readMPU6050();
  } else {
    // Valores por defecto si no hay MPU6050
    ax = ay = az = gx = gy = gz = 0;
  }
  
  // Control visual de LEDs
  digitalWrite(LED_1, boton1 > 0);
  digitalWrite(LED_2, boton2 > 0);
  digitalWrite(LED_3, pot1 > 2000);
  digitalWrite(LED_4, ldrValue > 2000);
  
  // Enviar datos al Serial Plotter
  Serial.print(boton1); Serial.print(",");
  Serial.print(boton2); Serial.print(",");
  Serial.print(boton3); Serial.print(",");
  Serial.print(boton4); Serial.print(",");
  Serial.print(pot1); Serial.print(",");
  Serial.print(pot2); Serial.print(",");
  Serial.print(pot3); Serial.print(",");
  Serial.print(pot4); Serial.print(",");
  Serial.print(ldrValue); Serial.print(",");
  Serial.print(micValue); Serial.print(",");
  Serial.print(map(ax, -32768, 32767, 0, 4095)); Serial.print(",");
  Serial.print(map(ay, -32768, 32767, 0, 4095)); Serial.print(",");
  Serial.print(map(az, -32768, 32767, 0, 4095)); Serial.print(",");
  Serial.print(map(gx, -32768, 32767, 0, 2048)); Serial.print(",");
  Serial.print(map(gy, -32768, 32767, 0, 2048)); Serial.print(",");
  Serial.println(map(gz, -32768, 32767, 0, 2048));
  
  // Diagnóstico periódico cada 5 segundos
  if (millis() - lastDiagnostic > 5000) {
    printDiagnostic();
    lastDiagnostic = millis();
  }
  
  delay(50);
}

void setupMPU6050() {
  Serial.print("Configurando MPU6050... ");
  
  // Despertar el MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // Registro PWR_MGMT_1
  Wire.write(0x00); // Despertar dispositivo
  byte error = Wire.endTransmission();
  
  if (error == 0) {
    delay(100);
    
    // Configurar acelerómetro (±2g)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1C); // Registro ACCEL_CONFIG
    Wire.write(0x00); // ±2g
    Wire.endTransmission();
    
    // Configurar giroscopio (±250°/s)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1B); // Registro GYRO_CONFIG
    Wire.write(0x00); // ±250°/s
    Wire.endTransmission();
    
    // Configurar filtro pasa-bajos
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1A); // Registro CONFIG
    Wire.write(0x03); // Filtro pasa-bajos a 44Hz
    Wire.endTransmission();
    
    mpuConnected = true;
    Serial.println("OK");
  } else {
    Serial.println("FALLO - Verifique conexiones I2C");
    mpuConnected = false;
  }
}

void readMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B); // Registro inicial de datos del acelerómetro
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  
  if (Wire.available() >= 14) {
    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();
    temp = (Wire.read() << 8) | Wire.read(); // Temperatura (no usada)
    gx = (Wire.read() << 8) | Wire.read();
    gy = (Wire.read() << 8) | Wire.read();
    gz = (Wire.read() << 8) | Wire.read();
  }
}

int readStableAnalog(int pin, int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(100);
  }
  return sum / samples;
}

void calibrateMicrophone() {
  Serial.print("Calibrando micrófono... ");
  
  // Tomar 100 muestras para calibrar el nivel base
  long sum = 0;
  micMin = 4095;
  micMax = 0;
  
  for (int i = 0; i < 100; i++) {
    int reading = analogRead(MIC_PIN);
    sum += reading;
    if (reading < micMin) micMin = reading;
    if (reading > micMax) micMax = reading;
    delay(10);
  }
  
  micBaseline = sum / 100;
  Serial.printf("OK (Base: %d, Min: %d, Max: %d)\n", micBaseline, micMin, micMax);
  micCalibrated = true;
}

int readMicrophoneLevel() {
  if (!micCalibrated) return 0;
  
  // Tomar múltiples muestras rápidas
  int maxDeviation = 0;
  int currentReading;
  
  for (int i = 0; i < 10; i++) {
    currentReading = analogRead(MIC_PIN);
    int deviation = abs(currentReading - micBaseline);
    if (deviation > maxDeviation) {
      maxDeviation = deviation;
    }
    delayMicroseconds(100);
  }
  
  // Mapear la desviación máxima a un rango visible
  return constrain(map(maxDeviation, 0, 500, 0, 4095), 0, 4095);
}

void printDiagnostic() {
  Serial.println("\n=== DIAGNÓSTICO ===");
  Serial.printf("MPU6050: %s\n", mpuConnected ? "Conectado" : "Desconectado");
  Serial.printf("LDR: %d (0-4095)\n", ldrValue);
  Serial.printf("Mic Base: %d, Nivel: %d\n", micBaseline, micValue);
  Serial.printf("Acelerómetro: X=%d Y=%d Z=%d\n", ax, ay, az);
  Serial.printf("Giroscopio: X=%d Y=%d Z=%d\n", gx, gy, gz);
  Serial.println("==================\n");
}

// Función para probar conexiones manualmente
void testConnections() {
  Serial.println("\n=== PRUEBA DE CONEXIONES ===");
  
  // Probar LDR
  Serial.print("LDR (Pin 26): ");
  int ldrTest = analogRead(LDR_PIN);
  Serial.printf("%d ", ldrTest);
  Serial.println(ldrTest > 10 ? "OK" : "PROBLEMA");
  
  // Probar Micrófono
  Serial.print("Micrófono (Pin 33): ");
  int micTest = analogRead(MIC_PIN);
  Serial.printf("%d ", micTest);
  Serial.println(micTest > 10 ? "OK" : "PROBLEMA");
  
  // Probar MPU6050
  Serial.print("MPU6050 I2C: ");
  Wire.beginTransmission(MPU6050_ADDR);
  byte error = Wire.endTransmission();
  Serial.println(error == 0 ? "OK" : "PROBLEMA");
  
  Serial.println("============================\n");
}
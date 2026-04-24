
// ==============================================================================================================================================
// PROTO-SYNTH V2 - VIDEO SYNTH CONTROLLER - GC Lab Chile
// ==============================================================================================================================================
// Desarrollado por: GC Lab Chile
// Licencia de Software: MIT License (https://opensource.org/licenses/MIT)
// Licencia de Hardware: CERN Open Hardware Licence v2 - Permissive (CERN-OHL-P)
//
// Puedes usar, modificar y distribuir este código y hardware, siempre que se mantenga
// la atribución a GC Lab Chile. Se entrega "tal cual", sin garantías de ningún tipo.
// ==============================================================================================================================================
// REPOSITORIO: https://github.com/GC-Lab-Gonzalo/proto-synth-v2
// ==============================================================================================================================================
// HARDWARE
// ==============================================================================================================================================
// - Microcontrolador ESP32 DevKit
// - 4 Botones con pull-up       |1 -> PIN 18, 2 -> PIN 4,  3 -> PIN 15, 4 -> PIN 19|
// - 4 LEDs indicadores          |1 -> PIN 23, 2 -> PIN 32, 3 -> PIN 5,  4 -> PIN 2 |
// - 4 Potenciómetros analógicos |1 -> PIN 13, 2 -> PIN 14, 3 -> PIN 12, 4 -> PIN 27|
// ==============================================================================================================================================

// ==============================================================================================================================================
// DESCRIPCIÓN
// ==============================================================================================================================================
// Firmware controlador para el Video Sample Synth. El Proto-Synth v2 actúa como interfaz
// física enviando el estado de los potenciómetros y botones vía serial (115200 baud) al
// sintetizador de video corriendo en PC, y recibiendo comandos para controlar los LEDs
// como feedback visual del estado del sintetizador.
// ==============================================================================================================================================

// ==============================================================================================================================================
// FUNCIONAMIENTO
// ==============================================================================================================================================
// CONTROLES DE EXPRESIÓN:
// - Potenciómetro 1 (PIN 13): Posición/scrub en el video
// - Potenciómetro 2 (PIN 14): Tamaño del grano (grain size)
// - Potenciómetro 3 (PIN 12): Dispersión del grano (scatter)
// - Potenciómetro 4 (PIN 27): Volumen de audio
// - Botón 1 (PIN 18): Play/Pause
// - Botón 2 (PIN 4):  Cambiar modo de efecto (GRANULAR → STUTTER → REVERSE → SCAN)
// - Botón 3 (PIN 15): Freeze frame
// - Botón 4 (PIN 19): Trigger manual de muestra
// - LED 1 (PIN 23): Estado Play/Pause (encendido = reproduciendo)
// - LED 2 (PIN 32): Cambio de modo (parpadea al cambiar)
// - LED 3 (PIN 5):  Freeze activo
// - LED 4 (PIN 2):  Trigger de muestra / actividad
//
// PROTOCOLO SERIAL (115200 baud):
//   ENVÍA:   pot1,pot2,pot3,pot4,btn1,btn2,btn3,btn4,imu_x\n  (aprox. 50 Hz)
//            imu_x: 0=inclinado izq (cam0) | 2048=nivelado (mezcla) | 4095=inclinado der (cam1)
//   RECIBE:  L:led1,led2,led3,led4\n  (feedback desde PC para controlar LEDs)
//
// MODO DE USO:
// 1. Asegúrate de que el Potenciómetro 3 esté girado al máximo antes de cargar
// 2. Carga el firmware en el ESP32
// 3. En la PC ejecuta: python video_synth.py <video.mp4> [COMX]
// ==============================================================================================================================================

// ==============================================================================================================================================
// COMENTARIOS
// ==============================================================================================================================================
// - Para subir código exitosamente, asegúrate de que el Potenciómetro 3 esté girado al máximo.
// - Los Pines 2,4,12,13,14,15,25,26,27 no van a funcionar si el Bluetooth/WiFi están activados
//   ya que están conectados al ADC2 del ESP32.
// - Los potenciómetros están invertidos en el hardware v2.0: se usa (4095 - analogRead) para
//   corregir la dirección de giro.
// - El suavizado EMA (alpha=0.15) reduce el ruido del ADC sin agregar latencia perceptible.
// ==============================================================================================================================================

// ==============================================================================================================================================
// CONFIGURACIÓN DE HARDWARE - PINES
// ==============================================================================================================================================
#define BOTON_1 18
#define BOTON_2 4
#define BOTON_3 15
#define BOTON_4 19

#define POT_1 13
#define POT_2 14
#define POT_3 12
#define POT_4 27

#define LED_1 23
#define LED_2 32
#define LED_3 5
#define LED_4 2

// ==============================================================================================================================================
// PROGRAMA
// ==============================================================================================================================================

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;
bool mpuOk = false;

// Suavizado exponencial (EMA) para potenciómetros y IMU
const float ALPHA_EMA = 0.15f;
float emaImuX = 2048.0f;
float emaPot1 = 0.0f;
float emaPot2 = 0.0f;
float emaPot3 = 0.0f;
float emaPot4 = 0.0f;

// Estado de botones (para detectar flancos)
int estadoBtn1 = 1, estadoBtn2 = 1, estadoBtn3 = 1, estadoBtn4 = 1;

// Buffer para recepción de comandos LED (lectura no-bloqueante)
String bufferSerial = "";

// Tiempo del último envío
unsigned long tiempoUltimoEnvio = 0;
const unsigned long INTERVALO_ENVIO = 20; // 20ms → 50Hz

// Pines de LEDs para iterar en la animación de inicio
const int pinesLeds[4] = {LED_1, LED_2, LED_3, LED_4};

// ==============================================================================================================================================
// SETUP
// ==============================================================================================================================================
void setup() {
  Serial.begin(115200);

  // Configurar botones con pull-up interno
  pinMode(BOTON_1, INPUT_PULLUP);
  pinMode(BOTON_2, INPUT_PULLUP);
  pinMode(BOTON_3, INPUT_PULLUP);
  pinMode(BOTON_4, INPUT_PULLUP);

  // Configurar LEDs como salida
  for (int i = 0; i < 4; i++) {
    pinMode(pinesLeds[i], OUTPUT);
    digitalWrite(pinesLeds[i], LOW);
  }

  // Configurar ADC a 12 bits (0-4095) con atenuación máxima para rango completo
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // Inicializar EMA con primera lectura real
  emaPot1 = 4095 - analogRead(POT_1);
  emaPot2 = 4095 - analogRead(POT_2);
  emaPot3 = 4095 - analogRead(POT_3);
  emaPot4 = 4095 - analogRead(POT_4);

  // Inicializar MPU6050 (I2C: SDA=21, SCL=22)
  Wire.begin(21, 22);
  mpuOk = mpu.begin();
  if (mpuOk) {
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  // Animación de inicio: barrido secuencial de LEDs
  animacionInicio();
}

// ==============================================================================================================================================
// LOOP PRINCIPAL
// ==============================================================================================================================================
void loop() {
  unsigned long ahora = millis();

  // ── Leer y suavizar potenciómetros (invertidos por errata hardware v2.0) ──
  float lecturaPot1 = (float)(4095 - analogRead(POT_1));
  float lecturaPot2 = (float)(4095 - analogRead(POT_2));
  float lecturaPot3 = (float)(4095 - analogRead(POT_3));
  float lecturaPot4 = (float)(4095 - analogRead(POT_4));

  emaPot1 = ALPHA_EMA * lecturaPot1 + (1.0f - ALPHA_EMA) * emaPot1;
  emaPot2 = ALPHA_EMA * lecturaPot2 + (1.0f - ALPHA_EMA) * emaPot2;
  emaPot3 = ALPHA_EMA * lecturaPot3 + (1.0f - ALPHA_EMA) * emaPot3;
  emaPot4 = ALPHA_EMA * lecturaPot4 + (1.0f - ALPHA_EMA) * emaPot4;

  // ── Leer botones (pull-up: LOW = presionado → invertir lógica) ──
  int btn1 = !digitalRead(BOTON_1);
  int btn2 = !digitalRead(BOTON_2);
  int btn3 = !digitalRead(BOTON_3);
  int btn4 = !digitalRead(BOTON_4);

  // ── Leer IMU y calcular mezcla de cámaras ──
  if (mpuOk) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    // acceleration.x en m/s²: -9.8 (izq) … 0 (nivelado) … +9.8 (der)
    float ax = a.acceleration.x;
    float imuRaw = ((ax / 9.8f) + 1.0f) * 0.5f * 4095.0f;
    imuRaw = constrain(imuRaw, 0.0f, 4095.0f);
    emaImuX = ALPHA_EMA * imuRaw + (1.0f - ALPHA_EMA) * emaImuX;
  }

  // ── Enviar datos al PC a 50Hz ──
  if (ahora - tiempoUltimoEnvio >= INTERVALO_ENVIO) {
    tiempoUltimoEnvio = ahora;

    // Enviar CSV: pot1,pot2,pot3,pot4,btn1,btn2,btn3,btn4,imu_x
    Serial.print((int)emaPot1); Serial.print(',');
    Serial.print((int)emaPot2); Serial.print(',');
    Serial.print((int)emaPot3); Serial.print(',');
    Serial.print((int)emaPot4); Serial.print(',');
    Serial.print(btn1); Serial.print(',');
    Serial.print(btn2); Serial.print(',');
    Serial.print(btn3); Serial.print(',');
    Serial.print(btn4); Serial.print(',');
    Serial.println((int)emaImuX);
  }

  // ── Leer comandos LED de la PC (no-bloqueante) ──
  leerComandoLED();
}

// ==============================================================================================================================================
// FUNCIONES AUXILIARES
// ==============================================================================================================================================

// Lee comandos entrantes de la forma "L:1,0,1,0\n" y aplica los LEDs
void leerComandoLED() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      // Procesar línea completa
      if (bufferSerial.startsWith("L:")) {
        aplicarComandoLED(bufferSerial.substring(2));
      }
      bufferSerial = "";
    } else if (c != '\r') {
      // Limitar tamaño del buffer para evitar desbordamiento
      if (bufferSerial.length() < 32) {
        bufferSerial += c;
      }
    }
  }
}

// Parsea "1,0,1,0" y enciende/apaga los 4 LEDs
void aplicarComandoLED(String datos) {
  int valores[4] = {0, 0, 0, 0};
  int idx = 0;
  int inicio = 0;

  for (int i = 0; i <= datos.length() && idx < 4; i++) {
    if (i == datos.length() || datos[i] == ',') {
      valores[idx] = datos.substring(inicio, i).toInt();
      idx++;
      inicio = i + 1;
    }
  }

  for (int i = 0; i < 4; i++) {
    digitalWrite(pinesLeds[i], valores[i] ? HIGH : LOW);
  }
}

// Animación de inicio: barrido de LEDs de izquierda a derecha (2 veces)
void animacionInicio() {
  for (int rep = 0; rep < 2; rep++) {
    for (int i = 0; i < 4; i++) {
      digitalWrite(pinesLeds[i], HIGH);
      delay(80);
      digitalWrite(pinesLeds[i], LOW);
    }
  }
  // Parpadeo final para confirmar listo
  for (int i = 0; i < 4; i++) digitalWrite(pinesLeds[i], HIGH);
  delay(150);
  for (int i = 0; i < 4; i++) digitalWrite(pinesLeds[i], LOW);
  delay(100);
}

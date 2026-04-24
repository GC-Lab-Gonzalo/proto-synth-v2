
// ==============================================================================================================================================
// PROTO-SYNTH V2 - ARPEGIADOR CELESTIAL - GC Lab Chile
// ==============================================================================================================================================
// Desarrollado por: Gonzalo - GC Lab Chile
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
// Arpegiador con 4 secuencias de arpegios celestiales (acordes Maj7, add9, Lydian) seleccionables por botón.
// Incluye morphing de forma de onda, envolvente AD, filtro paso bajo resonante controlado por IMU,
// y un modo secundario con patrones rítmicos, vibrato y control de volumen.
// ==============================================================================================================================================

// ==============================================================================================================================================
// FUNCIONAMIENTO
// ==============================================================================================================================================
// MODO PRINCIPAL (por defecto):
// - Botón 1: Arpegio 1 (Cmaj9 celestial)
// - Botón 2: Arpegio 2 (Fmaj7#11 celestial)
// - Botón 3: Arpegio 3 (D Frigio Dominante)
// - Botón 4: Arpegio 4 (B Frigio Dominante)
// - Potenciómetro 1: Velocidad del arpegio
// - Potenciómetro 2: Volumen general
// - Potenciómetro 3: Attack de la envolvente
// - Potenciómetro 4: Decay de la envolvente
// - IMU Eje X: Frecuencia de corte del filtro LPF
// - IMU Eje Y: Resonancia del filtro
// - LEDs: Secuencia ordenada al ritmo del arpegio
//
// MODO SECUNDARIO (mantener Botón 1 + Botón 4 para alternar):
// - Potenciómetro 1: Patrón de notas del arpegio
// - Potenciómetro 2: Vibrato Rate
// - Potenciómetro 3: Vibrato Depth
// - Potenciómetro 4: Forma de onda (cuadrada <-> diente de sierra)
//
// NOTA: Los controles solo se actualizan al mover el potenciómetro (sistema pickup).
// ==============================================================================================================================================

// ==============================================================================================================================================
// COMENTARIOS
// ==============================================================================================================================================
// - Los potenciómetros tienen lógica invertida (máximo valor = 0, mínimo valor = 4095).
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
const int SAMPLE_RATE = 22050;
const int AMPLITUDE = 127;
const int SDA_PIN = 21, SCL_PIN = 22, IMU_ADDRESS = 0x68;
const int POT1_PIN = 13, POT2_PIN = 14, POT3_PIN = 12, POT4_PIN = 27;
const int LED1_PIN = 23, LED2_PIN = 32, LED3_PIN = 5, LED4_PIN = 2;
const int BUTTON1_PIN = 18, BUTTON2_PIN = 4, BUTTON3_PIN = 15, BUTTON4_PIN = 19;

// ==============================================================================================================================================
// PROGRAMA
// ==============================================================================================================================================

// --- ARPEGIOS FRIGIO NEOCLÁSICO (frecuencias en Hz) ---
// Escala Frigia Dominante (modo 5 de menor armónica): 1 b2 3 4 5 b6 b7
// Cada arpegio tiene 8 notas con intervalos característicos del modo frigio neoclásico
const int NUM_NOTAS_ARPEGIO = 8;

// Arpegio 1: E Frigio Dominante (E2-F2-G#2-B2-D3-E3-F3-G#3)
const float arpegio1[] = {82.41, 87.31, 103.83, 123.47, 146.83, 164.81, 174.61, 207.65};
// Arpegio 2: A Frigio Dominante (A2-Bb2-C#3-E3-G3-A3-Bb3-C#4)
const float arpegio2[] = {110.00, 116.54, 138.59, 164.81, 196.00, 220.00, 233.08, 277.18};
// Arpegio 3: D Frigio Dominante (D2-Eb2-F#2-A2-C3-D3-Eb3-F#3)
const float arpegio3[] = {73.42, 77.78, 92.50, 110.00, 130.81, 146.83, 155.56, 185.00};
// Arpegio 4: B Frigio Dominante (B2-C3-D#3-F#3-A3-B3-C4-D#4)
const float arpegio4[] = {123.47, 130.81, 155.56, 185.00, 220.00, 246.94, 261.63, 311.13};

const float* arpegios[] = {arpegio1, arpegio2, arpegio3, arpegio4};

// --- PATRONES DE NOTAS ---
// Cada patrón define el orden en que se recorren las 8 notas del arpegio (índices 0-7)
const int NUM_PATRONES = 8;
const int patrones[NUM_PATRONES][NUM_NOTAS_ARPEGIO] = {
  {0, 1, 2, 3, 4, 5, 6, 7},  // Ascendente
  {7, 6, 5, 4, 3, 2, 1, 0},  // Descendente
  {0, 2, 4, 6, 7, 5, 3, 1},  // Péndulo
  {0, 4, 1, 5, 2, 6, 3, 7},  // Saltos intercalados
  {0, 7, 1, 6, 2, 5, 3, 4},  // Zigzag extremo-centro
  {0, 2, 0, 3, 0, 5, 0, 7},  // Pedal con tónica
  {0, 3, 6, 1, 4, 7, 2, 5},  // Terceras encadenadas
  {7, 0, 5, 2, 6, 1, 4, 3}   // Invertido alterno
};

// --- VARIABLES DE ESTADO ---
int arpegioActual = 0;
int pasoActual = 0;
bool reproduciendo = false; // Solo suena al mantener un botón presionado
bool botonPresionado = false; // Indica si algún botón de arpegio está presionado

// --- MODO ---
bool modoSecundario = false;

// --- TEMPORIZADORES ---
unsigned long ultimoPasoMicros = 0;
unsigned long tempoMicros = 250000; // velocidad del arpegio en microsegundos

// --- FORMA DE ONDA ---
float morphOnda = 0.0; // 0.0 = cuadrada, 1.0 = diente de sierra

// --- ENVOLVENTE AD ---
float attackTime = 0.01;  // en segundos
float decayTime = 0.3;    // en segundos
float attackRate, decayRate;
enum EstadoEnvolvente { INACTIVO, ATAQUE, DECAIMIENTO };
EstadoEnvolvente estadoEnv = INACTIVO;
float valorEnv = 0.0;

// --- OSCILADOR ---
float frecuenciaActual = 440.0;
float fase = 0.0;

// --- FILTRO ---
float filtro_x1 = 0, filtro_x2 = 0, filtro_y1 = 0, filtro_y2 = 0;
float filtro_a0, filtro_a1, filtro_a2, filtro_b1, filtro_b2;
float imu_x = 0.0, imu_y = 0.0, imu_filtrada_x = 0.0, imu_filtrada_y = 0.0;
const float IMU_ALPHA = 0.1;

// --- VIBRATO ---
float vibratoRate = 0.0;     // Hz
float vibratoDepth = 0.0;    // en semitonos (0 a 2)
float vibratoFase = 0.0;

// --- VOLUMEN ---
float volumen = 1.0;

// --- PATRÓN RÍTMICO ---
int patronActual = 0;

// --- DEBOUNCE ---
unsigned long ultimoBoton1 = 0, ultimoBoton2 = 0, ultimoBoton3 = 0, ultimoBoton4 = 0;
const unsigned long DEBOUNCE_MS = 200;
unsigned long ultimoCambioModo = 0;
const unsigned long DEBOUNCE_MODO_MS = 500;
bool ignorarSuelta = false; // Evita que al soltar el combo de modo se silencie

// --- SISTEMA PICKUP DE POTENCIÓMETROS ---
// Los potenciómetros solo actualizan su parámetro cuando se detecta movimiento
const int UMBRAL_POT = 60;
const int PINES_POT[] = {POT1_PIN, POT2_PIN, POT3_PIN, POT4_PIN};

// Valores ADC capturados al entrar a cada modo (referencia para detectar movimiento)
int potReferencia[2][4]; // [modo][pot] - valor ADC de referencia
bool potActivo[2][4];    // [modo][pot] - true si el pot ya fue "capturado" (movido)

// =================================================================
//                    INICIALIZACIÓN
// =================================================================
void setup() {
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(LED4_PIN, OUTPUT);
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(BUTTON3_PIN, INPUT_PULLUP);
  pinMode(BUTTON4_PIN, INPUT_PULLUP);

  dac_output_enable(DAC_CHANNEL_1);
  dac_output_voltage(DAC_CHANNEL_1, 128);

  inicializarIMU();
  inicializarPotPickup();
  calcularCoeficientesFiltro();

  attackRate = 1.0 / (attackTime * SAMPLE_RATE);
  decayRate = 1.0 / (decayTime * SAMPLE_RATE);
}

void inicializarIMU() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void inicializarPotPickup() {
  for (int m = 0; m < 2; m++) {
    for (int p = 0; p < 4; p++) {
      potReferencia[m][p] = 4095 - analogRead(PINES_POT[p]);
      potActivo[m][p] = false;
    }
  }
}

// =================================================================
//                    SISTEMA PICKUP DE POTENCIÓMETROS
// =================================================================
// Devuelve el valor del pot (0-4095) solo si fue movido desde su posición de referencia.
// Si no ha sido movido, devuelve -1.
int leerPotConPickup(int indicePot) {
  int modo = modoSecundario ? 1 : 0;
  int valorCrudo = 4095 - analogRead(PINES_POT[indicePot]);

  if (potActivo[modo][indicePot]) {
    // Ya fue capturado, actualizar normalmente
    return valorCrudo;
  } else {
    // Verificar si se movió lo suficiente desde la referencia
    if (abs(valorCrudo - potReferencia[modo][indicePot]) > UMBRAL_POT) {
      potActivo[modo][indicePot] = true;
      return valorCrudo;
    }
    return -1; // No se ha movido, no actualizar
  }
}

// Resetear el pickup al cambiar de modo
void resetearPickupModo(int modo) {
  for (int p = 0; p < 4; p++) {
    potReferencia[modo][p] = 4095 - analogRead(PINES_POT[p]);
    potActivo[modo][p] = false;
  }
}

// =================================================================
//                    LOOP PRINCIPAL
// =================================================================
void loop() {
  static unsigned long ultimoAudio = 0;
  static int contadorControl = 0;

  unsigned long ahora = micros();

  if (ahora - ultimoAudio >= (1000000 / SAMPLE_RATE)) {
    ultimoAudio = ahora;

    // Lectura de controles cada 256 muestras (~86 Hz)
    if (contadorControl++ % 256 == 0) {
      leerBotones();
      leerControles();
      leerIMU();
      calcularCoeficientesFiltro();
    }

    // Avanzar arpegio (tempo uniforme, el patrón solo cambia el orden de las notas)
    if (reproduciendo && (ahora - ultimoPasoMicros >= tempoMicros)) {
      avanzarPaso();
    }

    // Generar audio
    float env = procesarEnvolvente();
    if (env > 0.001) {
      // Calcular frecuencia con vibrato
      float frecConVibrato = frecuenciaActual;
      if (vibratoDepth > 0.01 && vibratoRate > 0.1) {
        vibratoFase += vibratoRate / SAMPLE_RATE;
        if (vibratoFase >= 1.0) vibratoFase -= 1.0;
        float modulacion = sin(2.0 * PI * vibratoFase) * vibratoDepth;
        frecConVibrato = frecuenciaActual * pow(2.0, modulacion / 12.0);
      }

      float muestrasPorCiclo = (float)SAMPLE_RATE / frecConVibrato;
      float faseNorm = fase / muestrasPorCiclo;

      // Morphing entre cuadrada y diente de sierra
      float muestra = generarOndaMorph(faseNorm, morphOnda);

      fase += 1.0;
      if (fase >= muestrasPorCiclo) fase -= muestrasPorCiclo;

      // Aplicar filtro
      float muestraFiltrada = aplicarFiltro(muestra);

      // Aplicar envolvente y volumen
      int valorDAC = 128 + (int)(muestraFiltrada * env * volumen * AMPLITUDE);
      valorDAC = constrain(valorDAC, 0, 255);
      dac_output_voltage(DAC_CHANNEL_1, valorDAC);
    } else {
      dac_output_voltage(DAC_CHANNEL_1, 128);
    }
  }
}

// =================================================================
//                    GENERADOR DE FORMA DE ONDA
// =================================================================
// Morphing continuo entre onda cuadrada (morph=0) y diente de sierra (morph=1)
float generarOndaMorph(float faseNorm, float morph) {
  // Onda cuadrada
  float cuadrada = (faseNorm < 0.5) ? 1.0 : -1.0;
  // Diente de sierra
  float sierra = (2.0 * faseNorm) - 1.0;
  // Interpolación lineal
  return cuadrada * (1.0 - morph) + sierra * morph;
}

// =================================================================
//                    LÓGICA DEL ARPEGIADOR
// =================================================================
void avanzarPaso() {
  ultimoPasoMicros = micros();

  pasoActual = (pasoActual + 1) % NUM_NOTAS_ARPEGIO;

  // Actualizar LEDs en secuencia ordenada
  actualizarLEDs(pasoActual);

  // Obtener la nota según el patrón actual (el patrón define el orden de las notas)
  int indiceNota = patrones[patronActual][pasoActual];
  frecuenciaActual = arpegios[arpegioActual][indiceNota];
  fase = 0.0;
  estadoEnv = ATAQUE;
  valorEnv = 0.0;
}

// =================================================================
//                    ENVOLVENTE AD
// =================================================================
float procesarEnvolvente() {
  switch (estadoEnv) {
    case ATAQUE:
      valorEnv += attackRate;
      if (valorEnv >= 1.0) {
        valorEnv = 1.0;
        estadoEnv = DECAIMIENTO;
      }
      break;
    case DECAIMIENTO:
      valorEnv -= decayRate;
      if (valorEnv <= 0.0) {
        valorEnv = 0.0;
        estadoEnv = INACTIVO;
      }
      break;
    default:
      valorEnv = 0.0;
      break;
  }
  return valorEnv;
}

// =================================================================
//                    FILTRO PASO BAJO RESONANTE
// =================================================================
void calcularCoeficientesFiltro() {
  // Mapear IMU X a frecuencia de corte (100 Hz - 10000 Hz)
  float freqCorte = mapFloat(imu_filtrada_x, -1.0, 1.0, 100.0, 10000.0);
  // Mapear IMU Y a resonancia (0.7 - 15.0)
  float resonancia = mapFloat(imu_filtrada_y, -1.0, 1.0, 0.7, 15.0);

  float freqNorm = constrain(freqCorte / (SAMPLE_RATE / 2.0), 0.01, 0.95);
  float omega = PI * freqNorm;
  float s = sin(omega);
  float c = cos(omega);
  float alpha = s / (2.0 * constrain(resonancia, 0.7, 20.0));

  float b0 = (1 - c) / 2;
  float b1_coef = 1 - c;
  float b2 = b0;
  float a0 = 1 + alpha;
  float a1_coef = -2 * c;
  float a2 = 1 - alpha;

  filtro_a0 = b0 / a0;
  filtro_a1 = b1_coef / a0;
  filtro_a2 = b2 / a0;
  filtro_b1 = a1_coef / a0;
  filtro_b2 = a2 / a0;
}

float aplicarFiltro(float entrada) {
  float salida = filtro_a0 * entrada + filtro_a1 * filtro_x1 + filtro_a2 * filtro_x2
                 - filtro_b1 * filtro_y1 - filtro_b2 * filtro_y2;
  filtro_x2 = filtro_x1;
  filtro_x1 = entrada;
  filtro_y2 = filtro_y1;
  filtro_y1 = salida;
  return salida;
}

float mapFloat(float x, float inMin, float inMax, float outMin, float outMax) {
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

// =================================================================
//                    LECTURA DE IMU
// =================================================================
void leerIMU() {
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU_ADDRESS, 6, true);
  if (Wire.available() >= 6) {
    int16_t raw_x = Wire.read() << 8 | Wire.read();
    int16_t raw_y = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read(); // descartar Z
    imu_filtrada_x = imu_filtrada_x * (1 - IMU_ALPHA) + constrain(raw_x / 16384.0, -1, 1) * IMU_ALPHA;
    imu_filtrada_y = imu_filtrada_y * (1 - IMU_ALPHA) + constrain(raw_y / 16384.0, -1, 1) * IMU_ALPHA;
  }
}

// =================================================================
//                    LECTURA DE CONTROLES
// =================================================================
void leerControles() {
  if (!modoSecundario) {
    // --- MODO PRINCIPAL ---
    // Pot 1: Velocidad del arpegio (50ms - 500ms por paso)
    int val = leerPotConPickup(0);
    if (val >= 0) {
      tempoMicros = map(val, 0, 4095, 500000, 50000);
    }

    // Pot 2: Volumen (0 - 100%)
    val = leerPotConPickup(1);
    if (val >= 0) {
      volumen = val / 4095.0;
    }

    // Pot 3: Attack (1ms - 500ms)
    val = leerPotConPickup(2);
    if (val >= 0) {
      attackTime = mapFloat(val, 0, 4095, 0.001, 0.5);
      attackRate = 1.0 / (attackTime * SAMPLE_RATE);
    }

    // Pot 4: Decay (10ms - 2000ms)
    val = leerPotConPickup(3);
    if (val >= 0) {
      decayTime = mapFloat(val, 0, 4095, 0.01, 2.0);
      decayRate = 1.0 / (decayTime * SAMPLE_RATE);
    }
  } else {
    // --- MODO SECUNDARIO ---
    // Pot 1: Patrón rítmico
    int val = leerPotConPickup(0);
    if (val >= 0) {
      patronActual = map(val, 0, 4095, 0, NUM_PATRONES - 1);
    }

    // Pot 2: Vibrato Rate (0 - 5 Hz)
    val = leerPotConPickup(1);
    if (val >= 0) {
      vibratoRate = mapFloat(val, 0, 4095, 0.0, 5.0);
    }

    // Pot 3: Vibrato Depth (0 - 0.3 semitonos)
    val = leerPotConPickup(2);
    if (val >= 0) {
      vibratoDepth = mapFloat(val, 0, 4095, 0.0, 0.3);
    }

    // Pot 4: Forma de onda (cuadrada <-> sierra)
    val = leerPotConPickup(3);
    if (val >= 0) {
      morphOnda = val / 4095.0;
    }
  }
}

// =================================================================
//                    LECTURA DE BOTONES
// =================================================================
void leerBotones() {
  unsigned long t = millis();
  bool btn1 = !digitalRead(BUTTON1_PIN);
  bool btn2 = !digitalRead(BUTTON2_PIN);
  bool btn3 = !digitalRead(BUTTON3_PIN);
  bool btn4 = !digitalRead(BUTTON4_PIN);

  // Detección de combo Botón 1 + Botón 4 para cambio de modo (toggle persistente)
  if (btn1 && btn4 && (t - ultimoCambioModo > DEBOUNCE_MODO_MS)) {
    modoSecundario = !modoSecundario;
    ultimoCambioModo = t;
    ignorarSuelta = true; // No silenciar al soltar el combo

    // Resetear pickup del modo al que se entra
    resetearPickupModo(modoSecundario ? 1 : 0);

    // Parpadear todos los LEDs para indicar cambio de modo
    parpadearLEDs();
    return;
  }

  // Si se acaba de soltar el combo de cambio de modo, ignorar esta suelta
  if (ignorarSuelta) {
    if (!btn1 && !btn4) {
      ignorarSuelta = false; // Ya se soltaron, listo para funcionar normal
    }
    return;
  }

  // Determinar si algún botón individual está presionado (funciona como tecla)
  bool algunoPresionado = false;

  if (btn1 && !btn4) {
    arpegioActual = 0;
    algunoPresionado = true;
  } else if (btn2) {
    arpegioActual = 1;
    algunoPresionado = true;
  } else if (btn3) {
    arpegioActual = 2;
    algunoPresionado = true;
  } else if (btn4 && !btn1) {
    arpegioActual = 3;
    algunoPresionado = true;
  }

  // Al presionar: iniciar arpegio. Al soltar: silencio.
  if (algunoPresionado && !botonPresionado) {
    // Recién se presionó un botón
    botonPresionado = true;
    reproduciendo = true;
    pasoActual = 0;
    ultimoPasoMicros = micros();
    fase = 0.0;
    frecuenciaActual = arpegios[arpegioActual][0];
    estadoEnv = ATAQUE;
    valorEnv = 0.0;
  } else if (!algunoPresionado && botonPresionado) {
    // Se soltó el botón: silenciar
    botonPresionado = false;
    reproduciendo = false;
    estadoEnv = INACTIVO;
    valorEnv = 0.0;
    // Apagar LEDs
    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, LOW);
    digitalWrite(LED3_PIN, LOW);
    digitalWrite(LED4_PIN, LOW);
  }
}

// =================================================================
//                    LEDS
// =================================================================
void actualizarLEDs(int paso) {
  if (modoSecundario) {
    // Modo secundario: todos los LEDs parpadean juntos al ritmo del arpegio
    bool encendido = (paso % 2 == 0);
    digitalWrite(LED1_PIN, encendido);
    digitalWrite(LED2_PIN, encendido);
    digitalWrite(LED3_PIN, encendido);
    digitalWrite(LED4_PIN, encendido);
  } else {
    // Modo principal: secuencia ordenada, cada LED cubre 2 pasos
    digitalWrite(LED1_PIN, paso < 2);
    digitalWrite(LED2_PIN, paso >= 2 && paso < 4);
    digitalWrite(LED3_PIN, paso >= 4 && paso < 6);
    digitalWrite(LED4_PIN, paso >= 6);
  }
}

void parpadearLEDs() {
  // Parpadeo rápido para indicar cambio de modo
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED1_PIN, HIGH); digitalWrite(LED2_PIN, HIGH);
    digitalWrite(LED3_PIN, HIGH); digitalWrite(LED4_PIN, HIGH);
    delay(50);
    digitalWrite(LED1_PIN, LOW); digitalWrite(LED2_PIN, LOW);
    digitalWrite(LED3_PIN, LOW); digitalWrite(LED4_PIN, LOW);
    delay(50);
  }
}

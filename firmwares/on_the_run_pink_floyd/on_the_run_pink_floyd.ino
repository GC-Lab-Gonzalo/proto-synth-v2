
// ==============================================================================================================================================
// PROTO-SYNTH V2 - SEQUENCER "ON THE RUN" - GC Lab Chile
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
// Sequenciador que reproduce la famosa secuencia de "On the Run" de Pink Floyd
// Mi4 → Sol4 → La4 → Sol4 → Re5 → Do5 → Re5 → Mi5
// ==============================================================================================================================================

// ==============================================================================================================================================
// FUNCIONAMIENTO
// ==============================================================================================================================================
// CONTROLES DE EXPRESIÓN:
// - Potenciómetro 1: Volumen general
// - Potenciómetro 2: Attack
// - Potenciómetro 3: Saturacion/Distorsión 
// - Potenciómetro 4: Decay
// - Botón 1: Play/Stop
// - Botón 2: Reducir el tempo
// - Botón 3: Aumentar el tempo
// - Botón 4: No se usa
// - LED 1: Indica el paso 1 y 5 de la secuencia
// - LED 2: Indica el paso 2 y 6 de la secuencia
// - LED 3: Indica el paso 3 y 7 de la secuencia
// - LED 4: Indica el paso 4 y 8 de la secuencia
// - IMU: Control de filtro LPF
// - LDR: No se usa
// - Micrófono: No se usa
// - Header 1: No se usa
// - Header 2: No se usa
// - Salida MIDI: No se usa
//
// MODO DE USO:
// 1. Presiona Botón 1 para iniciar/detener la secuencia
// 2. Usa Botón 2/3 para cambiar velocidad
// 3. Mueve el dispositivo para cambiar el filtro
// 4. Ajusta potenciómetros para cambiar el sonido
//
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
#include "driver/dac.h"  // Control del DAC (Digital to Analog Converter) del ESP32
#include "math.h"        // Funciones matemáticas (sin, cos, PI, etc.)
#include "Wire.h"        // Comunicación I2C para el sensor IMU

// ==============================================================================================================================================
// CONFIGURACIÓN DE HARDWARE - PINES
// ==============================================================================================================================================
// Pines I2C para el sensor de movimiento (IMU)
const int SDA_PIN = 21;        // Pin de datos I2C
const int SCL_PIN = 22;        // Pin de reloj I2C
const int IMU_ADDRESS = 0x68;  // Dirección I2C del MPU6050

// Configuración de audio
const int SAMPLE_RATE = 22050;  // Frecuencia de muestreo (22kHz = calidad aceptable)
const int AMPLITUDE = 90;       // Amplitud máxima del sonido (0-127 para DAC)

// Pines de potenciómetros (controles analógicos)
const int POT1_PIN = 13;  // Potenciómetro 1: Volumen general
const int POT2_PIN = 14;  // Potenciómetro 2: Attack (velocidad de inicio del sonido)
const int POT3_PIN = 12;  // Potenciómetro 3: Saturación/Distorsión
const int POT4_PIN = 27;  // Potenciómetro 4: Decay (velocidad de desvanecimiento)

// Pines de LEDs (indicadores visuales de la secuencia)
const int LED1_PIN = 23;  // LED 1 - Pasos 1 y 5
const int LED2_PIN = 32;  // LED 2 - Pasos 2 y 6
const int LED3_PIN = 5;   // LED 3 - Pasos 3 y 7  
const int LED4_PIN = 2;   // LED 4 - Pasos 4 y 8

// Pines de botones (controles digitales)
const int BUTTON_PIN = 18;     // Botón 1: Play/Stop del secuenciador
const int BUTTON2_PIN = 4;     // Botón 2: Disminuir velocidad (más lento)
const int BUTTON3_PIN = 15;    // Botón 3: Aumentar velocidad (más rápido)
const int BUTTON4_PIN = 19;    // Botón 4: No usado en esta versión

// ==============================================================================================================================================
// VARIABLES DEL SENSOR IMU (SENSOR DE MOVIMIENTO)
// ==============================================================================================================================================

// Valores brutos del acelerómetro
float imu_accel_x = 0.0;        // Aceleración en eje X
float imu_accel_y = 0.0;        // Aceleración en eje Y
unsigned long lastIMURead = 0;   // Tiempo de la última lectura del IMU
const unsigned long IMU_READ_INTERVAL = 20;  // Leer IMU cada 20ms

// Filtros para suavizar las lecturas del IMU (evita cambios bruscos)
float filtered_x = 0.0;         // Valor filtrado del eje X
float filtered_y = 0.0;         // Valor filtrado del eje Y
const float IMU_FILTER_ALPHA = 0.2;  // Factor de suavizado (0.0 = sin filtro, 1.0 = filtro máximo)

// ==============================================================================================================================================
// VARIABLES DEL FILTRO DE AUDIO (PROCESAMIENTO DEL SONIDO)
// ==============================================================================================================================================

// Variables para almacenar estados anteriores del filtro
float filter_x1 = 0, filter_x2 = 0;  // Entradas anteriores
float filter_y1 = 0, filter_y2 = 0;  // Salidas anteriores

// Coeficientes del filtro (se calculan dinámicamente)
float filter_a0, filter_a1, filter_a2;  // Coeficientes de entrada
float filter_b1, filter_b2;             // Coeficientes de retroalimentación

// ==============================================================================================================================================
// SECUENCIA MUSICAL "ON THE RUN"
// ==============================================================================================================================================

// Notas de la famosa secuencia (frecuencias en Hz)
const float SEQUENCE_NOTES[] = {
  329.63,  // Mi4 (E4) - Primera nota
  392.00,  // Sol4 (G4)
  440.00,  // La4 (A4) 
  392.00,  // Sol4 (G4) - Repetición
  587.33,  // Re5 (D5) - Octava más aguda
  523.25,  // Do5 (C5)
  587.33,  // Re5 (D5) - Repetición
  659.25   // Mi5 (E5) - Nota final
};
const int SEQUENCE_LENGTH = 8;  // Número total de pasos en la secuencia

// ==============================================================================================================================================
// VARIABLES DEL SECUENCIADOR (REPRODUCTOR DE LA SECUENCIA)
// ==============================================================================================================================================

bool sequencer_playing = false;        // Si el secuenciador está reproduciendo
int current_step = 0;                  // Paso actual en la secuencia (0-7)
unsigned long last_step_time = 0;      // Tiempo del último cambio de paso
unsigned long step_duration = 200;    // Duración de cada paso en milisegundos (120 BPM inicial)

// Límites de velocidad del secuenciador
const unsigned long MIN_STEP_DURATION = 80;   // Más rápido posible (750 BPM)
const unsigned long MAX_STEP_DURATION = 500;  // Más lento posible (60 BPM)

// ==============================================================================================================================================
// VARIABLES DEL ENVELOPE (ENVOLVENTE DE VOLUMEN)
// ==============================================================================================================================================
// El envelope controla cómo inicia y termina cada nota

float envelope_level = 0.0;        // Nivel actual del envelope (0.0 = silencio, 1.0 = máximo)
bool note_triggered = false;       // Si se acaba de disparar una nueva nota
unsigned long note_start_time = 0; // Tiempo cuando comenzó la nota actual

// Duraciones del envelope (se ajustan con potenciómetros)
float attack_duration = 50.0;      // Tiempo para llegar al volumen máximo (ms)
float decay_duration = 150.0;      // Tiempo para desvanecerse (ms)

// ==============================================================================================================================================
// VARIABLES DE CONTROL DE BOTONES (ANTIREBOTE)
// ==============================================================================================================================================

// Estados de los botones (para detectar cuando se presionan)
bool button_pressed = false;       // Estado del botón play/stop
bool button2_pressed = false;      // Estado del botón velocidad-
bool button3_pressed = false;      // Estado del botón velocidad+

// Tiempos para antirebote (evitar múltiples detecciones)
unsigned long last_button_time = 0;
unsigned long last_button2_time = 0;
unsigned long last_button3_time = 0;
const unsigned long BUTTON_DEBOUNCE = 200;  // 200ms de antirebote

// ==============================================================================================================================================
// VARIABLES DE GENERACIÓN DE AUDIO
// ==============================================================================================================================================

float currentFreq = 164.81;  // Frecuencia actual a reproducir (Mi3 inicial)
bool audioPlaying = false;   // Si el audio está sonando
int waveform_type = 2;       // Tipo de forma de onda (2 = sierra armónica)

// Variables para generación de formas de onda
float phase = 0.0;           // Fase actual de la onda (posición en el ciclo)
float modulation_phase = 0.0; // Fase para modulación (efectos adicionales)

// ==============================================================================================================================================
// FUNCIÓN DE INICIALIZACIÓN DEL IMU
// ==============================================================================================================================================

void initIMU() {
  // Inicializar comunicación I2C con el sensor de movimiento
  Wire.begin(SDA_PIN, SCL_PIN);    // Configurar pines I2C
  Wire.setClock(400000);           // Velocidad rápida (400kHz)
  
  // Despertar el MPU6050 (viene dormido por defecto)
  Wire.beginTransmission(IMU_ADDRESS);  // Iniciar comunicación
  Wire.write(0x6B);                     // Registro de gestión de energía
  Wire.write(0);                        // Escribir 0 para despertar
  Wire.endTransmission(true);           // Finalizar transmisión
  
  // Configurar el acelerómetro para rango ±2g (sensibilidad estándar)
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(0x1C);                     // Registro de configuración del acelerómetro
  Wire.write(0x00);                     // ±2g de rango
  Wire.endTransmission(true);
  
  delay(100);  // Esperar a que se estabilice el sensor
}

// ==============================================================================================================================================
// FUNCIÓN DE LECTURA DEL IMU
// ==============================================================================================================================================

void readIMU() {
  unsigned long currentTime = millis();
  
  // Solo leer el IMU cada cierto intervalo (no sobrecargar)
  if (currentTime - lastIMURead < IMU_READ_INTERVAL) {
    return;  // Salir sin hacer nada si es muy pronto
  }
  lastIMURead = currentTime;
  
  // Solicitar datos del acelerómetro
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(0x3B);                    // Registro donde empiezan los datos del acelerómetro
  Wire.endTransmission(false);         // Mantener conexión abierta
  Wire.requestFrom(IMU_ADDRESS, 6, true);  // Pedir 6 bytes (X, Y, Z = 2 bytes cada uno)
  
  if (Wire.available() >= 6) {         // Si recibimos todos los datos
    // Leer datos X e Y (ignorar Z para este proyecto)
    int16_t raw_x = Wire.read() << 8 | Wire.read();  // Combinar 2 bytes en valor X
    int16_t raw_y = Wire.read() << 8 | Wire.read();  // Combinar 2 bytes en valor Y
    Wire.read(); Wire.read();                        // Descartar bytes Z
    
    // Convertir valores brutos a unidades de gravedad (g)
    // El MPU6050 en ±2g devuelve valores de -32768 a +32767
    // Dividir por 16384 convierte a rango -2.0g a +2.0g
    imu_accel_x = raw_x / 16384.0;
    imu_accel_y = raw_y / 16384.0;
    
    // Aplicar filtro de suavizado (filtro pasa-bajos simple)
    // Esto evita cambios bruscos en el sonido al mover el dispositivo
    filtered_x = filtered_x * (1.0 - IMU_FILTER_ALPHA) + imu_accel_x * IMU_FILTER_ALPHA;
    filtered_y = filtered_y * (1.0 - IMU_FILTER_ALPHA) + imu_accel_y * IMU_FILTER_ALPHA;
  }
}

// ==============================================================================================================================================
// FUNCIÓN DE ACTUALIZACIÓN DEL FILTRO DE AUDIO
// ==============================================================================================================================================

void updateFilterCoefficients() {
  // CONTROL DE RESONANCIA con IMU Y (inclinación lateral)
  // Resonancia = qué tan "puntiagudo" suena el filtro
  float y_abs = abs(filtered_y);                    // Valor absoluto (siempre positivo)
  y_abs = constrain(y_abs, 0.0, 1.0);              // Limitar entre 0 y 1
  float resonance = 0.5 + y_abs * 19.5;            // Mapear a rango 0.5-20 (resonancia suave a extrema)
  
  // CONTROL DE FRECUENCIA DE CORTE con IMU X (inclinación adelante/atrás)
  // Frecuencia de corte = qué frecuencias pasan por el filtro
  float x_abs = abs(filtered_x);                    // Valor absoluto
  x_abs = constrain(x_abs, 0.0, 1.0);              // Limitar entre 0 y 1
  float cutoff_freq = 200.0 + x_abs * 7800.0;      // Mapear a rango 200Hz-8000Hz
  
  // CÁLCULO DE COEFICIENTES DEL FILTRO (matemáticas avanzadas)
  // Esto convierte frecuencia y resonancia en coeficientes para el filtro digital
  float norm_freq = cutoff_freq / (SAMPLE_RATE / 2.0);  // Normalizar frecuencia (0-1)
  norm_freq = constrain(norm_freq, 0.01, 0.95);         // Evitar valores extremos
  
  // Fórmulas del filtro pasa-bajos de 2° orden (Butterworth)
  float omega = PI * norm_freq;              // Frecuencia angular
  float sin_omega = sin(omega);              // Seno de la frecuencia
  float cos_omega = cos(omega);              // Coseno de la frecuencia
  float alpha = sin_omega / (2.0 * resonance);  // Factor de resonancia
  
  // Coeficientes del numerador (entrada)
  float b0 = (1.0 - cos_omega) / 2.0;
  float b1 = 1.0 - cos_omega;
  float b2 = (1.0 - cos_omega) / 2.0;
  
  // Coeficientes del denominador (retroalimentación)
  float a0_temp = 1.0 + alpha;
  float a1_temp = -2.0 * cos_omega;
  float a2_temp = 1.0 - alpha;
  
  // Normalizar coeficientes (dividir por a0)
  filter_a0 = b0 / a0_temp;
  filter_a1 = b1 / a0_temp;
  filter_a2 = b2 / a0_temp;
  filter_b1 = a1_temp / a0_temp;
  filter_b2 = a2_temp / a0_temp;
}

// ==============================================================================================================================================
// FUNCIÓN DE APLICACIÓN DEL FILTRO Y SATURACIÓN
// ==============================================================================================================================================

float applyFilter(float input) {
  // POTENCIÓMETRO 3: SATURACIÓN/DISTORSIÓN - INVERTIDO
  // Lee el potenciómetro 3 para controlar cuánta distorsión aplicar
  int pot3_value = analogRead(POT3_PIN);                    // Leer valor (0-4095)
  float pot3_normalized = 1.0 - (pot3_value / 4095.0);     // INVERTIR: 1.0 cuando pot=0, 0.0 cuando pot=4095
  float drive = 1.0 + pot3_normalized * 9.0;                // Mapear a rango 1x-10x (ganancia)
  
  // APLICAR SATURACIÓN (DISTORSIÓN)
  // Multiplicar la señal por el factor de ganancia
  float driven_input = input * drive;
  
  // Saturación tipo "soft clipping" (suena más musical que clipping duro)
  if (driven_input > 1.5) {
    driven_input = 0.9;        // Limitar máximo
  } else if (driven_input < -1.5) {
    driven_input = -0.9;       // Limitar mínimo
  } else if (driven_input > 0.8) {
    // Compresión suave en valores altos
    driven_input = 0.8 + (driven_input - 0.8) * 0.2;
  } else if (driven_input < -0.8) {
    // Compresión suave en valores bajos
    driven_input = -0.8 + (driven_input + 0.8) * 0.2;
  }
  
  // APLICAR FILTRO DIGITAL
  // Ecuación del filtro de 2° orden: y[n] = a0*x[n] + a1*x[n-1] + a2*x[n-2] - b1*y[n-1] - b2*y[n-2]
  float output = filter_a0 * driven_input + filter_a1 * filter_x1 + filter_a2 * filter_x2
                 - filter_b1 * filter_y1 - filter_b2 * filter_y2;
  
  // Actualizar memoria del filtro (guardar valores anteriores)
  filter_x2 = filter_x1;      // x[n-2] = x[n-1]
  filter_x1 = driven_input;   // x[n-1] = x[n]
  filter_y2 = filter_y1;      // y[n-2] = y[n-1]
  filter_y1 = output;         // y[n-1] = y[n]
  
  return output;  // Devolver señal filtrada
}

// ==============================================================================================================================================
// FUNCIÓN DE ACTUALIZACIÓN DEL SECUENCIADOR
// ==============================================================================================================================================

void updateSequencer() {
  // Si el secuenciador no está reproduciendo, silenciar todo
  if (!sequencer_playing) {
    audioPlaying = false;     // Detener audio
    envelope_level = 0.0;     // Silenciar envelope
    return;                   // Salir de la función
  }
  
  unsigned long current_time = millis();  // Tiempo actual
  
  // VERIFICAR SI ES TIEMPO PARA EL SIGUIENTE PASO
  if (current_time - last_step_time >= step_duration) {
    last_step_time = current_time;  // Actualizar tiempo del último paso
    
    // AVANZAR AL SIGUIENTE PASO (con repetición cíclica)
    current_step = (current_step + 1) % SEQUENCE_LENGTH;  // 0,1,2,3,4,5,6,7,0,1,2...
    
    // ESTABLECER NUEVA FRECUENCIA según el paso actual
    currentFreq = SEQUENCE_NOTES[current_step];
    
    // DISPARAR NUEVA NOTA (reiniciar envelope)
    note_triggered = true;      // Marcar que se disparó nueva nota
    note_start_time = current_time;  // Recordar cuándo empezó
    envelope_level = 0.0;       // Resetear nivel del envelope
    audioPlaying = true;        // Activar generación de audio
    
    // ACTUALIZAR LEDS para mostrar paso actual
    updateLEDs();
  }
  
  // ACTUALIZAR ENVELOPE (envolvente de volumen)
  updateEnvelope();
}

// ==============================================================================================================================================
// FUNCIÓN DE ACTUALIZACIÓN DEL ENVELOPE (ENVOLVENTE)
// ==============================================================================================================================================

void updateEnvelope() {
  // Si no hay audio, mantener envelope en cero
  if (!audioPlaying) {
    envelope_level = 0.0;
    return;
  }
  
  // LEER PARÁMETROS DE LOS POTENCIÓMETROS - INVERTIDOS
  int attack_pot = analogRead(POT2_PIN);   // Potenciómetro 2: Attack
  int decay_pot = analogRead(POT4_PIN);    // Potenciómetro 4: Decay
  
  // CONVERTIR A DURACIÓN EN MILISEGUNDOS - INVERTIDOS
  attack_duration = 10.0 + ((4095 - attack_pot) / 4095.0) * 90.0;   // INVERTIDO: 10ms a 100ms de attack
  decay_duration = 50.0 + ((4095 - decay_pot) / 4095.0) * 450.0;    // INVERTIDO: 50ms a 500ms de decay
  
  unsigned long note_time = millis() - note_start_time;  // Tiempo transcurrido desde inicio de nota
  
  // CALCULAR NIVEL DEL ENVELOPE según la fase actual
  if (note_time < attack_duration && note_triggered) {
    // FASE DE ATTACK: Subida rápida del volumen (0.0 → 1.0)
    envelope_level = note_time / attack_duration;  // Interpolación lineal
    if (envelope_level >= 1.0) {
      envelope_level = 1.0;           // Limitar a máximo
      note_triggered = false;         // Terminar fase de attack
    }
  }
  else if (note_time < attack_duration + decay_duration) {
    // FASE DE DECAY: Bajada gradual del volumen (1.0 → 0.0)
    note_triggered = false;  // Ya no estamos en attack
    float decay_progress = (note_time - attack_duration) / decay_duration;  // Progreso del decay (0-1)
    envelope_level = 1.0 - decay_progress;  // Interpolar hacia abajo
    if (envelope_level < 0.0) {
      envelope_level = 0.0;  // No bajar de cero
    }
  }
  else {
    // FASE DE SILENCIO: Nota terminada, esperar siguiente paso
    envelope_level = 0.0;
  }
}

// ==============================================================================================================================================
// FUNCIÓN DE ACTUALIZACIÓN DE LEDS
// ==============================================================================================================================================

void updateLEDs() {
  // APAGAR TODOS LOS LEDS primero
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  digitalWrite(LED3_PIN, LOW);
  digitalWrite(LED4_PIN, LOW);
  
  // ENCENDER LED según el paso actual
  // Como hay 8 pasos pero solo 4 LEDs, cada LED representa 2 pasos
  switch (current_step % 4) {  // Usar módulo 4 para mapear 8 pasos a 4 LEDs
    case 0: digitalWrite(LED1_PIN, HIGH); break;  // Pasos 0 y 4 (Mi4 y Re5)
    case 1: digitalWrite(LED2_PIN, HIGH); break;  // Pasos 1 y 5 (Sol4 y Do5)
    case 2: digitalWrite(LED3_PIN, HIGH); break;  // Pasos 2 y 6 (La4 y Re5)
    case 3: digitalWrite(LED4_PIN, HIGH); break;  // Pasos 3 y 7 (Sol4 y Mi5)
  }
}

// ==============================================================================================================================================
// FUNCIONES DE GENERACIÓN DE FORMAS DE ONDA
// ==============================================================================================================================================

// Genera onda triangular (subida y bajada lineal)
float generateTriangle(float normalized_phase) {
  if (normalized_phase < 0.5) {
    // Primera mitad: subir de -1 a +1
    return 4.0 * normalized_phase - 1.0;
  } else {
    // Segunda mitad: bajar de +1 a -1
    return 3.0 - 4.0 * normalized_phase;
  }
}

// Genera onda de pulso (onda cuadrada con ancho variable)
float generatePulse(float normalized_phase) {
  // El ancho del pulso se controla con IMU Y
  float duty_cycle = 0.3 + 0.4 * (filtered_y + 1.0) / 2.0;  // Mapear IMU Y a rango 0.3-0.7
  duty_cycle = constrain(duty_cycle, 0.2, 0.8);              // Limitar a rango seguro
  
  // Comparar fase actual con ciclo de trabajo
  return (normalized_phase < duty_cycle) ? 1.0 : -1.0;  // +1 o -1 según la fase
}

// Genera onda sierra armónica (suma de senoidales para sonido más rico)
float generateHarmonicSaw(float normalized_phase) {
  float output = 0.0;
  
  // SUMAR PRIMEROS 5 ARMÓNICOS (frecuencias múltiples)
  for (int harmonic = 1; harmonic <= 5; harmonic++) {
    float amplitude = 1.0 / harmonic;  // Cada armónico es más débil (1, 1/2, 1/3, 1/4, 1/5)
    // Sumar senoidal de cada armónico
    output += amplitude * sin(2.0 * PI * harmonic * normalized_phase);
  }
  
  return output * 0.5;  // Reducir amplitud total para evitar saturación
}

// ==============================================================================================================================================
// FUNCIÓN PRINCIPAL DE GENERACIÓN DE FORMAS DE ONDA
// ==============================================================================================================================================

float generateWaveform(float samples_per_cycle) {
  // ACTUALIZAR FASE (posición en el ciclo de la onda)
  phase += 1.0;                    // Avanzar un sample
  if (phase >= samples_per_cycle) {  // Si completamos un ciclo
    phase = 0.0;                   // Reiniciar fase
  }
  
  // NORMALIZAR FASE a rango 0.0-1.0 (0% a 100% del ciclo)
  float normalized_phase = phase / samples_per_cycle;
  float output = 0.0;
  
  // SELECCIONAR TIPO DE FORMA DE ONDA
  switch (waveform_type) {
    case 0:
      output = generateTriangle(normalized_phase);    // Onda triangular
      break;
    case 1:
      output = generatePulse(normalized_phase);       // Onda de pulso
      break;
    case 2:
      output = generateHarmonicSaw(normalized_phase); // Sierra armónica (por defecto)
      break;
    default:
      output = generateTriangle(normalized_phase);    // Fallback a triangular
      break;
  }
  
  // SUAVIZAR INICIO DE ONDA (evitar clicks al cambiar frecuencia)
  if (phase < 2.0) {
    output *= (phase / 2.0);  // Fade-in gradual en los primeros 2 samples
  }
  
  return output;
}

// ==============================================================================================================================================
// FUNCIÓN DE CONTROL DE BOTONES
// ==============================================================================================================================================

void checkButtons() {
  unsigned long current_time = millis();
  
  // LEER ESTADO ACTUAL DE LOS BOTONES
  // Los botones usan INPUT_PULLUP, así que LOW = presionado
  bool button_state = digitalRead(BUTTON_PIN);      // Botón 1: Play/Stop
  bool button2_state = digitalRead(BUTTON2_PIN);    // Botón 2: Velocidad-
  bool button3_state = digitalRead(BUTTON3_PIN);    // Botón 3: Velocidad+
  
  // BOTÓN 1: PLAY/STOP DEL SECUENCIADOR
  if (!button_state && !button_pressed && (current_time - last_button_time > BUTTON_DEBOUNCE)) {
    button_pressed = true;           // Marcar como presionado
    last_button_time = current_time; // Actualizar tiempo para antirebote
    
    // ALTERNAR ESTADO DEL SECUENCIADOR
    sequencer_playing = !sequencer_playing;  // Cambiar entre play/stop
    
    if (sequencer_playing) {
      // INICIAR SECUENCIA desde el principio
      current_step = 0;                           // Comenzar en el primer paso
      last_step_time = current_time;              // Marcar tiempo de inicio
      currentFreq = SEQUENCE_NOTES[current_step]; // Establecer primera frecuencia (Mi4)
      note_triggered = true;                      // Disparar primera nota
      note_start_time = current_time;             // Marcar inicio de nota
      audioPlaying = true;                        // Activar generación de audio
      updateLEDs();                               // Mostrar primer LED
    } else {
      // DETENER SECUENCIA
      audioPlaying = false;        // Desactivar audio
      envelope_level = 0.0;        // Silenciar envelope
      // Apagar todos los LEDs
      digitalWrite(LED1_PIN, LOW);
      digitalWrite(LED2_PIN, LOW);
      digitalWrite(LED3_PIN, LOW);
      digitalWrite(LED4_PIN, LOW);
    }
  }
  
  // BOTÓN 2: DISMINUIR VELOCIDAD (hacer más lento)
  if (!button2_state && !button2_pressed && (current_time - last_button2_time > BUTTON_DEBOUNCE)) {
    button2_pressed = true;           // Marcar como presionado
    last_button2_time = current_time; // Actualizar tiempo de antirebote
    
    step_duration += 10;              // Aumentar duración = más lento
    if (step_duration > MAX_STEP_DURATION) {
      step_duration = MAX_STEP_DURATION;  // No superar límite máximo
    }
  }
  
  // BOTÓN 3: AUMENTAR VELOCIDAD (hacer más rápido)
  if (!button3_state && !button3_pressed && (current_time - last_button3_time > BUTTON_DEBOUNCE)) {
    button3_pressed = true;           // Marcar como presionado
    last_button3_time = current_time; // Actualizar tiempo de antirebote
    
    step_duration -= 10;              // Disminuir duración = más rápido
    if (step_duration < MIN_STEP_DURATION) {
      step_duration = MIN_STEP_DURATION;  // No superar límite mínimo
    }
  }
  
  // DETECTAR CUANDO SE SUELTAN LOS BOTONES (para permitir nueva pulsación)
  if (button_state && button_pressed) {
    button_pressed = false;       // Liberar botón 1
  }
  if (button2_state && button2_pressed) {
    button2_pressed = false;      // Liberar botón 2
  }
  if (button3_state && button3_pressed) {
    button3_pressed = false;      // Liberar botón 3
  }
}

// ==============================================================================================================================================
// FUNCIÓN SETUP - CONFIGURACIÓN INICIAL (SE EJECUTA UNA VEZ)
// ==============================================================================================================================================

void setup() {
  // CONFIGURAR PINES DE LEDS como salidas digitales
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(LED4_PIN, OUTPUT);
  
  // CONFIGURAR BOTONES con resistencia pull-up interna
  // INPUT_PULLUP significa que el pin lee HIGH cuando no está presionado
  // y LOW cuando el botón se presiona (conecta a GND)
  pinMode(BUTTON_PIN, INPUT_PULLUP);    // Botón 1: Play/Stop
  pinMode(BUTTON2_PIN, INPUT_PULLUP);   // Botón 2: Velocidad-
  pinMode(BUTTON3_PIN, INPUT_PULLUP);   // Botón 3: Velocidad+
  
  // CONFIGURAR PINES DE POTENCIÓMETROS como entradas analógicas
  // (En realidad no es necesario hacer pinMode para pines analógicos,
  // pero es buena práctica documentarlo)
  pinMode(POT1_PIN, INPUT);   // Potenciómetro 1: Volumen
  pinMode(POT2_PIN, INPUT);   // Potenciómetro 2: Attack
  pinMode(POT3_PIN, INPUT);   // Potenciómetro 3: Saturación
  pinMode(POT4_PIN, INPUT);   // Potenciómetro 4: Decay
  
  // CONFIGURAR DAC (Digital to Analog Converter)
  dac_output_enable(DAC_CHANNEL_1);     // Habilitar DAC en pin 25
  dac_output_voltage(DAC_CHANNEL_1, 128); // Establecer voltaje medio (silencio)
  
  // INICIALIZAR SENSOR IMU
  initIMU();  // Configurar comunicación I2C y despertar sensor
  
  // CONFIGURACIÓN INICIAL DEL SISTEMA
  delay(100);                    // Esperar estabilización del hardware
  readIMU();                     // Leer valores iniciales del IMU
  updateFilterCoefficients();    // Calcular coeficientes iniciales del filtro
  
  // INICIALIZAR SECUENCIA
  currentFreq = SEQUENCE_NOTES[0];  // Establecer primera nota (Mi4)
}

// ==============================================================================================================================================
// FUNCIÓN LOOP - BUCLE PRINCIPAL (SE EJECUTA CONTINUAMENTE)
// ==============================================================================================================================================

void loop() {
  // VARIABLES ESTÁTICAS (mantienen su valor entre llamadas)
  static unsigned long lastAudioUpdate = 0;  // Tiempo de última actualización de audio
  static int filterUpdateCounter = 0;         // Contador para actualizar controles
  
  unsigned long currentTime = micros();  // Tiempo actual en microsegundos
  
  // ACTUALIZAR AUDIO A FRECUENCIA DE MUESTREO (22kHz)
  if (currentTime - lastAudioUpdate >= 45) {  // 45 microsegundos ≈ 22kHz
    lastAudioUpdate = currentTime;  // Actualizar tiempo de referencia
    
    // ACTUALIZAR CONTROLES cada ciertos samples (no en cada sample)
    // Esto evita sobrecargar el procesador con lecturas innecesarias
    if (filterUpdateCounter % 110 == 0) {  // Cada 110 samples ≈ 5ms
      readIMU();                    // Leer sensor de movimiento
      updateFilterCoefficients();   // Recalcular filtro según IMU
      updateSequencer();            // Actualizar secuenciador
      checkButtons();               // Verificar botones presionados
    }
    filterUpdateCounter++;  // Incrementar contador
    
    // GENERAR AUDIO si está activo
    if (audioPlaying) {
      // CALCULAR SAMPLES POR CICLO de la frecuencia actual
      float samples_per_cycle = SAMPLE_RATE / currentFreq;
      
      // GENERAR MUESTRA DE FORMA DE ONDA
      float waveform_sample = generateWaveform(samples_per_cycle);
      
      // APLICAR FILTRO (dos pasadas para mayor efecto)
      float filtered_sample = applyFilter(waveform_sample);     // Primera pasada
      filtered_sample = applyFilter(filtered_sample * 0.7);    // Segunda pasada (reducida)
      
      // APLICAR ENVELOPE (envolvente de volumen)
      filtered_sample *= envelope_level;  // Multiplicar por nivel actual del envelope
      
      // LEER POTENCIÓMETRO DE VOLUMEN (control maestro) - INVERTIDO
      int volume_pot = analogRead(POT1_PIN);        // Leer valor (0-4095)
      float volume = (4095 - volume_pot) / 4095.0;  // INVERTIR: máximo cuando pot=0, mínimo cuando pot=4095
      
      // CONVERTIR A VALOR PARA DAC (0-255)
      // El DAC usa 8 bits: 0 = 0V, 128 = 1.65V, 255 = 3.3V
      // 128 es el punto medio (silencio)
      int sample = 128 + (int)(AMPLITUDE * 0.6 * filtered_sample * volume);
      sample = constrain(sample, 0, 255);  // Asegurar rango válido
      
      // ENVIAR MUESTRA AL DAC (convertir digital a analógico)
      dac_output_voltage(DAC_CHANNEL_1, sample);
    } else {
      // Si no hay audio, mantener DAC en punto medio (silencio)
      dac_output_voltage(DAC_CHANNEL_1, 128);
    }
  }
}
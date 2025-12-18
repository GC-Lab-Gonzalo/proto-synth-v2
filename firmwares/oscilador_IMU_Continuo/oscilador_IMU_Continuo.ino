
// ==============================================================================================================================================
// PROTO-SYNTH V2 - IMU Controlled Synth con Formas de Onda Mejoradas - GC Lab Chile
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
// Sintetizador controlado por IMU con generación de ondas mejoradas, filtro dinámico y controles más precisos.
// ==============================================================================================================================================

// ==============================================================================================================================================
// FUNCIONAMIENTO
// ==============================================================================================================================================
// CONTROLES DE EXPRESIÓN:
// - Potenciómetro 1: Resonancia (Q)
// - Potenciómetro 2: Volumen general
// - Potenciómetro 3: Drive/Saturación antes del filtro
// - Potenciómetro 4: Frecuencia mínima del filtro (high-pass progresivo)
// - Botón 1: Cambio de rangos
// - Botón 2: Cambio de octava central
// - Botón 3: Nota central aleatoria
// - Botón 4: No se usa
// - LED 1: Agudos
// - LED 2: Medios
// - LED 3: Graves
// - LED 4: Silencio
// - IMU: Control de notas CONTINUO (rango de 3 notas centrado en E3) y filtro LPF (Resonancia al 75%)
// - LDR: No se usa
// - Micrófono: No se usa
// - Header 1: No se usa
// - Header 2: No se usa
// - Salida MIDI: No se usa
//
// MODO DE USO:
// 1. Inclina el protosynth en distintas posiciones y ve como varía tanto el tono como el filtro
// 2. Ajusta los potenciómetros para modificar la resonancia, rango de frecuencia del filtro, distorción y tipo de filtro
// ==============================================================================================================================================

// ==============================================================================================================================================
// COMENTARIOS
// ==============================================================================================================================================
// - Los potenciometros están invertidos, bajan cuando estan al maximo.
// - Para subir código exitosamente, asegúrate de que el Potenciómetro 3 esté girado al máximo.
// - Los Pines 2,4,12,13,14,15,25,26,27 no van a funcionar si el Bluetooth está activado ya que están conectados al ADC2 del ESP32.
// ==============================================================================================================================================

// ==============================================================================================================================================
// INCLUSIÓN DE LIBRERÍAS
// ==============================================================================================================================================
#include "driver/dac.h"
#include "math.h"
#include "Wire.h"

// Configuración de hardware
const int SDA_PIN = 21;
const int SCL_PIN = 22;
const int IMU_ADDRESS = 0x68;
const int SAMPLE_RATE = 22050;
const int AMPLITUDE = 90;

// Potenciómetros para control del filtro
const int POT1_PIN = 14; // Resonancia (Q)
const int POT2_PIN = 13; // Volumen general
const int POT3_PIN = 12; // Drive/Saturación antes del filtro
const int POT4_PIN = 27; // Frecuencia mínima del filtro (high-pass progresivo)

// Pines de LEDs (izquierda a derecha: agudo a silencio)
const int LED1_PIN = 23; // LED izquierda (agudos)
const int LED2_PIN = 32; // LED centro-izquierda
const int LED3_PIN = 5;  // LED centro-derecha  
const int LED4_PIN = 2;  // LED derecha (graves)

// Botones para control
const int BUTTON_PIN = 18;    // Botón para cambio de rangos
const int BUTTON2_PIN = 4;    // Botón para cambio de octava central
const int BUTTON3_PIN = 15;   // Botón para nota central aleatoria

// Variables del IMU
float imu_accel_x = 0.0;
float imu_accel_y = 0.0;
unsigned long lastIMURead = 0;
const unsigned long IMU_READ_INTERVAL = 20;

// *** CONFIGURACIÓN DE SENSIBILIDAD MEJORADA ***
// Filtros para suavizar IMU
float filtered_x = 0.0;
float filtered_y = 0.0;
const float IMU_FILTER_ALPHA = 0.3; // Aumentado para respuesta más rápida

// Rangos de sensibilidad reducidos (antes era ±1.0, ahora mucho menor)
const float X_SENSITIVITY_RANGE = 0.25; // Solo necesita ±0.25g (antes ±1.0g)
const float Y_SENSITIVITY_RANGE = 0.3;  // Solo necesita ±0.3g (antes ±1.0g)

// Multiplicadores para amplificar la señal del IMU
const float X_AMPLIFICATION = 4.0; // Amplifica X por 4
const float Y_AMPLIFICATION = 3.3; // Amplifica Y por 3.3

// Variables del filtro
float filter_x1 = 0, filter_x2 = 0;
float filter_y1 = 0, filter_y2 = 0;
float filter_a0, filter_a1, filter_a2;
float filter_b1, filter_b2;
int filter_type = 0; // 0=lowpass, 1=highpass, 2=bandpass

// Definir las tres octavas centrales posibles
const float E2_FREQ = 82.41;   // E2 - Octava grave
const float E3_FREQ = 164.81;  // E3 - Octava media (original)
const float E4_FREQ = 329.63;  // E4 - Octava aguda

// Escala menor armónica para notas centrales aleatorias (sonido distópico/tecnológico, octava 3)
const float MELODIC_SCALE[] = {
  130.81,  // C3
  146.83,  // D3
  155.56,  // Eb3 (D#3)
  174.61,  // F3
  196.00,  // G3
  207.65,  // Ab3 (G#3)
  246.94   // B3 (natural, no Bb)
};
const int MELODIC_SCALE_SIZE = 7;

// Definir frecuencias para los diferentes rangos continuos
// Estos son los INTERVALOS relativos que se aplicarán a cualquier octava central

// Rango 0: 3 notas (intervalo de tono y medio)
const float RANGE0_RATIO_LOW = 0.8909;   // D relativo (D3/E3 = 146.83/164.81)
const float RANGE0_RATIO_HIGH = 1.0594;  // F relativo (F3/E3 = 174.61/164.81)

// Rango 1: 5 notas (intervalo de tercera mayor + cuarta)
const float RANGE1_RATIO_LOW = 0.7937;   // C relativo (C3/E3 = 130.81/164.81)
const float RANGE1_RATIO_HIGH = 1.2599;  // G# relativo (G#3/E3 = 207.65/164.81)

// Rango 2: 1 octava
const float RANGE2_RATIO_LOW = 0.7492;   // B relativo (B2/E3 = 123.47/164.81)
const float RANGE2_RATIO_HIGH = 1.3348;  // A relativo (A3/E3 = 220.00/164.81)

// Rango 3: 1.5 octavas
const float RANGE3_RATIO_LOW = 0.6299;   // G# relativo (G#2/E3 = 103.83/164.81)
const float RANGE3_RATIO_HIGH = 1.5874;  // C relativo (C4/E3 = 261.63/164.81)

// Rango 4: 2 octavas
const float RANGE4_RATIO_LOW = 0.5297;   // F relativo (F2/E3 = 87.31/164.81)
const float RANGE4_RATIO_HIGH = 2.0000;  // E relativo (E4/E3 = 329.63/164.81)

// Rango 5: 3 octavas
const float RANGE5_RATIO_LOW = 0.3968;   // C relativo (C2/E3 = 65.41/164.81)
const float RANGE5_RATIO_HIGH = 3.1746;  // C relativo (C5/E3 = 523.25/164.81)

// Variables de control
int current_range = 0; // Rango actual (0-5)
int current_octave = 1; // Octava central actual (0=E2, 1=E3, 2=E4)
float custom_center_freq = 0.0; // Frecuencia central personalizada (0 = usar octava normal)
bool button_pressed = false;
bool button2_pressed = false;
bool button3_pressed = false;
unsigned long last_button_time = 0;
unsigned long last_button2_time = 0;
unsigned long last_button3_time = 0;
const unsigned long BUTTON_DEBOUNCE = 200; // 200ms debounce

// Variables de audio
float currentFreq = 164.81; // E3 inicial
bool audioPlaying = true; // Siempre tocando, sin silencio
int current_scale_index = 8; // Para mantener compatibilidad con LEDs

// Variables para generación de ondas
float phase = 0.0;
int waveform_type = 0; // 0=triangle, 1=pulse, 2=harmonic_saw

// Variables para modulación suave
float modulation_phase = 0.0;

void initIMU() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  
  // Despertar MPU6050
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  
  // Configurar acelerómetro ±2g (mantener para máxima resolución)
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true);
  
  delay(100);
}

void readIMU() {
  unsigned long currentTime = millis();
  if (currentTime - lastIMURead < IMU_READ_INTERVAL) {
    return;
  }
  lastIMURead = currentTime;
  
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU_ADDRESS, 6, true);
  
  if (Wire.available() >= 6) {
    int16_t raw_x = Wire.read() << 8 | Wire.read();
    int16_t raw_y = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read(); // Descartar Z
    
    // Convertir a g
    imu_accel_x = raw_x / 16384.0;
    imu_accel_y = raw_y / 16384.0;
    
    // *** APLICAR AMPLIFICACIÓN Y SENSIBILIDAD ***
    // Amplificar las señales para mayor sensibilidad
    float amplified_x = imu_accel_x * X_AMPLIFICATION;
    float amplified_y = imu_accel_y * Y_AMPLIFICATION;
    
    // Limitar a los rangos de sensibilidad reducidos
    amplified_x = constrain(amplified_x, -X_SENSITIVITY_RANGE, X_SENSITIVITY_RANGE);
    amplified_y = constrain(amplified_y, -Y_SENSITIVITY_RANGE, Y_SENSITIVITY_RANGE);
    
    // Normalizar a rango ±1.0 para el resto del código
    float normalized_x = amplified_x / X_SENSITIVITY_RANGE;
    float normalized_y = amplified_y / Y_SENSITIVITY_RANGE;
    
    // Suavizar con filtro más responsivo
    filtered_x = filtered_x * (1.0 - IMU_FILTER_ALPHA) + normalized_x * IMU_FILTER_ALPHA;
    filtered_y = filtered_y * (1.0 - IMU_FILTER_ALPHA) + normalized_y * IMU_FILTER_ALPHA;
  }
}

void updateFilterCoefficients() {
  // Eje Y controla frecuencia de corte (ahora más sensible)
  float y_abs = abs(filtered_y);
  y_abs = constrain(y_abs, 0.0, 1.0);
  
  // POT4 controla la frecuencia mínima del filtro
  int pot4_value = analogRead(POT4_PIN);
  float pot4_normalized = pot4_value / 4095.0; // 0.0 a 1.0
  float min_freq = 50.0 + pot4_normalized * 1950.0; // Mínimo de 50Hz a 2000Hz
  
  // Calcular frecuencia de corte con nuevo rango dinámico
  float max_freq = 8000.0; // Mantener máximo en 8000Hz
  float cutoff_freq = min_freq + y_abs * (max_freq - min_freq);
  
  // Leer potenciómetro para resonancia (Q de 0.5 a 20)
  int pot_value = analogRead(POT1_PIN);
  float pot_normalized = pot_value / 4095.0; // Normalizar a 0.0-1.0
  float resonance = 0.5 + pot_normalized * 19.5; // Q de 0.5 a 20
  
  // Normalizar frecuencia de corte
  float norm_freq = cutoff_freq / (SAMPLE_RATE / 2.0);
  norm_freq = constrain(norm_freq, 0.01, 0.95);
  
  float omega = PI * norm_freq;
  float sin_omega = sin(omega);
  float cos_omega = cos(omega);
  float alpha = sin_omega / (2.0 * resonance);
  
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

float applyFilter(float input) {
  // POT3: Drive/Saturación - MÁS AGRESIVO
  int pot3_value = analogRead(POT3_PIN);
  float pot3_normalized = pot3_value / 4095.0;
  float drive = 1.0 + pot3_normalized * 9.0; // Drive de 1x a 10x (más extremo)
  
  // Aplicar drive con saturación más notable
  float driven_input = input * drive;
  
  // Saturación tipo tanh para más carácter
  if (driven_input > 1.5) {
    driven_input = 0.9; // Clip duro
  } else if (driven_input < -1.5) {
    driven_input = -0.9; // Clip duro
  } else if (driven_input > 0.8) {
    driven_input = 0.8 + (driven_input - 0.8) * 0.2; // Saturación suave
  } else if (driven_input < -0.8) {
    driven_input = -0.8 + (driven_input + 0.8) * 0.2; // Saturación suave
  }
  
  // Aplicar filtro
  float output = filter_a0 * driven_input + filter_a1 * filter_x1 + filter_a2 * filter_x2
                 - filter_b1 * filter_y1 - filter_b2 * filter_y2;
  
  filter_x2 = filter_x1;
  filter_x1 = driven_input;
  filter_y2 = filter_y1;
  filter_y1 = output;
  
  return output;
}

void updateFrequency() {
  // Eje X controla frecuencia de manera CONTINUA (ahora más sensible)
  float x_value = filtered_x;
  
  // Ya está normalizado entre -1.0 y 1.0 por el nuevo sistema de sensibilidad
  x_value = constrain(x_value, -1.0, 1.0);
  
  // Obtener la frecuencia central
  float center_freq;
  
  if (custom_center_freq > 0.0) {
    // Usar frecuencia central personalizada (nota aleatoria)
    center_freq = custom_center_freq;
  } else {
    // Usar octava normal según current_octave
    switch (current_octave) {
      case 0: center_freq = E2_FREQ; break; // E2
      case 1: center_freq = E3_FREQ; break; // E3
      case 2: center_freq = E4_FREQ; break; // E4
      default: center_freq = E3_FREQ; break;
    }
  }
  
  // Obtener los ratios del rango actual
  float ratio_low, ratio_high;
  
  switch (current_range) {
    case 0: // Rango original: 3 notas
      ratio_low = RANGE0_RATIO_LOW;
      ratio_high = RANGE0_RATIO_HIGH;
      break;
      
    case 1: // Rango amplio: 5 notas
      ratio_low = RANGE1_RATIO_LOW;
      ratio_high = RANGE1_RATIO_HIGH;
      break;
      
    case 2: // Rango 1 octava
      ratio_low = RANGE2_RATIO_LOW;
      ratio_high = RANGE2_RATIO_HIGH;
      break;
      
    case 3: // Rango 1.5 octavas
      ratio_low = RANGE3_RATIO_LOW;
      ratio_high = RANGE3_RATIO_HIGH;
      break;
      
    case 4: // Rango 2 octavas
      ratio_low = RANGE4_RATIO_LOW;
      ratio_high = RANGE4_RATIO_HIGH;
      break;
      
    case 5: // Rango 3 octavas EXTREMO
      ratio_low = RANGE5_RATIO_LOW;
      ratio_high = RANGE5_RATIO_HIGH;
      break;
      
    default:
      ratio_low = RANGE0_RATIO_LOW;
      ratio_high = RANGE0_RATIO_HIGH;
      break;
  }
  
  // Calcular frecuencias absolutas aplicando los ratios a la frecuencia central
  float freq_low = center_freq * ratio_low;
  float freq_high = center_freq * ratio_high;
  
  // Mapear X de -1.0 a 1.0 hacia el rango de frecuencias seleccionado
  if (x_value >= 0.0) {
    // Rango positivo: de center a high
    currentFreq = center_freq + (x_value * (freq_high - center_freq));
  } else {
    // Rango negativo: de low a center
    currentFreq = center_freq + (x_value * (center_freq - freq_low));
  }
  
  // Siempre reproducir audio (sin silencio)
  audioPlaying = true;
  waveform_type = 2; // Mantener la forma de onda harmonic_saw
  
  // Actualizar LEDs basado en la frecuencia actual
  updateLEDs(x_value);
}

// Función para generar onda triangular (menos armónicos agudos)
float generateTriangle(float normalized_phase) {
  if (normalized_phase < 0.5) {
    return 4.0 * normalized_phase - 1.0; // Subida: -1 a 1
  } else {
    return 3.0 - 4.0 * normalized_phase; // Bajada: 1 a -1
  }
}

// Función para generar onda de pulso con duty cycle variable
float generatePulse(float normalized_phase) {
  float duty_cycle = 0.3 + 0.4 * (filtered_y + 1.0) / 2.0; // 30% a 70%
  duty_cycle = constrain(duty_cycle, 0.2, 0.8);
  
  return (normalized_phase < duty_cycle) ? 1.0 : -1.0;
}

// Función para generar onda diente de sierra con armónicos limitados
float generateHarmonicSaw(float normalized_phase) {
  float output = 0.0;
  
  // Sumar solo los primeros 5 armónicos para reducir ruido
  for (int harmonic = 1; harmonic <= 5; harmonic++) {
    float amplitude = 1.0 / harmonic; // Amplitud inversamente proporcional
    output += amplitude * sin(2.0 * PI * harmonic * normalized_phase);
  }
  
  return output * 0.5; // Reducir amplitud total
}

float generateWaveform(float samples_per_cycle) {
  phase += 1.0;
  if (phase >= samples_per_cycle) {
    phase = 0.0;
  }
  
  float normalized_phase = phase / samples_per_cycle; // 0.0 a 1.0
  float output = 0.0;
  
  switch (waveform_type) {
    case 0: // Onda triangular
      output = generateTriangle(normalized_phase);
      break;
      
    case 1: // Onda de pulso
      output = generatePulse(normalized_phase);
      break;
      
    case 2: // Diente de sierra con armónicos limitados
      output = generateHarmonicSaw(normalized_phase);
      break;
      
    default:
      output = generateTriangle(normalized_phase);
      break;
  }
  
  // Suavizar transiciones para evitar clicks
  if (phase < 2.0) {
    output *= (phase / 2.0);
  }
  
  return output;
}

void updateLEDs(float x_value) {
  // Apagar todos los LEDs primero
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  digitalWrite(LED3_PIN, LOW);
  digitalWrite(LED4_PIN, LOW);
  
  // Actualizar LEDs basado en la posición X (continua)
  if (x_value > 0.5) {
    // Agudos altos: LED1 (izquierda)
    digitalWrite(LED1_PIN, HIGH);
    
  } else if (x_value > 0.0) {
    // Agudos medios: LED2 (centro-izquierda)
    digitalWrite(LED2_PIN, HIGH);
    
  } else if (x_value > -0.5) {
    // Centro a graves: LED3 (centro-derecha)
    digitalWrite(LED3_PIN, HIGH);
    
  } else {
    // Graves profundos: LED4 (derecha)
    digitalWrite(LED4_PIN, HIGH);
  }
}

void checkButton() {
  unsigned long current_time = millis();
  bool button_state = digitalRead(BUTTON_PIN);
  bool button2_state = digitalRead(BUTTON2_PIN);
  bool button3_state = digitalRead(BUTTON3_PIN);
  
  // Botón 1: Cambio de rangos (pin 18)
  if (!button_state && !button_pressed && (current_time - last_button_time > BUTTON_DEBOUNCE)) {
    button_pressed = true;
    last_button_time = current_time;
    
    // Cambiar al siguiente rango (0-5, luego vuelve a 0)
    current_range = (current_range + 1) % 6;
    
    // NO delays - cambio instantáneo
  }
  
  // Botón 2: Cambio de octava central (pin 4)
  if (!button2_state && !button2_pressed && (current_time - last_button2_time > BUTTON_DEBOUNCE)) {
    button2_pressed = true;
    last_button2_time = current_time;
    
    // Cambiar a la siguiente octava (0=E2, 1=E3, 2=E4, luego vuelve a 0)
    current_octave = (current_octave + 1) % 3;
    
    // Resetear frecuencia personalizada para usar octavas normales
    custom_center_freq = 0.0;
    
    // NO delays - cambio instantáneo
  }
  
  // Botón 3: Nota central aleatoria melódica (pin 15)
  if (!button3_state && !button3_pressed && (current_time - last_button3_time > BUTTON_DEBOUNCE)) {
    button3_pressed = true;
    last_button3_time = current_time;
    
    // Elegir una nota aleatoria de la escala mayor
    int random_index = random(MELODIC_SCALE_SIZE);
    float base_note = MELODIC_SCALE[random_index];
    
    // Aplicar la nota en la octava actual
    float octave_multiplier;
    switch (current_octave) {
      case 0: octave_multiplier = 0.5; break;  // Octava 2 (grave)
      case 1: octave_multiplier = 1.0; break;  // Octava 3 (media)
      case 2: octave_multiplier = 2.0; break;  // Octava 4 (aguda)
      default: octave_multiplier = 1.0; break;
    }
    
    custom_center_freq = base_note * octave_multiplier;
    
    // NO delays - cambio instantáneo
  }
  
  // Detectar cuando se sueltan los botones
  if (button_state && button_pressed) {
    button_pressed = false;
  }
  if (button2_state && button2_pressed) {
    button2_pressed = false;
  }
  if (button3_state && button3_pressed) {
    button3_pressed = false;
  }
}

void setup() {
  // Configurar pines de LEDs
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(LED4_PIN, OUTPUT);
  
  // Configurar botones con pull-up interno
  pinMode(BUTTON_PIN, INPUT_PULLUP);   // Botón rangos (pin 18)
  pinMode(BUTTON2_PIN, INPUT_PULLUP);  // Botón octavas (pin 4)
  pinMode(BUTTON3_PIN, INPUT_PULLUP);  // Botón nota aleatoria (pin 15)
  
  // Configurar pines analógicos de potenciómetros
  pinMode(POT1_PIN, INPUT);
  pinMode(POT2_PIN, INPUT);
  pinMode(POT3_PIN, INPUT);
  pinMode(POT4_PIN, INPUT);
  
  // Configurar DAC
  dac_output_enable(DAC_CHANNEL_1);
  dac_output_voltage(DAC_CHANNEL_1, 128);
  
  // Inicializar IMU
  initIMU();
  
  // Leer IMU inicial
  delay(100);
  readIMU();
  updateFilterCoefficients();
}

void loop() {
  static unsigned long lastAudioUpdate = 0;
  static int filterUpdateCounter = 0;
  
  unsigned long currentTime = micros();
  
  // Actualizar audio a frecuencia de muestreo
  if (currentTime - lastAudioUpdate >= 45) { // 22kHz
    lastAudioUpdate = currentTime;
    
    // Leer IMU cada ciertos samples
    if (filterUpdateCounter % 110 == 0) { // Cada 5ms aprox
      readIMU();
      updateFrequency();
      updateFilterCoefficients();
      checkButton(); // Verificar botón para cambio de rangos
    }
    filterUpdateCounter++;
    
    if (audioPlaying) {
      float samples_per_cycle = SAMPLE_RATE / currentFreq;
      
      // Generar forma de onda seleccionada
      float waveform_sample = generateWaveform(samples_per_cycle);
      
      // Aplicar filtro doble para mejor limpieza
      float filtered_sample = applyFilter(waveform_sample);
      filtered_sample = applyFilter(filtered_sample * 0.7); // Segunda pasada más suave
      
      // Leer potenciómetro de volumen (POT2)
      int volume_pot = analogRead(POT2_PIN);
      float volume = volume_pot / 4095.0; // Normalizar a 0.0-1.0
      
      // Convertir a DAC (0-255) aplicando volumen
      int sample = 128 + (int)(AMPLITUDE * 0.6 * filtered_sample * volume);
      sample = constrain(sample, 0, 255);
      
      dac_output_voltage(DAC_CHANNEL_1, sample);
    } else {
      dac_output_voltage(DAC_CHANNEL_1, 128);
    }
  }
}
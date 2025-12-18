
// ==============================================================================================================================================
// PROTO-SYNTH V2 - IMU Controlled Synth - GC Lab Chile
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
// Sintetizador controlado por IMU que genera tonos basados en la inclinación del dispositivo.
// ==============================================================================================================================================

// ==============================================================================================================================================
// FUNCIONAMIENTO
// ==============================================================================================================================================
// CONTROLES DE EXPRESIÓN:
// - Potenciómetro 1: Resonancia (Q)
// - Potenciómetro 2: Rango de frecuencia del filtro (min/max)
// - Potenciómetro 3: Drive/Saturación antes del filtro
// - Potenciómetro 4: Tipo de filtro (LP/HP/BP)
// - Botón 1: No se usa
// - Botón 2: No se usa
// - Botón 3: No se usa
// - Botón 4: No se usa
// - LED 1: Agudos
// - LED 2: Medios
// - LED 3: Graves
// - LED 4: Silencio
// - IMU: Control de notas (escala flamenca en Mi) y filtro LPF (Resonancia al 75%)
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

// ==============================================================================================================================================
// CONFIGURACIÓN DE HARDWARE - PINES
// ==============================================================================================================================================
const int SDA_PIN = 21;
const int SCL_PIN = 22;
const int IMU_ADDRESS = 0x68;
const int SAMPLE_RATE = 22050;
const int AMPLITUDE = 90;

// Potenciómetros para control del filtro
const int POT1_PIN = 13;
const int POT2_PIN = 14;
const int POT3_PIN = 12;
const int POT4_PIN = 27;

// Pines de LEDs (izquierda a derecha: agudo a silencio)
const int LED1_PIN = 23; // LED izquierda (agudos)
const int LED2_PIN = 32; // LED centro-izquierda
const int LED3_PIN = 5;  // LED centro-derecha  
const int LED4_PIN = 2;  // LED derecha (silencio)

// ==============================================================================================================================================
// PROGRAMA
// ==============================================================================================================================================
// Variables del IMU
float imu_accel_x = 0.0;
float imu_accel_y = 0.0;
unsigned long lastIMURead = 0;
const unsigned long IMU_READ_INTERVAL = 20;

// Filtros para suavizar IMU
float filtered_x = 0.0;
float filtered_y = 0.0;
const float IMU_FILTER_ALPHA = 0.2;

// Variables del filtro
float filter_x1 = 0, filter_x2 = 0;
float filter_y1 = 0, filter_y2 = 0;
float filter_a0, filter_a1, filter_a2;
float filter_b1, filter_b2;
int filter_type = 0; // 0=lowpass, 1=highpass, 2=bandpass

// Escala flamenca en Mi extendida (D2 a A5)
const float FLAMENCO_SCALE_E[] = {
  73.42,   // D2 - Nueva nota más grave
  82.41,   // E2
  87.31,   // F2
  103.83,  // G#2
  110.00,  // A2
  123.47,  // B2
  130.81,  // C3
  146.83,  // D3
  164.81,  // E3 - Centro de la escala (índice 8)
  174.61,  // F3
  207.65,  // G#3
  220.00,  // A3
  246.94,  // B3
  261.63,  // C4
  293.66,  // D4
  329.63,  // E4
  349.23,  // F4
  415.30,  // G#4
  440.00,  // A4
  493.88,  // B4
  523.25,  // C5
  587.33,  // D5
  659.25,  // E5
  698.46,  // F5
  830.61,  // G#5
  880.00,  // A5
};

const int SCALE_SIZE = sizeof(FLAMENCO_SCALE_E) / sizeof(float);

// Variables de audio
float currentFreq = 164.81; // E3 inicial (ahora índice 8)
bool audioPlaying = false;
int current_scale_index = 8; // E3 en el centro de la nueva escala

// Variables para generación de ondas
float phase = 0.0;
int waveform_type = 0; // 0=triangle, 1=pulse, 2=harmonic_saw

void initIMU() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  
  // Despertar MPU6050
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  
  // Configurar acelerómetro ±2g
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
    
    // Suavizar
    filtered_x = filtered_x * (1.0 - IMU_FILTER_ALPHA) + imu_accel_x * IMU_FILTER_ALPHA;
    filtered_y = filtered_y * (1.0 - IMU_FILTER_ALPHA) + imu_accel_y * IMU_FILTER_ALPHA;
    
    audioPlaying = true;
  }
}

void updateFilterCoefficients() {
  // Eje Y controla frecuencia de corte - Rango mucho más amplio (50Hz - 8000Hz)
  float y_abs = abs(filtered_y);
  y_abs = constrain(y_abs, 0.0, 1.0);
  float cutoff_freq = 50.0 + y_abs * 7950.0; // Rango expandido: 50Hz a 8000Hz
  
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
  // Eje X controla las notas - ahora usa valores con signo
  float x_value = filtered_x; // Mantener el signo
  
  // Zona muerta muy pequeña en el centro (±0.02g) para estabilidad
  if (abs(x_value) < 0.02) {
    // En el centro exacto: E3 (índice 8)
    audioPlaying = true;
    current_scale_index = 8; // E3
    currentFreq = FLAMENCO_SCALE_E[8];
    waveform_type = 2;
    updateLEDs(x_value);
    return;
  }
  
  if (x_value > 0.02) {
    // Eje positivo: subir notas desde E3 hacia A5
    audioPlaying = true;
    
    // Mapear de 0.02 a 1.0 a índices 8 a 25 (E3 a A5)
    float x_mapped = (x_value - 0.02) / (1.0 - 0.02);
    x_mapped = constrain(x_mapped, 0.0, 1.0);
    
    current_scale_index = 8 + (int)(x_mapped * (SCALE_SIZE - 9)); // De índice 8 a 25
    current_scale_index = constrain(current_scale_index, 8, SCALE_SIZE - 1);
    
    currentFreq = FLAMENCO_SCALE_E[current_scale_index];
    waveform_type = 2;
    
  } else if (x_value < -0.02) {
    // Eje negativo: bajar desde E3 hacia D2, luego silencio
    float x_neg = abs(x_value); // Hacer positivo para cálculos
    
    if (x_neg > 0.8) {
      // En el extremo negativo: silencio
      audioPlaying = false;
      current_scale_index = -1;
      updateLEDs(x_value);
      return;
    }
    
    // Mapear de 0.02 a 0.8 a índices 8 a 0 (E3 a D2)
    float x_mapped = (x_neg - 0.02) / (0.8 - 0.02);
    x_mapped = constrain(x_mapped, 0.0, 1.0);
    
    current_scale_index = 8 - (int)(x_mapped * 8); // De índice 8 a 0
    current_scale_index = constrain(current_scale_index, 0, 8);
    
    audioPlaying = true;
    currentFreq = FLAMENCO_SCALE_E[current_scale_index];
    waveform_type = 2;
  }
  
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
  
  if (x_value > 0.5) {
    // Agudos altos: LED1 (izquierda)
    digitalWrite(LED1_PIN, HIGH);
    
  } else if (x_value > 0.02) {
    // Agudos medios: LED2 (centro-izquierda)
    digitalWrite(LED2_PIN, HIGH);
    
  } else if (x_value > -0.4) {
    // Centro y graves cercanos: LED3 (centro-derecha)
    digitalWrite(LED3_PIN, HIGH);
    
  } else {
    // Graves profundos y silencio: LED4 (derecha)
    digitalWrite(LED4_PIN, HIGH);
  }
}

void setup() {
  // Configurar pines de LEDs
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(LED4_PIN, OUTPUT);
  
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
    }
    filterUpdateCounter++;
    
    if (audioPlaying) {
      float samples_per_cycle = SAMPLE_RATE / currentFreq;
      
      // Generar forma de onda seleccionada
      float waveform_sample = generateWaveform(samples_per_cycle);
      
      // Aplicar filtro doble para mejor limpieza
      float filtered_sample = applyFilter(waveform_sample);
      filtered_sample = applyFilter(filtered_sample * 0.7); // Segunda pasada más suave
      
      // Convertir a DAC (0-255) con menos amplitud
      int sample = 128 + (int)(AMPLITUDE * 0.6 * filtered_sample);
      sample = constrain(sample, 0, 255);
      
      dac_output_voltage(DAC_CHANNEL_1, sample);
    } else {
      dac_output_voltage(DAC_CHANNEL_1, 128);
    }
  }
}

// ==============================================================================================================================================
// PROTO-SYNTH V2 - OSCILADOR CUANTIZADO - GC Lab Chile
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
// Sintetizador simple con onda cuadrada sintonizado para tocar backing tracks en La en diferentes escalas musicales
// ==============================================================================================================================================

// ==============================================================================================================================================
// FUNCIONAMIENTO
// ==============================================================================================================================================
// CONTROLES DE EXPRESIÓN:
// - Potenciómetro 1: Frecuencia/tono del oscilador
// - Potenciómetro 2: Resonancia del filtro
// - Potenciómetro 3: Volumen de salida 
// - Potenciómetro 4: Frecuencia de corte del filtro LPF
// - Botón 1: Cambia la escala musical
// - Botón 2: Activa/desactiva octava superior (toggle)
// - Botón 3: No se usa
// - Botón 4: No se usa
// - LED 1: No se usa
// - LED 2: No se usa
// - LED 3: No se usa
// - LED 4: No se usa
// - IMU: No se usa
// - LDR: Control de filtro LPF
// - Micrófono: No se usa
// - Header 1: No se usa
// - Header 2: No se usa
// - Salida MIDI: No se usa
//
// MODO DE USO:
// 1. Gira el potenciómetro 1 para cambiar la frecuencia
// 2. Gira el potenciómetro 2 para ajustar la resonancia del filtro
// 3. Gira el potenciómetro 3 para ajustar el volumen
// 4. Gira el potenciómetro 4 para ajustar la frecuencia de corte del filtro
// 5. Presiona el botón 1 para cambiar escalas musicales
// 6. Presiona el botón 2 para activar/desactivar octava superior
// 7. Gira el potenciómetro 1 al mínimo para silenciar el audio
//
// INFORMACIÓN DEL CODIGO:
// - ESCALAS MUSICALES DISPONIBLES:
//   1. Mayor (Do-Re-Mi-Fa-Sol-La-Si) - Sonido alegre
//   2. Menor Natural - Sonido melancólico
//   3. Menor Armónica - Sonido exótico
//   4. Árabe - Sonido oriental
//   5. Lidia - Sonido etéreo
//   6. Frigia - Sonido misterioso
//   7. Locria - Sonido muy oscuro/dissonante
// - Cuantización musical: Las frecuencias se "ajustan" a notas musicales reales
// - 2 octavas de rango base (La3 a La5) + controles de octava
// - Cambios instantáneos entre notas (sin glide/portamento)
// - Silenciado automático cuando el potenciómetro está al mínimo
// - 7 escalas musicales diferentes, todas centradas en LA
// - Onda cuadrada para sonido digital característico
// - Perfecto para backing tracks en tonalidad LA
// ==============================================================================================================================================

// ==============================================================================================================================================
// COMENTARIOS
// ==============================================================================================================================================
// - Tiene un sonido extraño, y fue un momento de experimentación que no logre hacerme el tiempo para pulir, hay que mejorar 
// - Para subir código exitosamente, asegúrate de que el Potenciómetro 3 esté girado al máximo.
// - Los Pines 2,4,12,13,14,15,25,26,27 no van a funcionar si el Bluetooth está activado ya que están conectados al ADC2 del ESP32.
// ==============================================================================================================================================

// ==============================================================================================================================================
// INCLUSIÓN DE LIBRERÍAS
// ==============================================================================================================================================
#include "driver/dac.h"  // Control del DAC (Digital to Analog Converter) del ESP32
#include "math.h"        // Funciones matemáticas avanzadas

// ==============================================================================================================================================
// CONFIGURACIÓN DE HARDWARE - PINES
// ==============================================================================================================================================
const int POT_FREQ_PIN = 13;     // Potenciómetro 1 - control de frecuencia
const int POT_RES_PIN = 14;      // Potenciómetro 2 - control de resonancia
const int POT_VOL_PIN = 12;      // Potenciómetro 3 - control de volumen
const int POT_FILTER_PIN = 27;   // Potenciómetro 4 - control de frecuencia de corte del filtro
const int SCALE_BTN_PIN = 18;    // Botón 1 - cambiar escala
const int OCT_BTN_PIN = 4;       // Botón 2 - toggle octava superior
const int DAC_PIN = 25;          // Salida de audio

// ==============================================================================================================================================
// CONFIGURACIÓN DEL OSCILADOR
// ==============================================================================================================================================
const int SAMPLE_RATE = 16000;   // Frecuencia de muestreo alta para mejor calidad
const float BASE_FREQ = 127.0;   // Subiendo más para alcanzar Si3 exacto
float current_frequency = 247.0; // Frecuencia actual (aproximadamente Si3)
float phase = 0.0;               // Fase del oscilador
int amplitude = 90;              // Amplitud actual (volumen)

// ==============================================================================================================================================
// VARIABLES DEL FILTRO LPF CON RESONANCIA
// ==============================================================================================================================================
float filter_cutoff = 1000.0;   // Frecuencia de corte del filtro (Hz)
float filter_resonance = 0.5;   // Resonancia del filtro (0.0 - 0.95)
float filter_x1 = 0.0;          // Estado del filtro - entrada anterior
float filter_x2 = 0.0;          // Estado del filtro - entrada anterior 2
float filter_y1 = 0.0;          // Estado del filtro - salida anterior
float filter_y2 = 0.0;          // Estado del filtro - salida anterior 2
float filter_a0, filter_a1, filter_a2; // Coeficientes del filtro
float filter_b1, filter_b2;     // Coeficientes del filtro

// ==============================================================================================================================================
// CONFIGURACIÓN MUSICAL
// ==============================================================================================================================================
int current_scale = 0;           // Escala actual (0-6)
int current_note = 0;            // Nota actual (0-15)

// Escalas musicales (intervalos en semitonos)
const int scales[][8] = {
  {0, 2, 4, 5, 7, 9, 11, 12},   // Mayor
  {0, 2, 3, 5, 7, 8, 10, 12},   // Menor Natural
  {0, 2, 3, 5, 7, 8, 11, 12},   // Menor Armónica
  {0, 1, 3, 4, 6, 8, 10, 12},   // Árabe
  {0, 2, 4, 6, 7, 9, 11, 12},   // Lidia
  {0, 1, 4, 5, 7, 8, 11, 12},   // Frigia
  {0, 1, 3, 5, 6, 8, 10, 12}    // Locria
};

// ==============================================================================================================================================
// VARIABLES DE CONTROL
// ==============================================================================================================================================
bool scale_btn_last = HIGH;
bool oct_btn_last = HIGH;
bool octave_active = false;     // Estado de la octava (on/off)
unsigned long last_scale_press = 0;
unsigned long last_oct_press = 0;
unsigned long last_update = 0;
bool is_silenced = false;

// ==============================================================================================================================================
// SETUP - CONFIGURACIÓN INICIAL
// ==============================================================================================================================================
void setup() {
  // Configurar pines
  pinMode(POT_FREQ_PIN, INPUT);
  pinMode(POT_RES_PIN, INPUT);
  pinMode(POT_VOL_PIN, INPUT);
  pinMode(POT_FILTER_PIN, INPUT);
  pinMode(SCALE_BTN_PIN, INPUT_PULLUP);
  pinMode(OCT_BTN_PIN, INPUT_PULLUP);
  
  // Configurar DAC
  dac_output_enable(DAC_CHANNEL_1);
  dac_output_voltage(DAC_CHANNEL_1, 128);
  
  // Deshabilitar logging
  esp_log_level_set("*", ESP_LOG_NONE);
  
  // Inicializar filtro
  update_filter_coefficients();
}

// ==============================================================================================================================================
// LOOP PRINCIPAL
// ==============================================================================================================================================
void loop() {
  unsigned long now = millis();
  
  // Leer controles cada 10ms
  if (now - last_update >= 10) {
    read_controls();
    last_update = now;
  }
  
  // Generar audio
  generate_audio();
}

// ==============================================================================================================================================
// LEER CONTROLES
// ==============================================================================================================================================
void read_controls() {
  // BOTÓN DE ESCALA (con antirebote)
  bool scale_btn_now = digitalRead(SCALE_BTN_PIN);
  if (scale_btn_now == LOW && scale_btn_last == HIGH && 
      (millis() - last_scale_press) > 200) {
    current_scale = (current_scale + 1) % 7;
    calculate_frequency();
    last_scale_press = millis();
  }
  scale_btn_last = scale_btn_now;
  
  // BOTÓN DE OCTAVA (toggle con antirebote)
  bool oct_btn_now = digitalRead(OCT_BTN_PIN);
  if (oct_btn_now == LOW && oct_btn_last == HIGH && 
      (millis() - last_oct_press) > 200) {
    octave_active = !octave_active;  // Cambiar estado on/off
    calculate_frequency();
    last_oct_press = millis();
  }
  oct_btn_last = oct_btn_now;
  
  // POTENCIÓMETRO DE FRECUENCIA (invertido)
  int freq_pot_raw = analogRead(POT_FREQ_PIN);
  int freq_pot = 4095 - freq_pot_raw;
  
  // POTENCIÓMETRO DE RESONANCIA (invertido)
  int res_pot_raw = analogRead(POT_RES_PIN);
  int res_pot = 4095 - res_pot_raw;
  float new_resonance = map(res_pot, 0, 4095, 0, 95) / 100.0;  // 0.0 - 0.95
  
  // POTENCIÓMETRO DE VOLUMEN (invertido)
  int vol_pot_raw = analogRead(POT_VOL_PIN);
  int vol_pot = 4095 - vol_pot_raw;
  amplitude = map(vol_pot, 0, 4095, 0, 120);
  
  // POTENCIÓMETRO 4 PARA FRECUENCIA DE CORTE DEL FILTRO (invertido)
  int filter_pot_raw = analogRead(POT_FILTER_PIN);
  int filter_pot = 4095 - filter_pot_raw;
  // Mapear potenciómetro a frecuencia de corte (100Hz - 4000Hz)
  float new_cutoff = map(filter_pot, 0, 4095, 100, 4000);
  
  // Actualizar filtro si cambió significativamente
  if (abs(new_cutoff - filter_cutoff) > 50 || abs(new_resonance - filter_resonance) > 0.05) {
    filter_cutoff = new_cutoff;
    filter_resonance = new_resonance;
    update_filter_coefficients();
  }
  
  // Procesar frecuencia
  if (freq_pot < 100) {
    is_silenced = true;
    current_frequency = 0;
  } else {
    is_silenced = false;
    // Mapear potenciómetro a nota (0-15)
    int new_note = map(freq_pot, 100, 4095, 0, 15);
    if (new_note != current_note) {
      current_note = new_note;
      calculate_frequency();
    }
  }
}

// ==============================================================================================================================================
// ACTUALIZAR COEFICIENTES DEL FILTRO LPF
// ==============================================================================================================================================
void update_filter_coefficients() {
  // Calcular frecuencia normalizada (0-1, donde 1 = Nyquist = SAMPLE_RATE/2)
  float omega = 2.0 * PI * filter_cutoff / SAMPLE_RATE;
  float sin_omega = sin(omega);
  float cos_omega = cos(omega);
  
  // Calcular Q del filtro (inverso de la resonancia)
  float Q = 1.0 / (2.0 - filter_resonance * 1.9);  // Resonancia suave
  float alpha = sin_omega / (2.0 * Q);
  
  // Coeficientes del filtro biquad pasa-bajos
  float norm = 1.0 + alpha;
  filter_a0 = (1.0 - cos_omega) / (2.0 * norm);
  filter_a1 = (1.0 - cos_omega) / norm;
  filter_a2 = filter_a0;
  filter_b1 = -2.0 * cos_omega / norm;
  filter_b2 = (1.0 - alpha) / norm;
}

// ==============================================================================================================================================
// APLICAR FILTRO LPF CON RESONANCIA
// ==============================================================================================================================================
float apply_filter(float input) {
  // Filtro biquad: y[n] = a0*x[n] + a1*x[n-1] + a2*x[n-2] - b1*y[n-1] - b2*y[n-2]
  float output = filter_a0 * input + filter_a1 * filter_x1 + filter_a2 * filter_x2 
                 - filter_b1 * filter_y1 - filter_b2 * filter_y2;
  
  // Actualizar estados del filtro
  filter_x2 = filter_x1;
  filter_x1 = input;
  filter_y2 = filter_y1;
  filter_y1 = output;
  
  return output;
}
// ==============================================================================================================================================
// CALCULAR FRECUENCIA SEGÚN NOTA Y OCTAVAS
// ==============================================================================================================================================
void calculate_frequency() {
  if (current_note < 0 || current_note > 15) {
    current_frequency = 0;
    return;
  }
  
  // Calcular octava y posición en escala
  int octave = current_note / 8;          // 0 o 1
  int scale_pos = current_note % 8;       // 0-7
  
  // Obtener semitono de la escala
  int semitone = scales[current_scale][scale_pos];
  
  // Calcular semitonos totales desde la base
  int total_semitones = (octave * 12) + semitone;
  
  // Aplicar modificadores de octava
  if (octave_active) {
    total_semitones += 12;  // +1 octava cuando está activado
  }
  
  // Calcular frecuencia final
  current_frequency = BASE_FREQ * pow(2.0, total_semitones / 12.0);
}

// ==============================================================================================================================================
// GENERAR AUDIO CON ONDA CUADRADA Y FILTRO LPF
// ==============================================================================================================================================
void generate_audio() {
  if (is_silenced || current_frequency <= 0) {
    dac_output_voltage(DAC_CHANNEL_1, 128);
    delayMicroseconds(62);  // Timing para 16kHz
    return;
  }
  
  // Generar onda cuadrada básica
  float raw_sample;
  if (phase < PI) {
    raw_sample = 1.0;
  } else {
    raw_sample = -1.0;
  }
  
  // Aplicar filtro LPF con resonancia
  float filtered_sample = apply_filter(raw_sample);
  
  // Convertir a DAC
  int dac_value = 128 + (int)(amplitude * filtered_sample);
  dac_value = constrain(dac_value, 0, 255);
  
  // Enviar al DAC
  dac_output_voltage(DAC_CHANNEL_1, dac_value);
  
  // Actualizar fase
  float samples_per_cycle = SAMPLE_RATE / current_frequency;
  phase += (2.0 * PI) / samples_per_cycle;
  
  if (phase >= 2.0 * PI) {
    phase -= 2.0 * PI;
  }
  
  // Timing para 16kHz: 1/16000 = 62.5 microsegundos
  delayMicroseconds(62);
}


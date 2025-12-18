
// ==============================================================================================================================================
// PROTO-SYNTH V2 - WaveShaping Pattern Sequencer - GC Lab Chile
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
// Sequenciador de patrones con síntesis de una voz basada en formas de onda modificables.
// ==============================================================================================================================================

// ==============================================================================================================================================
// FUNCIONAMIENTO
// ==============================================================================================================================================
// CONTROLES DE EXPRESIÓN:
// - Potenciómetro 1: Tempo
// - Potenciómetro 2: Volumen General
// - Potenciómetro 3: Gate / Duración de nota 
// - Potenciómetro 4: Ancho de Pulso (para onda de Pulso)
// - Botón 1: Start / Stop
// - Botón 2: Siguiente Patrón
// - Botón 3: Patrón Anterior
// - Botón 4: Cambiar Forma de Onda
// - LED 1: Indicador de sequencia
// - LED 2: Indicador de sequencia
// - LED 3: Indicador de sequencia
// - LED 4: Indicador de sequencia
// - IMU: Control de filtro LPF
// - LDR: Control de Octava
// - Micrófono: No se usa
// - Header 1: No se usa
// - Header 2: No se usa
// - Salida MIDI: No se usa
//
// MODO DE USO:
// 1. Inicia o detén la reproducción con el botón 1.
// 2. Establece el tempo con el potenciometro 1.
// 3. Ajusta el volumen general con el potenciometro 2.
// 4. Configura la duración de las notas con el potenciometro 3.
// 5. Selecciona la forma de onda con el botón 4.
// 6. Usa el potenciometro 4 para ajustar el ancho de pulso si la forma de onda seleccionada es Pulso.
// 7. Controla el cutoff y la resonancia del filtro con el IMU.
// 8. Cambia la octava con el LDR.
// 9. Cambia entre patrones con los botones 2 y 3.
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
const int SDA_PIN=21, SCL_PIN=22, IMU_ADDRESS=0x68;
const int POT1_PIN=13, POT2_PIN=14, POT3_PIN=12, POT4_PIN=27;
const int LED1_PIN=23, LED2_PIN=32, LED3_PIN=5, LED4_PIN=2;
const int BUTTON1_PIN=18, BUTTON2_PIN=4, BUTTON3_PIN=15, BUTTON4_PIN=19; // Botón 4 añadido
const int LDR_PIN = 26;

// ==============================================================================================================================================
// PROGRAMA
// ==============================================================================================================================================
// --- PATRONES ---
const int NUM_SEQUENCES = 21, NUM_STEPS = 8;
const int NOTES[NUM_SEQUENCES][NUM_STEPS] = { /* ... (los mismos patrones de antes) ... */ 
    {36,48,36,48,36,48,36,48}, {36,36,36,36,36,36,36,36},
    {36,36,36,48,36,36,36,48}, {36,36,48,42,36,36,36,48},
    {48,48,36,48,48,36,48,36}, {36,48,36,48,36,48,36,48},
    {36,36,48,36,36,48,36,48}, {36,37,38,39,40,41,42,43},
    {43,42,41,40,39,38,37,36}, {36,38,39,41,43,44,46,48},
    {48,46,44,43,41,39,38,36}, {36,38,40,43,45,48,50,52},
    {52,50,48,45,43,40,38,36}, {36,39,41,43,46,48,51,51},
    {51,48,46,43,41,39,36,36}, {36,39,41,42,44,46,48,51},
    {51,48,46,44,42,41,39,36}, {36,40,34,45,48,39,38,46},
    {36,38,35,45,37,43,50,41}, {36,46,44,37,48,35,42,45},
    {36,44,48,38,39,43,44,38}
};

// --- VARIABLES GLOBALES ---
int current_sequence_index = 0, current_step = 0;
bool is_playing = false;
unsigned long tempo_micros = 500000, last_step_time = 0;
int octave_offset = 0;
float master_volume = 1.0, gate_duration = 0.5;
float pulse_width = 0.5; // Para la onda de pulso
int waveform_type = 0; // 0:Saw, 1:Triangle, 2:Pulse, 3:Harmonic Saw

float current_freq = 440.0, phase = 0.0;
enum EnvState { IDLE, ATTACK, DECAY };
EnvState envelope_state = IDLE;
float envelope_value = 0.0;
const float ATTACK_RATE = 1.0 / (0.01 * SAMPLE_RATE);
float decay_rate;

float filter_x1=0, filter_x2=0, filter_y1=0, filter_y2=0;
float filter_a0, filter_a1, filter_a2, filter_b1, filter_b2;
float imu_accel_x=0.0, imu_accel_y=0.0, filtered_x=0.0, filtered_y=0.0;
const float IMU_FILTER_ALPHA = 0.1;

unsigned long last_button1_time=0, last_button2_time=0, last_button3_time=0, last_button4_time=0;
const unsigned long BUTTON_DEBOUNCE = 200;

// =================================================================
//                    INICIALIZACIÓN
// =================================================================
void setup() {
  pinMode(LED1_PIN, OUTPUT); pinMode(LED2_PIN, OUTPUT); pinMode(LED3_PIN, OUTPUT); pinMode(LED4_PIN, OUTPUT);
  pinMode(BUTTON1_PIN, INPUT_PULLUP); pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(BUTTON3_PIN, INPUT_PULLUP); pinMode(BUTTON4_PIN, INPUT_PULLUP); // Botón 4
  pinMode(LDR_PIN, INPUT);
  dac_output_enable(DAC_CHANNEL_1);
  dac_output_voltage(DAC_CHANNEL_1, 128);
  initIMU();
}

void initIMU() {
  Wire.begin(SDA_PIN, SCL_PIN); Wire.setClock(400000);
  Wire.beginTransmission(IMU_ADDRESS); Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true);
}

// =================================================================
//                    LOOP PRINCIPAL Y MOTOR DE AUDIO
// =================================================================
void loop() {
  static unsigned long lastAudioUpdate = 0;
  static int controlUpdateCounter = 0;

  if (micros() - lastAudioUpdate >= (1000000 / SAMPLE_RATE)) {
    lastAudioUpdate = micros();

    if (controlUpdateCounter++ % 256 == 0) {
      readControls(); readIMU(); updateFilterCoefficients();
    }
    
    if (is_playing && (micros() - last_step_time >= tempo_micros)) {
      advanceStep();
    }

    float env_amp = processEnvelope();
    if (env_amp > 0.001) {
      float raw_sample = 0.0;
      float samples_per_cycle = (float)SAMPLE_RATE / current_freq;
      float normalized_phase = phase / samples_per_cycle;
      
      // Selección de forma de onda
      switch(waveform_type) {
        case 0: raw_sample = generateSawtooth(normalized_phase); break;
        case 1: raw_sample = generateTriangle(normalized_phase); break;
        case 2: raw_sample = generatePulse(normalized_phase); break;
        case 3: raw_sample = generateHarmonicSaw(normalized_phase); break;
      }
      phase += 1.0;
      if (phase >= samples_per_cycle) phase = 0.0;
      
      float filtered_sample = applyFilter(raw_sample);
      int dac_sample = 128 + (int)(filtered_sample * env_amp * master_volume * AMPLITUDE);
      dac_sample = constrain(dac_sample, 0, 255);
      dac_output_voltage(DAC_CHANNEL_1, dac_sample);
    } else {
      dac_output_voltage(DAC_CHANNEL_1, 128);
    }
  }
}

// =================================================================
//              GENERADORES DE FORMAS DE ONDA
// =================================================================
float generateSawtooth(float norm_phase) {
  return (2.0 * norm_phase) - 1.0;
}
float generateTriangle(float norm_phase) {
  if (norm_phase < 0.5) return 4.0 * norm_phase - 1.0;
  else return 3.0 - 4.0 * norm_phase;
}
float generatePulse(float norm_phase) {
  return (norm_phase < pulse_width) ? 1.0 : -1.0;
}
float generateHarmonicSaw(float norm_phase) {
  float output = 0.0;
  for (int harmonic = 1; harmonic <= 5; harmonic++) {
    output += (1.0 / harmonic) * sin(2.0 * PI * harmonic * norm_phase);
  }
  return output * 0.5; // Reducir amplitud total
}

// =================================================================
//                    LÓGICA DEL SECUENCIADOR
// =================================================================
void advanceStep() {
  last_step_time = micros();
  current_step = (current_step + 1) % NUM_STEPS;
  updateLEDs(current_step);
  int note_midi = NOTES[current_sequence_index][current_step];
  if (note_midi > 0) {
    current_freq = mtof(note_midi + octave_offset);
    envelope_state = ATTACK;
  } else {
    envelope_state = IDLE; envelope_value = 0.0;
  }
}

float mtof(float midi_note) {
  return 440.0 * pow(2.0, (midi_note - 69.0) / 12.0);
}

// =================================================================
//              ENVOLVENTE, FILTRO Y CONTROLES
// =================================================================
float processEnvelope() {
  switch (envelope_state) {
    case ATTACK:
      envelope_value += ATTACK_RATE;
      if (envelope_value >= 1.0) { envelope_value = 1.0; envelope_state = DECAY; }
      break;
    case DECAY:
      decay_rate = 1.0 / ((gate_duration * (tempo_micros/1000000.0)) * SAMPLE_RATE);
      envelope_value -= decay_rate;
      if (envelope_value <= 0.0) { envelope_value = 0.0; envelope_state = IDLE; }
      break;
    default: envelope_value = 0.0; break;
  }
  return envelope_value;
}

void updateFilterCoefficients() { /* ... (sin cambios) ... */
  float cutoff_freq = map(filtered_x * 1000, -1000, 1000, 100, 10000);
  float resonance = map(filtered_y * 1000, -1000, 1000, 0.7, 15);
  float norm_freq = constrain(cutoff_freq / (SAMPLE_RATE / 2.0), 0.01, 0.95);
  float omega=PI*norm_freq, s=sin(omega), c=cos(omega), alpha=s/(2.0*constrain(resonance,0.7,20.0));
  float b0=(1-c)/2, b1=1-c, b2=b0, a0=1+alpha, a1=-2*c, a2=1-alpha;
  filter_a0=b0/a0; filter_a1=b1/a0; filter_a2=b2/a0; filter_b1=a1/a0; filter_b2=a2/a0;
}

float applyFilter(float input) { /* ... (sin cambios) ... */
  float o=filter_a0*input+filter_a1*filter_x1+filter_a2*filter_x2-filter_b1*filter_y1-filter_b2*filter_y2;
  filter_x2=filter_x1; filter_x1=input; filter_y2=filter_y1; filter_y1=o; return o;
}

void readControls() {
  int ldr_val = analogRead(LDR_PIN);
  if (ldr_val < 1024) octave_offset = -12; else if (ldr_val < 2048) octave_offset = 0;
  else if (ldr_val < 3072) octave_offset = 12; else octave_offset = 24;
  
  tempo_micros = map(4095-analogRead(POT1_PIN),0,4095,500000,62500);
  master_volume = (4095-analogRead(POT2_PIN))/4095.0;
  gate_duration = map(4095-analogRead(POT3_PIN),0,4095,1,100)/100.0;
  pulse_width = map(4095-analogRead(POT4_PIN),0,4095,5,95)/100.0; // Rango de 5% a 95%

  unsigned long t = millis();
  if (!digitalRead(BUTTON1_PIN) && (t - last_button1_time > BUTTON_DEBOUNCE)) {
    is_playing=!is_playing; if(is_playing){current_step=-1;last_step_time=micros()-tempo_micros;}else{updateLEDs(-1);}
    last_button1_time = t;
  }
  if (!digitalRead(BUTTON2_PIN) && (t - last_button2_time > BUTTON_DEBOUNCE)) {
    current_sequence_index=(current_sequence_index+1)%NUM_SEQUENCES; last_button2_time=t;
  }
  if (!digitalRead(BUTTON3_PIN) && (t - last_button3_time > BUTTON_DEBOUNCE)) {
    if(--current_sequence_index<0)current_sequence_index=NUM_SEQUENCES-1; last_button3_time=t;
  }
  if (!digitalRead(BUTTON4_PIN) && (t - last_button4_time > BUTTON_DEBOUNCE)) {
    waveform_type = (waveform_type + 1) % 4; // Cambia entre 0, 1, 2, 3
    last_button4_time = t;
  }
}

void readIMU() { /* ... (sin cambios) ... */
  Wire.beginTransmission(IMU_ADDRESS); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(IMU_ADDRESS, 6, true);
  if(Wire.available()>=6){
    int16_t raw_x=Wire.read()<<8|Wire.read(), raw_y=Wire.read()<<8|Wire.read(); Wire.read(); Wire.read();
    filtered_x=filtered_x*(1-IMU_FILTER_ALPHA)+constrain(raw_x/16384.0,-1,1)*IMU_FILTER_ALPHA;
    filtered_y=filtered_y*(1-IMU_FILTER_ALPHA)+constrain(raw_y/16384.0,-1,1)*IMU_FILTER_ALPHA;
  }
}
void updateLEDs(int step) { /* ... (sin cambios) ... */
  if(step==-1){digitalWrite(LED1_PIN,0);digitalWrite(LED2_PIN,0);digitalWrite(LED3_PIN,0);digitalWrite(LED4_PIN,0);return;}
  digitalWrite(LED1_PIN, step<2); digitalWrite(LED2_PIN, step>=2&&step<4);
  digitalWrite(LED3_PIN, step>=4&&step<6); digitalWrite(LED4_PIN, step>=6&&step<8);
}

// ==============================================================================================================================================
// PROTO-SYNTH V2 - OSCILADOR 4 ESCALAS - GC Lab Chile
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
// Sintetizador de dron basado en 4 osciladores de onda sierra, con 4 modos de escala musical seleccionables.
// ==============================================================================================================================================

// ==============================================================================================================================================
// FUNCIONAMIENTO
// ==============================================================================================================================================
// CONTROLES DE EXPRESIÓN:
// - Potenciómetro 1: Oscilador 1
// - Potenciómetro 2: Oscilador 2
// - Potenciómetro 3: Oscilador 3 
// - Potenciómetro 4: Oscilador 4
// - Botón 1: Modo frigio
// - Botón 2: Modo lidio  
// - Botón 3: Modo eólico
// - Botón 4: Menor armónica
// - LED 1: Indica modo frigio
// - LED 2: Indica modo lidio 
// - LED 3: Indica modo eólico
// - LED 4: Indica modo menor armónico
// - IMU: No se usa
// - LDR: Control de filtro LPF
// - Micrófono: No se usa
// - Header 1: No se usa
// - Header 2: No se usa
// - Salida MIDI: No se usa
//
// MODO DE USO:
// 1. Gira los potenciometros para definir los tonos de cada oscilador dentro de la escala seleccionada.
// 2. Presiona los botones para cambiar entre las 4 escalas musicales disponibles.
// 3. Ajusta el LDR para modificar la frecuencia de corte del filtro pasa-bajos aplicado a la mezcla de los osciladores.
//
// INFORMACIÓN DEL CODIGO:
// - Es necesario instalar la libreria Mozzi.
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
#include <Mozzi.h>
#include <Oscil.h>
#include <tables/saw2048_int8.h>
#include <LowPassFilter.h>

// ==============================================================================================================================================
// CONFIGURACIÓN DE HARDWARE - PINES
// ==============================================================================================================================================
#define MOZZI_CONTROL_RATE 256
#define POT1_PIN 12
#define POT2_PIN 13
#define POT3_PIN 14
#define POT4_PIN 27
#define LDR_PIN 26

// Pines de botones
#define BTN_FRIGIO_PIN 18
#define BTN_LIDIO_PIN 4
#define BTN_EOLICO_PIN 15
#define BTN_MENOR_ARM_PIN 19

// Pines de LEDs para mostrar estado de cada oscilador
#define LED1_PIN 5   // LED 1 - Oscilador 1
#define LED2_PIN 23  // LED 2 - Oscilador 2  
#define LED3_PIN 32  // LED 3 - Oscilador 3
#define LED4_PIN 2   // LED 4 - Oscilador 4

// ==============================================================================================================================================
// PROGRAMA
// ==============================================================================================================================================
// 4 osciladores de audio con onda sierra
Oscil<SAW2048_NUM_CELLS, MOZZI_AUDIO_RATE> aSaw0(SAW2048_DATA);
Oscil<SAW2048_NUM_CELLS, MOZZI_AUDIO_RATE> aSaw1(SAW2048_DATA);
Oscil<SAW2048_NUM_CELLS, MOZZI_AUDIO_RATE> aSaw2(SAW2048_DATA);
Oscil<SAW2048_NUM_CELLS, MOZZI_AUDIO_RATE> aSaw3(SAW2048_DATA);

// Filtro pasa-bajos
LowPassFilter lpf;

// Escalas musicales - todas en A (La)
// A FRIGIO: A - Bb - C - D - E - F - G
const float frigio_freqs[] = {
  // Octava 2
  55.00,   // A2
  58.27,   // Bb2
  65.41,   // C2
  73.42,   // D2
  82.41,   // E2
  87.31,   // F2
  98.00,   // G2
  
  // Octava 3
  110.00,  // A3
  116.54,  // Bb3
  130.81,  // C3
  146.83,  // D3
  164.81,  // E3
  174.61,  // F3
  196.00,  // G3
  
  // Octava 4
  220.00,  // A4
  233.08,  // Bb4
  261.63,  // C4
  293.66,  // D4
  329.63,  // E4
  349.23,  // F4
  392.00,  // G4
  
  // Octava 5
  440.00,  // A5
  466.16,  // Bb5
  523.25,  // C5
  587.33,  // D5
  659.25,  // E5
  698.46,  // F5
  783.99   // G5
};

// A LIDIO: A - B - C# - D# - E - F# - G#
const float lidio_freqs[] = {
  // Octava 2
  55.00,   // A2
  61.74,   // B2
  69.30,   // C#2
  77.78,   // D#2
  82.41,   // E2
  92.50,   // F#2
  103.83,  // G#2
  
  // Octava 3
  110.00,  // A3
  123.47,  // B3
  138.59,  // C#3
  155.56,  // D#3
  164.81,  // E3
  185.00,  // F#3
  207.65,  // G#3
  
  // Octava 4
  220.00,  // A4
  246.94,  // B4
  277.18,  // C#4
  311.13,  // D#4
  329.63,  // E4
  369.99,  // F#4
  415.30,  // G#4
  
  // Octava 5
  440.00,  // A5
  493.88,  // B5
  554.37,  // C#5
  622.25,  // D#5
  659.25,  // E5
  739.99,  // F#5
  830.61   // G#5
};

// A EÓLICO (A menor natural): A - B - C - D - E - F - G
const float eolico_freqs[] = {
  // Octava 2
  55.00,   // A2
  61.74,   // B2
  65.41,   // C2
  73.42,   // D2
  82.41,   // E2
  87.31,   // F2
  98.00,   // G2
  
  // Octava 3
  110.00,  // A3
  123.47,  // B3
  130.81,  // C3
  146.83,  // D3
  164.81,  // E3
  174.61,  // F3
  196.00,  // G3
  
  // Octava 4
  220.00,  // A4
  246.94,  // B4
  261.63,  // C4
  293.66,  // D4
  329.63,  // E4
  349.23,  // F4
  392.00,  // G4
  
  // Octava 5
  440.00,  // A5
  493.88,  // B5
  523.25,  // C5
  587.33,  // D5
  659.25,  // E5
  698.46,  // F5
  783.99   // G5
};

// A MENOR ARMÓNICA: A - B - C - D - E - F - G#
const float menor_armonica_freqs[] = {
  // Octava 2
  55.00,   // A2
  61.74,   // B2
  65.41,   // C2
  73.42,   // D2
  82.41,   // E2
  87.31,   // F2
  103.83,  // G#2
  
  // Octava 3
  110.00,  // A3
  123.47,  // B3
  130.81,  // C3
  146.83,  // D3
  164.81,  // E3
  174.61,  // F3
  207.65,  // G#3
  
  // Octava 4
  220.00,  // A4
  246.94,  // B4
  261.63,  // C4
  293.66,  // D4
  329.63,  // E4
  349.23,  // F4
  415.30,  // G#4
  
  // Octava 5
  440.00,  // A5
  493.88,  // B5
  523.25,  // C5
  587.33,  // D5
  659.25,  // E5
  698.46,  // F5
  830.61   // G#5
};

const int num_notes = 28; // 7 notas x 4 octavas

// Variables para manejo de escalas
enum Escala { FRIGIO, LIDIO, EOLICO, MENOR_ARMONICA };
Escala escala_actual = LIDIO; // Empezar con Lidio

// Variables para debounce de botones
unsigned long ultimo_cambio_boton = 0;
const unsigned long debounce_delay = 200;

void setup(){
  startMozzi();
  
  // Configurar pines de LEDs como salidas
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(LED4_PIN, OUTPUT);
  
  // Configurar botones con pull-up interno
  pinMode(BTN_FRIGIO_PIN, INPUT_PULLUP);
  pinMode(BTN_LIDIO_PIN, INPUT_PULLUP);
  pinMode(BTN_EOLICO_PIN, INPUT_PULLUP);
  pinMode(BTN_MENOR_ARM_PIN, INPUT_PULLUP);
  
  // Apagar todos los LEDs inicialmente
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  digitalWrite(LED3_PIN, LOW);
  digitalWrite(LED4_PIN, LOW);
  
  // Frecuencias iniciales apagadas
  aSaw0.setFreq(0);
  aSaw1.setFreq(0);
  aSaw2.setFreq(0);
  aSaw3.setFreq(0);
  
  // Configurar filtro pasa-bajos con resonancia moderada
  lpf.setCutoffFreq(255);
  lpf.setResonance(190);
}

// Función para obtener el array de frecuencias según la escala actual
const float* getFreqArray() {
  switch(escala_actual) {
    case FRIGIO: return frigio_freqs;
    case LIDIO: return lidio_freqs;
    case EOLICO: return eolico_freqs;
    case MENOR_ARMONICA: return menor_armonica_freqs;
    default: return lidio_freqs;
  }
}

// Función para leer y procesar botones
void leerBotones() {
  unsigned long tiempo_actual = millis();
  
  // Solo procesar si ha pasado suficiente tiempo desde el último cambio
  if (tiempo_actual - ultimo_cambio_boton > debounce_delay) {
    
    if (digitalRead(BTN_FRIGIO_PIN) == LOW) {
      escala_actual = FRIGIO;
      ultimo_cambio_boton = tiempo_actual;
    }
    else if (digitalRead(BTN_LIDIO_PIN) == LOW) {
      escala_actual = LIDIO;
      ultimo_cambio_boton = tiempo_actual;
    }
    else if (digitalRead(BTN_EOLICO_PIN) == LOW) {
      escala_actual = EOLICO;
      ultimo_cambio_boton = tiempo_actual;
    }
    else if (digitalRead(BTN_MENOR_ARM_PIN) == LOW) {
      escala_actual = MENOR_ARMONICA;
      ultimo_cambio_boton = tiempo_actual;
    }
  }
}

// Función para controlar LEDs basado en la actividad del oscilador
void updateLEDs(float freq0, float freq1, float freq2, float freq3) {
  // LED encendido si el oscilador está activo (freq > 0)
  digitalWrite(LED1_PIN, (freq0 > 0) ? HIGH : LOW);
  digitalWrite(LED2_PIN, (freq1 > 0) ? HIGH : LOW);
  digitalWrite(LED3_PIN, (freq2 > 0) ? HIGH : LOW);
  digitalWrite(LED4_PIN, (freq3 > 0) ? HIGH : LOW);
}

float quantizeToScale(int pot_value) {
  if (pot_value < 50) return 0; // Silencio si está muy bajo
  
  // Obtener el array de frecuencias de la escala actual
  const float* freq_array = getFreqArray();
  
  // Mapear el valor del potenciómetro al índice de notas
  int note_index = map(pot_value, 50, 4095, 0, num_notes - 1);
  
  // Constrain para asegurar que esté en rango
  note_index = constrain(note_index, 0, num_notes - 1);
  
  return freq_array[note_index];
}

void updateControl(){
  // Leer botones para cambiar escala
  leerBotones();
  
  // Leer potenciómetros
  int pot1 = mozziAnalogRead<12>(POT1_PIN);
  int pot2 = mozziAnalogRead<12>(POT2_PIN);
  int pot3 = mozziAnalogRead<12>(POT3_PIN);
  int pot4 = mozziAnalogRead<12>(POT4_PIN);
  
  // Leer LDR para controlar filtro
  int ldr_value = mozziAnalogRead<12>(LDR_PIN);
  
  // Invertir el sentido de los potenciómetros
  pot1 = 4095 - pot1;
  pot2 = 4095 - pot2;
  pot3 = 4095 - pot3;
  pot4 = 4095 - pot4;
  
  // Cuantizar cada potenciómetro a la escala actual
  float freq0 = quantizeToScale(pot1);
  float freq1 = quantizeToScale(pot2);
  float freq2 = quantizeToScale(pot3);
  float freq3 = quantizeToScale(pot4);
  
  // Establecer frecuencias de los osciladores
  aSaw0.setFreq(freq0);
  aSaw1.setFreq(freq1);
  aSaw2.setFreq(freq2);
  aSaw3.setFreq(freq3);
  
  // Actualizar LEDs basado en el estado de los osciladores
  updateLEDs(freq0, freq1, freq2, freq3);
  
  // Mapear LDR directamente a frecuencia de corte del filtro
  byte cutoff_freq = 5 + (ldr_value * 235) / 4095;
  lpf.setCutoffFreq(cutoff_freq);
}

AudioOutput updateAudio(){
  // Mezclar los 4 osciladores
  long asig = 
    (aSaw0.next() >> 3) +
    (aSaw1.next() >> 3) +
    (aSaw2.next() >> 3) +
    (aSaw3.next() >> 3);
    
  // Aplicar el filtro pasa-bajos
  int filtered_sig = lpf.next(asig) >> 2;
    
  // Límite suave
  if(filtered_sig > 240) filtered_sig = 240;
  if(filtered_sig < -240) filtered_sig = -240;
    
  return MonoOutput::from8Bit(filtered_sig);
}

void loop(){
  audioHook();
}
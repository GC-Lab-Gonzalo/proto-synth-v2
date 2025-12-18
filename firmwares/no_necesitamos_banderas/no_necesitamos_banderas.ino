
// ==============================================================================================================================================
// PROTO-SYNTH V2 - SEQUENCER "NO NECESITAMOS BANDERAS" - GC Lab Chile
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
// Sequenciador musical basado en el tema "No Necesitamos Banderas" de Los Prisioneros.
// ==============================================================================================================================================

// ==============================================================================================================================================
// FUNCIONAMIENTO
// ==============================================================================================================================================
// CONTROLES DE EXPRESIÓN:
// - Potenciómetro 1: Volumen general
// - Potenciómetro 2: Attack
// - Potenciómetro 3: Saturación/Distorsión 
// - Potenciómetro 4: Decay
// - Botón 1: Play/Stop
// - Botón 2: No se usa
// - Botón 3: No se usa
// - Botón 4: No se usa
// - LED 1: Indica sequencia
// - LED 2: Indica sequencia
// - LED 3: Indica sequencia
// - LED 4: Indica sequencia
// - IMU: Control de filtro LPF
// - LDR: No se usa
// - Micrófono: No se usa
// - Header 1: No se usa
// - Header 2: No se usa
// - Salida MIDI: No se usa
//
// MODO DE USO:
// 1. Presiona el Botón 1 para iniciar la secuencia. Vuelve a presionarlo para detenerla. 
// 2. Gira el Potenciómetro 1 para ajustar el volumen general de la secuencia. 
// 3. Gira el Potenciómetro 2 para controlar el ataque de las notas (de percusivo a suave). 
// 4. Gira el Potenciómetro 4 para ajustar el decaimiento de las notas (de cortas a largas). 
// 5. Gira el Potenciómetro 3 para añadir saturación y distorsión al sonido. 
// 6. Mueve o inclina el dispositivo. El sensor de movimiento (IMU) controla un filtro dinámico, alterando el timbre del sonido en tiempo real. 
// 6. Los LEDs parpadearán para indicar el progreso de la secuencia.
//
// INFORMACIÓN DEL CODIGO:
// - Datos extraídos directamente del archivo JSON con 862 notas
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

const int POT1_PIN = 13;
const int POT2_PIN = 14;
const int POT3_PIN = 12;
const int POT4_PIN = 27;

const int LED1_PIN = 23;  
const int LED2_PIN = 32;  
const int LED3_PIN = 5;   
const int LED4_PIN = 2;   

const int BUTTON_PIN = 18;

// ==============================================================================================================================================
// FUNCIÓN PARA CONVERTIR NOTA MIDI A FRECUENCIA
// ==============================================================================================================================================

float midiToFreq(int midiNote) {
  return 440.0 * pow(2.0, (midiNote - 69) / 12.0) * 4.0; // x4 para subir 2 octavas
}


// ==============================================================================================================================================
// SECUENCIA MUSICAL - DATOS CORRECTOS DEL JSON (862 NOTAS)
// ==============================================================================================================================================

const int SEQUENCE_NOTES[] = {
  33, 33, 33, 33, 33, 33, 36, 36, 36, 40, 40, 40, 45, 40, 33, 38, 38, 38, 38, 38, 38, 41, 41, 41, 45, 45, 45, 50, 45, 28,
  33, 33, 33, 33, 33, 33, 36, 36, 36, 40, 40, 40, 45, 40, 33, 38, 38, 38, 38, 38, 38, 41, 41, 41, 45, 45, 45, 50, 45, 28,
  33, 33, 33, 33, 33, 33, 36, 36, 36, 40, 40, 40, 45, 40, 33, 38, 38, 38, 38, 38, 38, 41, 41, 41, 45, 45, 45, 50, 45, 28,
  33, 33, 33, 33, 33, 33, 36, 36, 36, 40, 40, 40, 45, 40, 33, 38, 38, 38, 38, 38, 38, 41, 41, 41, 45, 45, 45, 50, 45, 28,
  33, 33, 40, 40, 33, 33, 33, 40, 33, 40, 38, 38, 45, 45, 38, 38, 38, 45, 38, 45, 33, 33, 40, 40, 33, 33, 33, 40, 33, 40,
  38, 38, 45, 45, 38, 38, 38, 45, 38, 45, 33, 33, 40, 40, 33, 33, 33, 40, 33, 40, 38, 38, 45, 45, 38, 38, 38, 45, 38, 45,
  33, 33, 40, 40, 33, 33, 33, 40, 33, 40, 38, 38, 45, 45, 38, 38, 38, 45, 38, 45, 33, 33, 40, 40, 33, 33, 33, 40, 33, 40,
  38, 38, 45, 45, 38, 38, 38, 45, 38, 45, 33, 33, 40, 40, 33, 33, 33, 40, 33, 40, 38, 38, 45, 45, 38, 38, 38, 45, 38, 45,
  33, 33, 40, 40, 33, 33, 33, 40, 33, 40, 38, 38, 45, 45, 38, 38, 38, 45, 38, 45, 31, 31, 38, 38, 31, 31, 31, 38, 31, 38,
  36, 36, 43, 43, 36, 36, 36, 43, 36, 43, 38, 38, 45, 45, 38, 38, 38, 45, 38, 45, 40, 40, 47, 47, 40, 40, 40, 47, 40, 47,
  33, 33, 40, 40, 33, 33, 33, 40, 33, 40, 38, 38, 45, 45, 38, 38, 38, 45, 38, 45, 33, 33, 40, 40, 33, 33, 33, 40, 33, 40,
  38, 38, 45, 45, 38, 38, 38, 45, 38, 45, 33, 33, 40, 40, 33, 33, 33, 40, 33, 40, 38, 38, 45, 45, 38, 38, 38, 45, 38, 45,
  33, 33, 40, 40, 33, 33, 33, 40, 33, 40, 38, 38, 45, 45, 38, 38, 38, 45, 38, 45, 33, 33, 40, 40, 33, 33, 33, 40, 33, 40,
  38, 38, 45, 45, 38, 38, 38, 45, 38, 45, 33, 33, 40, 40, 33, 33, 33, 40, 33, 40, 38, 38, 45, 45, 38, 38, 38, 45, 38, 45,
  33, 33, 40, 40, 33, 33, 33, 40, 33, 40, 38, 38, 45, 45, 38, 38, 38, 45, 38, 45, 31, 31, 38, 38, 31, 31, 31, 38, 31, 38,
  36, 36, 43, 43, 36, 36, 36, 43, 36, 43, 38, 38, 45, 45, 38, 38, 38, 45, 38, 45, 40, 40, 47, 47, 40, 40, 40, 47, 40, 47,
  38, 38, 38, 45, 38, 45, 38, 38, 38, 45, 38, 45, 38, 38, 38, 45, 38, 45, 38, 38, 38, 45, 38, 45, 38, 38, 38, 45, 38, 45,
  40, 40, 47, 47, 40, 40, 40, 47, 40, 47, 33, 33, 40, 40, 33, 33, 33, 40, 33, 40, 38, 38, 45, 45, 38, 38, 38, 45, 38, 45,
  33, 33, 40, 40, 33, 33, 33, 40, 33, 40, 38, 38, 45, 45, 38, 38, 38, 45, 38, 45, 33, 33, 40, 40, 33, 33, 33, 40, 33, 40,
  38, 38, 45, 45, 38, 38, 38, 45, 38, 45, 33, 33, 40, 40, 33, 33, 33, 40, 33, 40, 38, 38, 45, 45, 38, 38, 38, 45, 38, 45,
  33, 33, 40, 40, 33, 33, 33, 40, 33, 40, 38, 38, 45, 45, 38, 38, 38, 45, 38, 45, 31, 31, 38, 38, 31, 31, 31, 38, 31, 38,
  36, 36, 43, 43, 36, 36, 36, 43, 36, 43, 38, 38, 45, 45, 38, 38, 38, 45, 38, 45, 40, 40, 47, 47, 40, 40, 40, 47, 40, 47,
  38, 38, 38, 45, 38, 45, 38, 38, 38, 45, 38, 45, 38, 38, 38, 45, 38, 45, 38, 38, 38, 45, 38, 45, 38, 38, 38, 45, 38, 45,
  38, 38, 38, 45, 38, 45, 38, 38, 38, 45, 38, 45, 38, 38, 38, 45, 38, 45, 38, 38, 38, 45, 38, 45, 38, 38, 38, 45, 38, 45,
  40, 40, 47, 47, 40, 40, 40, 47, 40, 47, 33, 33, 40, 40, 33, 33, 33, 40, 33, 40, 38, 38, 45, 45, 38, 38, 38, 45, 38, 45,
  33, 33, 40, 40, 33, 33, 33, 40, 33, 40, 38, 38, 45, 45, 38, 38, 38, 45, 38, 45, 31, 31, 38, 38, 31, 31, 31, 38, 31, 38,
  36, 36, 43, 43, 36, 36, 36, 43, 36, 43, 38, 38, 45, 45, 38, 38, 38, 45, 38, 45, 40, 40, 47, 47, 40, 40, 47, 47, 40, 40,
  47, 47, 40, 40, 40, 47, 40, 47, 38, 38, 38, 45, 38, 45, 38, 38, 38, 45, 38, 45, 38, 38, 38, 45, 38, 45, 38, 38, 38, 45,
  38, 45, 38, 38, 38, 45, 38, 45, 38, 38, 38, 45, 38, 45, 38, 38, 38, 45, 38, 45, 33, 33, 40, 40, 33, 33, 33, 40, 33, 40,
  38, 38, 45, 45, 38, 38, 38, 45, 38, 45, 33, 33, 40, 40, 33, 33, 33, 40, 33, 40, 38, 38, 45, 45, 38, 38, 38, 45, 38, 45,
  33, 33, 33, 33, 33, 33, 36, 36, 36, 40, 40, 40, 45, 40, 33, 38, 38, 38, 38, 38, 38, 41, 41, 41, 45, 45, 45, 50, 45, 28,
  33, 33, 33, 33, 33, 33, 36, 36, 36, 40, 40, 40, 45, 40, 33, 38, 38, 38, 38, 38, 38, 41, 41, 41, 45, 45, 45, 50, 45, 28,
  33, 33, 33, 33, 33, 33, 36, 36, 36, 40, 40, 40, 45, 40, 33, 38, 38, 38, 38, 38, 38, 41, 41, 41, 45, 45, 45, 50, 45, 28
};

const int SEQUENCE_TIME_DELAYS[] = {
  0, 32, 32, 32, 32, 32, 32, 64, 64, 64, 64, 64, 64, 96, 72, 24, 32, 32, 32, 32, 32, 32, 64, 64, 64, 64, 64, 64, 96, 72,
  24, 32, 32, 32, 32, 32, 32, 64, 64, 64, 64, 64, 64, 96, 72, 24, 32, 32, 32, 32, 32, 32, 64, 64, 64, 64, 64, 64, 96, 72,
  24, 32, 32, 32, 32, 32, 32, 64, 64, 64, 64, 64, 64, 96, 72, 24, 32, 32, 32, 32, 32, 32, 64, 64, 64, 64, 64, 64, 96, 72,
  24, 32, 32, 32, 32, 32, 32, 64, 64, 64, 64, 64, 64, 96, 72, 24, 32, 32, 32, 32, 32, 32, 64, 64, 64, 64, 64, 64, 96, 72,
  24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72,
  24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72,
  24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72,
  24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72,
  24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72,
  24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72,
  24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72,
  24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72,
  24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72,
  24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72,
  24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72,
  24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72,
  24, 792, 64, 64, 64, 96, 72, 24, 64, 64, 64, 96, 72, 24, 792, 64, 64, 64, 96, 72, 24, 64, 64, 64, 96, 72, 24, 792, 64, 64,
  64, 96, 72, 24, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24,
  96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24,
  96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24,
  96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24,
  96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24,
  792, 64, 64, 64, 96, 72, 24, 64, 64, 64, 96, 72, 24, 792, 64, 64, 64, 96, 72, 24, 64, 64, 64, 96, 72, 24, 792, 64, 64,
  64, 96, 72, 24, 64, 64, 64, 96, 72, 24, 792, 64, 64, 64, 96, 72, 24, 64, 64, 64, 96, 72, 24, 792, 64, 64, 64, 96, 72,
  24, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96,
  96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96,
  96, 96, 96, 96, 96, 96, 96, 96, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 792, 64, 64, 64, 96, 72, 24, 64, 64, 64, 96, 72,
  24, 792, 64, 64, 64, 96, 72, 24, 64, 64, 64, 96, 72, 24, 792, 64, 64, 64, 96, 72, 24, 64, 64, 64, 96, 72, 24, 792, 64,
  64, 64, 96, 72, 24, 64, 64, 64, 96, 72, 24, 792, 64, 64, 64, 96, 72, 24, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64,
  64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 32, 32, 32, 32,
  32, 32, 64, 64, 64, 64, 64, 64, 96, 72, 24, 32, 32, 32, 32, 32, 32, 64, 64, 64, 64, 64, 64, 96, 72, 24, 32, 32, 32,
  32, 32, 32, 64, 64, 64, 64, 64, 64, 96, 72, 24, 32, 32, 32, 32, 32, 32, 64, 64, 64, 64, 64, 64, 96, 72, 24, 32, 32,
  32, 32, 32, 32, 64, 64, 64, 64, 64, 64, 96, 72
};

const int SEQUENCE_DURATIONS[] = {
  32, 32, 32, 32, 32, 32, 64, 64, 64, 64, 64, 64, 96, 72, 24, 32, 32, 32, 32, 32, 32, 64, 64, 64, 64, 64, 64, 96, 72, 24,
  32, 32, 32, 32, 32, 32, 64, 64, 64, 64, 64, 64, 96, 72, 24, 32, 32, 32, 32, 32, 32, 64, 64, 64, 64, 64, 64, 96, 72, 24,
  32, 32, 32, 32, 32, 32, 64, 64, 64, 64, 64, 64, 96, 72, 24, 32, 32, 32, 32, 32, 32, 64, 64, 64, 64, 64, 64, 96, 72, 24,
  32, 32, 32, 32, 32, 32, 64, 64, 64, 64, 64, 64, 96, 72, 24, 32, 32, 32, 32, 32, 32, 64, 64, 64, 64, 64, 64, 96, 72, 24,
  96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24,
  96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24,
  96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24,
  96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24,
  96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24,
  96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24,
  96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24,
  96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24,
  96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24,
  96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24,
  96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24,
  96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24,
  64, 64, 64, 96, 72, 24, 64, 64, 64, 96, 72, 24, 64, 64, 64, 96, 72, 24, 64, 64, 64, 96, 72, 24, 64, 64, 64, 96, 72, 24,
  96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24,
  96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24,
  96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24,
  96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24,
  96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24,
  64, 64, 64, 96, 72, 24, 64, 64, 64, 96, 72, 792, 64, 64, 64, 96, 72, 24, 64, 64, 64, 96, 72, 792, 64, 64, 64, 96, 72, 24,
  64, 64, 64, 96, 72, 792, 64, 64, 64, 96, 72, 24, 64, 64, 64, 96, 72, 792, 64, 64, 64, 96, 72, 24, 64, 64, 64, 96, 72, 24,
  96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24,
  96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 96, 96, 96, 96, 96, 96,
  96, 96, 96, 64, 64, 64, 96, 72, 24, 64, 64, 64, 96, 72, 792, 64, 64, 64, 96, 72, 24, 64, 64, 64, 96, 72, 792, 64, 64, 64,
  96, 72, 24, 64, 64, 64, 96, 72, 792, 64, 64, 64, 96, 72, 24, 64, 64, 64, 96, 72, 792, 64, 64, 64, 96, 72, 24, 64, 64, 64,
  96, 72, 792, 64, 64, 64, 96, 72, 24, 64, 64, 64, 96, 72, 792, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72,
  24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 96, 96, 96, 96, 64, 64, 64, 96, 72, 24, 32, 32, 32, 32, 32, 32, 64, 64, 64,
  64, 64, 64, 96, 72, 24, 32, 32, 32, 32, 32, 32, 64, 64, 64, 64, 64, 64, 96, 72, 24, 32, 32, 32, 32, 32, 32, 64, 64, 64,
  64, 64, 64, 96, 72, 24, 32, 32, 32, 32, 32, 32, 64, 64, 64, 64, 64, 64, 96, 72, 24, 32, 32, 32, 32, 32, 32, 64, 64, 64,
  64, 64, 64, 96, 72
};


const float SEQUENCE_VELOCITY = 0.63; // 80/127
const int SEQUENCE_LENGTH = 862;

// ==============================================================================================================================================
// VARIABLES DEL SISTEMA
// ==============================================================================================================================================

float imu_accel_x = 0.0, imu_accel_y = 0.0;        
unsigned long lastIMURead = 0;   
const unsigned long IMU_READ_INTERVAL = 20;  
float filtered_x = 0.0, filtered_y = 0.0;         
const float IMU_FILTER_ALPHA = 0.2;  

// Variables del filtro mejorado
float filter_x1 = 0, filter_x2 = 0;  
float filter_y1 = 0, filter_y2 = 0;  
float filter_a0, filter_a1, filter_a2;  
float filter_b1, filter_b2;

bool sequencer_playing = false;        
int current_step = 0;                  
unsigned long sequencer_start_time_us = 0;
unsigned long current_note_end_time_us = 0;
float tempo_multiplier = 0.22; // Tempo base          

float envelope_level = 0.0;        
bool note_triggered = false;       
unsigned long note_start_time = 0; 
float attack_duration = 50.0;      
float decay_duration = 150.0;      

bool button_pressed = false;       
unsigned long last_button_time = 0;
const unsigned long BUTTON_DEBOUNCE = 200;  

float currentFreq = 60.00;  
bool audioPlaying = false;   
int waveform_type = 2;       
float phase = 0.0;           

// Variables para timing absoluto
bool note_active = false;
unsigned long accumulated_time_us = 0;  // Tiempo acumulado desde el inicio
unsigned long next_note_time_us = 0;

// ==============================================================================================================================================
// FUNCIONES DEL SISTEMA
// ==============================================================================================================================================

void initIMU() {
  Wire.begin(SDA_PIN, SCL_PIN);    
  Wire.setClock(400000);           
  Wire.beginTransmission(IMU_ADDRESS);  
  Wire.write(0x6B); Wire.write(0);                        
  Wire.endTransmission(true);           
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(0x1C); Wire.write(0x00);                     
  Wire.endTransmission(true);
  delay(100);  
}

void readIMU() {
  unsigned long currentTime = millis();
  if (currentTime - lastIMURead < IMU_READ_INTERVAL) return;  
  lastIMURead = currentTime;
  
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(0x3B);                    
  Wire.endTransmission(false);         
  Wire.requestFrom(IMU_ADDRESS, 6, true);  
  
  if (Wire.available() >= 6) {         
    int16_t raw_x = Wire.read() << 8 | Wire.read();  
    int16_t raw_y = Wire.read() << 8 | Wire.read();  
    Wire.read(); Wire.read();  // Descartar Z
    
    float x_normalized = (raw_x <= 17936) ? 
      (float)(raw_x - 17936) / (17936 - 1596) : 
      (float)(raw_x - 17936) / (32767 - 17936);
    
    float y_normalized = (raw_y <= -580) ? 
      (float)(raw_y - (-580)) / (-580 - (-16532)) : 
      (float)(raw_y - (-580)) / (16304 - (-580));
    
    imu_accel_x = constrain(x_normalized, -1.0, 1.0);
    imu_accel_y = constrain(y_normalized, -1.0, 1.0);
    
    filtered_x = filtered_x * (1.0 - IMU_FILTER_ALPHA) + imu_accel_x * IMU_FILTER_ALPHA;
    filtered_y = filtered_y * (1.0 - IMU_FILTER_ALPHA) + imu_accel_y * IMU_FILTER_ALPHA;
  }
}

// ==============================================================================================================================================
// FILTRO MEJORADO - MAS DINAMICO Y ACIDO
// ==============================================================================================================================================

void updateFilterCoefficients() {
  // Mapeo del IMU más dramático para sonido ácido
  float y_abs = constrain(abs(filtered_y), 0.0, 1.0);
  float x_abs = constrain(abs(filtered_x), 0.0, 1.0);
  
  // RESONANCIA ALTA para sonido ácido/wah-wah (Q de 3 a 25)
  float resonance = 3.0 + pow(y_abs, 1.2) * 22.0;  // Curva más agresiva
  
  // FRECUENCIA con rango más amplio (30Hz a 12kHz)
  float cutoff_freq = 30.0 + pow(x_abs, 0.8) * 11970.0; // Curva logarítmica
  
  // Límites de seguridad pero permitiendo más rango
  float norm_freq = constrain(cutoff_freq / (SAMPLE_RATE / 2.0), 0.003, 0.95);
  
  float omega = PI * norm_freq;
  float sin_omega = sin(omega);
  float cos_omega = cos(omega);
  
  // Alpha con resonancia alta para efecto wah-wah
  float alpha = sin_omega / (2.0 * resonance);
  
  // Coeficientes del filtro lowpass
  float b0 = (1.0 - cos_omega) / 2.0;
  float b1 = 1.0 - cos_omega;
  float b2 = (1.0 - cos_omega) / 2.0;
  
  float a0_temp = 1.0 + alpha;
  float a1_temp = -2.0 * cos_omega;
  float a2_temp = 1.0 - alpha;
  
  // Normalización
  filter_a0 = b0 / a0_temp;
  filter_a1 = b1 / a0_temp;
  filter_a2 = b2 / a0_temp;
  filter_b1 = a1_temp / a0_temp;
  filter_b2 = a2_temp / a0_temp;
}

float applyFilter(float input) {
  // Saturación/distorsión del POT3
  int pot3_value = analogRead(POT3_PIN);                    
  float pot3_normalized = 1.0 - (pot3_value / 4095.0);     
  float drive = 1.0 + pot3_normalized * 4.0;  // Drive más suave (1x a 5x)
  
  float driven_input = input * drive;
  
  // Saturación suave tipo tape
  if (driven_input > 1.0) driven_input = 1.0 + (driven_input - 1.0) * 0.3;        
  else if (driven_input < -1.0) driven_input = -1.0 + (driven_input + 1.0) * 0.3;       
  
  // Clipping final
  driven_input = constrain(driven_input, -1.5, 1.5);
  
  // Aplicar filtro biquad
  float output = filter_a0 * driven_input + filter_a1 * filter_x1 + filter_a2 * filter_x2
                 - filter_b1 * filter_y1 - filter_b2 * filter_y2;
  
  // Actualizar delay lines
  filter_x2 = filter_x1; filter_x1 = driven_input;   
  filter_y2 = filter_y1; filter_y1 = output;         
  
  return output;  
}

float getMIDIVelocity() {
  int volume_pot = analogRead(POT1_PIN);
  float volume = (4095 - volume_pot) / 4095.0;
  return SEQUENCE_VELOCITY * volume;
}

// ==============================================================================================================================================
// FUNCIÓN DE SECUENCIADOR
// ==============================================================================================================================================

void updateSequencer() {
  if (!sequencer_playing) {
    audioPlaying = false;     
    envelope_level = 0.0;     
    return;                   
  }
  
  unsigned long current_time_us = micros();
  unsigned long elapsed_time_us = current_time_us - sequencer_start_time_us;
  
  // Verificar si es hora del siguiente paso
  if (elapsed_time_us >= accumulated_time_us) {
    // Obtener datos de la nota actual
    int midi_note = SEQUENCE_NOTES[current_step];
    int note_duration_us = (int)(SEQUENCE_DURATIONS[current_step] * 1000 / tempo_multiplier);
    
    // Configurar la nota si es válida
    if (midi_note >= 20) {
      currentFreq = midiToFreq(midi_note);
      note_triggered = true;
      note_start_time = millis();
      envelope_level = 0.0;
      audioPlaying = true;
      note_active = true;
      // Configurar cuándo debe terminar esta nota
      current_note_end_time_us = current_time_us + note_duration_us;
    } else {
      // Pausa - silencio
      audioPlaying = false;
      note_active = false;
      envelope_level = 0.0;
    }
    
    // Actualizar LEDs
    updateLEDs();
    
    // Avanzar al siguiente paso
    current_step = (current_step + 1) % SEQUENCE_LENGTH;
    
    // Acumular el tiempo hasta el siguiente paso
    accumulated_time_us += (unsigned long)(SEQUENCE_TIME_DELAYS[current_step] * 1000 / tempo_multiplier);
    
    // Reiniciar si completamos la secuencia
    if (current_step == 0) {
      accumulated_time_us = 0;
    }
  }
  
  // Verificar si la nota actual debe terminar
  if (note_active && current_time_us >= current_note_end_time_us) {
    audioPlaying = false;
    envelope_level = 0.0;
    note_active = false;
  }
  
  if (audioPlaying) updateEnvelope();
}

// ==============================================================================================================================================
// FUNCIONES DE ENVELOPE Y WAVEFORMS
// ==============================================================================================================================================

void updateEnvelope() {
  if (!audioPlaying) {
    envelope_level = 0.0;
    return;
  }
  
  int attack_pot = analogRead(POT2_PIN);   
  int decay_pot = analogRead(POT4_PIN);    
  
  attack_duration = 10.0 + ((4095 - attack_pot) / 4095.0) * 90.0;   
  decay_duration = 50.0 + ((4095 - decay_pot) / 4095.0) * 450.0;    
  
  unsigned long note_time = millis() - note_start_time;  
  
  if (note_time < attack_duration && note_triggered) {
    envelope_level = note_time / attack_duration;  
    if (envelope_level >= 1.0) {
      envelope_level = 1.0;           
      note_triggered = false;         
    }
  }
  else if (note_time < attack_duration + decay_duration) {
    note_triggered = false;  
    float decay_progress = (note_time - attack_duration) / decay_duration;  
    envelope_level = 1.0 - decay_progress;  
    if (envelope_level < 0.0) envelope_level = 0.0;  
  }
  else {
    envelope_level = 0.0;
  }
}

void updateLEDs() {
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  digitalWrite(LED3_PIN, LOW);
  digitalWrite(LED4_PIN, LOW);
  
  int led_index = current_step % 4;
  switch (led_index) {
    case 0: digitalWrite(LED1_PIN, HIGH); break;
    case 1: digitalWrite(LED2_PIN, HIGH); break;
    case 2: digitalWrite(LED3_PIN, HIGH); break;
    case 3: digitalWrite(LED4_PIN, HIGH); break;
  }
}

float generateTriangle(float normalized_phase) {
  return (normalized_phase < 0.5) ? 
    4.0 * normalized_phase - 1.0 : 
    3.0 - 4.0 * normalized_phase;
}

float generatePulse(float normalized_phase) {
  float duty_cycle = constrain(0.3 + 0.4 * (filtered_y + 1.0) / 2.0, 0.2, 0.8);              
  return (normalized_phase < duty_cycle) ? 1.0 : -1.0;  
}

float generateHarmonicSaw(float normalized_phase) {
  float output = 0.0;
  for (int harmonic = 1; harmonic <= 5; harmonic++) {
    float amplitude = 1.0 / harmonic;  
    output += amplitude * sin(2.0 * PI * harmonic * normalized_phase);
  }
  return output * 0.5;  
}

float generateWaveform(float samples_per_cycle) {
  phase += 1.0;                    
  if (phase >= samples_per_cycle) phase = 0.0;                   
  
  float normalized_phase = phase / samples_per_cycle;
  float output = 0.0;
  
  switch (waveform_type) {
    case 0: output = generateTriangle(normalized_phase); break;
    case 1: output = generatePulse(normalized_phase); break;
    case 2: output = generateHarmonicSaw(normalized_phase); break;
    default: output = generateTriangle(normalized_phase); break;
  }
  
  if (phase < 2.0) output *= (phase / 2.0);  
  return output;
}

void checkButtons() {
  unsigned long current_time = millis();
  bool button_state = digitalRead(BUTTON_PIN);      
  
  if (!button_state && !button_pressed && (current_time - last_button_time > BUTTON_DEBOUNCE)) {
    button_pressed = true;           
    last_button_time = current_time; 
    
    sequencer_playing = !sequencer_playing;  
    
    if (sequencer_playing) {
      current_step = 0;                           
      sequencer_start_time_us = micros();
      accumulated_time_us = 0;
      currentFreq = midiToFreq(SEQUENCE_NOTES[current_step]); 
      audioPlaying = false;
      note_active = false;
      envelope_level = 0.0;        
      updateLEDs();                               
    } else {
      audioPlaying = false;        
      envelope_level = 0.0;
      note_active = false;        
      digitalWrite(LED1_PIN, LOW);
      digitalWrite(LED2_PIN, LOW);
      digitalWrite(LED3_PIN, LOW);
      digitalWrite(LED4_PIN, LOW);
    }
  }
  
  if (button_state && button_pressed) button_pressed = false;       
}

// ==============================================================================================================================================
// SETUP Y LOOP
// ==============================================================================================================================================

void setup() {
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(LED4_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);    
  pinMode(POT1_PIN, INPUT);   
  pinMode(POT2_PIN, INPUT);   
  pinMode(POT3_PIN, INPUT);   
  pinMode(POT4_PIN, INPUT);   
  
  dac_output_enable(DAC_CHANNEL_1);     
  dac_output_voltage(DAC_CHANNEL_1, 128); 
  
  initIMU();  
  delay(100);                    
  readIMU();                     
  updateFilterCoefficients();    
  currentFreq = midiToFreq(SEQUENCE_NOTES[0]);
  
  // Inicializar variables del secuenciador
  current_step = 0;
  accumulated_time_us = 0;
  sequencer_playing = false;
  audioPlaying = false;
  note_active = false;
}

void loop() {
  static unsigned long lastAudioUpdate = 0;  
  static int filterUpdateCounter = 0;         
  
  unsigned long currentTime = micros();  
  
  if (currentTime - lastAudioUpdate >= 45) {  // ~22kHz
    lastAudioUpdate = currentTime;  
    
    if (filterUpdateCounter % 110 == 0) {  
      readIMU();                    
      updateFilterCoefficients();   
      updateSequencer();            
      checkButtons();               
    }
    filterUpdateCounter++;  
    
    if (audioPlaying) {
      float samples_per_cycle = SAMPLE_RATE / currentFreq;
      float waveform_sample = generateWaveform(samples_per_cycle);
      float filtered_sample = applyFilter(waveform_sample);     
      filtered_sample = applyFilter(filtered_sample * 0.7);    
      filtered_sample *= envelope_level;  
      float volume = getMIDIVelocity();
      
      int sample = 128 + (int)(AMPLITUDE * 0.6 * filtered_sample * volume);
      sample = constrain(sample, 0, 255);  
      dac_output_voltage(DAC_CHANNEL_1, sample);
    } else {
      dac_output_voltage(DAC_CHANNEL_1, 128);
    }
  }
}
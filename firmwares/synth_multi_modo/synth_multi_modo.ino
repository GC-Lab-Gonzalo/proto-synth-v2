
// ==============================================================================================================================================
// PROTO-SYNTH V2 - SYNTH MULTI-MODO E MAYOR - GC Lab Chile
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
// Sintetizador de 4 osciladores en escala E Mayor con 4 modos funcionales.
// Los botones cambian el modo activo y los potenciómetros cambian de función según el modo.
// ==============================================================================================================================================

// ==============================================================================================================================================
// FUNCIONAMIENTO
// ==============================================================================================================================================
// MODOS (seleccionados por botones):
//
// Botón 1 - MODO OSCILADORES (inicial):
//   - POT 1-4: Frecuencia de cada oscilador cuantizada a escala E Mayor
//
// Botón 2 - MODO FILTRO:
//   - POT 1: Cutoff del filtro pasa-bajos
//   - POT 2: Resonancia del filtro
//   - POT 3: Mezcla de forma de onda (sierra ↔ cuadrada)
//   - POT 4: Volumen general
//
// Botón 3 - MODO VELOCIDAD DE VIBRATO:
//   - POT 1-4: Velocidad del vibrato de cada oscilador (2-8 Hz)
//
// Botón 4 - MODO PROFUNDIDAD DE VIBRATO:
//   - POT 1-4: Profundidad de vibrato de cada oscilador
//
// LEDs: Indican el modo activo (1 LED encendido = 1 modo)
//
// SISTEMA PICK-UP:
// Los parámetros solo se actualizan cuando el potenciómetro correspondiente
// se mueve desde su posición actual. Esto evita saltos de valor al cambiar de modo.
//
// MODO DE USO:
// 1. Al encender, estás en Modo Osciladores. Gira los pots para elegir notas.
// 2. Presiona botones para cambiar de modo. Los parámetros NO cambian hasta mover un pot.
//
// COMENTARIOS:
// - Para subir código exitosamente, asegúrate de que el Potenciómetro 3 esté girado al máximo.
// - Los Pines 2,4,12,13,14,15,25,26,27 no van a funcionar si el Bluetooth está activado.
// - Es necesario instalar la librería Mozzi.
// ==============================================================================================================================================

// ==============================================================================================================================================
// INCLUSIÓN DE LIBRERÍAS
// ==============================================================================================================================================
#include <Mozzi.h>
#include <Oscil.h>
#include <tables/saw2048_int8.h>
#include <tables/square_no_alias_2048_int8.h>
#include <tables/sin2048_int8.h>
#include <LowPassFilter.h>

// ==============================================================================================================================================
// CONFIGURACIÓN DE HARDWARE - PINES
// ==============================================================================================================================================
#define MOZZI_CONTROL_RATE 256
#define POT1_PIN 12
#define POT2_PIN 13
#define POT3_PIN 14
#define POT4_PIN 27

#define BTN1_PIN 18
#define BTN2_PIN 4
#define BTN3_PIN 15
#define BTN4_PIN 19

#define LED1_PIN 5
#define LED2_PIN 23
#define LED3_PIN 32
#define LED4_PIN 2

// ==============================================================================================================================================
// PROGRAMA
// ==============================================================================================================================================

// 4 osciladores de sierra
Oscil<SAW2048_NUM_CELLS, MOZZI_AUDIO_RATE> osc_saw0(SAW2048_DATA);
Oscil<SAW2048_NUM_CELLS, MOZZI_AUDIO_RATE> osc_saw1(SAW2048_DATA);
Oscil<SAW2048_NUM_CELLS, MOZZI_AUDIO_RATE> osc_saw2(SAW2048_DATA);
Oscil<SAW2048_NUM_CELLS, MOZZI_AUDIO_RATE> osc_saw3(SAW2048_DATA);

// 4 osciladores de cuadrada (para mezcla de forma de onda)
Oscil<SQUARE_NO_ALIAS_2048_NUM_CELLS, MOZZI_AUDIO_RATE> osc_sq0(SQUARE_NO_ALIAS_2048_DATA);
Oscil<SQUARE_NO_ALIAS_2048_NUM_CELLS, MOZZI_AUDIO_RATE> osc_sq1(SQUARE_NO_ALIAS_2048_DATA);
Oscil<SQUARE_NO_ALIAS_2048_NUM_CELLS, MOZZI_AUDIO_RATE> osc_sq2(SQUARE_NO_ALIAS_2048_DATA);
Oscil<SQUARE_NO_ALIAS_2048_NUM_CELLS, MOZZI_AUDIO_RATE> osc_sq3(SQUARE_NO_ALIAS_2048_DATA);

// 4 LFOs sinusoidales para vibrato
Oscil<SIN2048_NUM_CELLS, MOZZI_CONTROL_RATE> lfo0(SIN2048_DATA);
Oscil<SIN2048_NUM_CELLS, MOZZI_CONTROL_RATE> lfo1(SIN2048_DATA);
Oscil<SIN2048_NUM_CELLS, MOZZI_CONTROL_RATE> lfo2(SIN2048_DATA);
Oscil<SIN2048_NUM_CELLS, MOZZI_CONTROL_RATE> lfo3(SIN2048_DATA);

// Filtro pasa-bajos
LowPassFilter lpf;

// Escala E Mayor: E - F# - G# - A - B - C# - D# (4 octavas)
const float e_mayor_freqs[] = {
  // Octava 2
  82.41,   // E2
  92.50,   // F#2
  103.83,  // G#2
  110.00,  // A2
  123.47,  // B2
  138.59,  // C#2
  155.56,  // D#2

  // Octava 3
  164.81,  // E3
  185.00,  // F#3
  207.65,  // G#3
  220.00,  // A3
  246.94,  // B3
  277.18,  // C#3
  311.13,  // D#3

  // Octava 4
  329.63,  // E4
  369.99,  // F#4
  415.30,  // G#4
  440.00,  // A4
  493.88,  // B4
  554.37,  // C#4
  622.25,  // D#4

  // Octava 5
  659.25,  // E5
  739.99,  // F#5
  830.61,  // G#5
  880.00,  // A5
  987.77,  // B5
  1108.73, // C#5
  1244.51  // D#5
};

const int num_notas = 28;

// Modos del sintetizador
enum Modo { OSCILADORES, FILTRO, VEL_VIBRATO, PROF_VIBRATO };
Modo modo_actual = OSCILADORES;

// Frecuencias base de cada oscilador (se mantienen entre modos)
float freq_base[4] = {0, 0, 0, 0};

// Parámetros de filtro y forma de onda
byte filtro_cutoff = 255;
byte filtro_resonancia = 190;
int waveform_mix = 0;     // 0 = sierra pura, 255 = cuadrada pura
int volumen = 255;         // 0-255

// Parámetros de vibrato
int vibrato_depth[4] = {0, 0, 0, 0};       // Profundidad por oscilador (0-127)
float vibrato_rate[4] = {5.0, 5.0, 5.0, 5.0}; // Velocidad por oscilador (2-8 Hz)

// Debounce
unsigned long ultimo_cambio_boton = 0;
const unsigned long debounce_delay = 200;

// Sistema pick-up: los parámetros solo se actualizan cuando el pot se mueve
int pot_anterior[4] = {-1, -1, -1, -1};
bool pot_enganchado[4] = {false, false, false, false};
const int UMBRAL_PICKIP = 150;

void setup() {
  startMozzi();

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(LED4_PIN, OUTPUT);

  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP);
  pinMode(BTN3_PIN, INPUT_PULLUP);
  pinMode(BTN4_PIN, INPUT_PULLUP);

  digitalWrite(LED1_PIN, HIGH); // Modo inicial = osciladores
  digitalWrite(LED2_PIN, LOW);
  digitalWrite(LED3_PIN, LOW);
  digitalWrite(LED4_PIN, LOW);

  // Frecuencias iniciales apagadas
  osc_saw0.setFreq(0); osc_sq0.setFreq(0);
  osc_saw1.setFreq(0); osc_sq1.setFreq(0);
  osc_saw2.setFreq(0); osc_sq2.setFreq(0);
  osc_saw3.setFreq(0); osc_sq3.setFreq(0);

  // LFOs de vibrato
  lfo0.setFreq(5.0f);
  lfo1.setFreq(5.0f);
  lfo2.setFreq(5.0f);
  lfo3.setFreq(5.0f);

  // Filtro
  lpf.setCutoffFreq(255);
  lpf.setResonance(190);
}

// Cuantizar valor de potenciómetro a la escala E Mayor
float cuantizarAEscala(int pot_value) {
  if (pot_value < 50) return 0;
  int indice = map(pot_value, 50, 4095, 0, num_notas - 1);
  indice = constrain(indice, 0, num_notas - 1);
  return e_mayor_freqs[indice];
}

// Resetear enganche de todos los pots (al cambiar de modo)
void resetearPickup() {
  for (int i = 0; i < 4; i++) {
    pot_enganchado[i] = false;
    pot_anterior[i] = -1;
  }
}

// Verificar si un pot se movió lo suficiente para engancharse
bool verificarPickup(int indice_pot, int lectura_actual) {
  if (pot_enganchado[indice_pot]) return true;
  if (pot_anterior[indice_pot] < 0) {
    pot_anterior[indice_pot] = lectura_actual;
    return false;
  }
  if (abs(lectura_actual - pot_anterior[indice_pot]) > UMBRAL_PICKIP) {
    pot_enganchado[indice_pot] = true;
    pot_anterior[indice_pot] = lectura_actual;
    return true;
  }
  return false;
}

// Leer botones y cambiar modo
void leerBotones() {
  unsigned long tiempo_actual = millis();
  if (tiempo_actual - ultimo_cambio_boton < debounce_delay) return;

  if (digitalRead(BTN1_PIN) == LOW) {
    modo_actual = OSCILADORES;
    resetearPickup();
    ultimo_cambio_boton = tiempo_actual;
  }
  else if (digitalRead(BTN2_PIN) == LOW) {
    modo_actual = FILTRO;
    resetearPickup();
    ultimo_cambio_boton = tiempo_actual;
  }
  else if (digitalRead(BTN3_PIN) == LOW) {
    modo_actual = VEL_VIBRATO;
    resetearPickup();
    ultimo_cambio_boton = tiempo_actual;
  }
  else if (digitalRead(BTN4_PIN) == LOW) {
    modo_actual = PROF_VIBRATO;
    resetearPickup();
    ultimo_cambio_boton = tiempo_actual;
  }
}

// Actualizar LEDs según modo activo
void actualizarLEDs() {
  digitalWrite(LED1_PIN, modo_actual == OSCILADORES ? HIGH : LOW);
  digitalWrite(LED2_PIN, modo_actual == FILTRO ? HIGH : LOW);
  digitalWrite(LED3_PIN, modo_actual == VEL_VIBRATO ? HIGH : LOW);
  digitalWrite(LED4_PIN, modo_actual == PROF_VIBRATO ? HIGH : LOW);
}

// Establecer frecuencias en ambos juegos de osciladores (saw + square)
void setOscFreq(int i, float freq) {
  switch(i) {
    case 0: osc_saw0.setFreq(freq); osc_sq0.setFreq(freq); break;
    case 1: osc_saw1.setFreq(freq); osc_sq1.setFreq(freq); break;
    case 2: osc_saw2.setFreq(freq); osc_sq2.setFreq(freq); break;
    case 3: osc_saw3.setFreq(freq); osc_sq3.setFreq(freq); break;
  }
}

void updateControl() {
  leerBotones();

  // Leer potenciómetros (invertidos por errata de hardware)
  int pot[4];
  pot[0] = 4095 - mozziAnalogRead<12>(POT1_PIN);
  pot[1] = 4095 - mozziAnalogRead<12>(POT2_PIN);
  pot[2] = 4095 - mozziAnalogRead<12>(POT3_PIN);
  pot[3] = 4095 - mozziAnalogRead<12>(POT4_PIN);

  switch(modo_actual) {

    case OSCILADORES: {
      for (int i = 0; i < 4; i++) {
        if (verificarPickup(i, pot[i])) {
          freq_base[i] = cuantizarAEscala(pot[i]);
        }
      }
      break;
    }

    case FILTRO: {
      if (verificarPickup(0, pot[0])) {
        filtro_cutoff = map(pot[0], 0, 4095, 5, 255);
      }
      if (verificarPickup(1, pot[1])) {
        filtro_resonancia = map(pot[1], 0, 4095, 0, 255);
      }
      if (verificarPickup(2, pot[2])) {
        waveform_mix = map(pot[2], 0, 4095, 0, 255);
      }
      if (verificarPickup(3, pot[3])) {
        volumen = map(pot[3], 0, 4095, 0, 255);
      }
      break;
    }

    case VEL_VIBRATO: {
      for (int i = 0; i < 4; i++) {
        if (verificarPickup(i, pot[i])) {
          vibrato_rate[i] = 1.0f + (pot[i] * 19.0f) / 4095.0f;
        }
      }
      break;
    }

    case PROF_VIBRATO: {
      for (int i = 0; i < 4; i++) {
        if (verificarPickup(i, pot[i])) {
          vibrato_depth[i] = map(pot[i], 0, 4095, 0, 127);
        }
      }
      break;
    }
  }

  // Actualizar velocidad de LFOs
  lfo0.setFreq(vibrato_rate[0]);
  lfo1.setFreq(vibrato_rate[1]);
  lfo2.setFreq(vibrato_rate[2]);
  lfo3.setFreq(vibrato_rate[3]);

  // Aplicar vibrato a las frecuencias
  int8_t lfo_val[4];
  lfo_val[0] = lfo0.next();
  lfo_val[1] = lfo1.next();
  lfo_val[2] = lfo2.next();
  lfo_val[3] = lfo3.next();

  for (int i = 0; i < 4; i++) {
    if (freq_base[i] > 0 && vibrato_depth[i] > 0) {
      float mod = 1.0f + ((float)lfo_val[i] / 128.0f) * ((float)vibrato_depth[i] / 127.0f) * 0.06f;
      setOscFreq(i, freq_base[i] * mod);
    } else {
      setOscFreq(i, freq_base[i]);
    }
  }

  // Aplicar filtro
  lpf.setCutoffFreq(filtro_cutoff);
  lpf.setResonance(filtro_resonancia);

  actualizarLEDs();
}

AudioOutput updateAudio() {
  // Obtener muestras de ambos juegos de osciladores
  int saw_mix = (osc_saw0.next() >> 3) + (osc_saw1.next() >> 3) +
                (osc_saw2.next() >> 3) + (osc_saw3.next() >> 3);

  int sq_mix = (osc_sq0.next() >> 3) + (osc_sq1.next() >> 3) +
               (osc_sq2.next() >> 3) + (osc_sq3.next() >> 3);

  // Mezclar sierra y cuadrada según waveform_mix (0=sierra, 255=cuadrada)
  long asig = ((long)saw_mix * (255 - waveform_mix) + (long)sq_mix * waveform_mix) >> 8;

  // Aplicar filtro pasa-bajos
  int filtered_sig = lpf.next(asig) >> 2;

  // Aplicar volumen
  filtered_sig = ((long)filtered_sig * volumen) >> 8;

  // Límite suave
  if (filtered_sig > 240) filtered_sig = 240;
  if (filtered_sig < -240) filtered_sig = -240;

  return MonoOutput::from8Bit(filtered_sig);
}

void loop() {
  audioHook();
}

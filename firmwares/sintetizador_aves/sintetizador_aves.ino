// ==============================================================================================================================================
// PROTO-SYNTH V2 - Sintetizador de Aves - GC Lab Chile
// ==============================================================================================================================================
// Desarrollado por: GC Lab Chile / Claude Code
// Licencia de Software: MIT License (https://opensource.org/licenses/MIT)
// Licencia de Hardware: CERN Open Hardware Licence v2 - Permissive (CERN-OHL-P)
//
// Inspirado en el proyecto "birds" de notthetup (WebAudio API):
// https://github.com/notthetup/birds
//
// Puedes usar, modificar y distribuir este código y hardware, siempre que se mantenga
// la atribución a GC Lab Chile. Se entrega "tal cual", sin garantías de ningún tipo.
// ==============================================================================================================================================
// REPOSITORIO: https://github.com/GC-Lab-Gonzalo/proto-synth-v2
// ==============================================================================================================================================
// HARDWARE
// ==============================================================================================================================================
// - Microcontrolador ESP32 DevKit
// - 4 Botones con pull-up     |1 -> PIN 18, 2 -> PIN 4, 3 -> PIN 15, 4 -> PIN 19|
// - 4 LEDs indicadores        |1 -> PIN 23, 2 -> PIN 32, 3 -> PIN 5,  4 -> PIN 2 |
// - 4 Potenciómetros          |1 -> PIN 13, 2 -> PIN 14, 3 -> PIN 12, 4 -> PIN 27|
// - Sensor de luz LDR         |Pin 26|
// - Jack de audio DAC         |Pin 25|
// ==============================================================================================================================================
// DESCRIPCIÓN
// ==============================================================================================================================================
// Sintetizador de cantos de aves usando síntesis FM + AM con envolventes EAD
// (Ataque-Decay Exponencial). Cada botón dispara el canto de un ave diferente.
// Los potenciómetros y el sensor de luz permiten modificar el sonido en tiempo real.
//
// Técnica: síntesis FM donde el modulador barre en frecuencia (EAD), creando el
// glissando característico de los trinos. Una segunda capa AM agrega tremolo.
// ==============================================================================================================================================
// FUNCIONAMIENTO
// ==============================================================================================================================================
// - Botón 1: Jilguero Errante  — trino agudo y flautado
// - Botón 2: Curruca Gris      — canto grave y redondeado
// - Botón 3: Canta-Pinos       — trino rápido con muchas armónicas
// - Botón 4: Tuco-Tuco         — canto expresivo y resonante
//
// - Potenciómetro 1: Tono (pitch) — gira hacia arriba para agudizar el canto
// - Potenciómetro 2: Ataque       — qué tan rápido sube el canto
// - Potenciómetro 3: Cola         — qué tan larga es la estela del sonido
// - Potenciómetro 4: Timbre       — profundidad de la modulación FM (más riqueza armónica)
// - LDR:                          — tremolo (más luz = más vibrato de amplitud)
//
// - LEDs: indica cuál ave está sonando durante el chirp
// ==============================================================================================================================================
// COMENTARIOS
// ==============================================================================================================================================
// - Para subir código exitosamente, asegúrate de que el Potenciómetro 3 esté girado al máximo.
// ==============================================================================================================================================

#include "driver/dac.h"
#include <math.h>

// ==============================================================================================================================================
// CONFIGURACIÓN DE HARDWARE - PINES
// ==============================================================================================================================================
const int BTN_PINS[4] = {18, 4, 15, 19};   // Botones (pull-up → LOW = presionado)
const int LED_PINS[4] = {23, 32, 5, 2};    // LEDs

const int POT_TONO   = 13;   // Potenciómetro 1: Tono (pitch)
const int POT_ATAQUE = 14;   // Potenciómetro 2: Tiempo de ataque
const int POT_COLA   = 12;   // Potenciómetro 3: Tiempo de decay (cola)
const int POT_TIMBRE = 27;   // Potenciómetro 4: Timbre / profundidad FM
const int LDR_PIN    = 26;   // Sensor de luz: profundidad AM (tremolo)

// ==============================================================================================================================================
// CONSTANTES DE SÍNTESIS
// ==============================================================================================================================================
#define SAMPLE_RATE     22050      // Frecuencia de muestreo (Hz)
#define SAMPLE_US       45         // delayMicroseconds entre muestras (≈ 22222 Hz)
#define FREQ_MULT       7000.0f
#define FREQ_OFFSET     300.0f
#define ENV_FREQ_MAX    3000.0f
#define MAX_ATK_DCY     0.9f
#define T60             6.90776f
#define AMPLITUD        110
#define MAX_MUESTRAS    50000      // Límite de seguridad (~2.25 seg a 22222 Hz)

// ==============================================================================================================================================
// PRESETS DE AVES
// ==============================================================================================================================================
struct PresetAve {
  const char* nombre;
  float ifrq, atk, dcy;
  float fmod1, atkf1, dcyf1;
  float fmod2, atkf2, dcyf2;
  float amod1, atka1, dcya1;
  float amod2, atka2, dcya2;
};

const PresetAve PRESETS[4] = {
  // Botón 1: Jilguero Errante — trino agudo y flautado (~4157 Hz base)
  { "Jilguero Errante",
    0.5510f, 0.5918f, 0.1878f,
    0.0716f, 0.0204f, 0.3469f,
    0.0204f, 0.5510f, 0.1224f,
    0.6327f, 1.0000f, 0.6122f,
    0.3469f, 0.8163f, 0.6531f },

  // Botón 2: Curruca Gris — canto grave y redondeado (~1588 Hz base)
  { "Curruca Gris",
    0.1837f, 0.5918f, 0.3878f,
    0.0104f, 0.5306f, 0.3469f,
    0.2449f, 0.5510f, 0.1224f,
    0.3878f, 1.0000f, 0.6122f,
    0.3469f, 0.8163f, 0.6531f },

  // Botón 3: Chincol — trino cálido y musical (~3050 Hz base)
  // El modulador FM barre a baja frecuencia (~180 Hz) con alta profundidad,
  // creando parciales densas y cálidas en lugar del sonido metálico que
  // produce un modulador rápido. Tremolo suave a ~120 Hz.
  { "Chincol",
    0.3930f, 0.0800f, 0.5000f,
    0.0600f, 0.0400f, 0.7500f,
    0.0400f, 0.2000f, 0.6500f,
    0.5800f, 0.4500f, 0.6500f,
    0.1800f, 0.3500f, 0.5500f },

  // Botón 4: Tuco-Tuco — canto grave expresivo con cola larga (~1441 Hz base)
  { "Tuco-Tuco",
    0.1633f, 0.2245f, 0.1837f,
    0.0031f, 0.1224f, 1.0000f,
    0.0612f, 1.0000f, 0.7755f,
    0.9796f, 0.2041f, 0.7347f,
    1.0000f, 0.1429f, 0.6122f }
};

// ==============================================================================================================================================
// ENVOLVENTE EAD — Ataque-Decay Exponencial
// ==============================================================================================================================================
struct EAD {
  float valor    = 0.0f;
  float min_v    = 0.0f;
  float max_v    = 1.0f;
  float atk_coef = 0.9f;
  float dcy_coef = 0.9f;
  float atk_time = 0.3f;
  float elapsed  = 0.0f;
  bool en_ataque = false;
  bool activo    = false;

  void configurar(float atk_s, float dcy_s, float max_val, float min_val = 0.0f) {
    min_v    = min_val;
    max_v    = max_val;
    atk_time = atk_s;
    float tau_a = (atk_s > 0.001f) ? (atk_s * SAMPLE_RATE / T60) : 1.0f;
    float tau_d = (dcy_s > 0.001f) ? (dcy_s * SAMPLE_RATE / T60) : 1.0f;
    atk_coef = expf(-1.0f / tau_a);
    dcy_coef = expf(-1.0f / tau_d);
  }

  void disparar() {
    valor     = min_v;
    elapsed   = 0.0f;
    en_ataque = true;
    activo    = true;
  }

  float procesar() {
    if (!activo) return min_v;
    elapsed += 1.0f / (float)SAMPLE_RATE;
    if (en_ataque) {
      valor = max_v + (valor - max_v) * atk_coef;
      if (elapsed >= atk_time) en_ataque = false;
    } else {
      valor = min_v + (valor - min_v) * dcy_coef;
      if (fabsf(valor - min_v) < 0.0001f) {
        valor  = min_v;
        activo = false;
      }
    }
    return valor;
  }
};

// ==============================================================================================================================================
// ESTADO DEL SINTETIZADOR
// ==============================================================================================================================================
EAD envPrincipal, envFrecFM, envFrecAM, envGanaFM, envGanaAM;

float fasePortador  = 0.0f;
float faseMod       = 0.0f;
float faseAM        = 0.0f;
float freqPortador  = 2000.0f;

// Frecuencia base del preset sin escalar por pot_tono.
// Necesario para recalcular freqPortador en tiempo real dentro de generarChirp.
float freqBasePreset = 2000.0f;

// Variación aleatoria de pitch para el chirp actual (modo sostenido).
// 1.0 = sin variación (toque simple).
float freqVariacion = 1.0f;

// Factores aleatorios de timbre y tremolo para el chirp actual.
// Se aplican como multiplicadores en tiempo real dentro de generarChirp.
// 1.0 = sin variación (toque simple).
float factorFM_chirp = 1.0f;
float factorAM_chirp = 1.0f;

// ==============================================================================================================================================
// FUNCIÓN DE SÍNTESIS — genera el chirp completo de forma bloqueante
// ==============================================================================================================================================
// Patrón igual al de los firmwares existentes (trance_synth_ldr, sequenciador):
// genera las muestras en un loop con delayMicroseconds en lugar de timer ISR,
// evitando los problemas de FPU en ISR del ESP32 core v3.x.
void generarChirp(int led_idx) {
  // Resetear fases y disparar todos los envolventes
  fasePortador = 0.0f;
  faseMod      = 0.0f;
  faseAM       = 0.0f;
  envPrincipal.disparar();
  envFrecFM.disparar();
  envFrecAM.disparar();
  envGanaFM.disparar();
  envGanaAM.disparar();

  // LED encendido durante el chirp
  digitalWrite(LED_PINS[led_idx], HIGH);

  // Lectura inicial de los controles en tiempo real
  // Las lecturas se escalonan cada 400 muestras para no introducir clicks:
  //   muestra % 400 == 100 → POT_TONO  → freqPortador
  //   muestra % 400 == 200 → POT_TIMBRE → escala_timbre
  //   muestra % 400 == 300 → LDR       → escala_ldr
  float escala_timbre = (0.05f + (float)(4095 - analogRead(POT_TIMBRE)) / 4095.0f * 0.95f)
                        * factorFM_chirp;
  float escala_ldr    = (float)analogRead(LDR_PIN) / 4095.0f * factorAM_chirp;

  int muestras = 0;

  while (envPrincipal.activo && muestras < MAX_MUESTRAS) {

    // ── Lectura escalonada de controles en tiempo real (~18 ms por ciclo) ─
    int fase_adc = muestras % 400;
    if (fase_adc == 100) {
      float v  = (float)(4095 - analogRead(POT_TONO)) / 4095.0f;
      freqPortador = freqBasePreset * freqVariacion * (0.5f + v * 1.5f);
    } else if (fase_adc == 200) {
      float v  = (float)(4095 - analogRead(POT_TIMBRE)) / 4095.0f;
      escala_timbre = (0.05f + v * 0.95f) * factorFM_chirp;
    } else if (fase_adc == 300) {
      float v  = (float)analogRead(LDR_PIN) / 4095.0f;
      escala_ldr = v * factorAM_chirp;
    }

    // ── Procesar envolventes ──────────────────────────────────────────────
    float ganancia = envPrincipal.procesar();
    float freq_fm  = envFrecFM.procesar();
    float freq_am  = envFrecAM.procesar();
    // Aplicar escalas en tiempo real: el envelope da la forma, el pot da el nivel
    float prof_fm  = envGanaFM.procesar() * escala_timbre;
    float prof_am  = envGanaAM.procesar() * escala_ldr;

    // ── Avanzar fases de modulador y AM ──────────────────────────────────
    faseMod += freq_fm / (float)SAMPLE_RATE;
    faseAM  += freq_am / (float)SAMPLE_RATE;
    if (faseMod >= 1.0f) faseMod -= 1.0f;
    if (faseAM  >= 1.0f) faseAM  -= 1.0f;

    // ── Síntesis FM ───────────────────────────────────────────────────────
    float desplazamiento = sinf(2.0f * (float)M_PI * faseMod) * prof_fm * freqPortador;
    fasePortador += (freqPortador + desplazamiento) / (float)SAMPLE_RATE;
    float señal_fm = sinf(2.0f * (float)M_PI * fasePortador);

    // ── Modulación de amplitud (AM / tremolo) ─────────────────────────────
    float señal_am = 1.0f - prof_am * sinf(2.0f * (float)M_PI * faseAM);

    // ── Mezcla y salida DAC ───────────────────────────────────────────────
    float salida = ganancia * señal_fm * señal_am;
    int dac_val = 128 + (int)(salida * (float)AMPLITUD);
    if (dac_val < 0)   dac_val = 0;
    if (dac_val > 255) dac_val = 255;
    dac_output_voltage(DAC_CHANNEL_1, dac_val);

    delayMicroseconds(SAMPLE_US);
    muestras++;
  }

  // Silencio al terminar
  dac_output_voltage(DAC_CHANNEL_1, 128);
  digitalWrite(LED_PINS[led_idx], LOW);
}

// ==============================================================================================================================================
// HELPER — configura los cinco envolventes para un preset dado
// ==============================================================================================================================================
// Se llama desde loop() antes de cada chirp. Separada de generarChirp() para
// poder llamarla dos veces con distintos factores (toque corto vs sostenido).
// pot_timbre y ldr_val ya NO se pasan aquí: generarChirp los lee en tiempo real
// y los aplica como multiplicadores sobre el resultado del envelope.
void configurarEnvolventes(int idx, float f_atk, float f_dcy) {
  const PresetAve& p = PRESETS[idx];

  envPrincipal.configurar(
    MAX_ATK_DCY * p.atk   * f_atk,
    MAX_ATK_DCY * p.dcy   * f_dcy,
    1.0f);

  envFrecFM.configurar(
    MAX_ATK_DCY * p.atkf1 * f_atk,
    MAX_ATK_DCY * p.dcyf1 * f_dcy,
    ENV_FREQ_MAX * p.fmod1);

  envFrecAM.configurar(
    MAX_ATK_DCY * p.atkf2 * f_atk,
    MAX_ATK_DCY * p.dcyf2 * f_dcy,
    ENV_FREQ_MAX * p.fmod2);

  // max_v = valor bruto del preset; la escala viva se aplica en generarChirp
  envGanaFM.configurar(
    MAX_ATK_DCY * p.atka1 * f_atk,
    MAX_ATK_DCY * p.dcya1 * f_dcy,
    p.amod1);

  envGanaAM.configurar(
    MAX_ATK_DCY * p.atka2 * f_atk,
    MAX_ATK_DCY * p.dcya2 * f_dcy,
    p.amod2);
}

// ==============================================================================================================================================
// VARIABLES DE CONTROL
// ==============================================================================================================================================
bool          ultimoEstado[4];
unsigned long ultimaPresion[4];
const unsigned long DEBOUNCE_MS = 50;

// ==============================================================================================================================================
// SETUP
// ==============================================================================================================================================
void setup() {
  esp_log_level_set("*", ESP_LOG_NONE);

  for (int i = 0; i < 4; i++) {
    pinMode(BTN_PINS[i], INPUT_PULLUP);
  }
  for (int i = 0; i < 4; i++) {
    pinMode(LED_PINS[i], OUTPUT);
    digitalWrite(LED_PINS[i], LOW);
  }
  pinMode(POT_TONO,   INPUT);
  pinMode(POT_ATAQUE, INPUT);
  pinMode(POT_COLA,   INPUT);
  pinMode(POT_TIMBRE, INPUT);
  pinMode(LDR_PIN,    INPUT);

  dac_output_enable(DAC_CHANNEL_1);
  dac_output_voltage(DAC_CHANNEL_1, 128);

  // Semilla aleatoria usando el LDR (varía con la luz ambiente)
  randomSeed(analogRead(LDR_PIN) ^ millis());

  // Inicializar estados de botones AHORA para evitar disparo espurio al arrancar
  for (int i = 0; i < 4; i++) {
    ultimoEstado[i]  = HIGH;
    ultimaPresion[i] = millis();
  }

  // Animación de inicio
  for (int i = 0; i < 4; i++) {
    digitalWrite(LED_PINS[i], HIGH);
    delay(120);
    digitalWrite(LED_PINS[i], LOW);
    delay(40);
  }
}

// ==============================================================================================================================================
// LOOP PRINCIPAL
// ==============================================================================================================================================
void loop() {
  unsigned long ahora = millis();

  // Leer potenciómetros (invertidos por hardware v2.0)
  float pot_tono   = (float)(4095 - analogRead(POT_TONO))   / 4095.0f;
  float pot_ataque = (float)(4095 - analogRead(POT_ATAQUE)) / 4095.0f;
  float pot_cola   = (float)(4095 - analogRead(POT_COLA))   / 4095.0f;
  float pot_timbre = (float)(4095 - analogRead(POT_TIMBRE)) / 4095.0f;
  float ldr_val    = (float)analogRead(LDR_PIN) / 4095.0f;

  for (int i = 0; i < 4; i++) {
    bool estado = digitalRead(BTN_PINS[i]);

    if (estado == LOW && ultimoEstado[i] == HIGH &&
        (ahora - ultimaPresion[i]) > DEBOUNCE_MS) {

      // Frecuencia base del preset (sin pot_tono); guardada globalmente para
      // que generarChirp pueda recalcular freqPortador en tiempo real.
      freqBasePreset = FREQ_OFFSET + FREQ_MULT * PRESETS[i].ifrq;
      freqVariacion  = 1.0f;
      factorFM_chirp = 1.0f;
      factorAM_chirp = 1.0f;
      freqPortador   = freqBasePreset * (0.5f + pot_tono * 1.5f);

      // Factores temporales leídos de los potenciómetros
      float f_atk = 0.2f + pot_ataque * 0.8f;
      float f_dcy = 0.2f + pot_cola   * 0.8f;

      // ── Primer chirp: duración completa ──────────────────────────────────
      configurarEnvolventes(i, f_atk, f_dcy);
      generarChirp(i);

      // ── Modo sostenido: canto aleatorio mientras el botón permanezca LOW ──
      bool repitiendo = true;

      while (repitiendo && digitalRead(BTN_PINS[i]) == LOW) {

        // ── Pausa aleatoria ─────────────────────────────────────────────────
        // 10% de probabilidad de "respiro" largo
        int pausa_ms = (random(10) == 0) ? random(320, 620) : random(45, 210);

        unsigned long t_pausa = millis();
        while (millis() - t_pausa < (unsigned long)pausa_ms) {
          if (digitalRead(BTN_PINS[i]) == HIGH) { repitiendo = false; break; }
          delay(5);
        }
        if (!repitiendo) break;

        // ── Variaciones aleatorias por chirp ──────────────────────────────
        // Pitch: ±13% (freqVariacion persiste dentro de generarChirp)
        freqVariacion  = 0.87f + random(27) / 100.0f;
        // Timbre FM: ±25% (aplicado en tiempo real como multiplicador)
        factorFM_chirp = 0.75f + random(51) / 100.0f;
        // Tremolo AM: ±35%
        factorAM_chirp = 0.65f + random(71) / 100.0f;
        // Duración y ataque
        float f_dcy_r  = 0.15f + random(51) / 100.0f;
        float f_atk_r  = 0.30f + random(41) / 100.0f;

        freqPortador = freqBasePreset * freqVariacion * (0.5f + pot_tono * 1.5f);

        // ── 25% de probabilidad: doble trino ─────────────────────────────
        if (random(4) == 0) {

          // Primer chirp del par
          freqVariacion = 0.92f + random(20) / 100.0f;
          freqPortador  = freqBasePreset * freqVariacion * (0.5f + pot_tono * 1.5f);
          configurarEnvolventes(i, f_atk * f_atk_r * 0.55f, f_dcy * f_dcy_r * 0.45f);
          generarChirp(i);

          // Micro pausa (15–45 ms)
          unsigned long t_micro = millis();
          while (millis() - t_micro < (unsigned long)random(15, 46)) {
            if (digitalRead(BTN_PINS[i]) == HIGH) { repitiendo = false; break; }
            delay(3);
          }
          if (!repitiendo) break;

          // Segundo chirp del par — pitch distinto
          freqVariacion = 0.88f + random(26) / 100.0f;
          freqPortador  = freqBasePreset * freqVariacion * (0.5f + pot_tono * 1.5f);
          configurarEnvolventes(i, f_atk * f_atk_r * 0.50f, f_dcy * f_dcy_r * 0.55f);
          generarChirp(i);

        } else {
          // Chirp simple
          configurarEnvolventes(i, f_atk * f_atk_r, f_dcy * f_dcy_r);
          generarChirp(i);
        }
      }

      ultimaPresion[i] = millis();
    }

    ultimoEstado[i] = estado;
  }

  delay(5);
}


// ==============================================================================================================================================
// PROTO-SYNTH V2 - Giorgio by Moroder MIDI - GC Lab Chile
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
// ==============================================================================================================================================

// ==============================================================================================================================================
// DESCRIPCIÓN
// ==============================================================================================================================================
// Secuenciador MIDI con la melodía completa de "Giorgio by Moroder" (Daft Punk).
// La secuencia se activa con un botón. Los potenciómetros controlan parámetros de
// síntesis vía MIDI CC y el sensor IMU controla el cutoff/brillo del filtro.
//
// El timer de hardware garantiza que el tiempo entre pasos sea exacto y no se
// pierdan notas por lecturas lentas del IMU o del bus I2C.
// ==============================================================================================================================================

// ==============================================================================================================================================
// FUNCIONAMIENTO
// ==============================================================================================================================================
// CONTROLES:
// - Botón 1: Play / Stop de la secuencia
// - Botón 2: Aumentar tempo
// - Botón 3: Disminuir tempo
// - Botón 4: Reiniciar secuencia al paso 1
// - Potenciómetro 1: Attack  → CC73 (Canal MIDI 1)
// - Potenciómetro 2: Decay   → CC72 (Canal MIDI 1)
// - Potenciómetro 3: Resonancia → CC71 (Canal MIDI 1)
// - Potenciómetro 4: Reverb  → CC91 (Canal MIDI 1)
// - IMU (inclinación eje Y): Cutoff/Brillo → CC74 (Canal MIDI 1)
// - LED 1-4: indican el grupo de 4 pasos actual dentro de la secuencia
//
// MODO DE USO:
// 1. Conectar Proto-Synth a DAW/sintetizador vía MIDI
//    (o USB-MIDI usando Hairless MIDI y cambiando el baudio a 115200)
// 2. Presionar Botón 1 para iniciar la secuencia (los LEDs comienzan a ciclar)
// 3. Ajustar Attack, Decay, Resonancia y Reverb con los potenciómetros
// 4. Inclinar el dispositivo (eje Y) para abrir/cerrar el cutoff del filtro
// 5. Botones 2 y 3 para cambiar el tempo en tiempo real
// 6. Botón 4 para reiniciar la secuencia al inicio
// 7. Presionar Botón 1 nuevamente para detener
// ==============================================================================================================================================

// ==============================================================================================================================================
// COMENTARIOS
// ==============================================================================================================================================
// - Para subir código exitosamente, asegúrate de que el Potenciómetro 3 esté girado al máximo.
// - Los pines 2,4,12,13,14,15,25,26,27 no funcionan si el Bluetooth está activado (ADC2).
// - La secuencia tiene 368 pasos (16 corcheas por compás, 23 compases) y se repite en bucle.
// - El timer de hardware dispara el avance de paso: la ISR SOLO activa una bandera y
//   la nota se envía desde loop(). Esto evita envíos MIDI dentro de la ISR y garantiza
//   que no se pierdan notas aunque el bus I2C sea lento.
// ==============================================================================================================================================

// ==============================================================================================================================================
// LIBRERÍAS
// ==============================================================================================================================================
#include <Bounce2.h>
#include <Wire.h>

// ==============================================================================================================================================
// CONFIGURACIÓN DE HARDWARE - PINES
// ==============================================================================================================================================
#define MPU6050_ADDR 0x68

const int POT1 = 13;   // Attack     → CC73
const int POT2 = 14;   // Decay      → CC72
const int POT3 = 12;   // Resonancia → CC71
const int POT4 = 27;   // Reverb     → CC91

const int BOTON1 = 18; // Play / Stop
const int BOTON2 = 4;  // Tempo Up
const int BOTON3 = 15; // Tempo Down
const int BOTON4 = 19; // Reiniciar secuencia

const int LED1 = 23;
const int LED2 = 32;
const int LED3 = 5;
const int LED4 = 2;

// ==============================================================================================================================================
// PARÁMETROS MIDI
// ==============================================================================================================================================
#define CANAL_MIDI     1    // Canal MIDI para notas y CCs
#define VELOCIDAD_NOTA 100  // Velocidad fija de las notas (0-127)

#define CC_CUTOFF      74   // IMU → Brightness / Cutoff Filter (estándar GM)
#define CC_RESONANCIA  71   // POT3 → Timbre / Resonancia
#define CC_DECAY       72   // POT2 → Release Time
#define CC_ATTACK      73   // POT1 → Attack Time
#define CC_REVERB      91   // POT4 → Effects 1 Depth (Reverb)

// ==============================================================================================================================================
// SECUENCIA MIDI - "Giorgio by Moroder" (Daft Punk)
// ==============================================================================================================================================
// Notas convertidas de frecuencias Hz a números MIDI.
// Referencia de conversión: note = round(69 + 12 * log2(freq / 440))
//
// Leyenda de notas usadas:
//   62=D4  64=E4  65=F4  67=G4  69=A4
//   74=D5  76=E5  77=F5  78=F#5  79=G5  81=A5  83=B5  84=C6  86=D6  88=E6
//
// Estructura de bloques (16 notas c/u):
//   Am  → A4 como bajo: 81,69,84,69,83,69,84,88,69,81,84,69,83,69,84,81
//   Em  → E4 como bajo: 76,64,79,64,78,64,79,83,64,76,79,64,78,64,79,76
//   Fm  → F5 como bajo: 77,77,81,77,79,77,81,84,77,77,81,77,79,77,81,77
//   Gm  → G5 como bajo: 79,79,83,79,81,79,83,86,79,79,83,79,81,79,83,79
//   Dm  → D4 como bajo: 74,62,77,62,76,62,77,81,62,74,77,62,76,62,77,74
//   FmV → F4 como bajo: 76,65,81,65,79,65,81,84,65,77,81,65,79,65,81,77
//   GmV → G4 como bajo: 79,67,83,67,81,67,83,86,67,79,83,67,81,67,83,79
// ==============================================================================================================================================
const int LONGITUD_SECUENCIA = 368;

const byte SECUENCIA[] = {
  // ── Am ×2 ───────────────────────────────────────────────────────────────────
  81,69,84,69,83,69,84,88,69,81,84,69,83,69,84,81,
  81,69,84,69,83,69,84,88,69,81,84,69,83,69,84,81,
  // ── Em ×2 ───────────────────────────────────────────────────────────────────
  76,64,79,64,78,64,79,83,64,76,79,64,78,64,79,76,
  76,64,79,64,78,64,79,83,64,76,79,64,78,64,79,76,
  // ── Am ×2 (reprise) ─────────────────────────────────────────────────────────
  81,69,84,69,83,69,84,88,69,81,84,69,83,69,84,81,
  81,69,84,69,83,69,84,88,69,81,84,69,83,69,84,81,
  // ── Em ×2 (reprise) ─────────────────────────────────────────────────────────
  76,64,79,64,78,64,79,83,64,76,79,64,78,64,79,76,
  76,64,79,64,78,64,79,83,64,76,79,64,78,64,79,76,
  // ── Fm ──────────────────────────────────────────────────────────────────────
  77,77,81,77,79,77,81,84,77,77,81,77,79,77,81,77,
  // ── Gm ──────────────────────────────────────────────────────────────────────
  79,79,83,79,81,79,83,86,79,79,83,79,81,79,83,79,
  // ── Dm ──────────────────────────────────────────────────────────────────────
  74,62,77,62,76,62,77,81,62,74,77,62,76,62,77,74,
  // ── Em ──────────────────────────────────────────────────────────────────────
  76,64,79,64,78,64,79,83,64,76,79,64,78,64,79,76,
  // ── FmV (raíz F4) ───────────────────────────────────────────────────────────
  76,65,81,65,79,65,81,84,65,77,81,65,79,65,81,77,
  // ── GmV (raíz G4) ───────────────────────────────────────────────────────────
  79,67,83,67,81,67,83,86,67,79,83,67,81,67,83,79,
  // ── Em ×2 ───────────────────────────────────────────────────────────────────
  76,64,79,64,78,64,79,83,64,76,79,64,78,64,79,76,
  76,64,79,64,78,64,79,83,64,76,79,64,78,64,79,76,
  // ── Gm ──────────────────────────────────────────────────────────────────────
  79,79,83,79,81,79,83,86,79,79,83,79,81,79,83,79,
  // ── Dm ──────────────────────────────────────────────────────────────────────
  74,62,77,62,76,62,77,81,62,74,77,62,76,62,77,74,
  // ── Em ──────────────────────────────────────────────────────────────────────
  76,64,79,64,78,64,79,83,64,76,79,64,78,64,79,76,
  // ── FmV (raíz F4) ───────────────────────────────────────────────────────────
  76,65,81,65,79,65,81,84,65,77,81,65,79,65,81,77,
  // ── GmV (raíz G4) ───────────────────────────────────────────────────────────
  79,67,83,67,81,67,83,86,67,79,83,67,81,67,83,79,
  // ── Em ×2 (final) ───────────────────────────────────────────────────────────
  76,64,79,64,78,64,79,83,64,76,79,64,78,64,79,76,
  76,64,79,64,78,64,79,83,64,76,79,64,78,64,79,76
};

// ==============================================================================================================================================
// VARIABLES DEL SECUENCIADOR
// ==============================================================================================================================================
bool secuenciadorActivo = false;
int  pasoActual         = LONGITUD_SECUENCIA - 1; // Primer avance → paso 0
int  nivelTempo         = 5;

// Velocidades disponibles (ms por paso de 16ava nota)
// Nivel 5 = 130ms ≈ 115 BPM | Rango: ~37 BPM (400ms) a ~375 BPM (40ms)
const int nivelesVelocidad[] = {400, 300, 250, 200, 160, 130, 110, 90, 70, 55, 40};
const int MAX_NIVEL_TEMPO = 10;
int intervalo = 130;

// Timer de hardware — la ISR solo activa la bandera; la nota se envía en loop()
hw_timer_t *timerPasos = NULL;
volatile bool flagPaso = false;

// Control de NoteOff y ventana de blackout para los CCs
byte  ultimaNotaTocada  = 0;
unsigned long tiempoUltimaNota = 0;

// ==============================================================================================================================================
// VARIABLES DEL IMU → CC CUTOFF
// ==============================================================================================================================================
int16_t accelY, accelZ;
float   anguloY        = 0;
float   anguloFiltrado = 0;
const float ALPHA_FILTRO = 0.3; // 0=muy suave, 1=sin filtro

unsigned long ultimaLecturaMPU  = 0;
const unsigned long INTERVALO_MPU = 10; // ms entre lecturas del MPU

int   ultimoCutoff     = 64;  // Valor previo enviado
int   cutoffSuavizado  = 64;  // Valor con suavizado extra
unsigned long ultimoEnvioCutoff = 0;
const unsigned long INTERVALO_MIN_CC_IMU = 30; // ms mínimo entre envíos de CC del IMU
const int CAMBIO_MIN_CUTOFF = 1;               // Cambio mínimo para enviar CC

// Ventanas de blackout alrededor de cada nota (evitan colisión en el bus MIDI)
const unsigned long BLACKOUT_POST_NOTA = 25; // ms tras enviar una nota
const unsigned long BLACKOUT_PRE_NOTA  = 8;  // ms antes del próximo paso

// ==============================================================================================================================================
// VARIABLES DE POTENCIÓMETROS → CCs (Attack, Decay, Reso, Reverb)
// ==============================================================================================================================================
int ultimoAtaque  = -1;
int ultimoDecay   = -1;
int ultimoReso    = -1;
int ultimoReverb  = -1;

unsigned long ultimaLecturaPots = 0;
const unsigned long INTERVALO_POTS = 50; // ms entre barridos de pots
const int DEADBAND_POT = 2;              // Cambio mínimo para enviar CC

// ==============================================================================================================================================
// EFECTOS VISUALES NO BLOQUEANTES
// ==============================================================================================================================================
enum EfectoVisual { SIN_EFECTO, EFECTO_TEMPO, EFECTO_PLAY, EFECTO_RESET };
EfectoVisual efectoActual = SIN_EFECTO;
unsigned long inicioEfecto = 0;
int pasoEfecto   = 0;
int ciclosEfecto = 0;

// ==============================================================================================================================================
// BOTONES (Bounce2 — anti-rebote por software)
// ==============================================================================================================================================
Bounce btnPlayStop  = Bounce();
Bounce btnTempoUp   = Bounce();
Bounce btnTempoDown = Bounce();
Bounce btnReset     = Bounce();

// ==============================================================================================================================================
// FUNCIONES MIDI INLINE
// ==============================================================================================================================================
inline void enviarNoteOn(byte nota, byte velocidad, byte canal) {
  Serial.write(0x90 + (canal - 1));
  Serial.write(nota);
  Serial.write(velocidad);
}

inline void enviarNoteOff(byte nota, byte velocidad, byte canal) {
  Serial.write(0x80 + (canal - 1));
  Serial.write(nota);
  Serial.write(velocidad);
}

inline void enviarCC(byte controlador, byte valor, byte canal) {
  Serial.write(0xB0 + (canal - 1));
  Serial.write(controlador);
  Serial.write(valor);
}

// ==============================================================================================================================================
// ISR DEL TIMER
// La ISR únicamente activa la bandera. Así el NoteOn se envía desde loop(),
// nunca desde dentro de la interrupción. Esto evita que el bus MIDI o el I2C
// queden bloqueados y que se pierdan pasos de la secuencia.
// ==============================================================================================================================================
void IRAM_ATTR ISR_Paso() {
  flagPaso = true;
}

void actualizarTimer() {
  timerAlarm(timerPasos, (uint64_t)intervalo * 1000, true, 0); // intervalo ms → µs
}

// ==============================================================================================================================================
// SETUP
// ==============================================================================================================================================
void setup() {
  // Suprimir logs internos del ESP32 y abrir Serial en modo MIDI
  esp_log_level_set("*", ESP_LOG_NONE);
  Serial.end();
  delay(100);
  Serial.begin(31250, SERIAL_8N1, 3, 1); // RX=3, TX=1 (TX0)

  // I2C rápido con timeout para evitar bloqueos
  Wire.begin();
  Wire.setClock(400000); // 400kHz
  Wire.setTimeOut(3);    // 3ms máximo por transacción
  inicializarMPU6050();

  // Pines de botones y LEDs
  pinMode(BOTON1, INPUT_PULLUP);
  pinMode(BOTON2, INPUT_PULLUP);
  pinMode(BOTON3, INPUT_PULLUP);
  pinMode(BOTON4, INPUT_PULLUP);

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);

  // Configurar Bounce2 con intervalo de debounce
  btnPlayStop.attach(BOTON1);  btnPlayStop.interval(15);
  btnTempoUp.attach(BOTON2);   btnTempoUp.interval(15);
  btnTempoDown.attach(BOTON3); btnTempoDown.interval(15);
  btnReset.attach(BOTON4);     btnReset.interval(15);

  intervalo = nivelesVelocidad[nivelTempo];

  // Timer de hardware — API ESP32 Arduino 3.x: timerBegin(Hz) → 1MHz = 1 tick/µs
  timerPasos = timerBegin(1000000);
  timerAttachInterrupt(timerPasos, &ISR_Paso);
  timerAlarm(timerPasos, (uint64_t)intervalo * 1000, true, 0);
  // El timer corre siempre; la bandera se ignora si secuenciadorActivo==false

  // Lecturas iniciales del MPU para estabilizar el filtro
  for (int i = 0; i < 10; i++) {
    leerMPU6050();
    delay(10);
  }

  secuenciaDeArranque();
}

// ==============================================================================================================================================
// LOOP PRINCIPAL
// ==============================================================================================================================================
void loop() {
  unsigned long ahora = millis();

  // ── Prioridad 1: Paso del secuenciador (timing exacto por hardware) ──────────
  if (flagPaso) {
    flagPaso = false;
    if (secuenciadorActivo) {
      avanzarPaso();
    }
  }

  // ── Prioridad 2: IMU → CC Cutoff (CC74) ─────────────────────────────────────
  actualizarCutoffIMU(ahora);

  // ── Prioridad 3: Potenciómetros → CCs de parámetros ─────────────────────────
  actualizarPots(ahora);

  // ── Prioridad 4: Botones ─────────────────────────────────────────────────────
  actualizarBotones();

  // ── Prioridad 5: Efectos visuales ────────────────────────────────────────────
  gestionarEfectosVisuales();
}

// ==============================================================================================================================================
// AVANZAR PASO Y ENVIAR NOTA
// ==============================================================================================================================================
void avanzarPaso() {
  // NoteOff de la nota anterior (crea articulación natural entre pasos)
  if (ultimaNotaTocada != 0) {
    enviarNoteOff(ultimaNotaTocada, 0, CANAL_MIDI);
    ultimaNotaTocada = 0;
  }

  pasoActual = (pasoActual + 1) % LONGITUD_SECUENCIA;

  if (efectoActual == SIN_EFECTO) {
    actualizarLEDs();
  }

  byte nota = SECUENCIA[pasoActual];
  enviarNoteOn(nota, VELOCIDAD_NOTA, CANAL_MIDI);
  ultimaNotaTocada   = nota;
  tiempoUltimaNota   = millis();
}

// ==============================================================================================================================================
// LEDs — ciclo de 4 (muestra en qué grupo de 4 pasos estamos)
// ==============================================================================================================================================
void actualizarLEDs() {
  int grupo = pasoActual % 4;
  digitalWrite(LED1, grupo == 0 ? HIGH : LOW);
  digitalWrite(LED2, grupo == 1 ? HIGH : LOW);
  digitalWrite(LED3, grupo == 2 ? HIGH : LOW);
  digitalWrite(LED4, grupo == 3 ? HIGH : LOW);
}

void apagarTodosLEDs() {
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);
}

// ==============================================================================================================================================
// IMU → CC CUTOFF (CC74)
// La inclinación del dispositivo en el eje Y controla el brillo/cutoff del filtro.
// Se aplican ventanas de silencio antes y después de cada nota para evitar
// colisiones en el bus MIDI que provocarían pérdida de notas.
// ==============================================================================================================================================
void actualizarCutoffIMU(unsigned long ahora) {
  if (ahora - ultimaLecturaMPU < INTERVALO_MPU) return;
  ultimaLecturaMPU = ahora;

  leerMPU6050Fast();

  if (abs(accelY) > 100 && abs(accelZ) > 100) {
    float ay = accelY / 16384.0;
    float az = accelZ / 16384.0;

    anguloY        = atan2(ay, az) * 180.0 / PI;
    anguloFiltrado = ALPHA_FILTRO * anguloY + (1.0 - ALPHA_FILTRO) * anguloFiltrado;

    float anguloClamped = constrain(anguloFiltrado, -30.0, 30.0);
    int   targetCutoff  = map((int)(anguloClamped * 100), -3000, 3000, 0, 127);
    targetCutoff        = constrain(targetCutoff, 0, 127);

    // Suavizado adicional para evitar escalones bruscos
    cutoffSuavizado = (cutoffSuavizado * 3 + targetCutoff) / 4;

    // Calcular tiempo hasta el próximo paso
    unsigned long proximoPaso  = tiempoUltimaNota + (unsigned long)intervalo;
    unsigned long hastaProximo = (ahora < proximoPaso) ? (proximoPaso - ahora) : 0;

    // Enviar CC solo fuera de las ventanas post-nota y pre-nota
    if (abs(cutoffSuavizado - ultimoCutoff) >= CAMBIO_MIN_CUTOFF &&
        ahora - ultimoEnvioCutoff >= INTERVALO_MIN_CC_IMU &&
        ahora - tiempoUltimaNota  >= BLACKOUT_POST_NOTA &&
        hastaProximo               > BLACKOUT_PRE_NOTA) {

      enviarCC(CC_CUTOFF, (byte)cutoffSuavizado, CANAL_MIDI);
      ultimoCutoff      = cutoffSuavizado;
      ultimoEnvioCutoff = ahora;
    }
  }
}

// ==============================================================================================================================================
// POTENCIÓMETROS → CCs (Attack, Decay, Resonancia, Reverb)
// Se respetan las mismas ventanas de silencio que el IMU para no saturar el bus.
// ==============================================================================================================================================
void actualizarPots(unsigned long ahora) {
  if (ahora - ultimaLecturaPots < INTERVALO_POTS) return;
  ultimaLecturaPots = ahora;

  // Respetar ventana de blackout alrededor de cada nota
  unsigned long proximoPaso  = tiempoUltimaNota + (unsigned long)intervalo;
  unsigned long hastaProximo = (ahora < proximoPaso) ? (proximoPaso - ahora) : 0;

  if (ahora - tiempoUltimaNota < BLACKOUT_POST_NOTA || hastaProximo <= BLACKOUT_PRE_NOTA) {
    return;
  }

  // POT1 → Attack (CC73) — pots invertidos en v2.0: 4095 - analogRead()
  int vAtaque = map(4095 - analogRead(POT1), 0, 4095, 0, 127);
  vAtaque = constrain(vAtaque, 0, 127);
  if (abs(vAtaque - ultimoAtaque) >= DEADBAND_POT) {
    enviarCC(CC_ATTACK, (byte)vAtaque, CANAL_MIDI);
    ultimoAtaque = vAtaque;
  }

  // POT2 → Decay / Release (CC72)
  int vDecay = map(4095 - analogRead(POT2), 0, 4095, 0, 127);
  vDecay = constrain(vDecay, 0, 127);
  if (abs(vDecay - ultimoDecay) >= DEADBAND_POT) {
    enviarCC(CC_DECAY, (byte)vDecay, CANAL_MIDI);
    ultimoDecay = vDecay;
  }

  // POT3 → Resonancia / Timbre (CC71)
  int vReso = map(4095 - analogRead(POT3), 0, 4095, 0, 127);
  vReso = constrain(vReso, 0, 127);
  if (abs(vReso - ultimoReso) >= DEADBAND_POT) {
    enviarCC(CC_RESONANCIA, (byte)vReso, CANAL_MIDI);
    ultimoReso = vReso;
  }

  // POT4 → Reverb Send (CC91)
  int vReverb = map(4095 - analogRead(POT4), 0, 4095, 0, 127);
  vReverb = constrain(vReverb, 0, 127);
  if (abs(vReverb - ultimoReverb) >= DEADBAND_POT) {
    enviarCC(CC_REVERB, (byte)vReverb, CANAL_MIDI);
    ultimoReverb = vReverb;
  }
}

// ==============================================================================================================================================
// BOTONES
// ==============================================================================================================================================
void actualizarBotones() {
  btnPlayStop.update();
  btnTempoUp.update();
  btnTempoDown.update();
  btnReset.update();

  // Botón 1: Play / Stop
  if (btnPlayStop.fell()) {
    secuenciadorActivo = !secuenciadorActivo;
    if (secuenciadorActivo) {
      iniciarEfecto(EFECTO_PLAY);
    } else {
      // Apagar la última nota al detener
      if (ultimaNotaTocada != 0) {
        enviarNoteOff(ultimaNotaTocada, 0, CANAL_MIDI);
        ultimaNotaTocada = 0;
      }
      apagarTodosLEDs();
    }
  }

  // Botón 2: Tempo Up
  if (btnTempoUp.fell()) {
    if (nivelTempo < MAX_NIVEL_TEMPO) {
      nivelTempo++;
      intervalo = nivelesVelocidad[nivelTempo];
      actualizarTimer();
      iniciarEfecto(EFECTO_TEMPO);
    }
  }

  // Botón 3: Tempo Down
  if (btnTempoDown.fell()) {
    if (nivelTempo > 0) {
      nivelTempo--;
      intervalo = nivelesVelocidad[nivelTempo];
      actualizarTimer();
      iniciarEfecto(EFECTO_TEMPO);
    }
  }

  // Botón 4: Reiniciar secuencia al paso 0
  if (btnReset.fell()) {
    if (ultimaNotaTocada != 0) {
      enviarNoteOff(ultimaNotaTocada, 0, CANAL_MIDI);
      ultimaNotaTocada = 0;
    }
    pasoActual = LONGITUD_SECUENCIA - 1; // El próximo avance irá al paso 0
    iniciarEfecto(EFECTO_RESET);
  }
}

// ==============================================================================================================================================
// EFECTOS VISUALES NO BLOQUEANTES
// ==============================================================================================================================================
void iniciarEfecto(EfectoVisual efecto) {
  efectoActual = efecto;
  inicioEfecto = millis();
  pasoEfecto   = 0;
  ciclosEfecto = 0;
}

void gestionarEfectosVisuales() {
  if (efectoActual == SIN_EFECTO) return;

  unsigned long ahora = millis();

  switch (efectoActual) {

    case EFECTO_TEMPO: {
      // Todos los LEDs parpadean 2 veces para confirmar cambio de tempo
      if (ahora - inicioEfecto >= 80) {
        inicioEfecto = ahora;
        bool enc = (pasoEfecto == 0);
        digitalWrite(LED1, enc); digitalWrite(LED2, enc);
        digitalWrite(LED3, enc); digitalWrite(LED4, enc);
        pasoEfecto = 1 - pasoEfecto;
        if (pasoEfecto == 0) {
          ciclosEfecto++;
          if (ciclosEfecto >= 2) {
            efectoActual = SIN_EFECTO;
            if (secuenciadorActivo) actualizarLEDs();
            else apagarTodosLEDs();
          }
        }
      }
      break;
    }

    case EFECTO_PLAY: {
      // LEDs barren de 1 a 4 para indicar inicio de reproducción
      if (ahora - inicioEfecto >= 70) {
        inicioEfecto = ahora;
        apagarTodosLEDs();
        switch (pasoEfecto) {
          case 0: digitalWrite(LED1, HIGH); break;
          case 1: digitalWrite(LED2, HIGH); break;
          case 2: digitalWrite(LED3, HIGH); break;
          case 3: digitalWrite(LED4, HIGH); break;
        }
        pasoEfecto++;
        if (pasoEfecto >= 4) {
          efectoActual = SIN_EFECTO;
          if (secuenciadorActivo) actualizarLEDs();
          else apagarTodosLEDs();
        }
      }
      break;
    }

    case EFECTO_RESET: {
      // Todos los LEDs parpadean rápido 500ms para confirmar reset
      bool enc = ((ahora - inicioEfecto) / 70) % 2 == 0;
      digitalWrite(LED1, enc); digitalWrite(LED2, enc);
      digitalWrite(LED3, enc); digitalWrite(LED4, enc);
      if (ahora - inicioEfecto >= 500) {
        efectoActual = SIN_EFECTO;
        if (secuenciadorActivo) actualizarLEDs();
        else apagarTodosLEDs();
      }
      break;
    }

    default:
      break;
  }
}

// ==============================================================================================================================================
// SECUENCIA DE ARRANQUE
// ==============================================================================================================================================
void secuenciaDeArranque() {
  for (int i = 0; i < 2; i++) {
    digitalWrite(LED1, HIGH); delay(80);
    digitalWrite(LED2, HIGH); delay(80);
    digitalWrite(LED3, HIGH); delay(80);
    digitalWrite(LED4, HIGH); delay(80);
    apagarTodosLEDs();
    delay(120);
  }
}

// ==============================================================================================================================================
// MPU6050 — Inicialización
// ==============================================================================================================================================
void inicializarMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); Wire.write(0x00); // Despertar chip
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1A); Wire.write(0x03); // DLPF_CFG=3 → 44Hz filtro pasa-bajos
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C); Wire.write(0x00); // Rango acelerómetro ±2g
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x19); Wire.write(0x07); // Sample Rate = 125Hz
  Wire.endTransmission(true);
}

// ==============================================================================================================================================
// MPU6050 — Lectura completa (para estabilización en setup)
// ==============================================================================================================================================
void leerMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);
  Wire.read(); Wire.read();                    // accelX (ignorado)
  accelY = Wire.read() << 8 | Wire.read();
  accelZ = Wire.read() << 8 | Wire.read();
}

// ==============================================================================================================================================
// MPU6050 — Lectura rápida (solo accelY y accelZ — usada en loop)
// ==============================================================================================================================================
void leerMPU6050Fast() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3D); // ACCEL_YOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 4, true);
  accelY = Wire.read() << 8 | Wire.read();
  accelZ = Wire.read() << 8 | Wire.read();
}

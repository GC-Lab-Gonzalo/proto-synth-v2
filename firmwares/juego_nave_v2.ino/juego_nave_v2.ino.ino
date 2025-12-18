
// ==============================================================================================================================================
// PROTO-SYNTH V2 - SPACE SHOOTER - GC Lab Chile
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
// Juego de disparos espacial que utiliza una pantalla OLED SH1106 para mostrar gráficos, botones y un potenciómetro para controles, 
// y el DAC para generar sonidos. Incluye funciones para manejar el menú, el juego, las colisiones, los gráficos y los sonidos, 
// con un sistema de puntuación y dificultad progresiva.
// ==============================================================================================================================================

// ==============================================================================================================================================
// FUNCIONAMIENTO
// ==============================================================================================================================================
// CONTROLES DE EXPRESIÓN:
// - Potenciómetro 1: No se usa
// - Potenciómetro 2: No se usa
// - Potenciómetro 3: Mover nave horizontalmente
// - Potenciómetro 4: No se usa
// - Botón 1: Iniciar juego y disparar 
// - Botón 2: Poder especial
// - Botón 3: Pausa y reiniciar
// - Botón 4: No se usa
// - LED 1: No se usa
// - LED 2: No se usa
// - LED 3: No se usa
// - LED 4: No se usa
// - IMU: Se reemplaza por una pantalla OLED I2C SH1106
// - LDR: No se usa
// - Micrófono: No se usa
// - Header 1: No se usa
// - Header 2: No se usa
// - Salida MIDI: No se usa
//
// MODO DE USO:
// 1. 
// MODO DE USO:.
// 1. Conecta el Proto-synth V2 y observa la pantalla OLED para verificar el inicio del juego.
// 2. Usa el potenciómetro 3 para mover la nave horizontalmente.
// 3. Presiona el botón 1 para iniciar el juego y disparar.
// 4. Presiona el botón 2 para activar el poder especial (cuando esté disponible).
// 5. Presiona el botón 3 para pausar o reiniciar el juego.
//
// INFORMACIÓN DEL CODIGO:
// - Se remplaza el uso del IMU por una pantalla OLED SH1106 para mostrar gráficos del juego.
// - Es necesaria la librería U8g2 para controlar la pantalla OLED.
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
#include <Arduino.h>    // Librería básica de Arduino (funciones principales)
#include <U8g2lib.h>    // Librería para controlar pantallas OLED y LCD
#include <Wire.h>       // Librería para comunicación I2C (pantalla OLED)

// ==============================================================================================================================================
// DEFINICIÓN DE PINES - ADAPTADO PARA PROTO-SYNTH V2
// ==============================================================================================================================================
#define POTENTIOMETER_PIN 12  // Potenciómetro 3 del Proto-synth (movimiento horizontal)
#define START_FIRE_BTN_PIN 18 // Botón 1 del Proto-synth (iniciar/disparar)
#define SPECIAL_BTN_PIN 4     // Botón 2 del Proto-synth (poder especial)
#define PAUSE_RESET_BTN_PIN 15 // Botón 3 del Proto-synth (pausa/reiniciar)
#define DAC_PIN 25            // DAC del Proto-synth (salida de audio)

// ==============================================================================================================================================
// CONFIGURACIÓN DE LA PANTALLA OLED
// ==============================================================================================================================================
// Crea objeto para controlar pantalla SH1106 de 128x64 píxeles via I2C
// U8G2_R0 = sin rotación, U8X8_PIN_NONE = sin pin de reset
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// ==============================================================================================================================================
// CONSTANTES DEL JUEGO (valores que no cambian)
// ==============================================================================================================================================
#define SCREEN_WIDTH 128      // Ancho de la pantalla en píxeles
#define SCREEN_HEIGHT 64      // Alto de la pantalla en píxeles
#define PLAYER_WIDTH 11       // Ancho de la nave del jugador
#define PLAYER_HEIGHT 7       // Alto de la nave del jugador
#define ENEMY_WIDTH 8         // Ancho de las naves enemigas
#define ENEMY_HEIGHT 6        // Alto de las naves enemigas
#define BULLET_SIZE 2         // Tamaño de las balas (cuadrado de 2x2)
#define MAX_ENEMIES 10        // Máximo número de enemigos en pantalla
#define MAX_BULLETS 12        // Máximo número de balas en pantalla
#define MAX_STARS 30          // Máximo número de estrellas de fondo
#define FIRE_RATE 100         // Tiempo entre disparos (100ms = disparo rápido)
#define SPECIAL_COOLDOWN 3000 // Tiempo de espera para poder especial (3 segundos)

// ==============================================================================================================================================
// ENUMERACIÓN - ESTADOS DEL JUEGO
// ==============================================================================================================================================
// Enum define los diferentes estados que puede tener el juego
enum GameState {
  MENU,       // Pantalla de menú principal
  PLAYING,    // Jugando activamente
  PAUSED,     // Juego pausado
  GAME_OVER   // Pantalla de fin de juego
};

// ==============================================================================================================================================
// ESTRUCTURAS DE DATOS - PLANTILLAS PARA OBJETOS DEL JUEGO
// ==============================================================================================================================================

// Estructura para las estrellas del fondo (efecto visual)
struct Star {
  int x;          // Posición horizontal de la estrella
  int y;          // Posición vertical de la estrella
  int speed;      // Velocidad de movimiento hacia abajo
  int brightness; // Brillo de la estrella (0-3)
};

// Estructura para las naves (jugador y enemigos)
struct Ship {
  float x;            // Posición horizontal (float para movimiento suave)
  float y;            // Posición vertical
  bool active;        // Si la nave está activa (visible y funcional)
  int type;           // Tipo de nave (diferentes sprites)
  int health;         // Puntos de vida de la nave
  int explosionFrame; // Frame de animación de explosión (-1 = no explota)
};

// Estructura para las balas
struct Bullet {
  float x;        // Posición horizontal
  float y;        // Posición vertical
  float dirX;     // Dirección horizontal (-1 a 1)
  float dirY;     // Dirección vertical (-1 a 1)
  bool active;    // Si la bala está activa
  int noteIndex;  // Índice de nota musical para sonido
};

// ==============================================================================================================================================
// VARIABLES GLOBALES DEL JUEGO
// ==============================================================================================================================================

// Estado actual del juego
GameState gameState = MENU;

// Variables del jugador
int playerX;                    // Posición horizontal del jugador
int playerLives = 3;            // Vidas restantes del jugador
int score = 0;                  // Puntuación actual

// Variables de tiempo (para controlar velocidad del juego)
unsigned long lastEnemyTime = 0;        // Última vez que apareció un enemigo
unsigned long lastBulletTime = 0;       // Última vez que se disparó
unsigned long lastStarTime = 0;         // Última vez que se movieron las estrellas
unsigned long gameTime = 0;             // Tiempo total de juego
float gameSpeed = 1.0;                  // Velocidad del juego (aumenta con puntuación)

// Variables para el sistema de sonido
unsigned long noteEndTime = 0;         // Cuándo termina la nota actual
bool soundPriority = false;            // Si hay un sonido prioritario reproduciéndose

// Variables para el poder especial
unsigned long lastSpecialTime = 0;     // Última vez que se usó poder especial
bool specialAvailable = true;          // Si el poder especial está disponible
int specialSequencePos = 0;            // Posición en la secuencia de disparo especial
unsigned long lastSpecialBulletTime = 0; // Tiempo del último disparo especial
const int specialBulletDelay = 80;     // Retraso entre balas del poder especial (rápido para arpegio rápido)
const int specialBulletCount = 8;      // Número de balas en poder especial
bool specialActivated = false;         // Si el poder especial está activo

// Arrays para almacenar objetos del juego
Star stars[MAX_STARS];        // Array de estrellas de fondo
Ship enemies[MAX_ENEMIES];    // Array de naves enemigas
Bullet bullets[MAX_BULLETS];  // Array de balas

// Variables para animaciones
unsigned long lastFrameTime = 0;  // Tiempo del último frame de animación
int frameDelay = 50;               // Retraso entre frames (50ms)

// Variables para control de botones (prevenir rebotes)
bool startFireButtonPressed = false;    // Estado del botón disparar
bool specialButtonPressed = false;      // Estado del botón especial
bool pauseResetButtonPressed = false;   // Estado del botón pausa
unsigned long lastButtonTime = 0;       // Tiempo del último botón presionado
const int buttonDebounceDelay = 50;     // Retraso antirebote (50ms)

// Variables para el potenciómetro
int potValue = 0;               // Valor actual del potenciómetro (0-4095)
int lastPotValue = 0;           // Último valor del potenciómetro
const int potDeadZone = 50;     // Zona muerta (evita movimiento nervioso)

// ==============================================================================================================================================
// ESCALAS MUSICALES Y SONIDOS
// ==============================================================================================================================================

// Notas musicales en escala menor armónica (frecuencias en Hz)
const int scaleHarmonic[] = {
  262, 294, 311, 349, 392, 415, 466, 523, 587, 622, 698, 784, 830, 932, 1046
};
const int scaleSize = sizeof(scaleHarmonic) / sizeof(int); // Número de notas
const int noteDuration = 80; // Duración de cada nota en milisegundos

// Melodías del juego (índices de la escala musical)
const int victoryMelody[] = {0, 4, 7, 12, 7, 4, 0};        // Melodía de victoria
const int gameOverMelody[] = {11, 9, 7, 5, 4, 2, 0};       // Melodía de game over
const int levelUpMelody[] = {0, 2, 4, 7, 9, 11, 12};       // Melodía de subir nivel
const int specialArpeggio[] = {0, 2, 4, 7, 9, 11, 12, 14}; // Arpegio para poder especial

// ==============================================================================================================================================
// SPRITES - GRÁFICOS DE LAS NAVES (FORMATO BITMAP)
// ==============================================================================================================================================

// Nave del jugador (11x7 píxeles, pero almacenada como 8 bits por fila)
const unsigned char playerShip[] = {
  0b00010000,  // ___*____ (fila 1)
  0b00111000,  // __***___ (fila 2)
  0b01111100,  // _*****__ (fila 3)
  0b11111110,  // *******_ (fila 4)
  0b11111110,  // *******_ (fila 5)
  0b10101010,  // *_*_*_*_ (fila 6)
  0b10000010   // *_____*_ (fila 7)
};

// Nave enemiga tipo 1
const unsigned char enemyShip1[] = {
  0b10000010,  // *_____*_
  0b01000100,  // _*___*__
  0b01111100,  // _*****__
  0b10111010,  // *_***_*_
  0b11111110,  // *******_
  0b00101000   // __*_*___
};

// Nave enemiga tipo 2
const unsigned char enemyShip2[] = {
  0b00111000,  // __***___
  0b01111100,  // _*****__
  0b11111110,  // *******_
  0b10101010,  // *_*_*_*_
  0b00111000,  // __***___
  0b01000100   // _*___*__
};

// Frames de animación de explosión (3 frames)
const unsigned char explosion1[] = {
  0b00100100,  // __*__*__
  0b00011000,  // ___**___
  0b00011000,  // ___**___
  0b00011000,  // ___**___
  0b00100100   // __*__*__
};

const unsigned char explosion2[] = {
  0b01000010,  // _*____*_
  0b00111100,  // __****__
  0b01000010,  // _*____*_
  0b00111100,  // __****__
  0b01000010   // _*____*_
};

const unsigned char explosion3[] = {
  0b10000001,  // *______*
  0b01000010,  // _*____*_
  0b00111100,  // __****__
  0b01000010,  // _*____*_
  0b10000001   // *______*
};

// ==============================================================================================================================================
// DECLARACIÓN DE FUNCIONES (prototipos)
// ==============================================================================================================================================
void initStars();           // Inicializar estrellas de fondo
void moveStars();           // Mover estrellas hacia abajo
void drawStars();           // Dibujar estrellas en pantalla
void readPotentiometer();   // Leer valor del potenciómetro
void handleButtons();       // Manejar estados de los botones
void drawPlayer();          // Dibujar nave del jugador
void fireBullet();          // Crear nueva bala
void fireSpecial();         // Disparar poder especial
void updateSpecial();       // Actualizar animación del poder especial
void moveBullets();         // Mover todas las balas
void drawBullets();         // Dibujar todas las balas
void spawnEnemy();          // Crear nuevo enemigo
void moveEnemies();         // Mover todos los enemigos
void drawEnemies();         // Dibujar todos los enemigos
void checkCollisions();     // Verificar colisiones entre objetos
void playDACNote(int frequency, int duration); // Reproducir nota por DAC
void playNote(int index, int duration = noteDuration); // Reproducir nota de escala
void playMelody(const int melody[], int length, int tempo); // Reproducir melodía
void showMenuScreen();      // Mostrar pantalla de menú
void showPauseScreen();     // Mostrar pantalla de pausa
void showGameOverScreen();  // Mostrar pantalla de game over
void resetGame();          // Reiniciar el juego
void drawHUD();            // Dibujar interfaz (vidas, puntuación)
void updateGameSpeed();    // Actualizar velocidad según puntuación

// ==============================================================================================================================================
// FUNCIÓN SETUP - SE EJECUTA UNA SOLA VEZ AL INICIAR
// ==============================================================================================================================================
void setup() {
  // Inicializar comunicación serie para debug (monitor serie)
  Serial.begin(115200);
  Serial.println("Iniciando Space Shooter adaptado para Proto-synth V2");
  
  // Configurar pines como entradas con resistencia pull-up interna
  // INPUT_PULLUP significa que el pin lee HIGH cuando no se presiona
  // y LOW cuando se presiona el botón
  pinMode(START_FIRE_BTN_PIN, INPUT_PULLUP);   // Botón 1 - Disparar
  pinMode(SPECIAL_BTN_PIN, INPUT_PULLUP);      // Botón 2 - Poder especial  
  pinMode(PAUSE_RESET_BTN_PIN, INPUT_PULLUP);  // Botón 3 - Pausa
  // Nota: El potenciómetro y DAC son entradas/salidas analógicas,
  // no necesitan configuración pinMode
  
  // Inicializar la pantalla OLED
  u8g2.begin();                    // Inicializar comunicación I2C con pantalla
  u8g2.setFont(u8g2_font_6x10_tr); // Establecer fuente de texto por defecto
  
  // Inicializar generador de números aleatorios
  // Usa ruido analógico del pin A0 como semilla para mayor aleatoriedad
  randomSeed(analogRead(A0));
  
  // Establecer estado inicial del juego
  gameState = MENU;
  showMenuScreen(); // Mostrar pantalla de bienvenida
}

// ==============================================================================================================================================
// FUNCIONES DE AUDIO - REPRODUCIR SONIDOS POR DAC
// ==============================================================================================================================================

// Reproduce una nota musical específica por el DAC
void playDACNote(int frequency, int duration) {
  // Para arpegios rápidos, reducir el número de ciclos para no bloquear
  int cycleCount = frequency * duration / 1000;
  // Calcular tiempo de medio período (para onda cuadrada)
  int halfPeriod = 500000 / frequency; // en microsegundos
  
  // Para sonidos rápidos del arpegio, limitar aún más los ciclos
  int maxCycles = (duration < 60) ? 8 : 15;  // Menos ciclos para sonidos cortos
  
  // Generar onda cuadrada
  for (int i = 0; i < cycleCount && i < maxCycles; i++) {
    dacWrite(DAC_PIN, 200);           // Voltaje alto (aproximadamente 2.5V)
    delayMicroseconds(halfPeriod);     // Esperar medio período
    dacWrite(DAC_PIN, 50);            // Voltaje bajo (aproximadamente 0.6V)
    delayMicroseconds(halfPeriod);     // Esperar medio período
  }
}

// Reproduce una nota de la escala musical
void playNote(int index, int duration) {
  index = index % scaleSize;    // Asegurar que el índice esté en rango válido
  
  // NO bloquear con soundPriority para sonidos de balas - solo para melodías largas
  // soundPriority = true;         // Comentado para permitir arpegios
  
  // Reproducir nota usando solo el DAC
  playDACNote(scaleHarmonic[index], duration);
  
  // Marcar cuándo terminará la nota (pero sin bloquear otros sonidos)
  noteEndTime = millis() + duration;
}

// Reproduce una melodía completa (secuencia de notas)
void playMelody(const int melody[], int length, int tempo) {
  soundPriority = true;  // Sonido prioritario - no interrumpir
  
  // Reproducir cada nota de la melodía
  for (int i = 0; i < length; i++) {
    playDACNote(scaleHarmonic[melody[i]], tempo);  // Reproducir nota
    delay(tempo + 20);  // Pausa entre notas (tempo + pequeña pausa)
  }
  
  soundPriority = false;    // Liberar prioridad de sonido
  noteEndTime = millis();   // Actualizar tiempo de fin
}

// ==============================================================================================================================================
// FUNCIONES DE PANTALLA - MOSTRAR DIFERENTES PANTALLAS DEL JUEGO
// ==============================================================================================================================================

// Muestra la pantalla de menú principal
void showMenuScreen() {
  u8g2.clearBuffer();  // Limpiar buffer de pantalla
  
  // Título del juego con fuente grande
  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.drawStr(10, 20, "SPACE GAME");
  
  // Instrucciones con fuente pequeña
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(5, 35, "Controles:");
  u8g2.drawStr(5, 45, "Pot12: Mover");      // Potenciómetro pin 12
  u8g2.drawStr(5, 55, "Pin18: Disparar");   // Botón 1
  u8g2.drawStr(70, 45, "Pin4: Especial");   // Botón 2
  u8g2.drawStr(70, 55, "Pin15: Pausa");     // Botón 3
  
  u8g2.drawStr(15, 64, "Pin18 para empezar");
  
  u8g2.sendBuffer();  // Enviar buffer a la pantalla física
}

// Muestra la pantalla de pausa
void showPauseScreen() {
  u8g2.clearBuffer();
  
  // Título "PAUSA"
  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.drawStr(35, 25, "PAUSA");
  
  // Mostrar puntuación actual
  u8g2.setFont(u8g2_font_6x10_tr);
  char scoreStr[16];  // Array de caracteres para formar texto
  sprintf(scoreStr, "Score: %d", score);  // Convertir número a texto
  u8g2.drawStr(35, 40, scoreStr);
  
  // Instrucciones de control
  u8g2.drawStr(10, 55, "Pin18: Continuar");
  u8g2.drawStr(10, 64, "Pin15: Reiniciar");
  
  u8g2.sendBuffer();
}

// Muestra la pantalla de fin de juego
void showGameOverScreen() {
  u8g2.clearBuffer();
  
  // Título "GAME OVER"
  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.drawStr(15, 20, "GAME OVER");
  
  // Mostrar puntuación final
  u8g2.setFont(u8g2_font_6x10_tr);
  char scoreStr[16];
  sprintf(scoreStr, "Score: %d", score);
  u8g2.drawStr(25, 35, scoreStr);
  
  // Opciones para continuar
  u8g2.drawStr(5, 50, "Pin18: Jugar otra vez");
  u8g2.drawStr(5, 60, "Pin15: Menu principal");
  
  u8g2.sendBuffer();
  
  // Reproducir melodía triste de game over
  playMelody(gameOverMelody, 7, 150);
}

// ==============================================================================================================================================
// FUNCIONES DE INICIALIZACIÓN Y REINICIO
// ==============================================================================================================================================

// Reinicia todas las variables del juego a su estado inicial
void resetGame() {
  // Inicializar posición y estadísticas del jugador
  playerX = (SCREEN_WIDTH - PLAYER_WIDTH) / 2;  // Centrar jugador
  playerLives = 3;   // Vidas iniciales
  score = 0;         // Puntuación inicial
  
  // Inicializar estrellas de fondo
  initStars();
  
  // Inicializar todas las balas como inactivas
  for (int i = 0; i < MAX_BULLETS; i++) {
    bullets[i].active = false;                    // Bala inactiva
    bullets[i].noteIndex = random(scaleSize);     // Nota aleatoria para sonido
    bullets[i].dirX = 0;                          // Dirección horizontal
    bullets[i].dirY = -2;                         // Dirección vertical (hacia arriba)
  }
  
  // Inicializar todos los enemigos como inactivos
  for (int i = 0; i < MAX_ENEMIES; i++) {
    enemies[i].active = false;          // Enemigo inactivo
    enemies[i].explosionFrame = -1;     // Sin explosión
  }
  
  // Reiniciar variables de tiempo
  lastEnemyTime = millis();     // Tiempo actual
  lastFrameTime = millis();     // Para animaciones
  lastSpecialTime = 0;          // Para poder especial
  specialAvailable = true;      // Poder especial disponible
  specialActivated = false;     // Poder especial no activo
  specialSequencePos = 0;       // Posición en secuencia especial
  gameTime = 0;                 // Tiempo de juego
  gameSpeed = 1.0;             // Velocidad inicial
  
  // Reiniciar estados de botones
  startFireButtonPressed = false;
  specialButtonPressed = false;
  pauseResetButtonPressed = false;
}

// Inicializa las estrellas de fondo con posiciones y velocidades aleatorias
void initStars() {
  for (int i = 0; i < MAX_STARS; i++) {
    stars[i].x = random(SCREEN_WIDTH);    // Posición X aleatoria
    stars[i].y = random(SCREEN_HEIGHT);   // Posición Y aleatoria
    stars[i].speed = random(1, 4);        // Velocidad aleatoria (1-3)
    stars[i].brightness = random(4);      // Brillo aleatorio (0-3)
  }
}

// ==============================================================================================================================================
// FUNCIONES DE MOVIMIENTO Y DIBUJO DE ESTRELLAS
// ==============================================================================================================================================

// Mueve todas las estrellas hacia abajo
void moveStars() {
  for (int i = 0; i < MAX_STARS; i++) {
    stars[i].y += stars[i].speed;  // Mover hacia abajo según velocidad
    
    // Si la estrella sale de la pantalla por abajo, reiniciarla arriba
    if (stars[i].y >= SCREEN_HEIGHT) {
      stars[i].y = 0;                           // Volver arriba
      stars[i].x = random(SCREEN_WIDTH);        // Nueva posición X aleatoria
      stars[i].speed = random(1, 4);            // Nueva velocidad
      stars[i].brightness = random(4);          // Nuevo brillo
    }
  }
}

// Dibuja todas las estrellas en la pantalla
void drawStars() {
  for (int i = 0; i < MAX_STARS; i++) {
    // Dibujar según el brillo de la estrella
    if (stars[i].brightness == 3) {
      // Estrella muy brillante - dibujar como cuadrado 2x2
      u8g2.drawPixel(stars[i].x, stars[i].y);
      u8g2.drawPixel(stars[i].x + 1, stars[i].y);
      u8g2.drawPixel(stars[i].x, stars[i].y + 1);
      u8g2.drawPixel(stars[i].x + 1, stars[i].y + 1);
    } else if (stars[i].brightness > 0) {
      // Estrella normal - dibujar como punto único
      u8g2.drawPixel(stars[i].x, stars[i].y);
    }
    // Si brightness == 0, no dibujar nada (estrella invisible)
  }
}

// ==============================================================================================================================================
// FUNCIONES DE CONTROL - POTENCIÓMETRO Y BOTONES
// ==============================================================================================================================================

// Lee el valor del potenciómetro y actualiza la posición del jugador
void readPotentiometer() {
  potValue = analogRead(POTENTIOMETER_PIN);  // Leer valor (0-4095)
  
  // Solo actualizar si hay cambio significativo (evita movimiento nervioso)
  if (abs(potValue - lastPotValue) > potDeadZone) {
    // Mapear valor del potenciómetro (0-4095) a posición de pantalla
    playerX = map(potValue, 0, 4095, 0, SCREEN_WIDTH - PLAYER_WIDTH);
    
    // Asegurar que el jugador no salga de los bordes
    if (playerX < 0) playerX = 0;
    if (playerX > SCREEN_WIDTH - PLAYER_WIDTH) playerX = SCREEN_WIDTH - PLAYER_WIDTH;
    
    lastPotValue = potValue;  // Recordar último valor
  }
}

// Maneja el estado de todos los botones (con antirebote)
void handleButtons() {
  unsigned long currentTime = millis();
  
  // Antirebote - no procesar botones muy rápido
  if (currentTime - lastButtonTime < buttonDebounceDelay) {
    return;  // Salir sin hacer nada
  }
  
  // Leer estado actual de todos los botones
  // digitalRead devuelve LOW cuando el botón está presionado (por INPUT_PULLUP)
  bool startFireBtn = (digitalRead(START_FIRE_BTN_PIN) == LOW);
  bool specialBtn = (digitalRead(SPECIAL_BTN_PIN) == LOW);
  bool pauseResetBtn = (digitalRead(PAUSE_RESET_BTN_PIN) == LOW);
  
  // Manejar botones según el estado actual del juego
  switch (gameState) {
    case MENU:  // En el menú principal
      // Solo el botón de inicio/disparo funciona
      if (startFireBtn && !startFireButtonPressed) {
        startFireButtonPressed = true;  // Marcar botón presionado
        resetGame();                    // Inicializar juego nuevo
        gameState = PLAYING;            // Cambiar a estado jugando
        playMelody(levelUpMelody, 7, 100);  // Melodía de inicio
        lastButtonTime = currentTime;   // Actualizar tiempo antirebote
      }
      break;
      
    case PLAYING:  // Durante el juego
      // Botón de disparo
      if (startFireBtn && !startFireButtonPressed) {
        // Primera vez que se presiona - disparar inmediatamente
        startFireButtonPressed = true;
        fireBullet();
        lastBulletTime = currentTime;
        lastButtonTime = currentTime;
      } else if (startFireBtn && startFireButtonPressed) {
        // Botón mantenido presionado - disparar según fire rate
        if (currentTime - lastBulletTime > FIRE_RATE) {
          fireBullet();
          lastBulletTime = currentTime;
        }
      }
      
      // Botón de poder especial
      if (specialBtn && !specialButtonPressed && specialAvailable) {
        specialButtonPressed = true;        // Marcar presionado
        specialActivated = true;            // Activar poder especial
        specialSequencePos = 0;             // Reiniciar secuencia
        lastSpecialBulletTime = currentTime;
        specialAvailable = false;           // Marcar como no disponible
        lastSpecialTime = currentTime;      // Iniciar tiempo de enfriamiento
        lastButtonTime = currentTime;
      }
      
      // Botón de pausa
      if (pauseResetBtn && !pauseResetButtonPressed) {
        pauseResetButtonPressed = true;
        gameState = PAUSED;              // Cambiar a estado pausado
        lastButtonTime = currentTime;
      }
      break;
      
    case PAUSED:  // Juego pausado
      if (startFireBtn && !startFireButtonPressed) {
        // Continuar juego
        startFireButtonPressed = true;
        gameState = PLAYING;             // Volver a jugar
        lastButtonTime = currentTime;
      } else if (pauseResetBtn && !pauseResetButtonPressed) {
        // Volver al menú principal
        pauseResetButtonPressed = true;
        gameState = MENU;
        showMenuScreen();
        lastButtonTime = currentTime;
      }
      break;
      
    case GAME_OVER:  // Fin del juego
      if (startFireBtn && !startFireButtonPressed) {
        // Jugar otra vez
        startFireButtonPressed = true;
        resetGame();
        gameState = PLAYING;
        lastButtonTime = currentTime;
      } else if (pauseResetBtn && !pauseResetButtonPressed) {
        // Volver al menú principal
        pauseResetButtonPressed = true;
        gameState = MENU;
        showMenuScreen();
        lastButtonTime = currentTime;
      }
      break;
  }
  
  // Actualizar estados de botones (liberar cuando no están presionados)
  if (!startFireBtn) startFireButtonPressed = false;
  if (!specialBtn) specialButtonPressed = false;
  if (!pauseResetBtn) pauseResetButtonPressed = false;
  
  // Verificar si el poder especial se ha enfriado (ya puede usarse otra vez)
  if (!specialAvailable && currentTime - lastSpecialTime > SPECIAL_COOLDOWN) {
    specialAvailable = true;  // Poder especial disponible otra vez
  }
}

// ==============================================================================================================================================
// FUNCIONES DE DIBUJO DE SPRITES
// ==============================================================================================================================================

// Dibuja la nave del jugador en su posición actual
void drawPlayer() {
  // Recorrer cada fila del sprite del jugador
  for (int y = 0; y < PLAYER_HEIGHT; y++) {
    // Recorrer cada bit (columna) de la fila actual
    for (int x = 0; x < 8; x++) {
      // Verificar si el bit actual está encendido (representa un píxel)
      if (playerShip[y] & (1 << (7-x))) {
        // Dibujar píxel en la posición correcta
        // playerX + x: posición horizontal
        // SCREEN_HEIGHT - PLAYER_HEIGHT - 2 + y: posición vertical (cerca del fondo)
        u8g2.drawPixel(playerX + x + ((PLAYER_WIDTH - 8) / 2), 
                       SCREEN_HEIGHT - PLAYER_HEIGHT - 2 + y);
      }
    }
  }
}

// ==============================================================================================================================================
// FUNCIONES DE BALAS Y DISPAROS
// ==============================================================================================================================================

// Crea una nueva bala desde la posición del jugador
void fireBullet() {
  // Buscar una bala inactiva para reutilizar
  for (int i = 0; i < MAX_BULLETS; i++) {
    if (!bullets[i].active) {  // Si encontramos una bala libre
      bullets[i].active = true;  // Activar la bala
      
      // Posicionar la bala en el centro de la nave del jugador
      bullets[i].x = playerX + PLAYER_WIDTH / 2 - BULLET_SIZE / 2;
      bullets[i].y = SCREEN_HEIGHT - PLAYER_HEIGHT - 2;  // Justo encima del jugador
      
      // Establecer dirección de movimiento (hacia arriba)
      bullets[i].dirX = 0;   // Sin movimiento horizontal
      bullets[i].dirY = -2;  // Movimiento hacia arriba (negativo = arriba)
      
      // Asignar nota musical aleatoria para el sonido
      bullets[i].noteIndex = random(scaleSize);
      
      // Reproducir sonido del disparo (con duración más corta para no bloquear)
      playNote(bullets[i].noteIndex, 50);  // Sonido más corto
      break;  // Salir del bucle - solo crear una bala
    }
  }
}

// Dispara el poder especial (múltiples balas en círculo)
void fireSpecial() {
  // Verificar si aún quedan balas por disparar en la secuencia
  if (specialSequencePos < specialBulletCount) {
    // Calcular ángulo para esta bala (distribución circular)
    float angle = (2.0 * PI * specialSequencePos) / specialBulletCount;
    
    // Buscar una bala libre para el disparo especial
    for (int i = 0; i < MAX_BULLETS; i++) {
      if (!bullets[i].active) {
        bullets[i].active = true;
        
        // Posición inicial (centro del jugador)
        bullets[i].x = playerX + PLAYER_WIDTH / 2 - BULLET_SIZE / 2;
        bullets[i].y = SCREEN_HEIGHT - PLAYER_HEIGHT - 2;
        
        // Calcular dirección basada en el ángulo
        bullets[i].dirX = sin(angle) * 2.0;  // Componente horizontal
        bullets[i].dirY = -cos(angle) * 2.0; // Componente vertical
        
        // Usar notas del arpegio especial para sonido único
        bullets[i].noteIndex = specialArpeggio[specialSequencePos % 8];
        
        // Reproducir sonido del arpegio (duración más corta para que se escuche toda la secuencia)
        playNote(bullets[i].noteIndex, 100);  // Duración ajustada para el arpegio
        
        // Avanzar en la secuencia
        specialSequencePos++;
        lastSpecialBulletTime = millis();
        break;  // Solo crear una bala por llamada
      }
    }
  } else {
    // Secuencia completa - desactivar poder especial
    specialActivated = false;
  }
}

// Actualiza la animación del poder especial
void updateSpecial() {
  if (specialActivated) {
    // Verificar si es tiempo de disparar la siguiente bala
    if (millis() - lastSpecialBulletTime > specialBulletDelay) {
      fireSpecial();  // Disparar siguiente bala de la secuencia
    }
  }
}

// Mueve todas las balas activas
void moveBullets() {
  for (int i = 0; i < MAX_BULLETS; i++) {
    if (bullets[i].active) {
      // Mover bala según su dirección
      bullets[i].x += bullets[i].dirX;  // Movimiento horizontal
      bullets[i].y += bullets[i].dirY;  // Movimiento vertical
      
      // Desactivar bala si sale de la pantalla
      if (bullets[i].y < 0 || bullets[i].y > SCREEN_HEIGHT || 
          bullets[i].x < 0 || bullets[i].x > SCREEN_WIDTH) {
        bullets[i].active = false;  // Bala fuera de pantalla
      }
    }
  }
}

// Dibuja todas las balas activas
void drawBullets() {
  for (int i = 0; i < MAX_BULLETS; i++) {
    if (bullets[i].active) {
      // Dibujar bala como cuadrado pequeño
      u8g2.drawBox(bullets[i].x, bullets[i].y, BULLET_SIZE, BULLET_SIZE);
    }
  }
}

// ==============================================================================================================================================
// FUNCIONES DE ENEMIGOS
// ==============================================================================================================================================

// Crea nuevos enemigos según la velocidad del juego
void spawnEnemy() {
  unsigned long currentTime = millis();
  // Calcular retraso entre enemigos (más rápido = más enemigos)
  unsigned long enemySpawnDelay = 1000 / gameSpeed;
  
  // Verificar si es tiempo de crear un nuevo enemigo
  if (currentTime - lastEnemyTime > enemySpawnDelay) {
    // Buscar un slot libre para el enemigo
    for (int i = 0; i < MAX_ENEMIES; i++) {
      // Verificar que esté inactivo y sin explosión
      if (!enemies[i].active && enemies[i].explosionFrame == -1) {
        enemies[i].active = true;  // Activar enemigo
        
        // Posición aleatoria en la parte superior
        enemies[i].x = random(SCREEN_WIDTH - ENEMY_WIDTH);
        enemies[i].y = 0;  // Aparecer en la parte superior
        
        // Tipo aleatorio de enemigo (diferentes sprites)
        enemies[i].type = random(2);  // 0 o 1
        enemies[i].health = 1;        // Vida inicial
        
        lastEnemyTime = currentTime;  // Actualizar tiempo
        break;  // Solo crear un enemigo por vez
      }
    }
  }
}

// Mueve todos los enemigos activos
void moveEnemies() {
  for (int i = 0; i < MAX_ENEMIES; i++) {
    if (enemies[i].active) {
      // Mover hacia abajo según velocidad del juego
      enemies[i].y += 0.5 * gameSpeed;
      
      // Movimiento especial para enemigo tipo 0 (patrón serpenteante)
      if (enemies[i].type == 0) {
        enemies[i].x += sin(enemies[i].y * 0.1) * 0.5;  // Movimiento sinusoidal
      }
      
      // Desactivar enemigo si sale de la pantalla
      if (enemies[i].y > SCREEN_HEIGHT) {
        enemies[i].active = false;
      }
    }
    
    // Manejar animación de explosión
    if (enemies[i].explosionFrame >= 0 && enemies[i].explosionFrame < 3) {
      // Avanzar frame de explosión según el tiempo
      if (millis() - lastFrameTime > frameDelay) {
        enemies[i].explosionFrame++;
        if (enemies[i].explosionFrame >= 3) {
          enemies[i].explosionFrame = -1;  // Terminar explosión
        }
      }
    }
  }
}

// Dibuja todos los enemigos y explosiones
void drawEnemies() {
  for (int i = 0; i < MAX_ENEMIES; i++) {
    if (enemies[i].active) {
      // Seleccionar sprite según el tipo de enemigo
      const unsigned char* enemySprite = (enemies[i].type == 0) ? enemyShip1 : enemyShip2;
      
      // Dibujar sprite del enemigo
      for (int y = 0; y < ENEMY_HEIGHT; y++) {
        for (int x = 0; x < 8; x++) {
          if (enemySprite[y] & (1 << (7-x))) {
            u8g2.drawPixel(enemies[i].x + x, enemies[i].y + y);
          }
        }
      }
    } else if (enemies[i].explosionFrame >= 0) {
      // Dibujar animación de explosión
      const unsigned char* explosionFrame;
      
      // Seleccionar frame de explosión
      switch (enemies[i].explosionFrame) {
        case 0: explosionFrame = explosion1; break;
        case 1: explosionFrame = explosion2; break;
        case 2: explosionFrame = explosion3; break;
      }
      
      // Dibujar frame de explosión
      for (int y = 0; y < 5; y++) {
        for (int x = 0; x < 8; x++) {
          if (explosionFrame[y] & (1 << (7-x))) {
            u8g2.drawPixel(enemies[i].x + x, enemies[i].y + y);
          }
        }
      }
    }
  }
}

// ==============================================================================================================================================
// FUNCIÓN DE DETECCIÓN DE COLISIONES
// ==============================================================================================================================================

// Verifica colisiones entre todos los objetos del juego
void checkCollisions() {
  // COLISIONES BALA-ENEMIGO
  for (int i = 0; i < MAX_BULLETS; i++) {
    if (bullets[i].active) {  // Solo verificar balas activas
      for (int j = 0; j < MAX_ENEMIES; j++) {
        if (enemies[j].active) {  // Solo verificar enemigos activos
          // Verificar si los rectángulos se superponen
          if (bullets[i].x < enemies[j].x + ENEMY_WIDTH &&           // Bala no está a la derecha
              bullets[i].x + BULLET_SIZE > enemies[j].x &&           // Bala no está a la izquierda
              bullets[i].y < enemies[j].y + ENEMY_HEIGHT &&          // Bala no está debajo
              bullets[i].y + BULLET_SIZE > enemies[j].y) {           // Bala no está arriba
            
            // ¡COLISIÓN DETECTADA!
            bullets[i].active = false;  // Desactivar bala
            enemies[j].health--;        // Reducir vida del enemigo
            
            if (enemies[j].health <= 0) {
              // Enemigo destruido
              enemies[j].active = false;        // Desactivar enemigo
              enemies[j].explosionFrame = 0;    // Iniciar animación de explosión
              score += 10;                      // Aumentar puntuación
              // Sonido largo de victoria
              playNote(bullets[i].noteIndex, noteDuration * 3);
            } else {
              // Enemigo dañado pero vivo
              // Sonido corto de impacto
              playNote(bullets[i].noteIndex, noteDuration * 2);
            }
            
            break;  // Salir del bucle de enemigos (bala ya impactó)
          }
        }
      }
    }
  }
  
  // COLISIONES JUGADOR-ENEMIGO
  for (int i = 0; i < MAX_ENEMIES; i++) {
    if (enemies[i].active) {
      // Verificar colisión con el jugador
      if (playerX < enemies[i].x + ENEMY_WIDTH &&                    // Jugador no está a la derecha
          playerX + PLAYER_WIDTH > enemies[i].x &&                   // Jugador no está a la izquierda
          SCREEN_HEIGHT - PLAYER_HEIGHT - 2 < enemies[i].y + ENEMY_HEIGHT && // Jugador no está debajo
          SCREEN_HEIGHT - 2 > enemies[i].y) {                        // Jugador no está arriba
        
        // ¡COLISIÓN CON EL JUGADOR!
        enemies[i].active = false;        // Destruir enemigo
        enemies[i].explosionFrame = 0;    // Explosión del enemigo
        playerLives--;                    // Perder una vida
        
        // Sonido complejo de impacto (secuencia de notas)
        playDACNote(scaleHarmonic[0], 100);  // Nota grave
        delay(100);
        playDACNote(scaleHarmonic[3], 100);  // Nota media
        delay(100);
        playDACNote(scaleHarmonic[1], 200);  // Nota final larga
        
        // Verificar si el juego terminó
        if (playerLives <= 0) {
          gameState = GAME_OVER;    // Cambiar a pantalla de fin
          showGameOverScreen();     // Mostrar pantalla
        }
        
        break;  // Solo una colisión por frame
      }
    }
  }
}

// ==============================================================================================================================================
// FUNCIÓN DE INTERFAZ DE USUARIO (HUD)
// ==============================================================================================================================================

// Dibuja la información del juego (puntuación, vidas, poder especial)
void drawHUD() {
  u8g2.setFont(u8g2_font_4x6_tr);  // Fuente pequeña para HUD
  
  // Mostrar puntuación en la esquina superior izquierda
  char scoreStr[16];
  sprintf(scoreStr, "Score: %d", score);  // Convertir número a texto
  u8g2.drawStr(0, 6, scoreStr);
  
  // Mostrar vidas en la esquina superior derecha
  char livesStr[8];
  sprintf(livesStr, "Lives: %d", playerLives);
  u8g2.drawStr(SCREEN_WIDTH - 36, 6, livesStr);
  
  // Mostrar estado del poder especial en el centro
  if (specialAvailable) {
    u8g2.drawStr(SCREEN_WIDTH / 2 - 12, 6, "SP:OK");  // Poder disponible
  } else {
    // Mostrar tiempo restante de enfriamiento
    int cooldownRemaining = (SPECIAL_COOLDOWN - (millis() - lastSpecialTime)) / 1000 + 1;
    if (cooldownRemaining > 0) {
      char specialStr[8];
      sprintf(specialStr, "SP:%d", cooldownRemaining);  // Segundos restantes
      u8g2.drawStr(SCREEN_WIDTH / 2 - 12, 6, specialStr);
    }
  }
}

// ==============================================================================================================================================
// FUNCIÓN DE VELOCIDAD DEL JUEGO
// ==============================================================================================================================================

// Actualiza la velocidad del juego según la puntuación
void updateGameSpeed() {
  // Aumentar velocidad gradualmente con la puntuación
  gameSpeed = 1.0 + (float)score / 500.0;  // +0.002 velocidad por punto
  
  // Limitar velocidad máxima para que siga siendo jugable
  if (gameSpeed > 2.0) gameSpeed = 2.0;  // Máximo 2x velocidad inicial
}

// ==============================================================================================================================================
// FUNCIÓN PRINCIPAL LOOP - SE EJECUTA CONTINUAMENTE
// ==============================================================================================================================================
void loop() {
  unsigned long currentTime = millis();  // Tiempo actual
  
  // Manejar botones en todos los estados del juego
  handleButtons();
  
  // Ejecutar lógica según el estado actual del juego
  switch (gameState) {
    case MENU:
      // En el menú solo esperamos input de botones
      // La pantalla ya se dibujó en showMenuScreen()
      break;
      
    case PLAYING:  // ¡El juego está activo!
      // Actualizar velocidad del juego según puntuación
      updateGameSpeed();
      
      // Leer posición del potenciómetro para mover al jugador
      readPotentiometer();
      
      // Mover todos los elementos del juego
      moveStars();     // Fondo de estrellas
      moveBullets();   // Balas del jugador
      moveEnemies();   // Naves enemigas
      
      // Actualizar sistemas especiales
      updateSpecial(); // Animación del poder especial
      
      // Crear nuevos enemigos según sea necesario
      spawnEnemy();
      
      // Verificar todas las colisiones
      checkCollisions();
      
      // DIBUJAR TODO EN PANTALLA
      u8g2.clearBuffer();  // Limpiar pantalla
      drawStars();         // Dibujar fondo de estrellas
      drawBullets();       // Dibujar balas
      drawEnemies();       // Dibujar enemigos y explosiones
      drawPlayer();        // Dibujar nave del jugador
      drawHUD();           // Dibujar interfaz (puntuación, vidas)
      u8g2.sendBuffer();   // Enviar todo a la pantalla física
      
      // Actualizar contador de frames para animaciones
      if (currentTime - lastFrameTime > frameDelay) {
        lastFrameTime = currentTime;
      }
      
      break;
      
    case PAUSED:
      // Mostrar pantalla de pausa
      showPauseScreen();
      break;
      
    case GAME_OVER:
      // La pantalla de game over ya se mostró en checkCollisions()
      // Solo esperamos input de botones para continuar
      break;
  }
  
  // Limpiar prioridad de sonido si el tiempo ha pasado
  if (soundPriority && currentTime > noteEndTime) {
    soundPriority = false;  // Liberar sistema de sonido
  }
  
  // Controlar velocidad del juego (aproximadamente 50 FPS)
  delay(20);  // Esperar 20ms entre frames
}
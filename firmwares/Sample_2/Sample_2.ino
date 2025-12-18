
// ==============================================================================================================================================
// PROTO-SYNTH V2 - Sample 2 - GC Lab Chile
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
// Sampler avanzado con grabación, reproducción en bucle, control de pitch, selección de fragmento y efecto de reversa.
// ==============================================================================================================================================

// ==============================================================================================================================================
// FUNCIONAMIENTO
// ==============================================================================================================================================
// CONTROLES DE EXPRESIÓN:
// - Potenciómetro 1: Control de velocidad/tempo
// - Potenciómetro 2: Punto de inicio del sample
// - Potenciómetro 3: Punto final/duración del sample 
// - Potenciómetro 4: Control de pitch (sin cambiar tempo)
// - Botón 1: Grabar
// - Botón 2: Play/stop
// - Botón 3: Reversa (mientras presionado)
// - Botón 4: Loop toggle
// - LED 1: Grabación/VU meter
// - LED 2: Reproducción/VU meter
// - LED 3: VU meter
// - LED 4: VU meter
// - IMU: No se usa
// - LDR: No se usa
// - Micrófono: Grabadora
// - Header 1: No se usa
// - Header 2: No se usa
// - Salida MIDI: No se usa
//
// MODO DE USO:
// 1. Graba pulsando el botón 1
// 2. Reproduce/para con el botón 2
// 3. Mantén presionado el botón 3 para reproducir en reversa
// 4. Activa/desactiva el modo loop con el botón 4
// 5. Ajusta la velocidad/tempo con el potenciómetro 1
// 6. Ajusta el punto de inicio del sample con el potenciómetro 2
// 7. Ajusta el punto final/duración del sample con el potenciómetro 3
// 8. Ajusta el pitch (sin cambiar tempo) con el potenciómetro 4 
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
#include <driver/adc.h>
#include <driver/dac.h>
#include <Arduino.h>

// ==============================================================================================================================================
// CONFIGURACIÓN DE HARDWARE - PINES
// ==============================================================================================================================================
// Pines Proto-Synth V2.0
const int MIC_PIN = 33;          // Micrófono
const int AUDIO_OUT_PIN = 25;    // Salida DAC

// Botones
const int REC_BTN = 18;          // Botón grabar
const int PLAY_BTN = 4;          // Botón play/stop
const int REVERSE_BTN = 15;      // Botón reversa (mientras presionado)
const int LOOP_BTN = 19;         // Botón loop toggle

// Potenciómetros
const int SPEED_POT = 12;        // Control de velocidad/tempo
const int START_POT = 13;        // Punto de inicio del sample
const int END_POT = 14;          // Punto final/duración del sample
const int PITCH_POT = 27;        // Control de pitch (sin cambiar tempo)

// LEDs para VU meter y estados
const int LED1 = 23;
const int LED2 = 32;
const int LED3 = 2;
const int LED4 = 5;


// ==============================================================================================================================================
// PROGRAMA
// ==============================================================================================================================================
// Configuración de audio
const int SAMPLE_RATE = 11000;   // 11kHz para ~5 segundos
const int MAX_SAMPLES = 55000;   // ~5 segundos a 11kHz (110KB)
const int VU_UPDATE_MS = 50;     // Actualización VU meter

// Buffer de audio en PSRAM si está disponible, sino en heap
uint16_t* audioBuffer;
volatile int totalSamples = 0;
volatile int playIndex = 0;
volatile bool hasRecording = false;
volatile bool isPlaying = false;
volatile bool isRecording = false;
volatile bool loopMode = false;
volatile bool reverseMode = false;

// Variables de fragmento
int startPoint = 0;
int endPoint = 0;
int currentDirection = 1; // 1 = normal, -1 = reversa
float pitchMultiplier = 1.0;
float samplePosition = 0.0; // Posición con decimales para pitch shifting

// VU Meter
unsigned long lastVuUpdate = 0;
int vuLevel = 0;

// Debounce
unsigned long lastButtonTime = 0;
const int DEBOUNCE_MS = 50;

void updateVuMeter(uint16_t audioLevel) {
    // Convertir nivel de audio a escala 0-4 para LEDs
    vuLevel = map(audioLevel, 0, 4095, 0, 4);
    
    // Mostrar VU meter en LEDs
    digitalWrite(LED1, vuLevel > 0 ? HIGH : LOW);
    digitalWrite(LED2, vuLevel > 1 ? HIGH : LOW);
    digitalWrite(LED3, vuLevel > 2 ? HIGH : LOW);
    digitalWrite(LED4, vuLevel > 3 ? HIGH : LOW);
}

void showRecording() {
    // Animación de grabación - parpadeo rítmico
    static unsigned long lastBlink = 0;
    static bool blinkState = false;
    
    if (millis() - lastBlink > 200) {
        blinkState = !blinkState;
        lastBlink = millis();
        
        digitalWrite(LED1, blinkState);
        digitalWrite(LED2, !blinkState);
        digitalWrite(LED3, blinkState);
        digitalWrite(LED4, !blinkState);
    }
}

void calculateFragment() {
    if (!hasRecording) return;
    
    // Leer potenciómetros
    int startPot = analogRead(START_POT);
    int endPot = analogRead(END_POT);
    
    // Mapear start point (0% al 49% del sample)
    startPoint = map(startPot, 0, 4095, 0, totalSamples * 49 / 100);
    
    // Mapear end point (50% al 100% del sample, pero desde startPoint)
    int minEnd = startPoint + (totalSamples * 5 / 100); // Mínimo 5% de duración
    endPoint = map(endPot, 0, 4095, minEnd, totalSamples - 1);
    
    // Asegurar que endPoint > startPoint
    if (endPoint <= startPoint) {
        endPoint = startPoint + 100; // Mínimo 100 samples
    }
}

void startRecording() {
    isRecording = true;
    isPlaying = false;
    totalSamples = 0;
    hasRecording = false;
    
    Serial.println("Iniciando grabación...");
}

void stopRecording() {
    isRecording = false;
    hasRecording = (totalSamples > 0);
    
    Serial.print("Grabación terminada. Samples: ");
    Serial.println(totalSamples);
}

void togglePlayback() {
    if (!hasRecording) return;
    
    isPlaying = !isPlaying;
    
    if (isPlaying) {
        calculateFragment();
        samplePosition = (float)startPoint;
        playIndex = startPoint;
        currentDirection = 1;
        Serial.println("Reproducción iniciada");
    } else {
        Serial.println("Reproducción detenida");
    }
}

void processAudio() {
    if (isRecording) {
        // Grabar audio
        if (totalSamples < MAX_SAMPLES) {
            uint16_t sample = analogRead(MIC_PIN);
            audioBuffer[totalSamples++] = sample;
            
            // Mostrar nivel en VU meter cada cierto tiempo
            if (millis() - lastVuUpdate > VU_UPDATE_MS) {
                updateVuMeter(sample);
                lastVuUpdate = millis();
            }
            
            // Sampling delay para 11kHz
            delayMicroseconds(90);
        } else {
            stopRecording();
        }
        return;
    }
    
    if (isPlaying && hasRecording) {
        // Reproducir audio
        calculateFragment(); // Recalcular en tiempo real
        
        // Leer potenciómetros
        int speedPot = analogRead(SPEED_POT);
        int pitchPot = analogRead(PITCH_POT);
        
        // Control de velocidad/tempo (10-500 microsegundos)
        int playbackDelay = map(speedPot, 0, 4095, 10, 500);
        
        // Control de pitch (0.25x a 4x)
        pitchMultiplier = map(pitchPot, 0, 4095, 25, 400) / 100.0;
        
        // Determinar dirección
        currentDirection = (digitalRead(REVERSE_BTN) == LOW) ? -1 : 1;
        
        // Obtener sample actual con interpolación para pitch shifting
        int currentSample = (int)samplePosition;
        if (currentSample >= startPoint && currentSample <= endPoint) {
            uint16_t sample;
            
            // Interpolación lineal para suavizar el pitch shifting
            if (currentSample < endPoint) {
                float fraction = samplePosition - currentSample;
                uint16_t sample1 = audioBuffer[currentSample];
                uint16_t sample2 = audioBuffer[currentSample + 1];
                sample = sample1 + (uint16_t)(fraction * (sample2 - sample1));
            } else {
                sample = audioBuffer[currentSample];
            }
            
            dac_output_voltage(DAC_CHANNEL_1, sample >> 4);
            
            // Mostrar nivel en VU meter
            if (millis() - lastVuUpdate > VU_UPDATE_MS) {
                updateVuMeter(sample);
                lastVuUpdate = millis();
            }
            
            // Avanzar posición con pitch multiplier
            float increment = pitchMultiplier * currentDirection;
            samplePosition += increment;
            playIndex = (int)samplePosition;
            
            // Control de límites y loop
            if (currentDirection > 0 && samplePosition > endPoint) {
                if (loopMode) {
                    samplePosition = (float)startPoint; // Reiniciar loop
                    playIndex = startPoint;
                } else {
                    isPlaying = false; // Parar reproducción
                }
            } else if (currentDirection < 0 && samplePosition < startPoint) {
                if (loopMode) {
                    samplePosition = (float)endPoint; // Reiniciar loop reverso
                    playIndex = endPoint;
                } else {
                    isPlaying = false; // Parar reproducción
                }
            }
        }
        
        delayMicroseconds(playbackDelay);
    }
    
    // Si no hay actividad de audio, mantener VU meter apagado
    if (!isRecording && !isPlaying) {
        if (millis() - lastVuUpdate > VU_UPDATE_MS) {
            updateVuMeter(0);
            lastVuUpdate = millis();
        }
    }
}

void handleButtons() {
    unsigned long currentTime = millis();
    
    // Debounce básico
    if (currentTime - lastButtonTime < DEBOUNCE_MS) return;
    
    // Botón grabar
    static bool lastRecState = HIGH;
    bool recState = digitalRead(REC_BTN);
    if (recState == LOW && lastRecState == HIGH) {
        if (!isRecording) {
            startRecording();
        } else {
            stopRecording();
        }
        lastButtonTime = currentTime;
    }
    lastRecState = recState;
    
    // Botón play/stop
    static bool lastPlayState = HIGH;
    bool playState = digitalRead(PLAY_BTN);
    if (playState == LOW && lastPlayState == HIGH) {
        togglePlayback();
        lastButtonTime = currentTime;
    }
    lastPlayState = playState;
    
    // Botón loop
    static bool lastLoopState = HIGH;
    bool loopState = digitalRead(LOOP_BTN);
    if (loopState == LOW && lastLoopState == HIGH) {
        loopMode = !loopMode;
        Serial.print("Loop mode: ");
        Serial.println(loopMode ? "ON" : "OFF");
        lastButtonTime = currentTime;
    }
    lastLoopState = loopState;
}

void setup() {
    Serial.begin(115200);
    Serial.println("Advanced 5-Second Sampler v1.0");
    
    // Allocar buffer de audio en heap
    audioBuffer = (uint16_t*)malloc(MAX_SAMPLES * sizeof(uint16_t));
    if (!audioBuffer) {
        Serial.println("ERROR: No se pudo allocar memoria para audio buffer!");
        while(1) delay(1000);
    }
    Serial.println("Buffer de audio allocado correctamente");
    
    // Configurar pines
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    pinMode(LED3, OUTPUT);
    pinMode(LED4, OUTPUT);
    
    pinMode(REC_BTN, INPUT_PULLUP);
    pinMode(PLAY_BTN, INPUT_PULLUP);
    pinMode(REVERSE_BTN, INPUT_PULLUP);
    pinMode(LOOP_BTN, INPUT_PULLUP);
    
    // Configurar ADC
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    analogSetWidth(12);
    
    // Configurar DAC
    dac_output_enable(DAC_CHANNEL_1);
    dac_output_voltage(DAC_CHANNEL_1, 128); // Valor neutro
    
    // Test inicial de LEDs
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED1, HIGH);
        digitalWrite(LED2, HIGH);
        digitalWrite(LED3, HIGH);
        digitalWrite(LED4, HIGH);
        delay(200);
        digitalWrite(LED1, LOW);
        digitalWrite(LED2, LOW);
        digitalWrite(LED3, LOW);
        digitalWrite(LED4, LOW);
        delay(200);
    }
    
    Serial.println("Sistema listo!");
    Serial.println("REC: Grabar/Parar grabación");
    Serial.println("PLAY: Reproducir/Parar");
    Serial.println("REVERSE: Mantener para reproducir en reversa");
    Serial.println("LOOP: Toggle modo loop");
    Serial.println("SPEED POT (12): Control de velocidad/tempo");
    Serial.println("PITCH POT (27): Control de pitch (0.25x - 4x)");
    Serial.println("START POT (13): Punto de inicio");
    Serial.println("END POT (14): Punto final");
}

void loop() {
    handleButtons();
    
    // Mostrar estado de grabación
    if (isRecording) {
        showRecording();
    }
    
    processAudio();
    
    // Pequeña pausa para estabilidad
    delayMicroseconds(10);
}
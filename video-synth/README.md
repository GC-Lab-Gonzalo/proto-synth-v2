# Video Synth — GC Lab Chile

El Proto-Synth v2 convertido en instrumento audiovisual. Hay dos modos:

- **Video Synth v2** (`video_synth_v2/`) — cargás un video y lo tocás como instrumento: el audio del video se loopea y el pitch, BPM y efectos se controlan con los potenciómetros y botones en tiempo real.
- **Video Synth Live** (`video_synth_live/`) — la cámara web y el micrófono son la fuente. El sonido del ambiente transforma la imagen en vivo.

---

## Estructura de carpetas

```
video-synth/
├── README.md                          ← esta guía
├── codigo_proto-synth/
│   └── video_synth_control/
│       └── video_synth_control.ino   ← firmware para el ESP32
├── video_synth_v2/
│   └── video_synth_v2.py             ← app de video desde archivo
└── video_synth_live/
    └── video_synth_live.py           ← app de cámara en tiempo real
```

---

## Requisitos

### Hardware
- Proto-Synth v2 (ESP32 DevKit v1)
- Cable USB para conectar el ESP32 al PC
- Computador con cámara web y micrófono (solo para Video Synth Live)

### Software
- **Python 3.10 o superior** — [python.org/downloads](https://www.python.org/downloads/)
- **Arduino IDE** — para cargar el firmware al ESP32
- **FFmpeg** — necesario para extraer el audio del video (solo Video Synth v2)

---

## 1. Instalar Python y librerías

Abrí una terminal (CMD o PowerShell en Windows) y ejecutá:

```bash
pip install opencv-python numpy pygame sounddevice soundfile pyserial imageio-ffmpeg
```

> En algunos sistemas puede ser `pip3` en lugar de `pip`.

### Verificar que quedó todo instalado

```bash
python -c "import cv2, numpy, pygame, sounddevice, soundfile, serial; print('OK')"
```

Si imprime `OK`, está todo listo.

---

## 2. Instalar FFmpeg (solo para Video Synth v2)

FFmpeg extrae el audio del video. Sin él, el Video Synth v2 funciona sin audio.

**Windows:**
1. Descargá FFmpeg desde [ffmpeg.org/download.html](https://ffmpeg.org/download.html) (versión "essentials" de gyan.dev)
2. Descomprimí la carpeta, por ejemplo en `C:\ffmpeg`
3. Agregá `C:\ffmpeg\bin` al PATH del sistema
4. Verificá en terminal: `ffmpeg -version`

**Mac (con Homebrew):**
```bash
brew install ffmpeg
```

**Linux (Ubuntu/Debian):**
```bash
sudo apt install ffmpeg
```

---

## 3. Cargar el firmware en el ESP32

El firmware `video_synth_control.ino` convierte el Proto-Synth en un controlador que envía el estado de los potenciómetros y botones al PC por USB.

1. Abrí Arduino IDE
2. Abrí el archivo `codigo_proto-synth/video_synth_control/video_synth_control.ino`
3. **Girá el Potenciómetro 3 al máximo** antes de conectar (pin de strapping del ESP32)
4. Seleccioná la placa: `ESP32 Dev Module`
5. Seleccioná el puerto COM correspondiente al ESP32
6. Hacé clic en **Subir** (flecha →)
7. Cuando termine, los 4 LEDs van a hacer un barrido de izquierda a derecha: eso confirma que el firmware cargó bien

> El firmware envía datos al PC a 50 Hz por el puerto serial a 115200 baudios. No necesitás tener abierto el Monitor Serie para usarlo.

---

## 4. Conectar el Proto-Synth al PC

1. Conectá el Proto-Synth al PC por USB
2. El script detecta el ESP32 automáticamente buscando puertos con chipset CP2102, CH340 o similar
3. Si tenés varios puertos y no lo detecta, podés especificarlo manualmente al ejecutar (ver sección 5 y 6)

**Ver qué puerto es en Windows:** Administrador de dispositivos → Puertos (COM & LPT) → buscar "Silicon Labs" o "CH340"

---

## 5. Ejecutar Video Synth v2 (video desde archivo)

```bash
python video_synth_v2/video_synth_v2.py
```

Al ejecutar, se abre un selector de archivos para elegir el video (`.mp4`, `.mov`, `.avi`, `.mkv`, etc.). Si el ESP32 está conectado, lo detecta solo.

**Forma alternativa pasando argumentos:**
```bash
python video_synth_v2/video_synth_v2.py mi_video.mp4
python video_synth_v2/video_synth_v2.py mi_video.mp4 COM5
```

La aplicación arranca en **pantalla completa**. Presioná `SPACE` para comenzar la reproducción.

### Controles — Video Synth v2

#### Botones

| Botón | Función |
|-------|---------|
| BTN1 | Play / Pause |
| BTN2 (pulsación corta) | Ciclar preset de batería (0=apagada → 1–10) |
| BTN2 (mantener) + POT1 | Controlar volumen de la batería |
| BTN3 | Ciclar modo FX: Normal → FX Audio → FX Video → Normal |
| BTN4 | Siguiente escala musical |

> El indicador de modo FX aparece en la esquina inferior izquierda de la pantalla (punto de color). Gris = Normal, Naranja = FX Audio, Azul = FX Video.

#### Potenciómetros — Modo Normal (punto gris)

| Pot | Función audio | Función video |
|-----|--------------|---------------|
| POT1 | Posición del sample (qué parte del video se toca) | Mueve el frame mostrado en pantalla |
| POT2 | Pitch cuantizado a la escala musical | Zoom + temperatura de color (grave=frío/azul, agudo=cálido/rojo) |
| POT3 | BPM (60–180) | — |
| POT4 | Volumen | Brillo y saturación de la imagen |

#### Potenciómetros — Modo FX Audio (punto naranja)

| Pot | Función audio | Función video |
|-----|--------------|---------------|
| POT1 | Filtro paso-bajos: frecuencia de corte (20 Hz – 20 kHz) | Efecto comic/bordes (filtro cerrado = más contornos) |
| POT2 | Resonancia del filtro (Q 0.5 – 8) | Aberración cromática (split de canales RGB) |
| POT3 | Reverb: tamaño de sala (decay) | Eco visual de frames |
| POT4 | Reverb: mezcla wet/dry | Distorsión de píxeles ondulante |

#### Potenciómetros — Modo FX Video (punto azul)

| Pot | Función audio | Función video |
|-----|--------------|---------------|
| POT1 | Bit crushing (fragmentación del sonido) | Tiles: divide la imagen en 1×1 hasta 4×4 repeticiones |
| POT2 | Ring modulation (timbre metálico/robótico) | Hue shift (giro de colores) |
| POT3 | Tremolo (pulso de amplitud 3–13 Hz) | Ghost / visión doble |
| POT4 | Flanger (barrido de fase) | Feedback espiral (rotación acumulada) |

#### Teclado (sin Proto-Synth)

| Tecla | Función |
|-------|---------|
| SPACE | Play / Pause |
| D | Ciclar preset de batería |
| 1–9 | Seleccionar preset de batería directamente |
| L | Cambiar largo del sample (0.5 / 1 / 2 / 4 beats) |
| S | Siguiente escala musical |
| E | Ciclar modo FX |
| H | Mostrar/ocultar ayuda en pantalla |
| Q / ESC | Salir |

---

## 6. Ejecutar Video Synth Live (cámara en tiempo real)

```bash
python video_synth_live/video_synth_live.py
```

La aplicación abre la cámara web (índice 0 por defecto) y el micrófono. La señal del micrófono transforma la imagen en tiempo real.

**Especificar puerto manualmente:**
```bash
python video_synth_live/video_synth_live.py COM5
```

> Si el micrófono tiene poca ganancia, acercate o hablá más fuerte. La ganancia está fija en 2.5× para funcionar bien con la mayoría de micrófonos sin ajuste.

### Controles — Video Synth Live

#### Botones

| Botón | Función |
|-------|---------|
| BTN1 | Silenciar / activar audio al altavoz |
| BTN3 | Ciclar modo FX: Normal → FX Audio → FX Video → Normal |
| BTN4 | Ciclar paleta de color (Natural, Térmica, Neón, Fosforescente, Monocromático) |

#### Potenciómetros — Modo Normal

| Pot | Función audio | Función video |
|-----|--------------|---------------|
| POT1 | Delay time: eco corto → largo (0–1.5 s) | Eco visual de frames (sincronizado con el delay de audio) |
| POT2 | Delay feedback: cantidad de repeticiones | Trail de movimiento (los trazos persisten en la imagen) |
| POT3 | Zoom + mezcla del delay | Zoom de la cámara |
| POT4 | Volumen de salida | Brillo y saturación de la imagen |

#### Potenciómetros — Modo FX Audio (punto naranja)

| Pot | Función audio | Función video |
|-----|--------------|---------------|
| POT1 | Filtro SVF resonante (cutoff 80 Hz – 10 kHz) | Desenfoque gaussiano (filtro cerrado = más borroso) |
| POT2 | Ring modulation: frecuencia portadora (voz robótica) | Efecto neón / bordes brillantes |
| POT3 | Chorus / vibrato (profundidad del LFO de pitch) | Shimmer VHS (desplazamiento de canales, efecto cinta vieja) |
| POT4 | Pitch shift ±1 octava (centro = sin cambio) | Temperatura de color (agudo=rojo/cálido, grave=azul/frío) |

> **POT4 en FX Audio:** la posición del centro (12 en punto) no cambia el pitch. Moverlo a la izquierda baja el tono, a la derecha lo sube.

#### Potenciómetros — Modo FX Video (punto azul)

| Pot | Función audio | Función video |
|-----|--------------|---------------|
| POT1 | Reverb Schroeder (reverberación tipo sala grande) | Feedback espiral (rota y acumula, se acelera con el audio) |
| POT2 | Harmonizer (agrega octava arriba, 30% mezcla) | Kaleidoscopio (simetría cuádruple → octal) |
| POT3 | Bit crushing (reduce profundidad de bits, 16→3) | Pixelación reactiva (el tamaño de pixel sube con el volumen) |
| POT4 | Flanger (delay corto modulado por LFO) | Warp ondulante (deformación sinusoidal, velocidad reactiva al audio) |

#### Efectos automáticos reactivos al audio (siempre activos)

Estos efectos no tienen control manual, reaccionan directamente al micrófono:

| Señal | Efecto visual |
|-------|--------------|
| Amplitud general | Zoom pulsante + flash de ataque |
| Bajos (20–200 Hz) | Tinte cálido (rojo/naranja) |
| Agudos (2–8 kHz) | Aberración cromática + tinte frío (azul) |
| Medios (200–2000 Hz) | Posterización (la imagen se aplana en bandas de color) |

#### Teclado (sin Proto-Synth)

| Tecla | Función |
|-------|---------|
| SPACE | Silenciar / activar audio |
| E | Ciclar modo FX |
| P | Ciclar paleta de color |
| H | Mostrar/ocultar ayuda en pantalla |
| Q / ESC | Salir |

---

## 7. Protocolo serial (para desarrollo)

El firmware envía una línea CSV cada 20 ms (50 Hz) por el puerto serial a **115200 baudios**:

```
pot1,pot2,pot3,pot4,btn1,btn2,btn3,btn4,imu_x\n
```

- `pot1–pot4`: valores ADC de 12 bits (0–4095)
- `btn1–btn4`: estado del botón (1 = presionado, 0 = suelto)
- `imu_x`: inclinación del IMU en eje X (0 = izquierda, 2048 = nivelado, 4095 = derecha) — usado en Video Synth Live para mezclar dos cámaras

El PC puede enviar de vuelta comandos para controlar los LEDs:

```
L:1,0,1,0\n
```

Donde cada valor (0 o 1) enciende o apaga el LED correspondiente.

---

## Solución de problemas

**El ESP32 no se detecta:**
Especificá el puerto manualmente: `python video_synth_v2.py mi_video.mp4 COM5` (Windows) o `python video_synth_v2.py mi_video.mp4 /dev/ttyUSB0` (Linux/Mac).

**No hay audio en Video Synth v2:**
Verificá que FFmpeg esté instalado y en el PATH. Probá en terminal: `ffmpeg -version`.

**La pantalla se ve lenta / bajo FPS:**
El Video Synth v2 carga el video en RAM. Con videos largos en alta resolución puede ser lento. Recomendado: videos de menos de 5 minutos a 1080p o menos.

**Error al cargar el firmware: "too many arguments to timerBegin":**
Estás usando Arduino Core v3.x. El firmware `video_synth_control.ino` ya es compatible con v3.x.

**La cámara no abre en Video Synth Live:**
En Windows se prueban automáticamente los backends MSMF y DirectShow. Si tenés más de una cámara, la segunda se detecta como segunda fuente y podés mezclarlas inclinando el Proto-Synth (IMU).

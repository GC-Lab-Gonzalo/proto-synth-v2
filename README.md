# 🎹 Proto-Synth v2 (ESP32 Open Source Synthesizer)

**Proto-Synth v2** es una plataforma de desarrollo de audio y síntesis experimental creada por **GC Lab Chile**. Basada en el ESP32, está diseñada para la educación en electrónica, programación de audio (DSP) y música electrónica DIY.

![Proto-Synth v2 Pinout](images/Pinout.png)


## 🚀 Características
El Proto-Synth v2 agrupa componentes esenciales para la síntesis de sonido en una sola PCB compacta:

* **Core:** ESP32 DevKit v1 (Dual Core, 240MHz).
* **Audio Out:** Salida Monofónica DAC (8-bit hardware) por Jack 3.5mm (Pin 25).
* **Sensores:**
    * **MPU6050:** Acelerómetro y Giroscopio para control gestual.
    * **LDR (Fotorresistencia):** Control por luz (Theremin óptico).
    * **Micrófono:** Entrada de audio básica para análisis o efectos.
* **Interfaz:**
    * 4x Potenciómetros.
    * 4x Botones (Configuración Pull-up).
    * 4x LEDs indicadores.
* **Conectividad:** MIDI Out (TX0) y pines de expansión.

## 📂 Contenido del Repositorio

* `/firmware`: Colección de **20 códigos de ejemplo** que incluyen:
    * Osciladores básicos y síntesis aditiva.
    * Samplers y loopers.
    * Secuenciadores de pasos.
    * Filtros controlados por luz (LDR).
    * Controladores MIDI vía giroscopio (MPU6050).
* `/hardware`:
    * **Gerbers:** Archivos listos para fabricación de PCB.
    * **Schematics:** Diagramas electrónicos.

## ⚠️ Erratas y Notas Técnicas (Hardware v2.0)
Si estás usando la placa v2.0 o planeas fabricarla tal cual está en los Gerbers, ten en cuenta estas "features" no intencionales:

1.  **Inversión de Potenciómetros:** El footprint en el PCB quedó invertido. Físicamente, girar a la derecha reduce la resistencia.
    * *Fix de Software:* Invertir la lectura: `int lectura = 4095 - analogRead(PIN_POT);`
2.  **Conflicto WiFi / ADC2:** Los 4 potenciómetros están conectados a pines que pertenecen al ADC2 del ESP32.
    * *Nota:* **No se pueden usar los potenciómetros simultáneamente con la librería WiFi.** Al activar WiFi, el ADC2 se deshabilita para uso de usuario.
3.  **Modo de Carga (Upload Issue):** El **Potenciómetro 3** está conectado al GPIO 12 (un pin de 'strapping' que determina el voltaje del flash interno al arrancar).
    * *Instrucción:* Para subir código exitosamente, asegúrate de que el **Potenciómetro 3 esté girado al máximo**.
4.  **Diodos/Puentes:** El esquemático original incluía diodos de protección que causaban caída de voltaje. En las placas ensambladas, estos han sido reemplazados por puentes de soldadura.

## 🛠️ Cómo empezar

### Requisitos
* Arduino IDE (con el board manager de ESP32 instalado).
* Librerías recomendadas: `Mozzi` (para audio avanzado), `Adafruit_MPU6050`.

### Flujo de Trabajo con IA
Recomendamos usar el **Pinout** incluido en la carpeta `/images` para generar código con asistentes de IA (ChatGPT, Claude, etc).
1. Sube la imagen del pinout a la IA.
2. Pide la funcionalidad deseada mapeando los pines visualmente.
3. ¡Compila y sube!

## 🤝 Contribuciones y Licencia
Este proyecto es **Open Source**.
Te invitamos a:
1.  Bajar los Gerbers y fabricar tu propia placa.
2.  Modificar el diseño en EasyEDA/KiCad para corregir las erratas (¡Pull requests bienvenidos!).
3.  Aportar nuevos códigos de sintetizadores a la carpeta de ejemplos.

Desarrollado con ❤️ por **GC Lab Chile**.

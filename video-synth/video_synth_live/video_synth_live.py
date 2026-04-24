#!/usr/bin/env python3
# Video Synth Live — GC Lab Chile
# Licencia: MIT License — https://opensource.org/licenses/MIT
#
# Cámara web y micrófono en tiempo real.
# El sonido transforma la imagen en vivo, frame a frame.
#
# CONTROLES HARDWARE (ESP32):
#   BTN1       → Silenciar / activar paso de audio al altavoz
#   BTN3       → Ciclar modo FX: Normal → FX Audio → FX Video → Normal
#   BTN4       → Ciclar paleta de color
#
# Modo Normal:
#   POT1 → Delay time (audio: eco corto → largo | video: eco visual de frames)
#   POT2 → Delay feedback (audio: repeticiones del eco | video: trail de movimiento)
#   POT3 → Zoom + delay mix (video: zoom cámara | audio: mezcla seco/wet del delay)
#   POT4 → Volumen (audio: volumen salida | video: brillo + saturación)
#   [ganancia del micrófono fija, óptima para la mayoría de micrófonos]
#
# Modo FX Audio (BTN3 ×1):
#   POT1 → Filtro SVF resonante (sweep cutoff 80 Hz – 10 kHz, Q alto)
#   POT2 → Ring modulation (frecuencia portadora → voz robótica/metálica)
#   POT3 → Chorus / vibrato (profundidad del LFO de pitch)
#   POT4 → Pitch shift (centro=normal, izq=grave, der=agudo; ±1 octava)
#
# Modo FX Video (BTN3 ×2):
#   POT1 → Feedback espiral (rotación acumulada, se acelera con el audio)
#   POT2 → Kaleidoscopio (0=apagado → simetría cuádruple → simetría octal)
#   POT3 → Pixelación reactiva (el tamaño de pixel sube con la amplitud)
#   POT4 → Warp ondulante (deformación sinusoidal, velocidad reactiva)
#
# Efectos siempre activos (reactivos al audio):
#   Amplitud  → zoom pulsante + flash de ataque (onset)
#   Bajos     → tinte cálido (rojo/naranja)
#   Agudos    → aberración cromática + tinte frío (azul)
#   Medios    → posterización (la imagen se aplana en bandas de color)
#
# Teclado:
#   SPACE=silenciar audio  E=ciclar FX  P=paleta  H=ayuda  Q/ESC=salir

import collections
import math
import sys
import threading
import time

import cv2
import numpy as np
import pygame
import sounddevice as sd
import serial
import serial.tools.list_ports

# ── Constantes ────────────────────────────────────────────────────────────────

SR         = 44100
BLOCK      = 512
ANAL_N     = 2048          # ventana FFT en muestras (~46 ms)
DELAY_MAX  = int(SR * 1.5) # 1.5 s de delay máximo
GHOST_HIST = 30            # frames guardados para eco visual

C_MODO_FX  = [(180, 180, 180), (255, 160, 20), (80, 140, 255)]
NOMBRES_FX = ["NORMAL", "FX AUDIO", "FX VIDEO"]

PALETAS = ["Natural", "Térmica", "Neón", "Fosforescente", "Monocromático"]

AFX_NEUTRO = [4095, 0, 0, 2048]  # filtro abierto, sin ring, sin chorus, pitch neutro
VFX_NEUTRO = [0, 0, 0, 0]


# ── Estado compartido ─────────────────────────────────────────────────────────

class Estado:
    """Lectura de pots/botones del ESP32 y estado global de la UI."""

    def __init__(self):
        self._l         = threading.Lock()
        self.pot        = [2048, 2048, 2048, 2048]
        self.btn        = [0, 0, 0, 0]
        self._prev_btn  = [0, 0, 0, 0]

        self.audio_on   = True
        self.modo_fx    = 0        # 0=normal 1=fx_audio 2=fx_video
        self.paleta_idx = 0
        self.imu_x      = 2048    # 0=cam0, 2048=mezcla, 4095=cam1

        # Valores efectivos por modo (soft-takeover al cambiar de modo)
        self.play_eff   = [2048, 2048, 2048, 2048]
        self._pl_enter  = [2048]*4;  self._pl_armed  = [True]*4

        self.afx_eff    = list(AFX_NEUTRO)
        self._afx_enter = [2048]*4;  self._afx_armed = [False]*4

        self.vfx_eff    = list(VFX_NEUTRO)
        self._vfx_enter = [2048]*4;  self._vfx_armed = [False]*4

        self.serial_ok  = False
        self.fps        = 0.0

    def update_serial(self, line: str):
        try:
            v = [int(x) for x in line.strip().split(',')]
        except ValueError:
            return
        if len(v) < 8:
            return
        with self._l:
            b, prev   = v[4:], self._prev_btn[:]
            self.pot  = v[:4]
            self.btn  = b[:]
            self._prev_btn = b[:]

            if b[0] and not prev[0]:
                self.audio_on = not self.audio_on
            if b[2] and not prev[2]:
                self._cambiar_fx((self.modo_fx + 1) % 3, v[:4])
            if b[3] and not prev[3]:
                self.paleta_idx = (self.paleta_idx + 1) % len(PALETAS)

            if len(v) >= 9:
                self.imu_x = max(0, min(4095, v[8]))

            self._actualizar_eff(v)

    def _cambiar_fx(self, nuevo: int, pots: list):
        self.modo_fx = nuevo
        if nuevo == 0:
            self._pl_enter  = pots[:]; self._pl_armed  = [False]*4
        elif nuevo == 1:
            self._afx_enter = pots[:]; self._afx_armed = [False]*4
        elif nuevo == 2:
            self._vfx_enter = pots[:]; self._vfx_armed = [False]*4

    def _actualizar_eff(self, v: list):
        def soft(vals, enter, armed):
            for i in range(4):
                if not armed[i] and abs(v[i] - enter[i]) > 80:
                    armed[i] = True
                if armed[i]:
                    vals[i] = v[i]
        if self.modo_fx == 0:
            soft(self.play_eff, self._pl_enter, self._pl_armed)
        elif self.modo_fx == 1:
            soft(self.afx_eff, self._afx_enter, self._afx_armed)
        elif self.modo_fx == 2:
            soft(self.vfx_eff, self._vfx_enter, self._vfx_armed)

    def snap(self) -> dict:
        with self._l:
            return {
                'play_eff':   self.play_eff[:],
                'afx_eff':    self.afx_eff[:],
                'vfx_eff':    self.vfx_eff[:],
                'audio_on':   self.audio_on,
                'modo_fx':    self.modo_fx,
                'paleta_idx': self.paleta_idx,
                'fps':        self.fps,
                'imu_x':      self.imu_x,
            }

    def toggle_audio(self):
        with self._l: self.audio_on = not self.audio_on

    def ciclar_fx(self):
        with self._l:
            nuevo = (self.modo_fx + 1) % 3
            self._cambiar_fx(nuevo, self.pot[:])

    def next_paleta(self):
        with self._l:
            self.paleta_idx = (self.paleta_idx + 1) % len(PALETAS)


# ── Análisis de audio ─────────────────────────────────────────────────────────

class AudioAnalisis:
    """Métricas espectrales del micrófono para conducir los efectos de video."""

    def __init__(self):
        self._l          = threading.Lock()
        self.rms         = 0.0
        self.bass        = 0.0   # energía 20–200 Hz
        self.mid         = 0.0   # energía 200–2000 Hz
        self.treble      = 0.0   # energía 2000–8000 Hz
        self.onset       = False # True cuando hay un ataque de sonido
        self._rms_smooth = 0.0

    def actualizar(self, buf: np.ndarray):
        """Llamar desde el hilo principal con el buffer circular del micrófono."""
        n = len(buf)
        if n < 64:
            return

        win = buf[-min(n, ANAL_N):] * np.hanning(min(n, ANAL_N)).astype(np.float32)
        rms = float(np.sqrt(np.mean(win**2)))

        spec         = np.abs(np.fft.rfft(win))
        hz           = SR / len(win)   # Hz por bin
        b_bass       = spec[max(1, int(20/hz))   : max(2, int(200/hz))]
        b_mid        = spec[max(1, int(200/hz))  : max(2, int(2000/hz))]
        b_treble     = spec[max(1, int(2000/hz)) : max(2, int(8000/hz))]
        norm         = max(1e-9, spec[1:].max())

        bass   = float(np.mean(b_bass))   / norm if len(b_bass)   else 0.0
        mid    = float(np.mean(b_mid))    / norm if len(b_mid)    else 0.0
        treble = float(np.mean(b_treble)) / norm if len(b_treble) else 0.0

        # Onset: la energía sube abruptamente por encima del nivel suavizado
        onset = (rms > self._rms_smooth * 1.7) and (rms > 0.015)
        self._rms_smooth = self._rms_smooth * 0.88 + rms * 0.12

        with self._l:
            alpha = 0.30
            self.rms    = self.rms    * (1-alpha) + rms    * alpha
            self.bass   = self.bass   * (1-alpha) + bass   * alpha
            self.mid    = self.mid    * (1-alpha) + mid    * alpha
            self.treble = self.treble * (1-alpha) + treble * alpha
            self.onset  = onset

    def snap(self) -> dict:
        with self._l:
            return {
                'rms':    self.rms,
                'bass':   self.bass,
                'mid':    self.mid,
                'treble': self.treble,
                'onset':  self.onset,
            }


# ── Motor de audio ────────────────────────────────────────────────────────────

class AudioEngine:
    """
    Stream dúplex (micrófono → DSP → altavoz).

    Cadena FX Audio (POT1-4 en modo FX Audio):
      1. Filtro SVF resonante  — cutoff sweep + resonancia alta tipo sintetizador
      2. Ring modulation       — voz robótica/metálica (multiplica por oscilador)
      3. Chorus / Vibrato      — LFO modula el pitch (pitch wobble)
      4. Pitch shift granular  — sube o baja el tono real de la voz

    El buffer de análisis expone la señal amplificada para los efectos de video.
    """

    GRAIN   = 1024          # tamaño de grano del pitch shifter (~23 ms)
    CH_LEN  = int(SR * 0.06)  # buffer del chorus (60 ms)

    def __init__(self, estado: Estado):
        self.estado = estado

        # Buffer circular para análisis espectral (hilo principal)
        self._anal_buf = np.zeros(ANAL_N, np.float32)
        self._anal_wr  = 0
        self._anal_l   = threading.Lock()

        # ── SVF (State Variable Filter) ───────────────────────────────────────
        # Mucho más estable que biquad a Q altos; produce LP, BP y HP limpios
        self._svf_low  = 0.0
        self._svf_band = 0.0

        # ── Ring modulation ───────────────────────────────────────────────────
        self._ring_ph = 0.0

        # ── Chorus / vibrato ──────────────────────────────────────────────────
        self._ch_buf = np.zeros(self.CH_LEN, np.float32)
        self._ch_wr  = 0
        self._lfo_ph = 0.0

        # ── Pitch shifter granular ────────────────────────────────────────────
        # Dos granos con ventana Hanning desfasados en GRAIN/2; se alternan
        # para producir salida continua sin clics en el punto de reset.
        ps_n = self.GRAIN * 8
        self._ps_buf  = np.zeros(ps_n, np.float32)
        self._ps_n    = ps_n
        self._ps_wr   = 0
        # Ventana Hanning precalculada (evita recomputar en cada callback)
        self._hann    = np.array(
            [0.5 * (1.0 - math.cos(2 * math.pi * k / self.GRAIN))
             for k in range(self.GRAIN)], dtype=np.float32)
        # Dos cabezas de lectura: posición fraccionaria + contador de grano
        self._ps_rd   = [float(ps_n - self.GRAIN * 2),
                         float(ps_n - self.GRAIN * 2 + self.GRAIN // 2)]
        self._ps_gc   = [0, self.GRAIN // 2]

        # ── Delay (controlado desde modo Normal) ──────────────────────────────
        self._dly_buf = np.zeros(DELAY_MAX + 2, np.float32)
        self._dly_wr  = 0

        # ── FX Video audio ────────────────────────────────────────────────────
        # POT1 → Reverb Schroeder (4 comb paralelos + 2 allpass en serie)
        _rv_cd = [1557, 1617, 1491, 1422]
        self._rv_cd = _rv_cd
        self._rv_cb = [np.zeros(d + 2, np.float32) for d in _rv_cd]
        self._rv_cw = [0] * 4
        _rv_ad = [225, 556]
        self._rv_ad = _rv_ad
        self._rv_ab = [np.zeros(d + 2, np.float32) for d in _rv_ad]
        self._rv_aw = [0] * 2

        # POT2 → Harmonizer: dos cabezas extra en el buffer del pitch shifter
        # leen a ratio 2.0 (octava arriba) mezclado suavemente
        self._vh_rd = [float(ps_n // 2),
                       float(ps_n // 2 + self.GRAIN // 2)]
        self._vh_gc = [0, self.GRAIN // 2]

        # POT4 → Flanger: delay corto modulado por LFO (~1-22 ms)
        _fl_n = int(0.022 * SR) + 4
        self._fl_buf = np.zeros(_fl_n, np.float32)
        self._fl_n   = _fl_n
        self._fl_wr  = 0
        self._fl_ph  = 0.0

    # ── Interfaz pública ──────────────────────────────────────────────────────

    def get_anal_buf(self) -> np.ndarray:
        """Retorna los últimos ANAL_N samples ordenados cronológicamente."""
        with self._anal_l:
            return np.roll(self._anal_buf.copy(), -self._anal_wr)

    def _push_anal(self, samples: np.ndarray):
        with self._anal_l:
            for x in samples:
                self._anal_buf[self._anal_wr] = x
                self._anal_wr = (self._anal_wr + 1) % ANAL_N

    # ── Callback de audio ─────────────────────────────────────────────────────

    def callback(self, indata: np.ndarray, outdata: np.ndarray,
                 frames: int, ti, st):
        s  = self.estado.snap()
        ep = s['afx_eff']

        # Canal izquierdo del micrófono como señal mono
        mic  = indata[:, 0].copy().astype(np.float32)

        # Ganancia fija: 2.5× + clip suave — funciona bien con la mayoría de micrófonos
        mic  = np.tanh(mic * 2.5)

        # Exponer al análisis visual
        self._push_anal(mic)

        sig = mic.copy()

        # ── 1. SVF Resonante (POT1 FX Audio = cutoff) ────────────────────────
        # Curva cuadrática: los primeros 2/3 del pot barre 80–2000 Hz (zona vocal),
        # el último tercio llega hasta 10 kHz (apertura brillante).
        cn      = ep[0] / 4095.0
        fc      = 80.0 + cn**1.8 * 9920.0          # 80 Hz … 10 kHz
        fc      = min(fc, SR * 0.46)
        Q       = 3.0 + cn * (8.0 - 3.0)           # Q 3…8: más agudo = más resonante
        f_c     = 2.0 * math.sin(math.pi * fc / SR)
        q_c     = 1.0 / Q

        lo = self._svf_low; ba = self._svf_band
        out_svf = np.empty(frames, np.float32)
        for i in range(frames):
            hi      = sig[i] - lo - q_c * ba
            ba     += f_c * hi
            lo     += f_c * ba
            # Mezcla LP+BP: gravan → LP oscuro; agudos → BP más resonante
            mix_bp  = min(1.0, cn * 1.6)            # 0=todo LP, 1=todo BP
            out_svf[i] = lo * (1.0 - mix_bp) + ba * mix_bp
        self._svf_low = lo; self._svf_band = ba

        if not np.isfinite(out_svf).all():
            out_svf = np.zeros(frames, np.float32)
            self._svf_low = self._svf_band = 0.0

        sig = np.clip(out_svf, -1.0, 1.0)

        # ── 2. Ring Modulation (POT2 FX Audio = freq portadora) ──────────────
        # Multiplica la voz por un oscilador sinusoidal → sonido robótico/metálico.
        # Se mezcla con la señal seca para conservar algo de inteligibilidad.
        rn = ep[1] / 4095.0
        if rn > 0.01:
            freq = 80.0 + rn**1.4 * 1120.0         # 80 Hz … 1200 Hz
            inc  = 2.0 * math.pi * freq / SR
            ph   = self._ring_ph
            t_vec = ph + inc * np.arange(frames, dtype=np.float32)
            carrier = np.sin(t_vec).astype(np.float32)
            self._ring_ph = float((ph + inc * frames) % (2.0 * math.pi))
            wet = min(0.85, rn * 1.0)
            # Mezclar ring-mod con seco para mantener algo de voz
            sig = sig * (1.0 - wet * 0.6) + sig * carrier * (wet * 0.6)
            sig = np.tanh(sig)                      # clip suave post-ring

        # ── 3. Chorus / Vibrato (POT3 FX Audio = profundidad LFO) ────────────
        # LFO sinusoidal modula el tiempo de delay → pitch wobble continuo.
        # Depth bajo → chorus sutil; depth alto → vibrato profundo.
        dn      = ep[2] / 4095.0
        ch_base = int(SR * 0.018)                   # delay base 18 ms
        ch_dep  = int(dn * SR * 0.014)              # ±14 ms de modulación máx
        lfo_hz  = 0.35 + dn * 1.8                  # 0.35 … 2.15 Hz
        inc_lfo = 2.0 * math.pi * lfo_hz / SR

        buf_ch = self._ch_buf; blen_ch = self.CH_LEN; wr_ch = self._ch_wr
        lp     = self._lfo_ph
        ch_out = np.empty(frames, np.float32)
        for i in range(frames):
            buf_ch[wr_ch] = sig[i]
            d  = ch_base + int(ch_dep * math.sin(lp))
            d  = max(1, min(d, blen_ch - 1))
            ch_out[i] = buf_ch[(wr_ch - d) % blen_ch]
            wr_ch = (wr_ch + 1) % blen_ch
            lp   += inc_lfo
        self._ch_wr  = wr_ch
        self._lfo_ph = float(lp % (2.0 * math.pi))

        if dn > 0.01:
            wet_ch = min(0.9, dn * 1.05)
            sig = sig * (1.0 - wet_ch) + ch_out * wet_ch

        # ── 4. Pitch Shift granular (POT4 FX Audio = ratio, centro = 1.0) ────
        # Dos granos Hanning desfasados se alternan para cubrir la salida sin gaps.
        # Rango: pot=0 → ratio=0.5 (–1 oct) | pot=2048 → ratio=1.0 | pot=4095 → ratio=2.0 (+1 oct)
        pn    = ep[3] / 4095.0
        ratio = 2.0 ** ((pn - 0.5) * 2.0)          # 0.5 … 2.0

        if abs(ratio - 1.0) > 0.03:
            G    = self.GRAIN; hann = self._hann
            ps_b = self._ps_buf; ps_n = self._ps_n; ps_wr = self._ps_wr
            rd   = self._ps_rd; gc   = self._ps_gc
            ps_out = np.zeros(frames, np.float32)

            for i in range(frames):
                # Escribir muestra en el buffer circular
                ps_b[ps_wr] = sig[i]
                ps_wr = (ps_wr + 1) % ps_n

                y = 0.0
                for h in range(2):
                    # Reiniciar grano cuando se completó
                    if gc[h] >= G:
                        rd[h]  = float((ps_wr - G * 2 + ps_n) % ps_n)
                        gc[h]  = 0

                    # Lectura interpolada
                    ri   = int(rd[h]) % ps_n
                    frac = rd[h] - int(rd[h])
                    s_r  = ps_b[ri] * (1.0 - frac) + ps_b[(ri + 1) % ps_n] * frac
                    y   += s_r * hann[gc[h]]

                    rd[h] = (rd[h] + ratio) % ps_n
                    gc[h] += 1

                ps_out[i] = y

            self._ps_wr = ps_wr; self._ps_rd = rd; self._ps_gc = gc
            # Mezcla 55% pitch + 45% seco: el pitch se oye claro, la voz se entiende
            sig = np.clip(sig * 0.45 + ps_out * 0.55, -1.0, 1.0)
        else:
            # Mantener el buffer activo aunque no se procese
            ps_b = self._ps_buf; ps_n = self._ps_n; ps_wr = self._ps_wr
            for i in range(frames):
                ps_b[ps_wr] = sig[i]; ps_wr = (ps_wr + 1) % ps_n
            self._ps_wr = ps_wr

        # ── Delay (POT1=time, POT2=feedback, POT3=mix — modo Normal) ─────────
        # El delay se aplica sobre la señal ya procesada por todos los FX.
        pp       = s['play_eff']
        dly_t    = pp[0] / 4095.0                  # 0..1 → 0..1.5 s
        dly_fb   = pp[1] / 4095.0 * 0.82           # máx 82% feedback
        dly_mx   = pp[2] / 4095.0 * 0.75           # máx 75% wet (voz sigue entendiéndose)
        d_smp    = max(1, int(dly_t * DELAY_MAX))
        buf_d    = self._dly_buf; blen_d = len(buf_d); wr_d = self._dly_wr
        dly_out  = np.empty(frames, np.float32)
        for i in range(frames):
            echo       = buf_d[(wr_d - d_smp) % blen_d]
            wet_sig    = sig[i] + echo * dly_fb
            dly_out[i] = sig[i] * (1.0 - dly_mx) + wet_sig * dly_mx
            buf_d[wr_d] = np.clip(wet_sig, -1.0, 1.0)
            wr_d = (wr_d + 1) % blen_d
        self._dly_wr = wr_d
        sig = np.clip(dly_out, -1.0, 1.0)

        # ── FX Video → Audio (POT1-4 modo FX Video) ──────────────────────────
        vp = s['vfx_eff']

        # POT1 (feedback espiral) → Reverb Schroeder: reverberaciones infinitas
        rv_n = vp[0] / 4095.0
        if rv_n > 0.01:
            decay = 0.55 + rv_n * 0.38     # 0.55..0.93
            wet   = rv_n * 0.42
            comb_out = np.zeros(frames, np.float32)
            for k in range(4):
                buf = self._rv_cb[k]; d = self._rv_cd[k]
                wr = self._rv_cw[k]; bl = len(buf)
                ok = np.empty(frames, np.float32)
                for i in range(frames):
                    rd = (wr - d) % bl
                    ok[i] = buf[rd]
                    buf[wr] = sig[i] + ok[i] * decay
                    wr = (wr + 1) % bl
                self._rv_cw[k] = wr
                comb_out += ok
            comb_out *= 0.25
            ap = comb_out
            for k in range(2):
                buf = self._rv_ab[k]; d = self._rv_ad[k]
                wr = self._rv_aw[k]; bl = len(buf); g = 0.5
                ok = np.empty(frames, np.float32)
                for i in range(frames):
                    rd = (wr - d) % bl
                    bd = buf[rd]; v = ap[i] + g * bd
                    ok[i] = bd - g * v; buf[wr] = v
                    wr = (wr + 1) % bl
                self._rv_aw[k] = wr
                ap = ok
            sig = sig * (1.0 - wet) + np.clip(ap, -1.0, 1.0) * wet

        # POT2 (kaleidoscopio) → Harmonizer: octava arriba mezclada al 30 %
        # Lee el mismo buffer del pitch shifter con dos cabezas a ratio 2.0
        harm_n = vp[1] / 4095.0
        if harm_n > 0.01:
            G = self.GRAIN; hann = self._hann
            ps_b = self._ps_buf; ps_n = self._ps_n; ps_wr = self._ps_wr
            rd = self._vh_rd; gc = self._vh_gc
            harm_out = np.zeros(frames, np.float32)
            for i in range(frames):
                y = 0.0
                for h in range(2):
                    if gc[h] >= G:
                        rd[h] = float((ps_wr - G * 2 + ps_n) % ps_n)
                        gc[h] = 0
                    ri = int(rd[h]) % ps_n
                    frac = rd[h] - int(rd[h])
                    s_r = ps_b[ri] * (1.0 - frac) + ps_b[(ri+1) % ps_n] * frac
                    y += s_r * hann[gc[h]]
                    rd[h] = (rd[h] + 2.0) % ps_n   # ratio 2.0 = octava arriba
                    gc[h] += 1
                harm_out[i] = y
            self._vh_rd = rd; self._vh_gc = gc
            wet = min(0.32, harm_n * 0.36)
            sig = np.clip(sig * (1.0 - wet) + harm_out * wet, -1.0, 1.0)

        # POT3 (pixelación) → Bitcrusher: reduce profundidad de bits
        # Pixelación visual = menor resolución; bitcrusher = menor resolución de audio
        pix_n = vp[2] / 4095.0
        if pix_n > 0.01:
            bits   = max(3, int(16 - pix_n * 13))   # 16 bits → 3 bits
            levels = float(1 << bits)
            sig    = np.round(sig * levels) / levels

        # POT4 (warp ondulante) → Flanger: delay corto con LFO
        warp_n = vp[3] / 4095.0
        if warp_n > 0.01:
            max_d  = max(2, int(warp_n * self._fl_n * 0.85))
            lfo_hz = 0.3 + warp_n * 2.7
            inc_fl = 2.0 * math.pi * lfo_hz / SR
            buf_fl = self._fl_buf; blen_fl = self._fl_n; wr_fl = self._fl_wr
            fl_ph  = self._fl_ph
            fl_out = np.empty(frames, np.float32)
            for i in range(frames):
                lv = math.sin(fl_ph); fl_ph += inc_fl
                d  = max(1, min(int(max_d * (0.5 + 0.5 * lv)), blen_fl - 1))
                fl_out[i] = sig[i] + buf_fl[(wr_fl - d) % blen_fl] * warp_n * 0.7
                buf_fl[wr_fl] = sig[i]
                wr_fl = (wr_fl + 1) % blen_fl
            self._fl_wr = wr_fl
            self._fl_ph = float(fl_ph % (2.0 * math.pi))
            sig = np.clip(fl_out, -1.0, 1.0)
        else:
            buf_fl = self._fl_buf; blen_fl = self._fl_n; wr_fl = self._fl_wr
            for i in range(frames):
                buf_fl[wr_fl] = sig[i]; wr_fl = (wr_fl + 1) % blen_fl
            self._fl_wr = wr_fl

        # ── Volumen y mute ────────────────────────────────────────────────────
        vol = s['play_eff'][3] / 4095.0
        out = np.clip(sig * vol, -1.0, 1.0) if s['audio_on'] else np.zeros(frames, np.float32)
        outdata[:, 0] = out
        outdata[:, 1] = out


# ── Captura de cámara ─────────────────────────────────────────────────────────

class CamaraCaptura:
    """Captura frames de la webcam en un hilo separado para no bloquear el loop."""

    def __init__(self, idx: int = 0, w: int = 640, h: int = 360):
        self._cap     = self._abrir(idx, w, h)
        self._frame:  np.ndarray | None = None
        self._l       = threading.Lock()
        self.ok       = self._cap is not None and self._cap.isOpened()
        self._running = True

        if self.ok:
            threading.Thread(target=self._loop, daemon=True).start()

    @staticmethod
    def _abrir(idx: int, w: int, h: int):
        """
        Prueba backends en orden hasta encontrar uno que funcione.
        En Windows: MSMF (cámara integrada) → DSHOW → sin backend.
        En otros: sin backend directamente.
        """
        if sys.platform == 'win32':
            candidatos = [
                (idx, cv2.CAP_MSMF),   # Microsoft Media Foundation — cámaras integradas
                (idx, cv2.CAP_DSHOW),  # DirectShow — cámaras USB externas
                (idx, cv2.CAP_ANY),    # dejar que OpenCV decida
            ]
            # Si el índice pedido falla, probar también el 1
            extras = [(1, cv2.CAP_MSMF), (1, cv2.CAP_DSHOW)] if idx == 0 else []
            candidatos += extras
        else:
            candidatos = [(idx, cv2.CAP_ANY)]

        for cam_idx, backend in candidatos:
            try:
                cap = cv2.VideoCapture(cam_idx, backend)
                if cap.isOpened():
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  w)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
                    cap.set(cv2.CAP_PROP_FPS, 30)
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    # Verificar que realmente devuelve frames
                    ret, _ = cap.read()
                    if ret:
                        print(f"Cámara abierta: índice={cam_idx} backend={backend}")
                        return cap
                    cap.release()
            except Exception:
                pass

        print("ERROR: No se encontró ninguna cámara disponible.")
        return None

    def _loop(self):
        while self._running and self._cap is not None:
            ret, frame = self._cap.read()
            if ret:
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                with self._l:
                    self._frame = rgb
            else:
                time.sleep(0.01)

    def get_frame(self) -> np.ndarray | None:
        with self._l:
            return self._frame.copy() if self._frame is not None else None

    def close(self):
        self._running = False
        if self._cap is not None:
            self._cap.release()


# ── Efectos de video ──────────────────────────────────────────────────────────

def _aplicar_paleta(frame: np.ndarray, idx: int) -> np.ndarray:
    if idx == 0:   # Natural
        return frame
    if idx == 1:   # Térmica — apariencia de cámara infrarroja
        g = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        return cv2.cvtColor(cv2.applyColorMap(g, cv2.COLORMAP_JET), cv2.COLOR_BGR2RGB)
    if idx == 2:   # Neón — tono brillante arco iris
        g = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        return cv2.cvtColor(cv2.applyColorMap(g, cv2.COLORMAP_HSV), cv2.COLOR_BGR2RGB)
    if idx == 3:   # Fosforescente — verde sobre negro
        g = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        out = np.zeros_like(frame)
        out[:, :, 1] = g   # solo canal verde
        return out
    # Monocromático
    g = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    return cv2.cvtColor(g, cv2.COLOR_GRAY2RGB)


def _kaleidoscopio(frame: np.ndarray, nivel: float) -> np.ndarray:
    """
    nivel 0.0 – 0.5 → simetría cuádruple (espejo H + V)
    nivel 0.5 – 1.0 → mezcla adicional con simetría diagonal (aspecto de mandala)
    """
    if nivel < 0.02:
        return frame
    h, w = frame.shape[:2]
    hh, hw = h // 2, w // 2

    q     = frame[:hh, :hw].copy()
    top   = np.concatenate([q, cv2.flip(q, 1)], axis=1)
    kali  = np.concatenate([top, cv2.flip(top, 0)], axis=0)
    if kali.shape[:2] != (h, w):
        kali = cv2.resize(kali, (w, h))

    if nivel > 0.5:
        alpha_diag = (nivel - 0.5) * 2.0
        diag = cv2.transpose(kali)
        if diag.shape[:2] != (h, w):
            diag = cv2.resize(diag, (w, h))
        kali = cv2.addWeighted(kali, 1.0 - alpha_diag * 0.5,
                               diag,  alpha_diag * 0.5, 0).astype(np.uint8)

    # Fundido suave al inicio para evitar corte abrupto
    if nivel < 0.12:
        alpha = nivel / 0.12
        kali  = cv2.addWeighted(frame, 1.0 - alpha, kali, alpha, 0)

    return kali.astype(np.uint8)


# ── Sintetizador principal ────────────────────────────────────────────────────

class VideoSynthLive:

    def __init__(self, puerto: str | None = None):
        self.puerto    = puerto
        self.estado    = Estado()
        self.analisis  = AudioAnalisis()

        self._engine:  AudioEngine | None    = None
        self._stream:  sd.Stream   | None    = None
        self._camara:  CamaraCaptura | None  = None
        self._camara2: CamaraCaptura | None  = None
        self._ser:     serial.Serial | None  = None
        self._running  = True

        self._frame_hist = collections.deque(maxlen=GHOST_HIST)
        self._prev_out:  np.ndarray | None   = None
        self._warp_t     = 0.0   # fase acumulada para el warp sinusoidal
        self._onset_ttl  = 0     # frames restantes del flash de onset

    # ── Serial ────────────────────────────────────────────────────────────────

    def _thread_serial(self):
        buf = ''
        while self._running and self._ser:
            try:
                if self._ser.in_waiting:
                    buf += self._ser.read(self._ser.in_waiting).decode('utf-8', errors='ignore')
                    while '\n' in buf:
                        line, buf = buf.split('\n', 1)
                        line = line.strip()
                        if line and not line.startswith('='):
                            self.estado.update_serial(line)
                time.sleep(0.02)
            except Exception:
                self.estado.serial_ok = False
                break

    # ── Pipeline de efectos de video ──────────────────────────────────────────

    def _aplicar_vfx(self, frame: np.ndarray, s: dict, a: dict) -> np.ndarray:
        pp  = s['play_eff']
        ep  = s['afx_eff']
        vp  = s['vfx_eff']
        rms    = a['rms']
        bass   = a['bass']
        mid    = a['mid']
        treble = a['treble']

        # ── FASE 1: Geométrico — zoom reactivo ───────────────────────────────
        # POT3 modo normal define el zoom base; la amplitud añade un pulso extra
        zoom_base  = 0.85 + (pp[2] / 4095.0) * 0.45   # 0.85..1.30
        zoom_audio = 1.0  + rms * 0.30                 # +0..+30 % según volumen
        zoom = zoom_base * zoom_audio
        if abs(zoom - 1.0) > 0.01:
            hf, wf = frame.shape[:2]
            nh = max(4, int(hf / zoom)); nw = max(4, int(wf / zoom))
            y0 = max(0, (hf - nh) // 2);  x0 = max(0, (wf - nw) // 2)
            nh = min(nh, hf - y0);         nw = min(nw, wf - x0)
            frame = cv2.resize(frame[y0:y0+nh, x0:x0+nw],
                               (wf, hf), interpolation=cv2.INTER_LINEAR)

        # ── FASE 2: Trail de movimiento (POT2 = delay feedback) ─────────────
        # El frame anterior se mezcla sobre el actual: cuanto más feedback,
        # más persisten los trazos de movimiento (efecto fantasma suave).
        trail_n = pp[1] / 4095.0
        if trail_n > 0.02 and self._prev_out is not None and self._prev_out.shape == frame.shape:
            # Curva cuadrática: el trail sube lento y nunca supera 0.55
            # → siempre hay al menos 45 % de imagen fresca visible
            alpha = min(0.55, trail_n ** 1.6 * 0.60)
            frame = cv2.addWeighted(frame, 1.0 - alpha, self._prev_out, alpha, 0)
            frame = np.clip(frame, 0, 255).astype(np.uint8)

        # Tinte cálido (rojo) proporcional a los bajos,
        # tinte frío (azul) proporcional a los agudos
        bass_t   = min(70, int(bass   * 100))
        treble_t = min(70, int(treble * 100))
        if bass_t > 3 or treble_t > 3:
            f = frame.astype(np.int16)
            f[:, :, 0] = np.clip(f[:, :, 0] + bass_t,   0, 255)
            f[:, :, 2] = np.clip(f[:, :, 2] + treble_t, 0, 255)
            frame = f.astype(np.uint8)

        # Aberración cromática: los agudos separan canales R / B lateralmente
        if treble > 0.04:
            shift = max(1, int(treble * 22))
            r = np.roll(frame[:, :, 0],  shift, axis=1)
            b = np.roll(frame[:, :, 2], -shift, axis=1)
            frame = np.stack([r, frame[:, :, 1], b], axis=-1).astype(np.uint8)

        # Posterización por medios: la imagen se aplana en bandas de color
        if mid > 0.06:
            niveles = max(2, int(10 - mid * 8))   # 10 niveles (suave) → 2 (muy aplanado)
            frame   = (frame // (256 // niveles) * (256 // niveles)).astype(np.uint8)

        # ── FASE 3: Onset — flash de ataque ───────────────────────────────────
        if a['onset']:
            self._onset_ttl = 4   # durar 4 frames
        if self._onset_ttl > 0:
            intensidad = int(55 * self._onset_ttl / 4)
            frame      = np.clip(frame.astype(np.int16) + intensidad, 0, 255).astype(np.uint8)
            self._onset_ttl -= 1

        # ── FASE 4: Paleta de color ───────────────────────────────────────────
        frame = _aplicar_paleta(frame, s['paleta_idx'])

        # Brillo + saturación por volumen de salida (POT4 modo normal)
        # Volumen bajo → imagen oscura y desaturada; alto → brillante e hipersaturada
        vol_n  = pp[3] / 4095.0
        bright = 0.15 + vol_n * 1.05   # 0.15 (casi negro) → 1.20 (sobrexpuesto)
        sat    = 0.15 + vol_n * 1.70   # 0.15 (escala de grises) → 1.85 (neón)
        if not (0.97 < bright < 1.03) or not (0.97 < sat < 1.03):
            hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV).astype(np.float32)
            hsv[:, :, 1] = np.clip(hsv[:, :, 1] * sat,    0, 255)
            hsv[:, :, 2] = np.clip(hsv[:, :, 2] * bright, 0, 255)
            frame = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2RGB)

        # ── FASE 5: FX Audio → Video ──────────────────────────────────────────

        # Cutoff bajo → desenfoque gaussiano (filtro suaviza la imagen)
        cutoff_n = ep[0] / 4095.0
        if cutoff_n < 0.75:
            k = max(1, int((1.0 - cutoff_n) * 17))
            k = k if k % 2 == 1 else k + 1
            frame = cv2.GaussianBlur(frame, (k, k), 0)

        # Distorsión alta → detección de bordes + fusión con imagen (efecto neon)
        drive_n = ep[1] / 4095.0
        if drive_n > 0.04:
            gray  = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            thr_l = max(8,  int(70 * (1.0 - drive_n)))
            thr_h = max(16, int(140 * (1.0 - drive_n)))
            edges = cv2.Canny(gray, thr_l, thr_h)
            edges_rgb = cv2.cvtColor(edges, cv2.COLOR_GRAY2RGB)
            frame = cv2.addWeighted(frame, 1.0, edges_rgb, drive_n * 2.2, 0)
            frame = np.clip(frame, 0, 255).astype(np.uint8)

        # Chorus → shimmer VHS: los canales RGB se desplazan ligeramente en H
        # cuanto más chorus, más "cinta magnética vieja" se ve
        ch_n = ep[2] / 4095.0
        if ch_n > 0.04:
            jitter = max(1, int(ch_n * 12))
            r = np.roll(frame[:, :, 0],  jitter, axis=1)
            b = np.roll(frame[:, :, 2], -jitter, axis=1)
            frame = np.stack([r, frame[:, :, 1], b], axis=-1).astype(np.uint8)

        # Pitch shift → temperatura de color: agudo=cálido(rojo), grave=frío(azul)
        pitch_n = ep[3] / 4095.0   # 0=grave, 0.5=normal, 1=agudo
        tint = int((pitch_n - 0.5) * 80)
        if abs(tint) > 4:
            f = frame.astype(np.int16)
            if tint > 0:   # agudo → más rojo/cálido
                f[:, :, 0] = np.clip(f[:, :, 0] + tint,      0, 255)
                f[:, :, 2] = np.clip(f[:, :, 2] - tint // 2, 0, 255)
            else:           # grave → más azul/frío
                f[:, :, 2] = np.clip(f[:, :, 2] - tint,      0, 255)
                f[:, :, 0] = np.clip(f[:, :, 0] + tint // 2, 0, 255)
            frame = f.astype(np.uint8)

        # ── FASE 6: Guardar en historial + eco visual (POT1 modo Normal) ─────
        self._frame_hist.append(frame.copy())

        # Eco visual: un frame del pasado aparece semitransparente desplazado.
        # El tiempo del eco coincide exactamente con el delay de audio (pp[0]).
        dly_vid = pp[0] / 4095.0
        if dly_vid > 0.01 and len(self._frame_hist) >= 2:
            hist  = list(self._frame_hist)
            d_idx = max(1, int(dly_vid * (len(hist) - 1)))
            eco   = hist[max(0, len(hist) - 1 - d_idx)]
            if eco.shape == frame.shape:
                desp  = int(dly_vid * 30)
                eco_s = np.roll(eco, desp, axis=1) if desp > 0 else eco
                alpha = min(0.72, dly_vid * 0.82)
                frame = cv2.addWeighted(frame, 1.0, eco_s, alpha, 0)
                frame = np.clip(frame, 0, 255).astype(np.uint8)

        # ── FASE 7: FX Video (controlados por POTs en modo FX Video) ──────────

        # POT1 → Feedback espiral: el frame anterior rota y se mezcla sobre sí mismo.
        # La velocidad de rotación se multiplica por la amplitud del micrófono.
        fb_amt = vp[0] / 4095.0
        if fb_amt > 0.02 and self._prev_out is not None:
            fb = self._prev_out
            if fb.shape == frame.shape:
                h, w  = fb.shape[:2]
                angulo = fb_amt * 9.0 * (1.0 + rms * 2.5)   # acelera con audio
                escala = max(0.88, 1.0 - fb_amt * 0.10)
                M      = cv2.getRotationMatrix2D((w//2, h//2), angulo, escala)
                fb_rot = cv2.warpAffine(fb, M, (w, h),
                                        flags=cv2.INTER_LINEAR,
                                        borderMode=cv2.BORDER_WRAP)
                alpha  = min(0.93, fb_amt * 0.96)
                frame  = cv2.addWeighted(frame, 1.0 - alpha, fb_rot, alpha, 0)
                frame  = np.clip(frame, 0, 255).astype(np.uint8)

        # POT2 → Kaleidoscopio (cuádruple → octal al pasar 50 %)
        kali_n = vp[1] / 4095.0
        frame  = _kaleidoscopio(frame, kali_n)

        # POT3 → Pixelación reactiva: el tamaño de pixel crece con el audio
        pix_n = vp[2] / 4095.0
        if pix_n > 0.01:
            h, w    = frame.shape[:2]
            pix_sz  = max(2, int(2 + pix_n * 28 + rms * 22))
            pix_sz  = min(pix_sz, min(h, w) // 3)
            small   = cv2.resize(frame,
                                 (max(1, w // pix_sz), max(1, h // pix_sz)),
                                 interpolation=cv2.INTER_LINEAR)
            frame   = cv2.resize(small, (w, h), interpolation=cv2.INTER_NEAREST)

        # POT4 → Warp ondulante: deformación sinusoidal de píxeles.
        # La velocidad de la onda y su amplitud crecen con el audio.
        warp_n = vp[3] / 4095.0
        if warp_n > 0.01:
            h, w = frame.shape[:2]
            amp  = warp_n * 20.0 * (1.0 + rms * 3.5)
            freq = 2.0 + warp_n * 6.0
            self._warp_t += 0.06 + rms * 0.25
            xs  = np.tile(np.arange(w, dtype=np.float32), (h, 1))
            ys  = np.tile(np.arange(h, dtype=np.float32).reshape(h, 1), (1, w))
            mx  = np.clip(xs + amp * np.sin(2*math.pi * ys/h * freq + self._warp_t),
                          0, w-1).astype(np.float32)
            my  = np.clip(ys + amp * np.sin(2*math.pi * xs/w * freq + self._warp_t * 0.65),
                          0, h-1).astype(np.float32)
            frame = cv2.remap(frame, mx, my,
                              cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP)

        self._prev_out = frame.copy()
        return frame

    # ── HUD ───────────────────────────────────────────────────────────────────

    def _dibujar_hud(self, screen, s: dict, a: dict, W: int, H: int):
        fnt   = pygame.font.SysFont("monospace", 10)
        mfx   = s['modo_fx']
        color = C_MODO_FX[mfx]

        # Indicador de modo FX
        pygame.draw.circle(screen, color, (14, H - 14), 4)
        if mfx > 0:
            t = fnt.render(NOMBRES_FX[mfx], True, color)
            screen.blit(t, (24, H - 18))

        # VU meter del micrófono (esquina inferior derecha)
        rms_db   = max(-60.0, 20.0 * math.log10(max(1e-9, a['rms'])))
        rms_norm = (rms_db + 60.0) / 60.0
        bar_w    = int(80 * rms_norm)
        color_vu = (0, 200, 100) if rms_norm < 0.75 else (255, 140, 0)
        pygame.draw.rect(screen, (25, 25, 25),  (W-100, H-14, 80,    6))
        pygame.draw.rect(screen, color_vu,      (W-100, H-14, bar_w, 6))
        t_mic = fnt.render("MIC", True, (55, 55, 55))
        screen.blit(t_mic, (W - 122, H - 18))

        # Estado audio ON / OFF
        ao_color = (0, 200, 100) if s['audio_on'] else (180, 40, 40)
        ao_txt   = "SPK" if s['audio_on'] else "MUT"
        screen.blit(fnt.render(ao_txt, True, ao_color), (W - 148, H - 18))

        # Paleta activa
        t_pal = fnt.render(PALETAS[s['paleta_idx']], True, (55, 55, 55))
        screen.blit(t_pal, (W - 155 - t_pal.get_width(), H - 18))

        # FPS
        t_fps = fnt.render(f"{s['fps']:.0f}fps", True, (40, 40, 40))
        screen.blit(t_fps, (W - 40, 8))

    # ── Pantalla de ayuda ─────────────────────────────────────────────────────

    def _dibujar_ayuda(self, screen, s: dict, W: int, H: int, fnt_md, fnt_sm):
        mfx   = s['modo_fx']
        color = C_MODO_FX[mfx]

        overlay = pygame.Surface((W, H))
        overlay.set_alpha(195)
        overlay.fill((0, 0, 0))
        screen.blit(overlay, (0, 0))

        def lbl(x, y, txt, col=(150, 150, 150)):
            screen.blit(fnt_sm.render(txt, True, col), (x, y))

        cx   = W // 2;  y0 = H // 2 - 145;  dy = 22
        col_l = cx - 260;  col_m = cx - 20
        col_r = cx + 120;  col_v = cx + 250

        t = fnt_md.render(f"Video Synth Live  —  {NOMBRES_FX[mfx]}", True, color)
        screen.blit(t, (cx - t.get_width() // 2, y0))

        t2 = fnt_sm.render(
            f"Paleta: {PALETAS[s['paleta_idx']]}  |  Audio: {'ON' if s['audio_on'] else 'OFF'}",
            True, (90, 190, 90))
        screen.blit(t2, (cx - t2.get_width() // 2, y0 + 22))

        pygame.draw.line(screen, (45,45,45), (col_l, y0+44), (col_v+100, y0+44), 1)

        lbl(col_m, y0+50, "NORMAL",   (200,200,200) if mfx==0 else (55,55,55))
        lbl(col_r, y0+50, "FX AUDIO", (255,160,20)  if mfx==1 else (55,55,55))
        lbl(col_v, y0+50, "FX VIDEO", (80,140,255)  if mfx==2 else (55,55,55))

        filas = [
            ("POT1", "Delay time   [eco vid]",  "Filtro SVF (cutoff)",  "Feedback espiral"),
            ("POT2", "Delay feedbk [trail vid]", "Ring mod (freq)",      "Kaleidoscopio"),
            ("POT3", "Zoom + delay mix",          "Chorus / vibrato",    "Pixelación"),
            ("POT4", "Vol [brillo+sat vid]",      "Pitch shift (±1 oct)","Warp ondulante"),
        ]
        for i, (pot, nm, afx, vfx) in enumerate(filas):
            y = y0 + 68 + dy * i
            lbl(col_l, y, pot,  (110,110,110))
            lbl(col_m, y, nm,   (200,200,200) if mfx==0 else (65,65,65))
            lbl(col_r, y, afx,  (255,160,20)  if mfx==1 else (65,65,65))
            lbl(col_v, y, vfx,  (80,140,255)  if mfx==2 else (65,65,65))

        pygame.draw.line(screen, (45,45,45), (col_l, y0+160), (col_v+100, y0+160), 1)

        btns = [
            "BTN1  Silenciar / activar altavoz",
            "BTN3  Ciclar modo FX:  Normal → FX Audio → FX Video → Normal",
            "BTN4  Ciclar paleta de color",
            "",
            "FX Audio: POT4 al centro = pitch sin cambio  |  Ganancia mic: fija",
        ]
        for i, b in enumerate(btns):
            lbl(col_l, y0 + 170 + dy * i, b)

        nota = fnt_sm.render(
            "SPACE=silenciar  E=ciclar FX  P=paleta  H=ayuda  Q/ESC=salir",
            True, (45, 45, 45))
        screen.blit(nota, (cx - nota.get_width() // 2, H - 26))

    # ── Bucle principal ───────────────────────────────────────────────────────

    def run(self):
        pygame.init()
        screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
        pygame.display.set_caption("Video Synth Live")
        W, H = screen.get_size()

        fnt_md = pygame.font.SysFont("monospace", 14, bold=True)
        fnt_sm = pygame.font.SysFont("monospace", 12)

        # Cámara
        self._camara = CamaraCaptura(idx=0, w=640, h=360)
        if not self._camara.ok:
            print("ADVERTENCIA: No se pudo abrir la cámara (índice 0).")
        self._camara2 = CamaraCaptura(idx=1, w=640, h=360)
        if self._camara2.ok:
            print("Segunda cámara detectada (índice 1) — mezcla por IMU activa.")
        else:
            self._camara2 = None

        # Audio dúplex
        self._engine = AudioEngine(self.estado)
        try:
            self._stream = sd.Stream(
                samplerate=SR, blocksize=BLOCK,
                channels=2, dtype='float32',
                latency='low',
                callback=self._engine.callback,
            )
            self._stream.start()
        except Exception as e:
            print(f"ADVERTENCIA: No se pudo iniciar el audio: {e}")

        # Serial
        if self._ser:
            threading.Thread(target=self._thread_serial, daemon=True).start()

        # Calcular tamaño y posición centrada del video
        cam_ar = 640 / 360
        if W / H > cam_ar:
            dh, dw = H, int(H * cam_ar)
        else:
            dw, dh = W, int(W / cam_ar)
        dx    = (W - dw) // 2
        dy_off = (H - dh) // 2
        surf_vid = pygame.Surface((dw, dh), 0, 24)

        reloj = pygame.time.Clock()
        t0, nf = time.time(), 0
        mostrar_ayuda = False

        while self._running:
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    self._running = False
                elif ev.type == pygame.KEYDOWN:
                    k = ev.key
                    if k in (pygame.K_q, pygame.K_ESCAPE):
                        self._running = False
                    elif k == pygame.K_SPACE:
                        self.estado.toggle_audio()
                    elif k == pygame.K_e:
                        self.estado.ciclar_fx()
                    elif k == pygame.K_p:
                        self.estado.next_paleta()
                    elif k == pygame.K_h:
                        mostrar_ayuda = not mostrar_ayuda

            # Análisis de audio desde buffer circular
            if self._engine is not None:
                self.analisis.actualizar(self._engine.get_anal_buf())
            a = self.analisis.snap()
            s = self.estado.snap()

            # Leer frame(s) de cámara y mezclar según IMU
            frame = self._camara.get_frame() if self._camara else None
            if frame is None:
                frame = np.zeros((360, 640, 3), np.uint8)
            if self._camara2 is not None:
                frame2 = self._camara2.get_frame()
                if frame2 is not None:
                    if frame2.shape != frame.shape:
                        frame2 = cv2.resize(frame2, (frame.shape[1], frame.shape[0]))
                    imu_a = s['imu_x'] / 4095.0
                    frame = cv2.addWeighted(frame,  1.0 - imu_a,
                                            frame2, imu_a, 0).astype(np.uint8)

            # Redimensionar al tamaño de visualización
            if frame.shape[1] != dw or frame.shape[0] != dh:
                frame = cv2.resize(frame, (dw, dh), interpolation=cv2.INTER_LINEAR)

            # Aplicar toda la cadena de efectos
            frame = self._aplicar_vfx(frame, s, a)

            screen.fill((0, 0, 0))
            try:
                pygame.surfarray.blit_array(surf_vid,
                    np.ascontiguousarray(frame.transpose(1, 0, 2)))
                screen.blit(surf_vid, (dx, dy_off))
            except Exception:
                pass

            self._dibujar_hud(screen, s, a, W, H)
            if mostrar_ayuda:
                self._dibujar_ayuda(screen, s, W, H, fnt_md, fnt_sm)

            pygame.display.flip()

            nf += 1
            now = time.time()
            if now - t0 >= 1.0:
                with self.estado._l:
                    self.estado.fps = nf / (now - t0)
                nf, t0 = 0, now

            reloj.tick(60)

        # Limpieza
        if self._stream:
            self._stream.stop()
            self._stream.close()
        if self._camara:
            self._camara.close()
        if self._camara2:
            self._camara2.close()
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
        pygame.quit()


# ── Utilidades ────────────────────────────────────────────────────────────────

def detectar_esp32() -> str | None:
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").upper()
        if any(c in desc for c in ["CP210", "CH340", "CP2102", "UART", "USB SERIAL"]):
            return p.device
    return None


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    puerto = sys.argv[1] if len(sys.argv) >= 2 else None
    if not puerto:
        puerto = detectar_esp32()

    synth = VideoSynthLive(puerto)

    if puerto:
        try:
            synth._ser = serial.Serial(puerto, 115200, timeout=0.1)
            time.sleep(2)
            synth.estado.serial_ok = True
            print(f"Serial conectado: {puerto}")
        except serial.SerialException as e:
            print(f"Error serial: {e}")

    print("Video Synth Live — GC Lab Chile")
    print("Controles: SPACE=silenciar  E=FX  P=paleta  H=ayuda  Q=salir")
    synth.run()


if __name__ == "__main__":
    main()

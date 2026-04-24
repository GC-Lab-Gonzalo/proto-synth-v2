#!/usr/bin/env python3
# Video Synth v2 — GC Lab Chile
# Licencia: MIT License — https://opensource.org/licenses/MIT
#
# Todo nace del video:
#   - Un sample del audio del video loopea en sincronía con la batería
#   - Pitch cuantizado a escalas musicales
#   - Filtro resonante + delay aplicados a la mezcla
#   - Efectos de video: tiles, hue, ghost, feedback espiral
#
# CONTROLES:
#   BTN1       → Play / Pause
#   BTN2 corto → Ciclar preset de batería (0=off, 1–10)
#   BTN2 hold  → POT1 controla volumen de batería
#   BTN3       → Ciclar FX: Normal → FX Audio → FX Video → Normal
#   BTN4       → Siguiente escala musical
#
# Modo Normal:
#   POT1 → Posición en el video (define qué parte del audio se usa como sample)
#   POT2 → Pitch del sample (cuantizado a la escala)
#   POT3 → BPM (60–180)
#   POT4 → Volumen
#
# Modo FX Audio (BTN3 ×1):
#   POT1 → Filtro: frecuencia de corte (20 Hz – 20 kHz)
#   POT2 → Resonancia del filtro (Q 0.5 – 8)
#   POT3 → Reverb decay (tamaño de sala)
#   POT4 → Reverb wet/dry (mezcla)
#
# Modo FX Video (BTN3 ×2):
#   POT1 → Tiles (1×1 → 4×4)
#   POT2 → Hue shift
#   POT3 → Ghost / visión doble
#   POT4 → Flanger espiral (barrido de fase, sin distorsión)
#
# Teclado:
#   SPACE=play/pause  D=batería  L=largo sample  S=escala  E=ciclar FX
#   1-4=patrón batería  H=ayuda  Q/ESC=salir

import collections
import io
import math
import os
import subprocess
import sys
import tempfile
import time
import threading

import cv2
import numpy as np
import pygame
import sounddevice as sd
import soundfile as sf
import serial
import serial.tools.list_ports

try:
    import tkinter as tk
    from tkinter import filedialog
    _TK = True
except ImportError:
    _TK = False

# ── Constantes ────────────────────────────────────────────────────────────────

SR          = 44100
BLOCK       = 512
LOOP_FADE_N = 220   # ~5 ms de fade en los extremos del loop para evitar clicks
C_MODO_FX   = [(180, 180, 180), (255, 160, 20), (80, 140, 255)]
DELAY_MAX_S = 2.0
MAX_FRAMES  = 1200
RAM_LIMIT   = 1_500_000_000
MIN_STORE_W = 640
MIN_STORE_H = 360
GHOST_HIST  = 40
VEL_MIN     = 0.25
VEL_MAX     = 4.0

BPM_MIN       = 60.0
BPM_MAX       = 180.0
STEPS_PER_BAR = 16

SAMPLE_BEATS_OPTS = [0.5, 1.0, 2.0, 4.0]   # opciones de largo en beats

ESCALAS = {
    "Pentatónica Menor": [0, 3, 5, 7, 10, 12, 15, 17, 19, 22, 24],
    "Dórica":            [0, 2, 3, 5, 7,  9, 10, 12, 14, 15, 17, 19],
    "Frigia":            [0, 1, 3, 5, 7,  8, 10, 12, 13, 15, 17, 19],
    "Blues":             [0, 3, 5, 6, 7, 10, 12, 15, 17, 18, 19, 22],
    "Árabe":             [0, 1, 4, 5, 7,  8, 11, 12, 13, 16, 17, 19],
    "Cromática":         list(range(25)),
}
NOMBRES_ESCALAS = list(ESCALAS.keys())

AFX_NEUTRO = [4095, 0, 0, 0]   # filtro abierto, sin reverb
VFX_NEUTRO = [0, 0, 0, 0]

KICK_PATTERNS = [
    [1,0,0,0, 1,0,0,0, 1,0,0,0, 1,0,0,0],  # 4 en el suelo
    [1,0,0,0, 1,0,1,0, 1,0,0,0, 1,0,1,0],  # sincopado suave
    [1,0,0,1, 0,0,1,0, 1,0,0,0, 1,0,0,0],  # sincopado medio
    [1,0,1,0, 0,0,1,0, 1,0,0,0, 0,1,0,0],  # sincopado intenso
    [1,0,0,0, 0,0,1,0, 0,1,0,0, 1,0,0,0],  # reggaeton/latin
    [1,0,0,0, 1,0,0,1, 1,0,0,0, 1,0,0,0],  # with extra 16th
]
HIHAT_PATTERNS = [
    [1,0,1,0, 1,0,1,0, 1,0,1,0, 1,0,1,0],  # 8vos cerrado
    [1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1],  # 16vos
    [1,0,0,1, 1,0,1,0, 1,0,0,1, 1,0,1,0],  # clave
    [0,1,0,1, 0,1,0,1, 0,1,0,1, 0,1,0,1],  # contratiempo
    [1,0,1,1, 0,1,1,0, 1,0,1,1, 0,1,1,0],  # shuffle
    [1,1,0,1, 1,0,1,1, 1,1,0,1, 1,0,1,0],  # denso irregular
]
CLAP_PATTERNS = [
    [0,0,0,0, 1,0,0,0, 0,0,0,0, 1,0,0,0],  # 2 y 4 clásico
    [0,0,1,0, 1,0,0,0, 0,0,1,0, 1,0,0,0],  # anticipado
    [0,0,0,0, 0,1,0,0, 0,0,0,0, 0,1,0,0],  # off-beat
    [0,0,0,0, 1,0,0,1, 0,0,0,0, 1,0,1,0],  # sincopado
    [0,1,0,0, 1,0,0,1, 0,1,0,0, 1,0,0,0],  # latin
    [0,0,1,0, 0,0,1,0, 1,0,0,0, 1,0,1,0],  # irregular
]

# Presets de batería: (kick_idx, hihat_idx, clap_idx, kick_on, hihat_on, clap_on)
# El primer elemento es None = sin batería
DRUM_PRESETS = [
    None,                              # 0: sin batería
    (0, 0, 0, True,  False, False),    # 1: solo bombo
    (0, 0, 0, True,  True,  False),    # 2: bombo + hihat 8vos
    (0, 1, 0, True,  True,  False),    # 3: bombo + hihat 16vos
    (0, 4, 0, True,  True,  False),    # 4: bombo + hihat shuffle
    (0, 0, 0, True,  True,  True),     # 5: bombo + hihat + caja clásica
    (1, 0, 1, True,  True,  True),     # 6: bombo sincopado + hihat + caja anticipada
    (2, 1, 0, True,  True,  True),     # 7: bombo sincopado + hihat 16vos + caja
    (3, 4, 3, True,  True,  True),     # 8: patrón intenso shuffle
    (4, 2, 4, True,  True,  True),     # 9: patrón latin
    (5, 5, 5, True,  True,  True),     # 10: patrón complejo
]


# ── Síntesis de percusión ─────────────────────────────────────────────────────

def _sintetizar_kick(sr=SR, dur=0.55):
    t     = np.linspace(0, dur, int(sr * dur), endpoint=False)
    freq  = 90.0 * np.exp(-t * 22.0) + 45.0
    phase = 2.0 * np.pi * np.cumsum(freq) / sr
    env   = np.exp(-t * 10.0)
    click = np.zeros_like(t)
    cn = int(sr * 0.005)
    click[:cn] = np.random.default_rng(42).standard_normal(cn).astype(np.float32) * 0.35
    return np.clip(np.sin(phase) * env + click, -1.0, 1.0).astype(np.float32)

def _sintetizar_hihat(sr=SR, dur=0.06):
    t   = np.linspace(0, dur, int(sr * dur), endpoint=False)
    env = np.exp(-t * 90.0)
    return np.clip(np.random.default_rng(7).standard_normal(len(t)).astype(np.float32) * env * 0.45, -1.0, 1.0)

def _sintetizar_clap(sr=SR, dur=0.14):
    t   = np.linspace(0, dur, int(sr * dur), endpoint=False)
    env = np.exp(-t * 40.0) + 0.35 * np.exp(-t * 18.0)
    env /= env.max()
    return np.clip(np.random.default_rng(13).standard_normal(len(t)).astype(np.float32) * env * 0.65, -1.0, 1.0)


# ── Estado compartido ─────────────────────────────────────────────────────────

class Estado:
    def __init__(self):
        self._l        = threading.Lock()
        self.pot       = [2048, 2048, 2048, 2048]
        self.btn       = [0, 0, 0, 0]
        self._prev_btn = [0, 0, 0, 0]
        self._btn2_t   = 0.0

        self.play           = False
        self.modo_fx        = 0        # 0=normal, 1=fx audio, 2=fx video
        self.drum_preset    = 0        # índice en DRUM_PRESETS (0=off)
        self.drum_volume    = 0.75
        self._btn2_held     = False
        self._btn2_pot1_enter = 2048   # posición de POT1 al presionar BTN2
        self._btn2_vol_armed  = False  # True cuando POT1 se movió ≥ umbral
        self.escala_idx     = 0
        self.sample_beat_idx = 1       # índice en SAMPLE_BEATS_OPTS (1 = 1 beat)

        # Soft-takeover por modo
        self.play_eff  = [2048, 2048, 2048, 2048]
        self._pl_enter = [2048]*4;  self._pl_armed = [True]*4

        self.afx_eff   = list(AFX_NEUTRO)
        self._afx_enter = [2048]*4; self._afx_armed = [False]*4

        self.vfx_eff   = list(VFX_NEUTRO)
        self._vfx_enter = [2048]*4; self._vfx_armed = [False]*4

        self.serial_ok = False
        self.fps       = 0.0

    def update_serial(self, line: str):
        try:
            v = [int(x) for x in line.strip().split(',')]
        except ValueError:
            return
        if len(v) != 8:
            return
        with self._l:
            b, prev = v[4:], self._prev_btn[:]
            self.pot   = v[:4]
            self.btn   = b[:]
            self._prev_btn = b[:]

            if b[0] and not prev[0]:
                self.play = not self.play

            if b[1] and not prev[1]:
                self._btn2_t         = time.time()
                self._btn2_pot1_enter = v[0]   # capturar posición inicial de POT1
                self._btn2_vol_armed  = False
            if b[1]:
                self._btn2_held = True
                # Solo cambiar drum_volume si POT1 se movió ≥ 80 desde el press
                if not self._btn2_vol_armed and abs(v[0] - self._btn2_pot1_enter) > 80:
                    self._btn2_vol_armed = True
                if self._btn2_vol_armed:
                    self.drum_volume = v[0] / 4095.0
            if not b[1] and prev[1]:
                held = time.time() - self._btn2_t
                if held < 0.8 and not self._btn2_vol_armed:
                    # Pulsación corta sin mover POT1: avanzar preset
                    self.drum_preset = (self.drum_preset + 1) % len(DRUM_PRESETS)
                # Rearmar soft-takeover para todos los modos al soltar
                self._pl_enter  = v[:4][:]
                self._pl_armed  = [False] * 4
                self._afx_enter = v[:4][:]
                self._afx_armed = [False] * 4
                self._vfx_enter = v[:4][:]
                self._vfx_armed = [False] * 4
                self._btn2_held      = False
                self._btn2_vol_armed = False

            if b[2] and not prev[2]:
                nuevo = (self.modo_fx + 1) % 3
                self._cambiar_fx(nuevo, v[:4])

            if b[3] and not prev[3]:
                self.escala_idx = (self.escala_idx + 1) % len(NOMBRES_ESCALAS)

            self._actualizar_eff(v)

    def _cambiar_fx(self, nuevo: int, pot_actual: list):
        self.modo_fx = nuevo
        if nuevo == 0:
            self._pl_enter  = pot_actual[:]; self._pl_armed  = [False]*4
        elif nuevo == 1:
            self._afx_enter = pot_actual[:]; self._afx_armed = [False]*4
        elif nuevo == 2:
            self._vfx_enter = pot_actual[:]; self._vfx_armed = [False]*4

    def _actualizar_eff(self, v: list):
        if self._btn2_held:
            return  # bloquear todos los pots mientras BTN2 está presionado
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
                'pot':              self.pot[:],
                'btn':              self.btn[:],
                'play_eff':         self.play_eff[:],
                'afx_eff':          self.afx_eff[:],
                'vfx_eff':          self.vfx_eff[:],
                'play':             self.play,
                'modo_fx':          self.modo_fx,
                'drum_preset':      self.drum_preset,
                'drum_volume':      self.drum_volume,
                'escala_idx':       self.escala_idx,
                'sample_beat_idx':  self.sample_beat_idx,
                'serial':           self.serial_ok,
                'fps':              self.fps,
            }

    def toggle_play(self):
        with self._l: self.play = not self.play

    def ciclar_fx(self):
        with self._l:
            nuevo = (self.modo_fx + 1) % 3
            self._cambiar_fx(nuevo, self.pot[:])

    def next_drum_preset(self):
        with self._l: self.drum_preset = (self.drum_preset + 1) % len(DRUM_PRESETS)

    def next_escala(self):
        with self._l: self.escala_idx = (self.escala_idx + 1) % len(NOMBRES_ESCALAS)

    def ciclar_largo(self):
        with self._l:
            self.sample_beat_idx = (self.sample_beat_idx + 1) % len(SAMPLE_BEATS_OPTS)


# ── Motor de audio ────────────────────────────────────────────────────────────

class AudioEngine:
    """
    El sample es un fragmento del audio del video que loopea en sincronía
    con el BPM. POT1 define la posición, POT2 el pitch, POT3 el BPM.
    La batería corre en paralelo con el mismo clock.
    DSP post-mezcla: filtro biquad LPF resonante + delay con feedback.
    """

    def __init__(self, audio: np.ndarray, estado: Estado):
        self.audio  = audio if len(audio) > 1 else np.zeros(SR * 5, np.float32)
        self.n      = len(self.audio)
        self.estado = estado

        # Posición del loop de sample (en muestras de salida)
        self._beat_pos = 0

        # Batería
        self._kick_smp  = _sintetizar_kick()
        self._hihat_smp = _sintetizar_hihat()
        self._clap_smp  = _sintetizar_clap()
        self._drum_csr  = {'kick': -1, 'hihat': -1, 'clap': -1}
        self._beat_phase = 0.0
        self._beat_count = 0
        # Filtro LP de un polo para suavizar los agudos de la batería (~5 kHz)
        _fc_drum = 5000.0
        self._drum_lpf_alpha = 1.0 - math.exp(-2.0 * math.pi * _fc_drum / SR)
        self._drum_lpf_z     = 0.0

        # Parámetros bloqueados por ciclo de loop: se actualizan solo en _beat_pos==0
        # Así sample_start y loop_out son constantes dentro de cada ciclo → sin clicks
        self._locked_sample_start = 0
        self._locked_loop_out     = max(1, int(SR * 60.0 / 120.0))  # default 120 BPM
        self._locked_pitch        = 1.0

        # Filtro biquad LPF (TDF2)
        self._bq_z1 = 0.0
        self._bq_z2 = 0.0

        # Reverb Schroeder: 4 filtros comb en paralelo + 2 all-pass en serie
        _rv_cd = [1557, 1617, 1491, 1422]
        self._rv_cd = _rv_cd
        self._rv_cb = [np.zeros(d + 2, np.float32) for d in _rv_cd]
        self._rv_cw = [0] * 4
        _rv_ad = [225, 556]
        self._rv_ad = _rv_ad
        self._rv_ab = [np.zeros(d + 2, np.float32) for d in _rv_ad]
        self._rv_aw = [0] * 2

        # Ring modulation, tremolo y flanger (FX Video → Audio): phases continuas entre callbacks
        self._ring_phase    = 0.0
        self._tremolo_phase = 0.0
        self._flanger_phase = 0.0
        _fl_max = int(0.022 * SR) + 2   # ~22 ms de buffer para flanger
        self._fl_max        = _fl_max
        self._flanger_buf   = np.zeros(_fl_max, np.float32)
        self._fl_wr         = 0

        # Fade-in al iniciar playback (evita transitorio de arranque)
        _fade_len           = int(SR * 0.05)   # 50 ms
        self._fade_len      = _fade_len
        self._fade_pos      = _fade_len        # inicia "terminado" → no aplica si ya estaba play
        self._prev_play     = False

    def get_beat_phase(self) -> float:
        return self._beat_phase % 1.0

    def get_beat_count(self) -> int:
        return self._beat_count

    def get_beat_step(self) -> int:
        return int(self._beat_phase) % STEPS_PER_BAR

    # ── Helpers ──────────────────────────────────────────────────────────────

    @staticmethod
    def _bpm_desde_pot(val: int) -> float:
        return BPM_MIN + (val / 4095.0) * (BPM_MAX - BPM_MIN)

    @staticmethod
    def _pitch_cuantizado(val: int, escala_idx: int) -> float:
        notas  = ESCALAS[NOMBRES_ESCALAS[escala_idx]]
        n      = len(notas)
        idx    = int((val / 4095.0) * (n - 1) + 0.5)
        idx    = max(0, min(n - 1, idx))
        centro = n // 2
        return 2.0 ** ((notas[idx] - notas[centro]) / 12.0)

    # ── Sample loop ───────────────────────────────────────────────────────────

    def _mezclar_sample(self, out: np.ndarray, frames: int, s: dict):
        if not s['play']:
            return

        pp    = s['play_eff']
        bpm   = self._bpm_desde_pot(pp[2])
        pitch = self._pitch_cuantizado(pp[1], s['escala_idx'])
        pitch = max(VEL_MIN, min(VEL_MAX, pitch))

        # Calcular parámetros deseados desde los pots actuales
        beat_out     = max(1, int(SR * 60.0 / bpm))
        n_beats      = SAMPLE_BEATS_OPTS[s['sample_beat_idx']]
        new_loop_out = max(1, int(beat_out * n_beats))
        max_start    = max(0, self.n - new_loop_out * 4)
        new_sample_start = int((pp[0] / 4095.0) * max_start)
        new_pitch    = pitch

        # Generar frames de salida con manejo de wrap-around del loop
        remaining = frames
        offset    = 0

        while remaining > 0:
            # Al inicio de cada ciclo del loop, adoptar los parámetros actuales.
            # Así los valores son constantes dentro del ciclo → sin clicks mid-loop,
            # pero el cambio de pot tiene efecto en el próximo ciclo.
            if self._beat_pos == 0:
                self._locked_sample_start = new_sample_start
                self._locked_loop_out     = new_loop_out
                self._locked_pitch        = new_pitch

            # Sanitizar si beat_pos quedó fuera del loop bloqueado
            if self._beat_pos >= self._locked_loop_out:
                self._beat_pos = 0
                continue

            frames_to_end = self._locked_loop_out - self._beat_pos
            f = min(remaining, frames_to_end)
            if f <= 0:
                self._beat_pos = 0
                break

            at_loop_start = (self._beat_pos == 0)
            at_loop_end   = (f == frames_to_end)

            # Posición en el audio fuente (con pitch)
            src_start = self._locked_sample_start + int(self._beat_pos * self._locked_pitch)
            src_end   = self._locked_sample_start + int((self._beat_pos + f) * self._locked_pitch)
            n_read    = max(2, src_end - src_start)

            src_start = max(0, min(self.n - 1, src_start))
            raw = self.audio[src_start : src_start + n_read]
            if len(raw) < n_read:
                raw = np.pad(raw, (0, n_read - len(raw)))

            chunk = np.interp(
                np.linspace(0, n_read - 1, f),
                np.arange(n_read), raw
            ).astype(np.float32)

            # Fade-in al inicio del ciclo y fade-out al final: evita click en el punto de loop
            if at_loop_start and f >= 2:
                n = min(LOOP_FADE_N, f)
                chunk[:n] *= np.linspace(0.0, 1.0, n, dtype=np.float32)
            if at_loop_end and f >= 2:
                n = min(LOOP_FADE_N, f)
                chunk[-n:] *= np.linspace(1.0, 0.0, n, dtype=np.float32)

            out[offset : offset + f] += chunk
            self._beat_pos = (self._beat_pos + f) % self._locked_loop_out
            offset    += f
            remaining -= f

    # ── Batería ───────────────────────────────────────────────────────────────

    def _mezclar_drums(self, out: np.ndarray, frames: int, s: dict):
        preset = DRUM_PRESETS[s['drum_preset']]
        if preset is None:
            return
        ki, hi, ci, k_on, h_on, c_on = preset
        bpm = self._bpm_desde_pot(s['play_eff'][2])
        steps_per_sample = (STEPS_PER_BAR * bpm) / (4.0 * 60.0 * SR)
        kick_p  = KICK_PATTERNS[ki]
        hihat_p = HIHAT_PATTERNS[hi]
        clap_p  = CLAP_PATTERNS[ci]
        mix     = np.zeros(frames, np.float32)

        for i in range(frames):
            prev_step = int(self._beat_phase) % STEPS_PER_BAR
            self._beat_phase += steps_per_sample
            step = int(self._beat_phase) % STEPS_PER_BAR
            if step != prev_step:
                if k_on and kick_p[step]:
                    self._drum_csr['kick']  = 0
                    self._beat_count += 1
                if h_on and hihat_p[step]:
                    self._drum_csr['hihat'] = 0
                if c_on and clap_p[step]:
                    self._drum_csr['clap']  = 0

            for nombre, smp in (('kick', self._kick_smp),
                                 ('hihat', self._hihat_smp),
                                 ('clap', self._clap_smp)):
                c = self._drum_csr[nombre]
                if 0 <= c < len(smp):
                    mix[i] += smp[c]
                    self._drum_csr[nombre] = c + 1

        # Filtro LP para quitar agudos de hihats y claps
        a = self._drum_lpf_alpha
        z = self._drum_lpf_z
        for i in range(frames):
            z += a * (mix[i] - z)
            mix[i] = z
        self._drum_lpf_z = z

        out += mix * s['drum_volume']

    # ── DSP: filtro biquad LPF + delay ───────────────────────────────────────

    def _aplicar_dsp(self, chunk: np.ndarray, frames: int, ep: list, vp: list) -> np.ndarray:
        fc_hz  = 20.0 * (1000.0 ** (ep[0] / 4095.0))
        fc_hz  = min(fc_hz, SR * 0.49)
        Q      = 0.5 + (ep[1] / 4095.0) * 7.5
        w0     = 2.0 * math.pi * fc_hz / SR
        cw     = math.cos(w0)
        sw     = math.sin(w0)
        alpha  = sw / (2.0 * Q)
        a0i    = 1.0 / (1.0 + alpha)
        b0n    = (1.0 - cw) * 0.5 * a0i
        b1n    = (1.0 - cw)       * a0i
        b2n    = (1.0 - cw) * 0.5 * a0i
        a1n    = -2.0 * cw        * a0i
        a2n    = (1.0 - alpha)    * a0i

        filt = np.empty(frames, np.float32)
        z1, z2 = self._bq_z1, self._bq_z2
        for i in range(frames):
            y       = b0n * chunk[i] + z1
            z1      = b1n * chunk[i] - a1n * y + z2
            z2      = b2n * chunk[i] - a2n * y
            filt[i] = y
        self._bq_z1, self._bq_z2 = z1, z2
        chunk = np.clip(filt, -1.0, 1.0)

        if not np.isfinite(chunk).all():
            self._bq_z1 = self._bq_z2 = 0.0
            chunk = np.zeros(frames, np.float32)

        # Reverb Schroeder (4 comb paralelos + 2 all-pass en serie)
        # ep[2] → decay 0.50..0.88 | ep[3] → mezcla wet 0..0.38
        decay = 0.50 + (ep[2] / 4095.0) * 0.38
        wet   = (ep[3] / 4095.0) * 0.38
        if wet > 0.005:
            comb_out = np.zeros(frames, np.float32)
            for k in range(4):
                buf = self._rv_cb[k]
                d   = self._rv_cd[k]
                wr  = self._rv_cw[k]
                bl  = len(buf)
                ok  = np.empty(frames, np.float32)
                for i in range(frames):
                    rd      = (wr - d) % bl
                    ok[i]   = buf[rd]
                    buf[wr] = chunk[i] + ok[i] * decay
                    wr      = (wr + 1) % bl
                self._rv_cw[k] = wr
                comb_out += ok
            comb_out *= 0.25  # normalizar 4 comb

            ap = comb_out
            for k in range(2):
                buf = self._rv_ab[k]
                d   = self._rv_ad[k]
                wr  = self._rv_aw[k]
                bl  = len(buf)
                g   = 0.5
                ok  = np.empty(frames, np.float32)
                for i in range(frames):
                    rd      = (wr - d) % bl
                    buf_d   = buf[rd]
                    v       = ap[i] + g * buf_d
                    ok[i]   = buf_d - g * v
                    buf[wr] = v
                    wr      = (wr + 1) % bl
                self._rv_aw[k] = wr
                ap = ok

            chunk = chunk * (1.0 - wet) + np.clip(ap, -1.0, 1.0) * wet

        # ── FX Video → Audio ─────────────────────────────────────────────────

        # Tiles → bit crushing (más tiles = sonido más fragmentado)
        # levels baja linealmente de 256 (casi transparente) a ~51 (80% del efecto máximo)
        tiles_norm = vp[0] / 4095.0
        if tiles_norm > 0.01:
            levels = max(26.0, 256.0 * (1.0 - tiles_norm * 0.90))
            chunk  = np.round(chunk * levels) / levels

        # Hue shift → ring modulation (cambio de color = cambio de timbre)
        hue_norm = vp[1] / 4095.0
        if hue_norm > 0.01:
            freq      = 80.0 + hue_norm * 320.0         # 80..400 Hz
            phase_inc = 2.0 * math.pi * freq / SR
            phases    = self._ring_phase + phase_inc * np.arange(frames, dtype=np.float32)
            carrier   = np.cos(phases).astype(np.float32)
            self._ring_phase = float((self._ring_phase + phase_inc * frames) % (2 * math.pi))
            wet   = hue_norm * 0.45
            chunk = np.tanh(chunk * (1.0 - wet) + chunk * carrier * wet)

        # Ghost → tremolo (visión doble = pulso de amplitud)
        ghost_norm = vp[2] / 4095.0
        if ghost_norm > 0.02:
            lfo_freq  = 3.0 + ghost_norm * 10.0         # 3..13 Hz
            phase_inc = 2.0 * math.pi * lfo_freq / SR
            phases    = self._tremolo_phase + phase_inc * np.arange(frames, dtype=np.float32)
            lfo       = (0.5 + 0.5 * np.cos(phases)).astype(np.float32)
            self._tremolo_phase = float((self._tremolo_phase + phase_inc * frames) % (2 * math.pi))
            depth = ghost_norm * 0.65
            chunk = chunk * (1.0 - depth + depth * lfo)

        # Feedback espiral → flanger (más espiral = barrido más profundo y rápido)
        fl_norm = vp[3] / 4095.0
        if fl_norm > 0.02:
            lfo_freq  = 0.3 + fl_norm * 2.7          # 0.3..3 Hz
            phase_inc = 2.0 * math.pi * lfo_freq / SR
            max_depth = max(2, int(fl_norm * self._fl_max * 0.8))
            lfo_ph    = self._flanger_phase
            out_fl    = np.empty(frames, np.float32)
            for i in range(frames):
                lfo_val   = math.sin(lfo_ph)
                lfo_ph   += phase_inc
                d         = max(1, int(max_depth * (0.5 + 0.5 * lfo_val)))
                d         = min(d, self._fl_max - 1)
                rd        = (self._fl_wr - d) % self._fl_max
                out_fl[i] = chunk[i] + self._flanger_buf[rd] * fl_norm * 0.6
                self._flanger_buf[self._fl_wr] = chunk[i]
                self._fl_wr = (self._fl_wr + 1) % self._fl_max
            self._flanger_phase = float(lfo_ph % (2.0 * math.pi))
            chunk = np.clip(out_fl, -1.0, 1.0)
        else:
            for i in range(frames):
                self._flanger_buf[self._fl_wr] = chunk[i]
                self._fl_wr = (self._fl_wr + 1) % self._fl_max

        return chunk

    # ── Callback ──────────────────────────────────────────────────────────────

    def callback(self, outdata: np.ndarray, frames: int, ti, st):
        s   = self.estado.snap()

        # Detectar arranque de playback: resetear filtro + armar fade-in
        if s['play'] and not self._prev_play:
            self._bq_z1   = 0.0
            self._bq_z2   = 0.0
            self._fade_pos = 0
        self._prev_play = s['play']

        out = np.zeros(frames, np.float32)

        self._mezclar_sample(out, frames, s)
        self._mezclar_drums(out, frames, s)

        # Pre-limiter suave: evita clipping duro cuando sample + batería suman alto
        out = np.tanh(out * 0.55)

        out = self._aplicar_dsp(out, frames, s['afx_eff'], s['vfx_eff'])

        # Fade-in de 50 ms al arrancar para suprimir transitorio inicial
        if self._fade_pos < self._fade_len:
            ramp_end = min(frames, self._fade_len - self._fade_pos)
            ramp = np.linspace(self._fade_pos / self._fade_len,
                               (self._fade_pos + ramp_end) / self._fade_len,
                               ramp_end, dtype=np.float32)
            out[:ramp_end] *= ramp
            self._fade_pos += frames

        vol = s['play_eff'][3] / 4095.0
        out = np.clip(out * vol, -1.0, 1.0)

        outdata[:, 0] = out
        outdata[:, 1] = out


# ── Sintetizador principal ────────────────────────────────────────────────────

class VideoSynthV2:

    def __init__(self, ruta: str, puerto: str | None = None):
        self.ruta    = ruta
        self.puerto  = puerto
        self.estado  = Estado()

        self.frames:    list[np.ndarray] = []
        self.fps_vid:   float = 25.0
        self.disp_size  = (640, 360)
        self.store_size = (640, 360)
        self.disp_off   = (0, 0)

        self._engine:  AudioEngine | None  = None
        self._stream:  sd.OutputStream | None = None
        self._ser:     serial.Serial | None = None
        self._running  = True

        self._frame_hist: collections.deque = collections.deque(maxlen=GHOST_HIST)
        self._prev_out:   np.ndarray | None = None

    # ── Carga ─────────────────────────────────────────────────────────────────

    def _progress(self, screen, msg: str, ratio: float, fnt):
        W, H = screen.get_size()
        screen.fill((0, 0, 0))
        bw = int(W * 0.38); bx = (W - bw) // 2; by = H // 2
        pygame.draw.rect(screen, (20, 20, 20), (bx, by, bw, 2))
        if ratio > 0:
            pygame.draw.rect(screen, (130, 130, 130), (bx, by, int(bw * ratio), 2))
        if msg:
            t = fnt.render(msg, True, (55, 55, 55))
            screen.blit(t, (bx, by - 18))
        pygame.display.flip()
        pygame.event.pump()

    def cargar(self, screen):
        W, H = screen.get_size()
        fnt  = pygame.font.SysFont("monospace", 11)
        self._progress(screen, "abriendo video...", 0.0, fnt)

        cap = cv2.VideoCapture(self.ruta)
        if not cap.isOpened():
            raise FileNotFoundError(f"No se puede abrir: {self.ruta}")

        total  = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 1
        fps_v  = cap.get(cv2.CAP_PROP_FPS) or 25.0
        vw     = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        vh     = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.fps_vid = fps_v

        ar = vw / max(1, vh)
        if W / H > ar:
            dh, dw = H, int(H * ar)
        else:
            dw, dh = W, int(W / ar)
        self.disp_size = (dw, dh)
        self.disp_off  = ((W - dw) // 2, (H - dh) // 2)

        step   = max(1, total // MAX_FRAMES)
        n_load = total // step
        max_bpf = RAM_LIMIT / max(1, n_load)
        dw_s = int(math.sqrt(max_bpf * ar / 3))
        dh_s = max(2, int(dw_s / ar))
        dw_s = max(MIN_STORE_W, min(dw, dw_s))
        dh_s = max(MIN_STORE_H, min(dh, dh_s))
        dw_s = (dw_s // 2) * 2; dh_s = (dh_s // 2) * 2
        self.store_size = (dw_s, dh_s)

        fi = cargados = 0
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            if fi % step == 0:
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                if (rgb.shape[1], rgb.shape[0]) != (dw_s, dh_s):
                    rgb = cv2.resize(rgb, (dw_s, dh_s), interpolation=cv2.INTER_LINEAR)
                self.frames.append(rgb)
                cargados += 1
                if cargados % 60 == 0:
                    self._progress(screen, f"cargando... {cargados}/{n_load}",
                                   cargados / max(1, n_load), fnt)
            fi += 1
        cap.release()

        if not self.frames:
            raise ValueError("No se pudieron cargar frames.")

        self._progress(screen, "extrayendo audio...", 0.92, fnt)
        audio = self._extraer_audio()
        self._engine = AudioEngine(audio, self.estado)
        self._progress(screen, "", 1.0, fnt)
        time.sleep(0.1)

    def _extraer_audio(self) -> np.ndarray:
        ffmpeg = 'ffmpeg'
        try:
            import imageio_ffmpeg
            ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
        except ImportError:
            pass
        tmp = None
        try:
            fd, tmp = tempfile.mkstemp(suffix='.wav')
            os.close(fd)
            cmd = [ffmpeg, '-y', '-i', self.ruta,
                   '-vn', '-ac', '1', '-ar', str(SR), tmp]
            r = subprocess.run(cmd, capture_output=True, timeout=600)
            if r.returncode == 0 and os.path.getsize(tmp) > 44:
                a, _ = sf.read(tmp, dtype='float32', always_2d=False)
                return (a.mean(axis=1) if a.ndim > 1 else a).astype(np.float32)
        except Exception:
            pass
        finally:
            if tmp and os.path.exists(tmp):
                try:
                    os.unlink(tmp)
                except Exception:
                    pass
        return np.zeros(SR * 10, np.float32)

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

    # ── Video: frame según posición POT1 ─────────────────────────────────────

    def _get_frame(self, s: dict) -> np.ndarray:
        n = len(self.frames)
        if n == 0:
            dw, dh = self.disp_size
            return np.zeros((dh, dw, 3), np.uint8)

        # El frame muestra la posición del sample (POT1)
        base_idx = int((s['play_eff'][0] / 4095.0) * (n - 1))

        # Cuando la batería está activa, secuenciar entre frames en sync con el beat
        if s['drum_preset'] > 0 and self._engine is not None:
            beat_step = self._engine.get_beat_step()
            paso = max(1, n // STEPS_PER_BAR)
            idx = (base_idx + beat_step * paso) % n
        else:
            idx = base_idx

        idx   = max(0, min(n - 1, idx))
        frame = self.frames[idx]

        if self.store_size != self.disp_size:
            frame = cv2.resize(frame, self.disp_size, interpolation=cv2.INTER_CUBIC)
        else:
            frame = frame.copy()

        return self._aplicar_vfx(frame, s)

    def _aplicar_vfx(self, frame: np.ndarray, s: dict) -> np.ndarray:
        ep = s['vfx_eff']
        af = s['afx_eff']
        beat_phase = self._engine.get_beat_phase() if self._engine else 0.0
        beat_count = self._engine.get_beat_count() if self._engine else 0

        # ── FX Audio → Video ─────────────────────────────────────────────────

        # Filtro → bordes estilo comic/dibujo (filtro cerrado = más contornos)
        filtro_norm = af[0] / 4095.0   # 1.0=abierto/neutro  0.0=cerrado
        if filtro_norm < 0.97:
            amount = 1.0 - filtro_norm
            gray   = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            hi_thr = max(20, int(120 * filtro_norm))
            edges  = cv2.Canny(gray, hi_thr // 2, hi_thr)
            kernel = np.ones((2, 2), np.uint8)
            edges  = cv2.dilate(edges, kernel, iterations=1)
            edges_rgb = cv2.cvtColor(edges, cv2.COLOR_GRAY2RGB)
            frame  = cv2.addWeighted(frame, 1.0, edges_rgb, amount * 2.5, 0)
            frame  = np.clip(frame, 0, 255).astype(np.uint8)

        # Resonancia → aberración cromática (split de canales RGB)
        res_norm = af[1] / 4095.0     # 0.0=sin resonancia  1.0=mucha
        if res_norm > 0.01:
            shift = max(1, int(res_norm * 22))
            r = np.roll(frame[:, :, 0],  shift, axis=1)
            b = np.roll(frame[:, :, 2], -shift, axis=1)
            frame = np.stack([r, frame[:, :, 1], b], axis=-1).astype(np.uint8)

        # Pitch → zoom central + temperatura de color
        # play_eff[1] = POT2 modo normal = pitch cuantizado a la escala
        # nota baja → zoom out + tinte frío (azul)
        # nota alta → zoom in  + tinte cálido (rojo/naranja)
        pitch_norm = s['play_eff'][1] / 4095.0
        zoom = 0.82 + pitch_norm * 0.42   # 0.82 (alejado) … 1.24 (acercado)
        if abs(zoom - 1.0) > 0.02:
            h_f, w_f = frame.shape[:2]
            new_h = max(4, int(h_f / zoom))
            new_w = max(4, int(w_f / zoom))
            y0 = max(0, (h_f - new_h) // 2)
            x0 = max(0, (w_f - new_w) // 2)
            new_h = min(new_h, h_f - y0)
            new_w = min(new_w, w_f - x0)
            frame = cv2.resize(frame[y0:y0+new_h, x0:x0+new_w],
                               (w_f, h_f), interpolation=cv2.INTER_LINEAR)
        tint = int((pitch_norm - 0.5) * 60)   # -30..+30
        if abs(tint) > 3:
            f = frame.astype(np.int16)
            if tint > 0:   # notas altas → más rojo, menos azul
                f[:, :, 0] = np.clip(f[:, :, 0] + tint,      0, 255)
                f[:, :, 2] = np.clip(f[:, :, 2] - tint // 2, 0, 255)
            else:          # notas bajas → más azul, menos rojo
                f[:, :, 2] = np.clip(f[:, :, 2] - tint,      0, 255)
                f[:, :, 0] = np.clip(f[:, :, 0] + tint // 2, 0, 255)
            frame = f.astype(np.uint8)

        # Tiles
        n_tiles = 1 + int((ep[0] / 4095.0) * 9.5)
        n_tiles = max(1, min(10, n_tiles))
        if n_tiles > 1:
            frame = self._efecto_tiles(frame, n_tiles, beat_phase, beat_count)

        # Hue shift
        if ep[1] > 40:
            hue_shift = int((ep[1] / 4095.0) * 90)
            hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV).copy()
            hsv[:, :, 0] = (hsv[:, :, 0].astype(np.int32) + hue_shift) % 180
            frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)

        self._frame_hist.append(frame)

        # Delay → eco visual (el delay del audio se ve como un eco temporal de frames)
        delay_norm = af[2] / 4095.0
        if delay_norm > 0.01 and len(self._frame_hist) >= 2:
            hist    = list(self._frame_hist)
            d_idx   = max(1, int(delay_norm * (len(hist) - 1)))
            eco     = hist[max(0, len(hist) - 1 - d_idx)]
            if eco.shape == frame.shape:
                desp  = int(delay_norm * 18)
                eco_s = np.roll(eco, desp, axis=1) if desp > 0 else eco
                alpha = min(0.55, delay_norm * 0.65)
                frame = cv2.addWeighted(frame, 1.0, eco_s, alpha, 0)
                frame = np.clip(frame, 0, 255).astype(np.uint8)

        # Ghost
        ghost_amt = ep[2] / 4095.0
        if ghost_amt > 0.02 and len(self._frame_hist) >= 3:
            hist  = list(self._frame_hist)
            d_idx = max(1, int(ghost_amt * (len(hist) - 1)))
            ghost = hist[max(0, len(hist) - 1 - d_idx)]
            if ghost.shape == frame.shape:
                shift   = int(ghost_amt * 40)
                ghost_r = np.roll(ghost, shift, axis=1) if 0 < shift < frame.shape[1] // 3 else ghost
                alpha   = min(0.75, ghost_amt * 0.85)
                frame   = cv2.addWeighted(frame, 1.0, ghost_r, alpha, 0)
                frame   = np.clip(frame, 0, 255).astype(np.uint8)
                if ghost_amt > 0.55:
                    ghost_l = np.roll(ghost, -shift, axis=1)
                    frame   = cv2.addWeighted(frame, 1.0, ghost_l, alpha * 0.45, 0)
                    frame   = np.clip(frame, 0, 255).astype(np.uint8)

        # Feedback espiral
        fb_amt = ep[3] / 4095.0
        if fb_amt > 0.02 and self._prev_out is not None:
            fb = self._prev_out
            if fb.shape == frame.shape:
                h, w  = fb.shape[:2]
                angle = fb_amt * 5.0
                scale = max(0.92, 1.0 - fb_amt * 0.06)
                M     = cv2.getRotationMatrix2D((w // 2, h // 2), angle, scale)
                fb_w  = cv2.warpAffine(fb, M, (w, h),
                                       flags=cv2.INTER_LINEAR,
                                       borderMode=cv2.BORDER_WRAP)
                alpha = min(0.88, fb_amt * 0.92)
                frame = cv2.addWeighted(frame, 1.0 - alpha, fb_w, alpha, 0)
                frame = np.clip(frame, 0, 255).astype(np.uint8)

        # Feedback audio → warp ondulante de píxeles (el feedback se vuelve distorsión visual)
        fb_audio_norm = af[3] / 4095.0
        if fb_audio_norm > 0.02:
            h, w = frame.shape[:2]
            amp  = fb_audio_norm * 14.0
            fy   = 2.0 + fb_audio_norm * 4.0
            fx   = 3.0 + fb_audio_norm * 5.0
            xs   = np.tile(np.arange(w, dtype=np.float32), (h, 1))
            ys   = np.tile(np.arange(h, dtype=np.float32).reshape(h, 1), (1, w))
            mx   = np.clip(xs + amp * np.sin(2 * math.pi * ys / h * fy), 0, w - 1).astype(np.float32)
            my   = np.clip(ys + amp * np.sin(2 * math.pi * xs / w * fx), 0, h - 1).astype(np.float32)
            frame = cv2.remap(frame, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP)

        # Volumen → brillo global + saturación
        # vol bajo (0.0) → imagen oscura y desaturada / vol alto (1.0) → imagen normal y saturada
        vol_norm = s['play_eff'][3] / 4095.0
        bright   = 0.15 + vol_norm * 0.85          # 0.15 (casi negro) … 1.0 (normal)
        sat      = vol_norm * 1.5                   # 0.0 (escala de grises) … 1.5 (hipersaturado)
        if bright < 0.98 or not (0.98 < sat < 1.02):
            hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV).astype(np.float32)
            hsv[:, :, 1] = np.clip(hsv[:, :, 1] * sat,    0, 255)
            hsv[:, :, 2] = np.clip(hsv[:, :, 2] * bright, 0, 255)
            frame = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2RGB)

        self._prev_out = frame
        return frame

    def _efecto_tiles(self, frame: np.ndarray, n_tiles: int,
                      beat_phase: float, beat_count: int) -> np.ndarray:
        h, w = frame.shape[:2]
        tw, th = w // n_tiles, h // n_tiles
        if tw < 4 or th < 4:
            return frame
        tile_base = cv2.resize(frame, (tw, th), interpolation=cv2.INTER_LINEAR)
        filas = []
        for ry in range(n_tiles):
            columna = []
            for rx in range(n_tiles):
                t = tile_base.copy()
                hue_idx = ry * n_tiles + rx
                hue_off = int((hue_idx * 25 + beat_count * 8) % 180)
                zoom = 1.0 + 0.04 * math.sin(beat_phase * 2 * math.pi + hue_idx)
                if zoom != 1.0:
                    zw, zh = int(tw * zoom), int(th * zoom)
                    t = cv2.resize(t, (zw, zh), interpolation=cv2.INTER_LINEAR)
                    ox = (zw - tw) // 2; oy = (zh - th) // 2
                    t = t[oy:oy+th, ox:ox+tw]
                    if t.shape[0] != th or t.shape[1] != tw:
                        t = cv2.resize(t, (tw, th))
                if hue_off > 3:
                    hsv = cv2.cvtColor(t, cv2.COLOR_RGB2HSV).copy()
                    hsv[:, :, 0] = (hsv[:, :, 0].astype(np.int32) + hue_off) % 180
                    t = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)
                columna.append(t)
            filas.append(np.concatenate(columna, axis=1))
        tiled = np.concatenate(filas, axis=0)
        if tiled.shape[1] != w or tiled.shape[0] != h:
            tiled = cv2.resize(tiled, (w, h), interpolation=cv2.INTER_LINEAR)
        return tiled

    # ── HUD ───────────────────────────────────────────────────────────────────

    def _dibujar_beat_flash(self, screen, s: dict, W: int, H: int):
        if s['drum_preset'] == 0 or self._engine is None:
            return
        phase = self._engine.get_beat_phase()
        if phase < 0.07:
            alpha = int(255 * (1.0 - phase / 0.07) * 0.35)
            flash = pygame.Surface((W, H), pygame.SRCALPHA)
            flash.fill((255, 255, 255, alpha))
            screen.blit(flash, (0, 0))

    def _dibujar_hud(self, screen, s: dict, W: int, H: int):
        fnt   = pygame.font.SysFont("monospace", 10)
        mfx   = s['modo_fx']
        color = C_MODO_FX[mfx]

        # Punto de modo FX
        pygame.draw.circle(screen, color, (14, H - 14), 4)

        # Etiqueta modo FX activo
        if mfx == 1:
            t = fnt.render("FX AUDIO", True, (255, 160, 20))
            screen.blit(t, (24, H - 18))
        elif mfx == 2:
            t = fnt.render("FX VIDEO", True, (80, 140, 255))
            screen.blit(t, (24, H - 18))

        # Batería
        if s['drum_preset'] > 0 and self._engine:
            phase = self._engine.get_beat_phase()
            r = 4 + int(phase < 0.1) * 2
            bpm_val = AudioEngine._bpm_desde_pot(s['play_eff'][2])
            vol_pct = int(s['drum_volume'] * 100)
            pygame.draw.circle(screen, (0, 230, 90), (100, H - 14), r)
            t = fnt.render(f"{bpm_val:.0f}bpm  drum {s['drum_preset']}/{len(DRUM_PRESETS)-1}  vol:{vol_pct}%",
                           True, (0, 180, 70))
            screen.blit(t, (112, H - 18))

        # Sample info — esquina inferior derecha
        n_beats = SAMPLE_BEATS_OPTS[s['sample_beat_idx']]
        escala  = NOMBRES_ESCALAS[s['escala_idx']]
        t1 = fnt.render(f"{n_beats}beat  {escala}", True, (80, 80, 80))
        screen.blit(t1, (W - t1.get_width() - 8, H - 18))

    # ── Overlay de ayuda ──────────────────────────────────────────────────────

    def _dibujar_ayuda(self, screen, s: dict, W: int, H: int, fnt_md, fnt_sm):
        mfx   = s['modo_fx']
        color = C_MODO_FX[mfx]

        overlay = pygame.Surface((W, H)); overlay.set_alpha(190)
        overlay.fill((0, 0, 0)); screen.blit(overlay, (0, 0))

        def lbl(x, y, txt, col=(160, 160, 160)):
            screen.blit(fnt_sm.render(txt, True, col), (x, y))

        cx = W // 2; y0 = H // 2 - 150; dy = 21
        col_l = cx - 260; col_m = cx - 30; col_r = cx + 130

        nombres_fx = ["NORMAL", "FX AUDIO", "FX VIDEO"]
        t = fnt_md.render(f"Video Synth v2  —  {nombres_fx[mfx]}", True, color)
        screen.blit(t, (cx - t.get_width() // 2, y0))

        escala  = NOMBRES_ESCALAS[s['escala_idx']]
        n_beats = SAMPLE_BEATS_OPTS[s['sample_beat_idx']]
        drum_str = "OFF" if s['drum_preset'] == 0 else f"preset {s['drum_preset']}"
        t2 = fnt_sm.render(f"Escala: {escala}  |  Sample: {n_beats} beat(s)  |  Batería: {drum_str}",
                           True, (100, 200, 100))
        screen.blit(t2, (cx - t2.get_width() // 2, y0 + 22))

        pygame.draw.line(screen, (50,50,50), (col_l, y0+46), (col_r+200, y0+46), 1)

        lbl(col_l, y0+52, "     ", (90,90,90))
        lbl(col_m, y0+52, "NORMAL",   (200,200,200) if mfx==0 else (60,60,60))
        lbl(col_r, y0+52, "FX AUDIO", (255,160,20)  if mfx==1 else (60,60,60))
        lbl(col_r+100, y0+52, "FX VIDEO", (80,140,255) if mfx==2 else (60,60,60))

        rows = [
            ("POT1", "Posición del sample",  "Filtro cutoff",   "Tiles (1–4×)"),
            ("POT2", "Pitch (escala)",        "Resonancia Q",    "Hue shift"),
            ("POT3", "BPM (60–180)",          "Reverb decay",    "Ghost"),
            ("POT4", "Volumen",               "Reverb wet/dry",  "Flanger espiral"),
        ]
        for i, (pot, norm, afx, vfx) in enumerate(rows):
            y = y0 + 70 + dy * i
            lbl(col_l,   y, pot,  (120,120,120))
            lbl(col_m,   y, norm, (200,200,200) if mfx==0 else (70,70,70))
            lbl(col_r,   y, afx,  (255,160,20)  if mfx==1 else (70,70,70))
            lbl(col_r+100, y, vfx, (80,140,255) if mfx==2 else (70,70,70))

        pygame.draw.line(screen, (50,50,50), (col_l, y0+162), (col_r+200, y0+162), 1)

        btns = [
            "BTN1  Play / Pause",
            "BTN2 corto  Ciclar preset de batería  (0=off → 1–10)",
            "BTN2 hold + POT1  Volumen de batería",
            "BTN3  Ciclar FX:  Normal → FX Audio → FX Video → Normal",
            "BTN4  Siguiente escala musical",
        ]
        for i, b in enumerate(btns):
            lbl(col_l, y0 + 172 + dy * i, b)

        nota = fnt_sm.render(
            "H=ayuda  SPACE=play  D=batería(ciclar)  1-9=preset  L=largo  S=escala  E=FX  Q=salir",
            True, (50, 50, 50))
        screen.blit(nota, (cx - nota.get_width() // 2, H - 28))

    # ── Bucle principal ───────────────────────────────────────────────────────

    def run(self):
        pygame.init()
        screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
        pygame.display.set_caption("Video Synth v2")
        W, H = screen.get_size()

        self.cargar(screen)

        self._stream = sd.OutputStream(
            samplerate=SR, blocksize=BLOCK, channels=2,
            dtype='float32', callback=self._engine.callback,
        )
        self._stream.start()

        if self._ser:
            threading.Thread(target=self._thread_serial, daemon=True).start()

        dw, dh = self.disp_size; dx, dy = self.disp_off
        reloj  = pygame.time.Clock()
        t0, nf = time.time(), 0

        surf_vid      = pygame.Surface((dw, dh), 0, 24)
        fnt_md        = pygame.font.SysFont("monospace", 14, bold=True)
        fnt_sm        = pygame.font.SysFont("monospace", 12)
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
                        self.estado.toggle_play()
                    elif k == pygame.K_d:
                        self.estado.next_drum_preset()
                    elif k == pygame.K_l:
                        self.estado.ciclar_largo()
                    elif k == pygame.K_s:
                        self.estado.next_escala()
                    elif k == pygame.K_e:
                        self.estado.ciclar_fx()
                    elif k == pygame.K_h:
                        mostrar_ayuda = not mostrar_ayuda
                    elif pygame.K_1 <= k <= pygame.K_9:
                        self.estado.drum_preset = k - pygame.K_0

            s     = self.estado.snap()
            frame = self._get_frame(s)
            screen.fill((0, 0, 0))

            try:
                pygame.surfarray.blit_array(surf_vid,
                    np.ascontiguousarray(frame.transpose(1, 0, 2)))
                screen.blit(surf_vid, (dx, dy))
            except Exception:
                pass

            self._dibujar_beat_flash(screen, s, W, H)
            self._dibujar_hud(screen, s, W, H)

            if mostrar_ayuda or (s['btn'][0] and s['btn'][3]):
                self._dibujar_ayuda(screen, s, W, H, fnt_md, fnt_sm)

            pygame.display.flip()

            nf += 1
            now = time.time()
            if now - t0 >= 1.0:
                with self.estado._l:
                    self.estado.fps = nf / (now - t0)
                nf, t0 = 0, now

            reloj.tick(60)

        if self._stream:
            self._stream.stop()
            self._stream.close()
        if self._ser:
            try: self._ser.close()
            except Exception: pass
        pygame.quit()


# ── Utilidades ────────────────────────────────────────────────────────────────

def pedir_archivo() -> str | None:
    if _TK:
        root = tk.Tk(); root.withdraw(); root.attributes('-topmost', True)
        ruta = filedialog.askopenfilename(
            title="Seleccionar video",
            filetypes=[("Video", "*.mp4 *.mov *.avi *.mkv *.webm *.m4v *.flv"),
                       ("Todos", "*.*")])
        root.destroy()
        return ruta if ruta else None
    return None

def detectar_esp32() -> str | None:
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").upper()
        if any(c in desc for c in ["CP210", "CH340", "CP2102", "UART", "USB SERIAL"]):
            return p.device
    return None


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    ruta, puerto = None, None
    if len(sys.argv) >= 2: ruta   = sys.argv[1]
    if len(sys.argv) >= 3: puerto = sys.argv[2]

    if not ruta:
        ruta = pedir_archivo()
    if not ruta or not os.path.isfile(ruta):
        print("No se seleccionó archivo de video.")
        sys.exit(0)

    if not puerto:
        puerto = detectar_esp32()

    synth = VideoSynthV2(ruta, puerto)
    if puerto:
        try:
            synth._ser = serial.Serial(puerto, 115200, timeout=0.1)
            time.sleep(2)
            synth.estado.serial_ok = True
            print(f"Serial conectado: {puerto}")
        except serial.SerialException as e:
            print(f"Error serial: {e}")

    synth.run()


if __name__ == "__main__":
    main()

# Author: Oleksandr Teteria
# v1.0.2
# 13.01.2026
# Implemented and tested on Pi Pico with RP2040
# Released under the MIT license

import machine
import neopixel
import time
import array
import micropython
import _thread
import math
import adc_dma, fastfft
from neo_matrix import NeoMatrixFast

# ======================================
# Конфігурація ADC
# ======================================
SAMPLE_FREQ = 44_000  # Hz
ADC0 = 0              # (GPIO26)

# ===============================================================
# Конфігурація FFT, DSP
# ===============================================================
# лог-подібний розподіл до ~18 кГц
IND_BANDS = (1, 1, 1, 2, 2, 3, 5, 7, 9, 14, 19, 27, 39, 65, 95, 131)
# Варіант A: до ~16 кГц, більше деталізації НЧ/СЧ
#IND_BANDS = (1, 1, 1, 2, 3, 4, 5, 7, 10, 14, 21, 28, 42, 58, 81, 93)
# варіант B, до ~16 кГц, “чистіше” логарифмічне
# IND_BANDS = (1, 1, 1, 1, 1, 3, 4, 6, 9, 12, 19, 26, 38, 55, 79, 115)
# Варіант C: до 22 кГц (вся смуга до Nyquist, без k=512)
# IND_BANDS = (1, 1, 1, 2, 3, 4, 5, 7, 10, 14, 21, 30, 44, 63, 91, 214)

NUM_BAND = 16 # кількість смуг
FFT_SIZE = 1024
# Опорна потужність повномасштабного синуса (Standard AES17 Reference)
FS_RMS2 = 32767**2 / 2

# ======================================
# Буфери та синхронізація
# ======================================
M = 16

spec_front = bytearray(M)
spec_back  = bytearray(M)
max_front = bytearray(M)
max_back = bytearray(M)

lock = _thread.allocate_lock()
ready = False

# Якщо True — Core0 не блокується, а перезаписує "останній кадр" (можливі пропуски).
# Якщо False — Core0 чекає, поки Core1 забере кадр (без пропусків).
DROP_FRAMES = True

# ===============================================================
# Динамічний масштаб та шумовий поріг(в "dB над шумовим порогом")
# ===============================================================
# Поріг шуму по смугам (в "dB над шумовим порогом") для max4466
# лог-подібний розподіл до ~18 кГц
NOISE_THRESHOLD = (48, 48, 51, 51, 52, 51, 51, 51, 51, 51, 51, 51, 54, 57, 62, 69)
# Варіант A
# NOISE_THRESHOLD = (43, 44, 48, 49, 51, 53, 55, 56, 56, 57, 57, 57, 53, 60, 63, 71)
# Варіант B
# NOISE_THRESHOLD = (51, 51, 53, 55, 56, 52, 52, 51, 50, 49, 49, 49, 47, 54, 56, 63)

_scale_db = 20.0       # стартове значення 
SCALE_MIN_DB = 6.0     # не даємо масштабу впасти нижче
SCALE_DECAY_DB = 0.10  # release: на скільки dB/кадр зменшувати масштаб, якщо сигнал слабшає
HEADROOM_DB = 0.4      # “запас” зверху, щоб 16 не забивалось постійно
GAMMA = 1.8            # <1 піднімає тихі смуги, >1 “стискає” низ
_tmp_adj = array.array('f', [0.0] * NUM_BAND)  # без алокацій у кадрі


def core1_led_worker():
    global ready

    # локальні буфери Core1 (не діляться з Core0)
    spec_work = bytearray(M)
    max_work  = bytearray(M)

    while True:
        lock.acquire()
        if ready:
            spec_work[:] = spec_front
            max_work[:]  = max_front
            ready = False
            lock.release()

            nm.apply_spectrum_buf(spec_work, max_work, button_peaks_en.value())  # viper2 + write()
        else:
            lock.release()
            time.sleep_us(50)


def band_dbfs(spec, i, j):
    # вертає значення dBFS для смуги частот (для діапазону бінів [i, j[ )
    # e = сума енергій бінів у смузі (очікується, що spec[k] >= 0)
    e = 0
    for k in range(i, j):
        e += spec[k]
    
    if e <= 0:
        return -120
   
    return 10.0 * math.log10((2.0 * e) / FS_RMS2)

num_dbfs = 0
dbfs_all = [0] * 16
def build_band_spectr_test(spec, out_buf):
    # тестова версія функції
    # використовується для вимірювання шумового порогу
    global num_dbfs, dbfs_all
    ind = 1
    dbfs_l = []
    for i, s in enumerate(IND_BANDS):
        dbfs = band_dbfs(spec, ind, ind+s)
        dbfs_l.append(dbfs)
        # тут маємо піднімати шумовий поріг та масштабувати під (1...16)?
        # шумовий поріг різний для частотних смуг, залежить від NOISE_THRESHOLD
        # масштаб має бути динамічним в залежності від максимального рівня
        # нижче - просто для проби щоб побачити, що працює
        val = (round(dbfs) + 70) // 3 
        if val > 16:
            val = 16
        elif val <= 0:
            val = 1
        out_buf[i] = val
        ind += s
    
    #вимірювання шумового порогу    
    num_dbfs = (num_dbfs + 1) % 1000
    dbfs_all = [a + b for a, b in zip(dbfs_l, dbfs_all)]
    if not num_dbfs:
        dbfs_all = [round(x / 1000) for x in dbfs_all]
        print(dbfs_all)
        dbfs_all = [0] * 16
   

def build_band_spectr(spec, out_buf):
    """
    out_buf[i] = 0..16
      0  -> смуга нижче шумового порогу (вимкнено)
      1..16 -> висота стовпця

    NOISE_THRESHOLD[i] трактуємо як |noise_dbfs|, тобто noise_dbfs = -NOISE_THRESHOLD[i]
    """
    global _scale_db

    ind = 1
    peak_adj = 0.0

    # 1) Рахуємо adj_db = max(0, dbfs - noise_dbfs) = max(0, dbfs + NOISE_THRESHOLD[i])
    for i, w in enumerate(IND_BANDS):
        db = band_dbfs(spec, ind, ind + w)

        adj = db + float(NOISE_THRESHOLD[i])
        if adj < 0.0:
            adj = 0.0

        _tmp_adj[i] = adj
        if adj > peak_adj:
            peak_adj = adj

        ind += w

    # 2) Динамічний масштаб (attack/release) + headroom
    target = peak_adj + HEADROOM_DB

    if target > _scale_db:
        # attack: швидко підхоплюємо
        _scale_db = target
    else:
        # release: повільно відпускаємо
        _scale_db -= SCALE_DECAY_DB
        if _scale_db < target:
            _scale_db = target
        if _scale_db < SCALE_MIN_DB:
            _scale_db = SCALE_MIN_DB

    denom = _scale_db if _scale_db > 1e-6 else 1e-6

    # 3) Масштабування у 0..16 (з гамма-кривою)
    for i in range(NUM_BAND):
        adj = _tmp_adj[i]
        if adj <= 0.0:
            out_buf[i] = 0
            continue

        x = adj / denom
        if x > 1.0:
            x = 1.0

        # Підняти низ (GAMMA<1) без складних таблиць
        y = x ** GAMMA

        lvl = 1 + int(y * 15.0 + 0.5)   # 1..16
        if lvl > 16:
            lvl = 16
        out_buf[i] = lvl

# ---------------- Core0 main loop ----------------
def core0_main_loop():
    global ready, spec_front, spec_back, max_front, max_back

    delay_max_level = 2 # затримка спаду макимумів
    num_frame = 0

    # peak-hold стан (лише Core0)
    max_state = bytearray(M)   # 0..16

    while True:
        t0 = time.ticks_us()

        # 1) Новий спектр у spec_back
        adc_dma.start(ADC0, SAMPLE_FREQ, FFT_SIZE)
        while adc_dma.busy():
            time.sleep_us(5)
        # Коли готово — отримуємо буфер потім FFT на даних буфера
        buf, peak = adc_dma.buffer_i16('auto', 10_000)
        spectr = fastfft.rfft(buf, True) 
        adc_dma.close()
        # Будуєомо об'єднаний спектр (об'єднуючи біни за схемою IND_BANDS)
        build_band_spectr(spectr, spec_back)

        # 2) Спад 1 раз на delay_max_level кадрів (ПЕРЕД max())
        num_frame = (num_frame + 1) % delay_max_level
        if num_frame == 0:
            for j in range(M):
                if max_state[j] > 0:
                    max_state[j] -= 1

        # 3) Peak-hold: max_state = max(max_state, spec_back)
        for j in range(M):
            v = spec_back[j]
            if v > max_state[j]:
                max_state[j] = v

        # 4) Знімок стану в max_back (це max_spectr кадру)
        max_back[:] = max_state

        # 5) Публікація кадру 
        #    БЕЗ swap посилань, лише копія
        if DROP_FRAMES:
            lock.acquire()
            spec_front[:] = spec_back
            max_front[:]  = max_back
            ready = True
            lock.release()
        else:
            while True:
                lock.acquire()
                if not ready:
                    spec_front[:] = spec_back
                    max_front[:]  = max_back
                    ready = True
                    lock.release()
                    break
                lock.release()
                time.sleep_us(50)

        t1 = time.ticks_us()
        # print(time.ticks_diff(t1, t0))

# --------------------------------------
# START
# --------------------------------------
if __name__ == '__main__':
    n = 16
    m = 16
    nm = NeoMatrixFast(row=n, col=m, neo_pin=20)
    nm.clear()
    # тумблер переключення режимів відображення піків (1/0 - вкл/викл)
    button_peaks_en = machine.Pin(16, machine.Pin.IN, machine.Pin.PULL_UP)

    _thread.start_new_thread(core1_led_worker, ())
    core0_main_loop()



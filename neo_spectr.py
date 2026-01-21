# Author: Oleksandr Teteria
# v1.0.5
# 21.01.2026
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
import os


# ======================================
# Конфігурація ADC
# ======================================
SAMPLE_FREQ = 40_000  # Hz
ADC0 = 0              # (GPIO26)

# ===============================================================
# Конфігурація FFT, DSP
# ===============================================================

# для модуля MAX9814 (Gain=40dB), ≈ 39 Гц … 15.55 кГц
IND_BANDS = (2, 1, 1, 1, 1, 1, 1, 5, 6, 11, 15, 24, 35, 53, 80, 160)

NUM_BAND = 16 # кількість смуг
FFT_SIZE = 1024
# Опорна потужність повномасштабного синуса, берем за 0 dB (Standard AES17 Reference)
FS_RMS2 = 32767**2 / 2

# ======================================
# Буфери та синхронізація
# ======================================
M = 16

lock = _thread.allocate_lock()

# 1-слотовий обмін спектром (memoryview від fastfft)
spectr_front = None        # посилання на memoryview
spectr_busy  = False       # True: Core1 ще НЕ завершив роботу зі spectr_front

# ===============================================================
# Динамічний масштаб та шумовий поріг(в "dB над шумовим порогом")
# ===============================================================

NOISE_THRESHOLD = (72, 80, 81, 81, 83, 86, 86, 74, 74, 72, 71, 69, 68, 68, 66, 63)

# файл з поточним шумовим порогом (для вимірювання)
filename = 'NOISE_THRESHOLD.txt'

_scale_db = 20.0       # стартове значення 
SCALE_MIN_DB = 6.0     # не даємо масштабу впасти нижче
SCALE_DECAY_DB = 0.05  # release: на скільки dB/кадр зменшувати масштаб, якщо сигнал слабшає
HEADROOM_DB = 0.4      # “запас” зверху, щоб 16 не забивалось постійно
GAMMA = 1.8            # <1 піднімає тихі смуги, >1 “стискає” низ
# dB підсилення для кожної смуги (довжина = NUM_BAND)
BAND_GAIN_DB = (
    2, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 2, 6,
    4, 4, 6, 10
    )

_tmp_adj = array.array('f', [0.0] * NUM_BAND)  


def core1_dsp_led_worker():
    global spectr_front, spectr_busy

    # локальні буфери Core1
    spec_work = bytearray(M)
    max_work  = bytearray(M)

    # peak-hold стан (лише Core1)
    delay_max_level = 2
    num_frame = 0
    max_state = bytearray(M)

    while True:
        # --- забрати спектр (memoryview) ---
        lock.acquire()
        if spectr_busy:
            spectr = spectr_front  # локальне посилання на memoryview
            lock.release()

            # --- DSP: смуги + AGC ---
            build_band_spectr(spectr, spec_work)
            # або тест:
            # build_band_spectr_test(spectr, spec_work)

            # --- peak-hold (як було на Core0) ---
            num_frame = (num_frame + 1) % delay_max_level
            if num_frame == 0:
                for j in range(M):
                    if max_state[j] > 0:
                        max_state[j] -= 1

            for j in range(M):
                v = spec_work[j]
                if v > max_state[j]:
                    max_state[j] = v

            max_work[:] = max_state

            # --- render + np.write() ---
            nm.apply_spectrum_buf(spec_work, max_work, button_peaks_en.value())

            # --- дозволяємо Core0 робити наступний rfft ---
            lock.acquire()
            spectr_busy = False
            lock.release()

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


if filename in os.listdir():
    os.rename(filename, filename[:-4] + '_old.txt')
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
        # нижче - просто для проби щоб побачити, що працює, шумовий поріг 70
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
        dbfs_all = [-(round(x / 1000)+2) for x in dbfs_all]
        print(dbfs_all)
        with open(filename, 'a') as f:
            print(dbfs_all, file=f)
        dbfs_all = [0] * 16
           

def build_band_spectr(spec, out_buf):
    global _scale_db

    ind = 1
    peak_adj = 0.0

    # 1) adj без gain (тільки шумовий поріг)
    for i, w in enumerate(IND_BANDS):
        db = band_dbfs(spec, ind, ind + w)

        adj = db + float(NOISE_THRESHOLD[i])
        if adj < 0.0:
            adj = 0.0

        _tmp_adj[i] = adj
        if adj > peak_adj:
            peak_adj = adj

        ind += w

    # 2) Масштабування (як було)
    target = peak_adj + HEADROOM_DB
    if target > _scale_db:
        _scale_db = target
    else:
        _scale_db -= SCALE_DECAY_DB
        if _scale_db < target:
            _scale_db = target
        if _scale_db < SCALE_MIN_DB:
            _scale_db = SCALE_MIN_DB

    denom = _scale_db if _scale_db > 1e-6 else 1e-6

    # 3) Мапінг у 0..16, але з частотозалежним gain
    for i in range(NUM_BAND):
        adj = _tmp_adj[i]
        if adj <= 0.0:
            out_buf[i] = 0
            continue

        adj_eff = adj + float(BAND_GAIN_DB[i])   # <-- підсилення смуги
        if adj_eff < 0.0:
            adj_eff = 0.0

        x = adj_eff / denom
        if x > 1.0:
            x = 1.0

        y = x ** GAMMA
        lvl = 1 + int(y * 15.0 + 0.5)
        if lvl > 16:
            lvl = 16
        out_buf[i] = lvl

# ---------------- Core0 main loop ----------------
def core0_main_loop():
    global spectr_front, spectr_busy

    while True:
        t0 = time.ticks_us()
        
        # 1) Захват ADC
        adc_dma.start(ADC0, SAMPLE_FREQ, FFT_SIZE)
        while adc_dma.busy():
            time.sleep_us(5)

        # отримуємо буфер (тут важливо НЕ робити close() до завершення FFT)
        buf, peak = adc_dma.buffer_i16('auto', 10_000)

        # 2) Перед викликом rfft() чекаємо, поки Core1 завершив читання попереднього спектра
        while True:
            lock.acquire()
            busy = spectr_busy
            lock.release()
            if not busy: # Core1 вже завершив обробку
                break
            time.sleep_us(50)

        # 3) FFT (повертає memoryview на внутрішній буфер fastfft)
        spectr = fastfft.rfft(buf, True)

        # 4) Тепер можна закрити adc_dma (бо FFT вже прочитав buf)
        adc_dma.close()

        # 5) Публікація спектра для Core1
        lock.acquire()
        spectr_front = spectr
        spectr_busy = True
        lock.release()
        
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

    _thread.start_new_thread(core1_dsp_led_worker, ())
    core0_main_loop()


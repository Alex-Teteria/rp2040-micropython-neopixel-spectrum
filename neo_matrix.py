# Author: Oleksandr Teteria
# v1.0.2
# 13.01.2026
# Implemented and tested on Pi Pico with RP2040
# Released under the MIT license

import machine
import neopixel
import time
import random
import array
import micropython


class NeoMatrixFast:
    def __init__(self, row, col, neo_pin):
        self.n = row
        self.m = col

        self.np = neopixel.NeoPixel(machine.Pin(neo_pin), self.n * self.m)
        self.buf = self.np.buf  # bytearray

        # WS2812 у MicroPython NeoPixel зазвичай в порядку GRB
        def grb(rgb):
            r, g, b = rgb
            return (g, r, b)

        self.nothing = grb((0, 0, 0))
        self.red = grb((32, 0, 0))
        self.orange = grb((24, 8, 0))
        self.yellow = grb((24, 16, 0))
        self.green_yellow = grb((12, 20, 0))
        self.green = grb((0, 32, 0))
        self.blue_light = grb((0, 16, 16))
        self.color_max = grb((22, 0, 10))
        
        # офсети в buf (uint16), плоский масив: off[j*n + i] = 3*pix_index
        self.off = array.array('H', [0] * (self.m * self.n))
        for j in range(self.m):
            base = j * self.n
            for i in range(self.n):
                # "змійка" по рядках
                pix = (self.m * i + j) if (i % 2) else (self.m - j - 1 + self.m * i)
                self.off[base + i] = 3 * pix

        # шаблон зводимо до кольору по рядку -> rowgrb[n*3]
        self.rowgrb = bytearray(self.n * 3)
        for i in range(self.n):
            g, r, b = self._row_color_grb(i)
            p = 3 * i
            self.rowgrb[p] = g
            self.rowgrb[p + 1] = r
            self.rowgrb[p + 2] = b

        # багаторазові буфери спектру (щоб не алокувати щораз)
        self.spec = bytearray(self.m)
        self.maxb = bytearray(self.m)

    def _row_color_grb(self, i):
        # Зонування шаблону за кольорами (за потреби підправити)
        if 0 <= i < 3:
            return self.red
        elif 3 <= i < 6:
            return self.orange
        elif 6 <= i < 9:
            return self.yellow
        elif 9 <= i < 12:
            return self.green_yellow
        elif 12 <= i < 15:
            return self.green
        else:
            return self.blue_light

    def clear(self):
        # швидке занулення всього буфера
        self.buf[:] = b"\x00" * len(self.buf)
        self.np.write()

    @micropython.viper
    def _apply_spec_viper(self, spec_ptr):  # spec_ptr -> ptr8
        buf = ptr8(self.buf)
        off = ptr16(self.off)
        row = ptr8(self.rowgrb)

        n = int(self.n)
        m = int(self.m)

        for j in range(m):
            v = int(ptr8(spec_ptr)[j])
            if v > n:
                v = n
            cutoff = n - v
            base = j * n

            # верх: off
            for i in range(cutoff):
                o = int(off[base + i])
                buf[o] = 0
                buf[o + 1] = 0
                buf[o + 2] = 0

            # низ: pattern (колір залежить тільки від row=i)
            for i in range(cutoff, n):
                o = int(off[base + i])
                p = 3 * i
                buf[o] = row[p]
                buf[o + 1] = row[p + 1]
                buf[o + 2] = row[p + 2]

    @micropython.viper
    def _apply_spec_viper2(self, spec_ptr, max_ptr):
        buf = ptr8(self.buf)
        off = ptr16(self.off)
        row = ptr8(self.rowgrb)

        n = int(self.n)
        m = int(self.m)

        spec = ptr8(spec_ptr)
        mx   = ptr8(max_ptr)

        # color_max (GRB)
        gmx = int(self.color_max[0])
        rmx = int(self.color_max[1])
        bmx = int(self.color_max[2])

        for j in range(m):
            v = int(spec[j])
            if v > n:
                v = n
            cutoff = n - v
            base = j * n

            # верх: off
            for i in range(cutoff):
                o = int(off[base + i])
                buf[o] = 0
                buf[o + 1] = 0
                buf[o + 2] = 0

            # низ: pattern
            for i in range(cutoff, n):
                o = int(off[base + i])
                p = 3 * i
                buf[o] = row[p]
                buf[o + 1] = row[p + 1]
                buf[o + 2] = row[p + 2]

            # --- максимум: led_matrix[n - max_spectr[j]][j] = color_max, якщо max > 1 ---
            mv = int(mx[j])
            if mv > n:
                mv = n
            if mv > 1:
                r = n - mv          # row index
                o = int(off[base + r])
                buf[o] = gmx
                buf[o + 1] = rmx
                buf[o + 2] = bmx

    def apply_spectrum(self, spectrum, max_spectr):
        '''
        Виконує задачі:
        1. Уніфікація входу:
           приймає spectrum і max_spectr будь-якого типу/довжини
           (list/tuple/bytearray, коротші за m тощо) і підставляє 0,
           якщо елементів не вистачає.
        2. Клемп + копія в внутрішні bytearray:
           - клемпує значення в 0..n;
           - копіює в self.spec і self.maxb, які гарантовано viper-friendly (ptr8)
        3. Виконує рендер і вивід:
           - self._apply_spec_viper2(self.spec, self.maxb)
           - self.np.write()
           
        spectrum[j]    : 0..n (висота стовпця)
        max_spectr[j]  : 0..n (позиція піку)
        '''
        n = self.n
        Ls = len(spectrum)
        Lm = len(max_spectr)

        # clamp + копія в bytearray (viper-friendly)
        for j in range(self.m):
            v = spectrum[j] if j < Ls else 0
            if v < 0: v = 0
            elif v > n: v = n
            self.spec[j] = v

            mv = max_spectr[j] if j < Lm else 0
            if mv < 0: mv = 0
            elif mv > n: mv = n
            self.maxb[j] = mv

        self._apply_spec_viper2(self.spec, self.maxb)
        self.np.write()
    
    def apply_spectrum_buf(self, spec_buf, max_buf, show_peaks=True):
        '''
        Варіант, коли spec_buf та max_buf вже як bytearray(m) з клемпом 0..n.
        spec_buf, max_buf: bytearray length m, значення 0..n
        '''
        if show_peaks: # відображати з піками чи без (viper2 або viper)
            self._apply_spec_viper2(spec_buf, max_buf)
        else:
            self._apply_spec_viper(spec_buf)
            
        self.np.write()
        
    def apply_spectrum_timed(self, spectrum):
        t0 = time.ticks_us()
        # fill
        L = len(spectrum)
        n = self.n
        for j in range(self.m):
            v = spectrum[j] if j < L else 0
            if v < 0:
                v = 0
            elif v > n:
                v = n
            self.spec[j] = v

        self._apply_spec_viper(self.spec)
        t1 = time.ticks_us()
        # write
        self.np.write()
        t2 = time.ticks_us()

        fill_us = time.ticks_diff(t1, t0)
        write_us = time.ticks_diff(t2, t1)
        total_us = time.ticks_diff(t2, t0)
        return total_us, fill_us, write_us


class NeoMatrix:
    def __init__(self, row, col, neo_pin):
        self.n = row
        self.m = col

        self.np = neopixel.NeoPixel(machine.Pin(neo_pin), self.n * self.m)

        # Кольори 
        self.green = (0, 32, 0)
        self.green_yellow = (12, 20, 0)
        self.red = (32, 0, 0)
        self.orange = (24, 8, 0)
        self.blue_light = (0, 16, 16)
        self.yellow = (24, 16, 0)

        self.nothing = (0, 0, 0)

        # ---------- попередній розрахунок індексів ----------
        # col_pix[j][i] = neopixel index для (row=i, col=j)
        self.col_pix = []
        for j in range(self.m):
            idxs = [0] * self.n
            for i in range(self.n):
                # "змійка" по рядках 
                idxs[i] = (self.m * i + j) if (i % 2) else (self.m - j - 1 + self.m * i)
            self.col_pix.append(idxs)

        # ---------- Шаблон (по рядках) як 1D-логіка ----------
        # колір залежить тільки від row -> достатньо row_color[i]
        self.row_color = [self._row_color(i) for i in range(self.n)]

    def _row_color(self, i):
        # зонування по кольорам
        if 0 <= i < 3:
            return self.red
        elif 3 <= i < 6:
            return self.orange
        elif 6 <= i < 9:
            return self.yellow
        elif 9 <= i < 12:
            return self.green_yellow
        elif 12 <= i < 15:
            return self.green
        else:
            return self.blue_light

    def clear(self):
        # якщо NeoPixel підтримує fill()
        self.np.fill(self.nothing)
        self.np.write()

    # ---------- без 2D-матриці, пишемо напряму в self.np ----------
    def apply_spectrum(self, spectrum):
        # spectrum: список довжини m, значення 0..n (висота стовпця)
        for j in range(self.m):
            v = spectrum[j] if j < len(spectrum) else 0
            if v < 0:
                v = 0
            elif v > self.n:
                v = self.n

            cutoff = self.n - v  # rows [0..cutoff-1] -> off, [cutoff..n-1] -> pattern
            idxs = self.col_pix[j]

            # верх: вимкнути
            for i in range(cutoff):
                self.np[idxs[i]] = self.nothing

            # низ: увімкнути згідно шаблону
            for i in range(cutoff, self.n):
                self.np[idxs[i]] = self.row_color[i]

        self.np.write()

    # вимір окремо: Python-fill та np.write()
    def apply_spectrum_timed(self, spectrum):
        t0 = time.ticks_us()
        for j in range(self.m):
            v = spectrum[j] if j < len(spectrum) else 0
            if v < 0:
                v = 0
            elif v > self.n:
                v = self.n

            cutoff = self.n - v
            idxs = self.col_pix[j]

            for i in range(cutoff):
                self.np[idxs[i]] = self.nothing
            for i in range(cutoff, self.n):
                self.np[idxs[i]] = self.row_color[i]

        t1 = time.ticks_us()
        self.np.write()
        t2 = time.ticks_us()

        fill_us = time.ticks_diff(t1, t0)
        write_us = time.ticks_diff(t2, t1)
        total_us = time.ticks_diff(t2, t0)
        return total_us, fill_us, write_us


# ---------------- main loop ----------------
def main_loop():
    '''тест роботи NeoMatrixFast на випадкових числах'''
    
    num_frame = 0

    while True:
        t0 = time.ticks_us()

        #1) Новий спектр (приклад)
        for j in range(M): # для тесту
            spec[j] = random.randint(0, 16)
        
        # 2) Спад 1 раз на delay_max_level кадрів (ПЕРЕД max())
        num_frame = (num_frame + 1) % delay_max_level
        if num_frame == 0:
            for j in range(M):
                if max_state[j] > 0:
                    max_state[j] -= 1

        # 3) Peak-hold: max_state = max(max_state, spec_back)
        for j in range(M):
            v = spec[j]
            if v > max_state[j]:
                max_state[j] = v

        # viper + write()
        nm.apply_spectrum_buf(spec, max_state, button_peaks_en.value())

        t1 = time.ticks_us()
        print('Затримка:', time.ticks_diff(t1, t0), 'мкс')
        time.sleep(0.2)

# --------------------------------------
# START
# --------------------------------------
if __name__ == '__main__':
    n = 16
    m = 16
    nm = NeoMatrixFast(row=n, col=m, neo_pin=20)
    nm.clear()
    
    # Буфери 
    M = 16
    spec = bytearray(M)
    max_state = bytearray(M)

    # затримка спаду макимумів
    delay_max_level = 2
    # вхід дозволу відображення максимумів
    button_peaks_en = machine.Pin(16, machine.Pin.IN, machine.Pin.PULL_UP)
    
    # тест NeoMatrixFast
    # main_loop()
    
    # тест NeoMatrix
    nm = NeoMatrix(row=n, col=m, neo_pin=20)
    nm.clear()
    print(type(nm.np.buf), len(nm.np.buf))

    while True:
        spec = [random.randint(0, 16) for _ in range(16)]

        total_us, fill_us, write_us = nm.apply_spectrum_timed(spec)
        print("total:", total_us, "us | fill:", fill_us, "us | write:", write_us, "us")





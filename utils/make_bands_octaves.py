import math


def make_ind_bands_octaves(Fs, N, fmin, fmax, bands=16, k_start=1,
                           ignore_nyquist=False):
    """
    Розбиття [fmin..fmax] на 'bands' смуг з рівним кроком в октавах (log2-шкала).

    Повертає:
      df                 - Hz/bin
      k_edges            - фактичні межі в бінах (довжина bands+1)
      ind_bands          - кількість бінів у кожній смузі (довжина bands)
      edges_hz_real      - фактичні межі в Гц (довжина bands+1), = [k*df]
      band_ranges_hz_real- фактичні діапазони смуг у Гц: [(f_lo, f_hi), ...]
      edges_req          - "запитані" межі (лог2), для діагностики
    """
    df = Fs / N
    nyq = Fs / 2

    # обмежуємо верхню межу реальним діапазоном rFFT
    fmax = min(fmax, nyq)

    if fmin <= 0 or fmax <= fmin:
        raise ValueError("Invalid fmin/fmax")

    # log2-розбиття (запитані межі)
    total_oct = math.log2(fmax / fmin)
    step_oct = total_oct / bands
    ratio = 2.0 ** step_oct
    edges_req = [fmin * (ratio ** i) for i in range(bands + 1)]
    edges_req[-1] = fmax  # фіксуємо точно

    # фактичні межі в бінах (після round + строго зростаючі)
    k_edges = [k_start]
    k_limit = (N // 2 - 1) if ignore_nyquist else (N // 2)

    for f in edges_req[1:]:
        ki = int(round(f / df))
        if ki <= k_edges[-1]:
            ki = k_edges[-1] + 1
        if ki > k_limit:
            raise ValueError("Too many bands for given N/Fs/f-range (ran out of FFT bins)")
        k_edges.append(ki)

    ind_bands = tuple(k_edges[i + 1] - k_edges[i] for i in range(bands))

    # Реальні межі в Гц, які утворилися після квантування в біни
    edges_hz_real = [k * df for k in k_edges]
    band_ranges_hz_real = [(edges_hz_real[i], edges_hz_real[i + 1]) for i in range(bands)]

    return df, k_edges, ind_bands, edges_hz_real, band_ranges_hz_real, edges_req


# приклад:
df, k_edges, IND_BANDS, edges_hz_real, band_ranges_real, edges_req = make_ind_bands_octaves(
    40_000, 1024, 30, 14000, 15, k_start=3, ignore_nyquist=False
)

print("df =", df, "Hz/bin")
print("IND_BANDS =", IND_BANDS, "sum =", sum(IND_BANDS))
print("\nReal band ranges (Hz) from bins:")
for i, (lo, hi) in enumerate(band_ranges_real):
    print(f"{i:2d}: {lo:8.2f} .. {hi:8.2f}   (bins {k_edges[i]}..{k_edges[i+1]})")

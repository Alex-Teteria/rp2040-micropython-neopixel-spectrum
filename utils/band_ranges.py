def band_ranges_hz(Fs, N, IND_BANDS, k_start=1):
    """
    Вхід:
      Fs        - частота дискретизації, Гц
      N         - розмір FFT
      IND_BANDS - кортеж/список ширин смуг у бінах
      k_start   - стартовий бін (типово 1, щоб ігнорувати DC)

    Вихід:
      df            - Hz/bin
      k_edges       - межі смуг у бінах (довжина len(IND_BANDS)+1)
      edges_hz      - межі смуг у Гц (довжина len(IND_BANDS)+1)
      band_ranges   - список діапазонів у Гц: [(f_lo, f_hi), ...]
    """
    df = Fs / N
    k_edges = [k_start]
    for w in IND_BANDS:
        k_edges.append(k_edges[-1] + int(w))

    edges_hz = [k * df for k in k_edges]
    band_ranges = [(edges_hz[i], edges_hz[i + 1]) for i in range(len(IND_BANDS))]
    return df, k_edges, edges_hz, band_ranges


# приклад:
Fs = 40_000
N = 1024
IND_BANDS = (2, 1, 1, 1, 1, 1, 1, 5, 6, 11, 15, 24, 35, 53, 80, 160)
 
df, k_edges, edges_hz, ranges = band_ranges_hz(Fs, N, IND_BANDS, k_start=1)
print("df =", df, "Hz/bin")
print("k_edges =", k_edges)
for i, (lo, hi) in enumerate(ranges):
    print(f"{i+1:2d}: {lo:8.2f} .. {hi:8.2f} Hz   (bins [{k_edges[i]}..{k_edges[i+1]}) )")

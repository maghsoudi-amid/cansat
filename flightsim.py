import numpy as np

REC = np.dtype([('t', '<u4'), ('p', '<i4'), ('ax', '<i2'), ('ay', '<i2'), ('az', '<i2'), ('v', 'u1'), ('pad', 'u1')])

def gen_flight(seed):
    rng = np.random.default_rng(10000 + seed)
    p0 = rng.uniform(95000, 103000)
    pad_n = int(rng.integers(1500, 3000))
    asc_v = rng.uniform(6, 15); apogee = rng.uniform(600, 1200); desc_v = rng.uniform(3, 6)
    asc_n = int(apogee / asc_v * 100); desc_n = int(apogee / desc_v * 100 + 1); gnd_n = 1500
    v = np.concatenate([np.zeros(pad_n), np.full(asc_n, asc_v), np.full(desc_n, -desc_v), np.zeros(gnd_n)])
    alt = np.maximum(np.cumsum(v) * 0.01, 0.0)
    n = len(alt); i_ap = pad_n + asc_n; ar = np.arange(n)

    def idx_at(h):
        return int(np.argmax((alt <= h) & (ar > i_ap)))

    if rng.random() < 0.4:
        i = idx_at(rng.uniform(40, 70)); L = int(rng.integers(300, 800)); amp = rng.uniform(20, 50)
        alt[i:i + L] += amp * np.sin(np.pi * np.arange(L) / L)
    i_td = int(np.argmax((alt <= 0.0) & (ar > i_ap)))

    pg = p0 + rng.uniform(-20, 20) * np.minimum(ar / pad_n, 1.0)
    def p_of(a, g): return g * (1 - a / 44330.0) ** 5.2553
    press = np.rint(p_of(alt, pg) + rng.normal(0, 3, n)).astype(np.int64)
    valid = np.ones(n, np.uint8)

    for _ in range(int(rng.integers(0, 4))):
        s = int(rng.integers(400, pad_n - 100)); L = int(rng.integers(5, 26))
        press[s:s + L] -= int(rng.uniform(150, 500))
    if rng.random() < 0.7:
        s = i_ap + int(rng.integers(0, 20)); L = int(rng.integers(10, 41))
        press[s:s + L] += int(rng.uniform(800, 2000))
    hi = idx_at(120) - 200
    for _ in range(int(rng.integers(0, 3))):
        s = int(rng.integers(i_ap + 50, hi)); L = int(rng.integers(5, 151))
        press[s:s + L] = int(round(pg[s])); valid[s:s + L] = 1
    if rng.random() < 0.5:
        s = idx_at(rng.uniform(100, 140)); L = int(rng.integers(20, 81))
        press[s:s + L] = np.rint(p_of(alt[s:s + L] - 18.0, pg[s:s + L])).astype(np.int64)
    if rng.random() < 0.7:
        s = idx_at(rng.uniform(90, 200)); L = int(rng.integers(20, 201))
        press[s:s + L] = int(rng.choice([0, 101325, 12345, -1])); valid[s:s + L] = 0
    if rng.random() < 0.3:
        s = int(rng.integers(pad_n + 100, i_ap - 300)); L = int(rng.integers(20, 201))
        press[s:s + L] = int(rng.choice([0, 101325])); valid[s:s + L] = 0

    acc = np.zeros((n, 3)); acc[:, 2] = 1000
    acc += rng.normal(0, 15, (n, 3))
    acc[pad_n:i_ap] += rng.normal(0, 100, (i_ap - pad_n, 3))
    acc[i_ap:i_td, 2] += rng.normal(0, 250, i_td - i_ap)
    acc[i_ap:i_td, 0:2] += rng.normal(0, 150, (i_td - i_ap, 2))
    acc[i_td:i_td + 3] = 6000
    acc = np.clip(np.rint(acc), -6000, 6000)

    t0 = (2 ** 32 - int(rng.integers(0, n)) * 10) % 2 ** 32 if rng.random() < 0.3 else 0
    rec = np.zeros(n, REC)
    rec['t'] = ((t0 + 10 * ar) % 2 ** 32).astype(np.uint32)
    rec['p'] = np.clip(press, -2**31, 2**31 - 1); rec['v'] = valid
    rec['ax'], rec['ay'], rec['az'] = acc[:, 0], acc[:, 1], acc[:, 2]
    truth = dict(alt=alt, i_launch=pad_n, i_ap=i_ap, i_td=i_td, n=n)
    return rec.tobytes(), truth

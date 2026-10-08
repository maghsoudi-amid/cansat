import subprocess, sys, re, time
import numpy as np
from flightsim import gen_flight

NF = 40

def build(src):
    r = subprocess.run(f'gcc -std=c99 -Wall -Wextra -O2 {src} harness.c -o harness_cand -lm', shell=True, capture_output=True, text=True)
    if r.returncode: print(r.stderr[:3000]); sys.exit(1)
    subprocess.run(f'gcc -std=c99 -c {src} -o cand.o', shell=True, check=True)
    nm = subprocess.run('nm cand.o', shell=True, capture_output=True, text=True).stdout
    return [s for s in ('malloc', 'calloc', 'realloc', 'free') if re.search(r' U ' + s + r'\b', nm)]

RX = re.compile(r'^SHD,(\d+),(-?\d+),([0-3])$')

def check(data, tr):
    out = subprocess.run(['./harness_cand'], input=data, capture_output=True).stdout.decode().splitlines()
    ST, SV, BZ, T, errs = [], [], [], [], []
    N = 0
    for l in out:
        a = l.split(' ', 2)
        if a[0] == 'ST': ST.append((int(a[1]), int(a[2])))
        elif a[0] == 'SV': SV.append((int(a[1]), int(a[2])))
        elif a[0] == 'BZ': BZ.append((int(a[1]), int(a[2])))
        elif a[0] == 'T': T.append((int(a[1]), a[2]))
        elif a[0] == 'N': N = int(a[1])
        else: errs.append('telemetry buffer/format error from harness')
    alt = tr['alt']
    if N != tr['n']: errs.append('did not process all samples')
    if [s for _, s in ST] != [0, 1, 2, 3]: errs.append(f'state sequence {[s for _, s in ST]}'); return errs
    si = {s: i for i, s in ST}
    if not (tr['i_launch'] <= si[1] <= tr['i_launch'] + 1500): errs.append(f'FLIGHT state at {si[1]}, launch at {tr["i_launch"]}')
    if [s for _, s in SV] != [1000, 2000]: errs.append(f'servo changes {SV}')
    else:
        f = SV[1][0]
        if f <= tr['i_ap']: errs.append('separation before apogee')
        elif not (70.0 <= alt[f] <= 103.0): errs.append(f'separation at true altitude {alt[f]:.1f} m (allowed 70-103)')
        if si[2] != f: errs.append('SEPARATED state not set on the sample where servo moves')
    if not (tr['i_td'] - 20 <= si[3] <= tr['i_td'] + 1000): errs.append(f'LANDED at {si[3]}, touchdown at {tr["i_td"]}')
    if [(i, s) for i, s in BZ] != [(0, 0), (si[3], 1)]: errs.append('buzzer must be 0 until LANDED, then 1')
    if [i for i, _ in T] != list(range(0, tr['n'], 100)): errs.append('telemetry cadence wrong')
    else:
        stt = np.zeros(tr['n'], int)
        for i, s in ST: stt[i:] = s
        for k, (i, s) in enumerate(T):
            g = RX.match(s)
            if not g: errs.append(f'bad packet {s!r}'); break
            if int(g.group(1)) != k: errs.append('seq wrong'); break
            if int(g.group(3)) != stt[i]: errs.append(f'state field wrong at {i}'); break
            if abs(int(g.group(2)) - alt[i]) > 60: errs.append(f'agl field {g.group(2)} vs truth {alt[i]:.0f} at {i}'); break
    return errs

if __name__ == '__main__':
    src = sys.argv[1] if len(sys.argv) > 1 else 'flight_ref.c'
    nf = int(sys.argv[2]) if len(sys.argv) > 2 else NF
    heap = build(src)
    if heap: print('FAIL heap symbols used:', heap)
    failed = 0
    for seed in range(nf):
        data, tr = gen_flight(seed)
        e = check(data, tr)
        if e: failed += 1
        print(('PASS ' if not e else 'FAIL ') + f'flight {seed:02d}', '' if not e else e[:2])
    print(f'FLIGHTS FAILED: {failed} / {nf}')
    print('RESULT:', 'PASS' if failed == 0 and not heap else 'FAIL')

import subprocess, sys, json, re, glob, os, math
import numpy as np

def build(src):
    r = subprocess.run(f'gcc -std=c99 -Wall -Wextra -O2 {src} harness.c -o harness_cand -lm', shell=True, capture_output=True, text=True)
    if r.returncode: print(r.stderr); sys.exit(1)
    subprocess.run(f'gcc -std=c99 -c {src} -o cand.o', shell=True, check=True)
    nm = subprocess.run('nm cand.o', shell=True, capture_output=True, text=True).stdout
    return [s for s in ('malloc', 'calloc', 'realloc', 'free') if re.search(r' U ' + s + r'\b', nm)]

def run(name):
    out = subprocess.run(['./harness_cand', f'traces/{name}.csv'], capture_output=True, text=True).stdout.splitlines()
    ev = dict(P=[], S=[], T=[], bad=[], N=0)
    for l in out:
        a = l.split(' ', 2)
        if a[0] == 'P': ev['P'].append((int(a[1]), int(a[2])))
        elif a[0] == 'S': ev['S'].append((int(a[1]), int(a[2])))
        elif a[0] == 'T': ev['T'].append((int(a[1]), a[2]))
        elif a[0] == 'N': ev['N'] = int(a[1])
        else: ev['bad'].append(l)
    return ev

def check(name):
    m = json.load(open(f'traces/{name}.json'))
    rows = np.genfromtxt(f'traces/{name}.csv', delimiter=',', skip_header=1)
    alt, press, acc = rows[:, 6], rows[:, 1].astype(int), rows[:, 3:6].astype(int)
    ev = run(name); errs = []
    if ev['N'] != m['n']: errs.append('did not process all samples')
    if ev['bad']: errs.append('output flag errors: ' + str(ev['bad'][:2]))
    ph = [p for _, p in ev['P']]
    if ph != [0, 1, 2, 3, 4]: errs.append(f'phase sequence {ph}')
    pi = {p: i for i, p in ev['P']}
    if 1 in pi and not (m['i_launch'] <= pi[1] <= m['i_launch'] + 400): errs.append(f'ASCENT at {pi[1]}')
    if 2 in pi and not (m['i_ap'] + 1900 <= pi[2] <= m['i_ap'] + 2300): errs.append(f'DESCENT at {pi[2]}')
    fires = [i for i, s in ev['S'] if s == 2000]
    if [s for _, s in ev['S']] != [1000, 2000]: errs.append(f'servo transitions {ev["S"]}')
    if fires:
        a = alt[fires[0]]
        if not (m['fire_alt_lo'] <= a <= m['fire_alt_hi']): errs.append(f'separation at true altitude {a:.1f} m')
    if 4 in pi and not (m['i_td'] + 95 <= pi[4] <= m['i_td'] + 115): errs.append(f'LANDED at {pi[4]} (touchdown {m["i_td"]})')
    T = ev['T']; n = m['n']
    if [i for i, _ in T] != list(range(0, n, 100)): errs.append('telemetry cadence wrong')
    rx = re.compile(r'^SHD,(\d+),(\d+)\.(\d{3}),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),([0-4])$')
    prev = None
    for k, (i, s) in enumerate(T):
        g = rx.match(s)
        if not g: errs.append(f'bad packet format at {i}: {s!r}'); break
        seq, ts, tms, agl, pp, ax, ay, az, phs = g.groups()
        tr = int(ts) * 1000 + int(tms)
        if int(seq) != k: errs.append(f'seq {seq} != {k}'); break
        if (int(pp), int(ax), int(ay), int(az)) != (press[i], *acc[i]): errs.append(f'raw fields wrong at {i}'); break
        if abs(int(agl) - alt[i]) > 12: errs.append(f'agl {agl} vs truth {alt[i]:.0f} at {i}'); break
        if int(phs) >= 2:
            if prev is not None and prev >= 0 and tr != prev + 1000: errs.append(f't_rel step wrong at {i}: {prev}->{tr}'); break
            prev = tr
        else:
            if tr != 0: errs.append(f't_rel nonzero before DESCENT at {i}'); break
            prev = -1
    return errs

if __name__ == '__main__':
    src = sys.argv[1] if len(sys.argv) > 1 else 'flight_ref.c'
    heap = build(src)
    total = 0
    if heap: print('FAIL heap symbols used:', heap)
    for f in sorted(glob.glob('traces/*.json')):
        name = os.path.basename(f)[:-5]
        e = check(name)
        print(('PASS ' if not e else 'FAIL ') + name, '' if not e else e)
        total += bool(e)
    print('TOTAL FAILED TRACES:', total + bool(heap))

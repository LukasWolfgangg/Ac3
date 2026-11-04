"""tripod_check.py - does the rudder rev B stern tripod carry the cassette's side load? (30 Oct 2026)

Geometry from Hull_v2 version 21 (Rudder_A). Hull frame: x from the transom, y to starboard,
z from the design waterline; mm and N throughout.

Load: the upper bushing's side reaction, 68 N with x1.3 at the thick-root throw limit
(Mechanisms - Rudder section 6). It has two parallel paths to the hull:
  (1) the cassette itself: carbon tube + PA12-CF carrier as a cantilever off the 2.5 mm trunnion pin,
      whose two bosses on the well walls take the moment as a couple (centres 16.75 mm apart);
  (2) the tube touching the hub slot once the 0.2 mm side gap closes: hub -> three 3 mm carbon
      struts -> feet on the deck.
The struts are Euler-Bernoulli beams (pultruded carbon, E 120 GPa, G 5 GPa, assumed), the hub is
rigid, and the strut ends are bonded rigidly (best case) or pinned. The carrier's bending section
was sliced from the CAD body (I about x: 1600 mm^4 at the foot, 2181 through the tiller opening,
3077 at the socket); the material stiffnesses are assumptions, bracketed soft / nominal / stiff.

CAD note (Hull_v2 version 21): every strut is cut flush against the hub's underside and side and
against the top of its foot - butt joints of 5-10 mm^2, no sockets - so the bonded case below is
a best case the drawn joints cannot deliver.
Fixed in Hull_v2 version 22 (30 Oct 2026): the aft struts' lines pass through the tube's mid-rake
bushing point (20, 0, 80) and every strut end sits in a 9 mm deep, 3.1 mm socket (6.5 mm PA12-CF
bosses under the hub and on the feet; the feet are trimmed to the deck). Checked in CAD over rake
+-5.49 deg: no interference, tube to hub 0.20 mm (the slot), tube to the struts and bosses 2.0 mm
or more. Tripod 7.04 g (5.41 g before).
"""
import numpy as np

HUB_REF = np.array([20.0, 0.0, 80.0])
AS_DRAWN = [((8.5, 4.0, 80.0), (5.0, 75.0, 14.64)),      # aft, starboard
            ((8.5, -4.0, 80.0), (5.0, -75.0, 14.64)),    # aft, port
            ((32.0, 0.0, 80.0), (110.0, 0.0, 36.0))]     # forward, centreline
AFT_AT_X20 = [((20.0, 8.5, 80.0), (5.0, 75.0, 14.64)),   # variant: aft struts meet the hub's sides at x 20
              ((20.0, -8.5, 80.0), (5.0, -75.0, 14.64)),
              AS_DRAWN[2]]


def _aimed(pt=np.array([20.0, 0.0, 80.0]), y_sock=8.5):
    """variant: the aft struts' lines pass through the tube's mid-rake bushing point; each strut ends
    where its line reaches y = +-8.5 (in a socket boss under the hub)"""
    out = []
    for sg in (1, -1):
        foot = np.array([5.0, 75.0 * sg, 14.64])
        out.append((tuple(pt + y_sock / 75.0 * (foot - pt)), tuple(foot)))
    return out


AFT_AIMED = _aimed() + [AS_DRAWN[2]]
# as built in Hull_v2 version 22 (30 Oct 2026): rod ends at the socket bottoms (9 mm sockets at both ends)
def _v22():
    out = []
    for top, foot, s_hub in (((20.0, 0.0, 80.0), (5.0, 75.0, 14.64), 10.5), ((20.0, 0.0, 80.0), (5.0, -75.0, 14.64), 10.5),
                             ((32.0, 0.0, 80.0), (110.0, 0.0, 36.0), 6.0)):
        top, foot = np.array(top), np.array(foot)
        u = (foot - top) / np.linalg.norm(foot - top)
        out.append((tuple(top + s_hub * u), tuple(foot - 3.0 * u)))   # (18.43, +-7.83, 73.18) -> (5.45, +-72.76, 16.59); (37.23, 0, 77.05) -> (107.39, 0, 37.47)
    return out


V22 = _v22()
TUBE_X = ((10.6, 'tube aft end of slot'), (20.0, 'tube centred (no rake)'), (29.4, 'tube fwd end of slot'))
P, GAP = 68.0, 0.2
ARM = 80.0 + 14.05            # upper bushing above the trunnion pin, mm
BOSS_S = 7.5 - (-9.25)        # pin boss centres, mm
STRUT_Z = np.pi * 3.0 ** 3 / 32


def beam_k(p1, p2, E, G, d):
    """12x12 global stiffness of a round beam element"""
    A = np.pi * d * d / 4
    I = np.pi * d ** 4 / 64
    L = np.linalg.norm(np.subtract(p2, p1))
    k = np.zeros((12, 12))
    EA, GJ = E * A / L, G * 2 * I / L
    k[0, 0] = k[6, 6] = EA
    k[0, 6] = k[6, 0] = -EA
    k[3, 3] = k[9, 9] = GJ
    k[3, 9] = k[9, 3] = -GJ
    c1, c2, c3, c4 = 12 * E * I / L ** 3, 6 * E * I / L ** 2, 4 * E * I / L, 2 * E * I / L
    for v, t, sg in ((1, 5, 1), (2, 4, -1)):
        idx = [v, t, v + 6, t + 6]
        m = np.array([[c1, sg * c2, -c1, sg * c2], [sg * c2, c3, -sg * c2, c4],
                      [-c1, -sg * c2, c1, -sg * c2], [sg * c2, c4, -sg * c2, c3]])
        for a in range(4):
            for b in range(4):
                k[idx[a], idx[b]] += m[a, b]
    ex = np.subtract(p2, p1) / L
    ref = np.array([0, 0, 1.0]) if abs(ex[2]) < 0.9 else np.array([1.0, 0, 0])
    ey = np.cross(ref, ex)
    ey /= np.linalg.norm(ey)
    R = np.array([ex, ey, np.cross(ex, ey)])
    T = np.zeros((12, 12))
    for i in range(4):
        T[3 * i:3 * i + 3, 3 * i:3 * i + 3] = R
    return T.T @ k @ T


def skew(r):
    return np.array([[0, -r[2], r[1]], [r[2], 0, -r[0]], [-r[1], r[0], 0]])


def tripod(x_load, struts=AS_DRAWN, F=1.0, bonded=True, E=120e3, G=5e3, d=3.0, n=8):
    """hub stiffness in y at the tube position x_load (N/mm), and per strut the axial force (N)
    and the largest end bending moment (N mm) under F newtons there"""
    dof, elems = 6, []
    for h, f in struts:
        h, f = np.array(h), np.array(f)
        ids = []
        for i in range(n + 1):
            p = h + (f - h) * i / n
            if i == 0:
                ids.append(('hub', p, None) if bonded else ('hubpin', p, dof))
                dof += 0 if bonded else 3
            elif i == n:
                ids.append(('fixed', p, None) if bonded else ('footpin', p, dof))
                dof += 0 if bonded else 3
            else:
                ids.append(('free', p, dof))
                dof += 6
        for i in range(n):
            elems.append((ids[i], ids[i + 1]))

    def T_of(nd):
        kind, p, d0 = nd
        T = np.zeros((6, dof))
        if kind in ('hub', 'hubpin'):
            T[0:3, 0:3] = np.eye(3)
            T[0:3, 3:6] = -skew(p - HUB_REF)
            if kind == 'hub':
                T[3:6, 3:6] = np.eye(3)
            else:
                T[3:6, d0:d0 + 3] = np.eye(3)
        elif kind == 'free':
            T[:, d0:d0 + 6] = np.eye(6)
        elif kind == 'footpin':
            T[3:6, d0:d0 + 3] = np.eye(3)
        return T

    K = np.zeros((dof, dof))
    for a, b in elems:
        Tm = np.vstack([T_of(a), T_of(b)])
        K += Tm.T @ beam_k(a[1], b[1], E, G, d) @ Tm
    Tl = np.zeros((3, dof))
    Tl[:, 0:3] = np.eye(3)
    Tl[:, 3:6] = -skew(np.array([x_load, 0, 80.0]) - HUB_REF)
    u = np.linalg.solve(K + 1e-9 * np.eye(dof), Tl.T @ np.array([0, F, 0]))
    per = []
    for s in range(len(struts)):
        Mmax, N = 0.0, 0.0
        for a, b in (elems[s * n], elems[s * n + n - 1]):
            fe = beam_k(a[1], b[1], E, G, d) @ (np.vstack([T_of(a), T_of(b)]) @ u)
            ex = (b[1] - a[1]) / np.linalg.norm(b[1] - a[1])
            for mv in (fe[3:6], fe[9:12]):
                Mmax = max(Mmax, np.linalg.norm(mv - (mv @ ex) * ex))
            N = abs(fe[0:3] @ ex)
        per.append((N, Mmax))
    dy = (Tl @ u)[1]
    return (F / dy if abs(dy) > 1e-12 else float('inf')), per


def cassette(E_tube=70e3, E_pa=4.0e3, k_boss=(16e3, 30e3), parts=False):
    """lateral stiffness of the cassette top (z 80) with no hub, N/mm"""
    z_pin, z_top, z_load, z_tube0 = -14.05, 17.8, 80.0, 9.5
    I_tube = np.pi / 64 * (10 ** 4 - 8 ** 4)

    def I_car(z):                      # sliced from Cassette_carrier_PA12CF, mm^4
        if z < -8.0:
            return 1600.0
        if z < 9.5:
            return 2181.0              # tiller opening: back wall and cheeks only
        if z < 15.2:
            return 3077.0
        return 1612.0

    zs = np.linspace(z_pin, z_top, 4001)
    dz = zs[1] - zs[0]
    f_car = sum((z_load - z) ** 2 / (E_pa * I_car(z) + (E_tube * I_tube if z >= z_tube0 else 0.0)) * dz for z in zs)
    f_tube = (z_load - z_top) ** 3 / (3 * E_tube * I_tube)
    k1, k2 = k_boss                    # pin-in-boss bearing stiffness, assumed (about E x boss length)
    f_boss = (z_load - z_pin) ** 2 / (BOSS_S ** 2 * k1 * k2 / (k1 + k2))
    if parts:
        return f_car, f_tube, f_boss
    return 1.0 / (f_car + f_tube + f_boss)


def share(kc, kt):
    """tripod and cassette shares of P; the cassette alone takes the load until the 0.2 mm gap closes"""
    fc0 = min(P, kc * GAP)
    ft = (P - fc0) * kt / (kt + kc)
    return ft, P - ft


def pin(fc):
    m = fc * ARM
    return m / 1000, m / BOSS_S, m / BOSS_S / (2.5 * 4)   # Nm, N per boss, MPa in the 4 mm (starboard) boss


if __name__ == '__main__':
    kc = cassette()
    fcar, ftube, fboss = cassette(parts=True)
    tot = fcar + ftube + fboss
    print('Cassette alone, nominal (tube 70 GPa, PA12-CF 4 GPa): %.0f N/mm at the upper bushing; flexibility from the carrier %.0f%%, tube %.0f%%, pin bosses %.0f%%' % (
        kc, 100 * fcar / tot, 100 * ftube / tot, 100 * fboss / tot))
    m, b, s = pin(P)
    print('  without the tripod: %.0f N moves the tube top %.2f mm and puts %.2f Nm on the trunnion pin: %.0f N per boss, %.0f MPa bearing in the 4 mm boss' % (
        P, P / kc, m, b, s))
    for Et, Ep, kb, lab in ((55e3, 3.0e3, (8e3, 15e3), 'soft'), (100e3, 5.5e3, (30e3, 60e3), 'stiff')):
        print('  %s bracket (tube %.0f GPa, PA12-CF %.1f GPa): %.0f N/mm' % (lab, Et / 1e3, Ep / 1e3, cassette(Et, Ep, kb)))
    f1g = 1.9 * 157.5 / 94.5
    print('  everyday load: a steady 1 g turn takes about 1.9 N of rudder side force, %.1f N at the upper bushing: the tube top moves %.3f mm, the 0.2 mm gap to the hub never closes' % (
        f1g, f1g / kc))
    print()
    for lab, st in (('as drawn in version 21 (butt joints)', AS_DRAWN), ('version 21 without the forward strut', AS_DRAWN[:2]),
                    ('aft struts moved to the hub sides at x 20', AFT_AT_X20),
                    ('aft struts aimed at the tube\'s mid-rake bushing point (20, 0, 80), ends in sockets at (%.1f, +-8.5, %.1f)' % (
                        AFT_AIMED[0][0][0], AFT_AIMED[0][0][2]), AFT_AIMED),
                    ('AS BUILT, Hull_v2 version 22: aft struts aimed at (20, 0, 80), 9 mm sockets at both ends of all three', V22)):
        print('%s (bonded joints, best case):' % lab)
        print('  pinned joints, tube centred: k %.0f N/mm' % tripod(20.0, st, bonded=False)[0])
        for xl, tag in TUBE_X:
            kt = tripod(xl, st)[0]
            ft, fc = share(kc, kt)
            per = tripod(xl, st, F=ft)[1]
            m, b, s = pin(fc)
            print('  %-24s k %5.0f N/mm  tripod %4.1f N, cassette %4.1f N -> pin %.2f Nm (%3.0f N per boss, %2.0f MPa); struts axial %s N, joint moments %s N mm (rod bending up to %.0f MPa)' % (
                tag, kt, ft, fc, m, b, s, '/'.join('%.0f' % a for a, _ in per), '/'.join('%.0f' % c for _, c in per), max(c for _, c in per) / STRUT_Z))
        print('  same load at the aft struts\' apex: k %.0f N/mm' % tripod(st[0][0][0], st)[0])
        print()

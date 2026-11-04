"""29 Oct 2026: checks at the top-reach structural basis (12 kt of wind, TWA 105 deg, 22.15 kt; top_reach.py) that are not in
gimbal_revE_calcs.py: (1) the steel-to-carbon joint at 135 mm under the strut side force, from the capacities in
strength_v4.py (Foilarm_v4 sections; Foilarm_v5 keeps the same spigot, sleeve and bond); (2) the rudder.
Side force: the sail's steady side force at the top reach is 38.0 N (the heel cap binds) and the leeward strut carries it
(the rig's centre of effort is estimated within about 10 mm of the strut station, so the rudder takes almost none)."""
import io, contextlib, json, math, subprocess, os
if not os.path.exists('strength_v4_out.json'):
    with contextlib.redirect_stdout(io.StringIO()):
        import strength_v4
cap = json.load(open('strength_v4_out.json'))['B4_capacity']
F = 38.0
print('(1) steel-to-carbon joint at 135 mm, strut side force %.1f N (x1.3 in the moment, uniform over the 135 mm wet length): joint moment %.2f Nm' % (F, 1.3 * F * 0.0675))
for k, bar, lab in (('steel tab/boss yields', 3.0, 'steel tab and boss, on yield'), ('bond reaches 3x (15 MPa wet, peak 2.5x)', None, 'bond (assumed 15 MPa wet, peak 2.5x)'),
                    ('carbon sleeve reaches its allowable', 4.0, 'carbon sleeve, on the allowable'), ('steel strut at z=105 mm yields', 3.0, 'steel strut at 105 mm, on yield')):
    Fc = cap[k]['side_force_N']
    m = 3.0 * Fc / F if bar is None else Fc / F
    b = 3.0 if bar is None else bar
    print('  %-40s capacity %6.1f N at its %s -> %.2fx (bar %.0fx) %s' % (lab, Fc, 'bar' if bar is None else 'limit', m, b, 'PASS' if m >= b else 'FAIL'))
print('  (the same joint at the old 4 deg / 14.5 kt case, 31.9 N: tab %.2fx, bond %.2fx, sleeve %.2fx)' % (
    cap['steel tab/boss yields']['side_force_N'] / 31.9, 3 * cap['bond reaches 3x (15 MPa wet, peak 2.5x)']['side_force_N'] / 31.9, cap['carbon sleeve reaches its allowable']['side_force_N'] / 31.9))

with contextlib.redirect_stdout(io.StringIO()):
    import struts2
from struts2 import sect_t
print('(2) rudder: the T-foil downforce is axial in the blade; the blade and post are bent by the blade side force (steering) and by the T-foil drag')
kt=0.514444; rho=999.0
V=22.15*kt; q=0.5*rho*V*V
R=14.01; Sr=0.0036; cr=0.030
c,tc,n,Lb,Lw=34.0,0.17,4,150.0,119.0
s=sect_t(c,n,tc); allow=(373*1+186.5*(n-1))/n
Zw=s['Ith']/(c*tc/2)
print('q %.0f Pa; T-foil CL %.3f; blade 4-ply Ith %.1f mm4, Z %.1f mm3, allowable %.0f MPa' % (q, R/(q*Sr), s['Ith'], Zw, allow))
for lab, AR in (('free surface as a pressure-release plane (AR = wetted span/chord)', Lw/c), ('free surface as a wall (AR doubled, the strut model)', 2*Lw/c)):
    a=2*math.pi*AR/(AR+2)
    F1=q*(c/1000)*(Lw/1000)*a*math.radians(1)
    M1=1.3*F1*(Lb-Lw/2)              # N mm at the blade root (hull bottom), x1.3
    sig1=M1/Zw
    print('  %s: CL_alpha %.2f/rad; side force %.1f N per degree; root moment %.2f Nm per degree (x1.3); the blade reaches 4x on its allowable at %.2f deg of blade incidence' % (lab, a, F1, M1/1000, allow/4/sig1))
# T-foil strength: each half-span a cantilever from the blade, elliptic load, x1.3
t=0.0749*cr*1000; Ith=0.036*cr*1000*t**3; Z=Ith/(t/2)
M=1.3*R/2*(4*60/(3*math.pi))
print('T-foil S9033 30 mm solid: t %.2f mm, Z %.1f mm3 (Ith ~0.036 c t^3); half-span root moment %.0f N mm -> %.0f MPa (x1.3); solid carbon at the 218 MPa ply-rule allowable %.1fx, PA12-CF print ~80 MPa %.1fx' % (t, Z, M, M/Z, 218/(M/Z), 80/(M/Z)))
# rake hold about a transverse axis at the post (the whole blade rakes): T-foil drag x blade length + blade drag x its wetted centroid + T-foil pitching moment
Dt=1.503; Db=2.973; CMt=-0.0061
Mr=Dt*Lb/1000 + Db*(Lb-Lw/2)/1000 + abs(CMt)*q*Sr*cr
print('rudder rake moment at 22.15 kt (pivot at the post, 150 mm blade): %.2f Nm steady, %.2f Nm x1.3, plus %.0f mNm per mm that the T-foil quarter chord sits off the rake axis' % (Mr, 1.3*Mr, R))

import io, contextlib, json
with contextlib.redirect_stdout(io.StringIO()):
    from r250 import *
import strutopt
W=2.8; reach=0.25
print("=== steel/carbon hybrid candidates at W=%.1f reach=%.0f mm (17-4PH H1025, joint 280 mm from foil)"%(W,reach*1e3))
for cc,tcc in ((38,0.17),(40,0.18),(40,0.20)):   # 40/0.20 = chosen carbon upper arm (25 Sep 2026)
  for cs,tcs in ((20,0.20),(22,0.18),(24,0.16),(26,0.15)):
    r=hybrid(cs,tcs,280,cc,tcc,6,W,reach,MAT['17-4PH H1025'])
    # tang check: steel section thinned to fit inside carbon cavity (inner thickness)
    t_in=cc*tcc-2*6*0.2325
    tang=steel_sect(cs*0.95,min(t_in-0.2,cs*tcs)/ (cs*0.95))
    sig_t=r['Mj']*1e3*(tang['t']/2)/tang['Ith']
    print(" steel %2d x %.2f (t %.1f) + carbon %d/%.2f | twist %.2f (steel %.2f) | steel margin %.1fx | carbon root margin %.1fx | drag10 %.2f N | mass %.0f g (steel %.0f) | Mj %.1f Nm xoff %.0f | tang t %.1f mm -> margin %.1fx (cavity %.1f mm)"%(
        cs,tcs,r['ts'],cc,tcc,r['tw'],r['tw_s'],r['marg_s'],r['marg_c'],r['drag10'],r['mass'],r['m_s'],r['Mj'],r['xoff'],tang['t'],1000/sig_t,t_in))
# blade
rud=W*g*175/550
print("rudder downforce %.2f N ; structural design cant torque %.2f Nm (1.3 x (weight + rudder downforce) x reach); physical RM %.2f Nm"%(rud,1.3*(W*g+rud)*reach,W*g*reach))
for tc in (0.17,):
    for cb in (30,32,34):
        b=strutopt.blade_check(cb,tc,rud)
        print(" blade %d mm t/c %.2f:"%(cb,tc), None if b is None else (b[0],round(b[1],1),round(b[2],2), round(b[3]['P']*1e-3*0.15*b[0]*0.339*1e3,1),'g'))

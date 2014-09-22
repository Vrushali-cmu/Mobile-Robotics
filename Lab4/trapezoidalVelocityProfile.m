function uref = trapezoidalVelocityProfile(t, amax, vmax, dist, sgn)
global tf;
tf = (abs(dist)+vmax*vmax/amax)/vmax;
tramp = vmax/amax;
if (t<=0 || t>=tf)
    uref=0;
elseif (t>0 && t<=tramp)
    uref = sgn*amax*t;
elseif (t>=(tf-tramp) && t<tf)
    uref = sgn*amax*(tf-t)
else
    uref=sgn*vmax
end
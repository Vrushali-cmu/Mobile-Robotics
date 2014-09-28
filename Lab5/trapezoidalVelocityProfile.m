function uref = trapezoidalVelocityProfile(t, amax, vmax,sgn,tf,tPause)
tramp = vmax/amax;
if (t<=0 || t>=tf)
    uref=0;
elseif (t>0 && t<=tPause)
    uref = 0
elseif (t>tPause && t<=tPause+tramp)
    uref = sgn*amax*(t-tPause);
elseif (t>=(tf-tramp-tPause) && t<(tf-tPause))
    uref = sgn*amax*(tf-tPause-t)
else
    uref=sgn*vmax
end
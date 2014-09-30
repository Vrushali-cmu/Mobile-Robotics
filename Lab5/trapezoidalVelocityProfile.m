function uref = trapezoidalVelocityProfile(t, amax, vmax,sgn,tf,tPause)
tramp = vmax/amax;
if (t<=0 || t>=tf-tPause)
    uref=0;
elseif (t>0 && t<=tPause)
    uref = 0;
elseif (t>tPause && t<=tPause+tramp)
    uref = sgn*amax*(t-tPause);
elseif (t>tPause+tramp && t<=tf-tramp-tPause)
    uref = sgn*vmax;
elseif (t>=(tf-tramp-tPause) && t<(tf-tPause))
    uref = sgn*amax*(tf-tPause-t);
else
    uref=0;
end
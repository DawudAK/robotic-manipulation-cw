function [th, isReachable, debug] = ik_pos_only_between_jaws(p, phi, elbowSign, params)

x = p(1); y = p(2); z = p(3);

h         = params.h;
L1        = params.L1;
L2        = params.L2;
Le        = params.Le;
delta     = params.delta;
deltaSign = params.deltaSign;

% Base yaw
th1 = atan2(y, x);
r   = hypot(x, y);

% Wrist-centre offset in planar frame
rw = r - Le*cos(phi);
zw = z - Le*sin(phi);

% Shoulder-plane relative z
zp = zw - h;

% Planar 2R IK
c3 = (rw^2 + zp^2 - L1^2 - L2^2) / (2*L1*L2);

debug     = struct();
debug.r   = r;
debug.rw  = rw;
debug.zw  = zw;
debug.zp  = zp;
debug.c3  = c3;

if abs(c3) > 1
    th = [NaN NaN NaN NaN];
    isReachable = false;
    return;
end

s3  = elbowSign * sqrt(max(0, 1 - c3^2));
th3 = atan2(s3, c3);

beta    = atan2(zp, rw);
gamma   = atan2(L2*sin(th3), L1 + L2*cos(th3));
th2_eff = beta - gamma;

% Recover th2 from effective angle (delta kink)
th2 = th2_eff - deltaSign*delta;

% Wrist pitch from pitch constraint
th4 = phi - th2_eff - th3;

th = [th1 th2 th3 th4];
isReachable = true;

debug.th1     = th1;
debug.th2     = th2;
debug.th3     = th3;
debug.th4     = th4;
debug.beta    = beta;
debug.gamma   = gamma;
debug.th2_eff = th2_eff;

end
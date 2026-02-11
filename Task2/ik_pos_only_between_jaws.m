% ===========================
% FILE 1: ik_pos_only_between_jaws.m
% ===========================
function [th, isReachable, debug] = ik_pos_only_between_jaws(p, phi, elbowSign, params)
% Analytic IK (position-only) for an end-effector frame located between the jaws.
%
% Inputs
%   p         : [x;y;z] desired jaw-centre position in base frame (m)
%   phi       : desired gripper pitch angle in the (r,z) plane (rad)
%               phi = 0 means "forward/horizontal" in the r-direction.
%   elbowSign : +1 = elbow-up,  -1 = elbow-down (choose whichever is valid)
%   params    : struct with fields:
%               h, L1, L2, Le, delta, deltaSign
%
% Outputs
%   th         : [th1 th2 th3 th4] joint angles in your DH/joint convention (rad)
%   isReachable: true/false
%   debug      : struct with intermediate values (useful for debugging)

% Unpack
x = p(1); y = p(2); z = p(3);

h         = params.h;
L1        = params.L1;
L2        = params.L2;
Le        = params.Le;
delta     = params.delta;
deltaSign = params.deltaSign;   % your FK: -1

% 1) Base yaw
th1 = atan2(y, x);
r  = hypot(x, y);

% 2) Back off from jaw-centre to wrist-centre in planar (r,z)
%    (Position-only constraint: choose phi)
rw = r - Le*cos(phi);
zw = z - Le*sin(phi);

% remove base height (shoulder plane origin)
zp = zw - h;

% 3) Planar 2R IK to reach (rw, zp)
c3 = (rw^2 + zp^2 - L1^2 - L2^2) / (2*L1*L2);

debug = struct();
debug.r  = r;
debug.rw = rw;
debug.zw = zw;
debug.zp = zp;
debug.c3 = c3;

if abs(c3) > 1
    th = [NaN NaN NaN NaN];
    isReachable = false;
    return;
end

s3  = elbowSign * sqrt(max(0, 1 - c3^2));
th3 = atan2(s3, c3);

beta  = atan2(zp, rw);
gamma = atan2(L2*sin(th3), L1 + L2*cos(th3));
th2_eff = beta - gamma;

% 4) Undo your FK "kink" modelling:
%    FK used: th2_eff = th2 + deltaSign*delta  (in your code th_eff(2)=th2 + deltaSign*delta)
%    => th2 = th2_eff - deltaSign*delta
th2 = th2_eff - deltaSign*delta;

% 5) Wrist pitch to satisfy chosen phi:
%    phi = th2_eff + th3 + th4  => th4 = phi - th2_eff - th3
th4 = phi - th2_eff - th3;

th = [th1 th2 th3 th4];
isReachable = true;

debug.th1 = th1;
debug.th2 = th2;
debug.th3 = th3;
debug.th4 = th4;
debug.beta = beta;
debug.gamma = gamma;
debug.th2_eff = th2_eff;
end



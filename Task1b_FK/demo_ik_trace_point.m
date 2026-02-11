% ===========================
% FILE 3 (OPTIONAL): demo_ik_trace_point.m
% ===========================
% Minimal "move to target" using your FK plot helpers (no GUI).
% Requires: fk_chain_from_DH.m, plot_frames.m
%
% Edit p below, run, it plots the stickman at the IK solution.

clear; clc; close all;

% Params (same as test file)
v_sh = 0.128; x_sh = 0.024;
h  = 0.077;
L1 = hypot(v_sh, x_sh);
L2 = 0.124;
L3 = 0.126;
Lg = 0.205/2;
delta = atan2(x_sh, v_sh);
deltaSign = -1;

% Home offsets (same convention as your FK demo)
th0 = [0, pi/2 + delta, -pi/2, 0];

params = struct('h',h,'L1',L1,'L2',L2,'Le',L3+Lg,'delta',delta,'deltaSign',deltaSign);

% Target
p   = [0.18; 0.00; 0.22];
phi = 0;
[th, ok] = ik_pos_only_between_jaws(p, phi, -1, params);
if ~ok
    [th, ok] = ik_pos_only_between_jaws(p, phi, +1, params);
end
if ~ok, error('Unreachable'); end

q = th - th0;

% Build FK chain to jaw centre
th_eff = th; th_eff(2) = th_eff(2) + deltaSign*delta;
DH = [ 0,   pi/2, h,  th_eff(1);
       L1,  0,    0,  th_eff(2);
       L2,  0,    0,  th_eff(3);
       L3,  0,    0,  th_eff(4);
       Lg,  0,    0,  0 ];

[Ts, Ps] = fk_chain_from_DH(DH);

% Plot
figure('Color','w'); ax = axes; hold(ax,'on'); grid(ax,'on'); axis(ax,'equal'); view(ax,3);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
xlim([-0.25 0.25]); ylim([-0.25 0.25]); zlim([0 0.35]);

for i=1:size(Ps,1)-1
    line([Ps(i,1) Ps(i+1,1)],[Ps(i,2) Ps(i+1,2)],[Ps(i,3) Ps(i+1,3)],'LineWidth',5);
end
hF = plot_frames(Ts, 0.03); %#ok<NASGU>
plot3(p(1),p(2),p(3),'kx','MarkerSize',10,'LineWidth',2);
title(sprintf('IK demo: q=[%.3f %.3f %.3f %.3f] rad', q(1),q(2),q(3),q(4)));



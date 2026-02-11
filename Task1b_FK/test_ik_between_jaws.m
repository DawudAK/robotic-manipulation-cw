% ===========================
% FILE 2: test_ik_between_jaws.m
% ===========================
% This script:
%  1) Defines the same geometry constants you used in FK
%  2) Solves IK for a desired jaw-centre position p
%  3) Converts th -> q using your th0 (home offsets)
%  4) (Optional) Verifies by running FK and printing the resulting position

clear; clc;

%% ---- Parameters matching your FK model ----
% Shoulder "kink" geometry (spec)
v_sh = 0.128;
x_sh = 0.024;

params.h  = 0.077;
params.L1 = hypot(v_sh, x_sh);         % ~= 0.1302
params.L2 = 0.124;
params.delta = atan2(x_sh, v_sh);      % ~= 0.185
params.deltaSign = -1;                 % YOU set this in FK and it matched the diagram

% Wrist->gripper-body (spec)
L3 = 0.126;

% Jaw-centre offset approx:
% 0.205 is jaw-to-jaw "length" shown; centre is ~half from pivot plane
Lg = 0.205/2;

params.Le = L3 + Lg;                   % wrist -> jaw-centre effective

%% ---- Your desired home pose offsets ----
% Home: arm up, forearm forward, gripper forward
% We adjusted th0(2) so that th2_eff becomes pi/2 at q=0
th0 = [0,  pi/2 + params.delta,  -pi/2,  0];

%% ---- IK target and settings ----
p   = [0.14; 0.02; 0.26];     % desired jaw-centre position (m)
phi = 0;                      % "gripper forward/horizontal" in (r,z) plane

% Try elbow-down first, then elbow-up if needed
elbowChoices = [-1, +1];

ok = false;
for elbowSign = elbowChoices
    [th, isReachable, dbg] = ik_pos_only_between_jaws(p, phi, elbowSign, params);
    if isReachable
        ok = true;
        break;
    end
end

if ~ok
    error('Target not reachable with either elbow configuration. Try a closer p or different phi.');
end

q = th - th0;

disp('--- IK RESULT ---');
disp(['elbowSign = ', num2str(elbowSign)]);
disp('th (rad) ='); disp(th);
disp('q  (rad) ='); disp(q);

%% ---- Optional: FK verification (requires your fk_chain_from_DH helper) ----
% This checks the jaw-centre position produced by FK using the same modelling
doFKcheck = true;

if doFKcheck
    % Reconstruct th and apply delta modelling exactly like your FK demo
    th_eff = th;
    th_eff(2) = th_eff(2) + params.deltaSign * params.delta;

    % Standard DH chain (Craig) to jaw-centre (tool) point:
    % [a alpha d theta]
    DH = [ 0,          pi/2, params.h,  th_eff(1);
           params.L1,  0,    0,         th_eff(2);
           params.L2,  0,    0,         th_eff(3);
           L3,         0,    0,         th_eff(4);
           Lg,         0,    0,         0       ];

    [Ts, Ps] = fk_chain_from_DH(DH); %#ok<ASGLU>
    p_fk = Ps(end,:).';   % end position

    disp('--- FK CHECK ---');
    disp('p_desired ='); disp(p);
    disp('p_fk      ='); disp(p_fk);
    disp('error (m) ='); disp(p_fk - p);
    disp(['|error| = ', num2str(norm(p_fk - p)), ' m']);
end

%% ---- How to use with your GUI demo ----
% If you want to set your FK demo sliders/edits:
%   - Replace the q values in your GUI (q1..q4) with the q printed above.



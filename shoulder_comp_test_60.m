% shoulder_comp_test.m
% Tests shoulder compensation values across 3 positions to find the optimal degree
% Requires: r = RobotDriver_sync() already set up with set_comp function
%
% USAGE:
%   1. Make sure robot is connected and r is initialised
%   2. Run this script
%   3. Look at the summary table at the end to pick the best compensation

%% ---- Settings ----

r = RobotDriver_sync()
r.setup();
phi = -pi/3;   % consistent wrist angle for all tests

% Three test positions representing your actual task workspace
pos1 = [0.08,  0.00, 0.10];   % close in  — radius=0.08m, safe at -pi/3
pos2 = [0.15,  0.00, 0.10];   % mid range — radius=0.15m, safe at -pi/3
pos3 = [0.20, -0.15, 0.08];   % extended  — radius=0.25m, safe at -pi/3 only

% Compensation values to sweep (degrees)
comp_values = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0];

% Movement settings
move_time  = 3.0;   % seconds per move
settle_time = 0.8;  % seconds to wait after move before reading

%% ---- Storage ----
n = numel(comp_values);
results = zeros(n, 10);  % [comp, pos1 x/y/z offsets, pos2 x/y/z offsets, pos3 x/y/z offsets]

%% ---- Validate positions are reachable at this phi ----
fprintf('======================================\n');
fprintf(' Shoulder Compensation Test\n');
fprintf('======================================\n');
fprintf('Checking positions are reachable at phi=%.2f rad (%.1f deg)...\n', phi, phi*180/pi);

test_positions = {pos1, pos2, pos3};
pos_names = {'pos1 (close)', 'pos2 (mid)', 'pos3 (extended)'};
all_ok = true;

% Robot params needed for IK check
v_sh = 0.128; x_sh = 0.024;
ik_p.h         = 0.077;
ik_p.L1        = hypot(v_sh, x_sh);
ik_p.L2        = 0.124;
ik_p.Le        = 0.126;
ik_p.delta     = atan2(x_sh, v_sh);
ik_p.deltaSign = -1;

for chk = 1:3
    pos = test_positions{chk};
    [~, ok] = ik_check(pos, phi, ik_p);
    if ok
        fprintf('  [OK]   %s: [%.3f, %.3f, %.3f]\n', pos_names{chk}, pos(1), pos(2), pos(3));
    else
        fprintf('  [FAIL] %s: [%.3f, %.3f, %.3f] NOT REACHABLE at phi=%.1f deg\n', ...
            pos_names{chk}, pos(1), pos(2), pos(3), phi*180/pi);
        fprintf('         Run workspace_map.m to find safe positions for this phi\n');
        all_ok = false;
    end
end

if ~all_ok
    error('One or more test positions unreachable at phi=%.1f deg. Check workspace_map.m for safe alternatives.', phi*180/pi);
end

fprintf('All positions reachable. Starting sweep...\n\n');

%% ---- Safe Start ----
fprintf('Moving to safe home position first...\n');
r.move_cubic([0.15, 0.0, 0.20], phi, 4.0);
pause(1.0);

%% ---- Sweep ----
for i = 1:n
    comp = comp_values(i);
    fprintf('\n--- Testing %.1f degrees ---\n', comp);
    r.set_comp(comp);

    % ---- Position 1: Close in ----
    fprintf('  Moving to pos1 (close in)...\n');
    r.move_cubic(pos1, phi, move_time);
    pause(settle_time);
    actual1 = r.get_xyz();
    off1 = actual1 - pos1;

    % Return to safe height before next move
    r.move_cubic([pos1(1), pos1(2), 0.18], phi, 2.0);
    pause(0.5);

    % ---- Position 2: Mid range ----
    fprintf('  Moving to pos2 (mid range)...\n');
    r.move_cubic(pos2, phi, move_time);
    pause(settle_time);
    actual2 = r.get_xyz();
    off2 = actual2 - pos2;

    r.move_cubic([pos2(1), pos2(2), 0.18], phi, 2.0);
    pause(0.5);

    % ---- Position 3: Extended ----
    fprintf('  Moving to pos3 (extended)...\n');
    r.move_cubic(pos3, phi, move_time);
    pause(settle_time);
    actual3 = r.get_xyz();
    off3 = actual3 - pos3;

    r.move_cubic([0.15, 0.0, 0.20], phi, 3.0);
    pause(0.5);

    % ---- Store results ----
    results(i,:) = [comp, off1, off2, off3];

    % ---- Print per-position summary ----
    fprintf('  Pos1 (close):    X=%+.2fmm  Y=%+.2fmm  Z=%+.2fmm  |  total=%+.2fmm\n', ...
        off1(1)*1000, off1(2)*1000, off1(3)*1000, norm(off1)*1000);
    fprintf('  Pos2 (mid):      X=%+.2fmm  Y=%+.2fmm  Z=%+.2fmm  |  total=%+.2fmm\n', ...
        off2(1)*1000, off2(2)*1000, off2(3)*1000, norm(off2)*1000);
    fprintf('  Pos3 (extended): X=%+.2fmm  Y=%+.2fmm  Z=%+.2fmm  |  total=%+.2fmm\n', ...
        off3(1)*1000, off3(2)*1000, off3(3)*1000, norm(off3)*1000);
end

%% ---- Reset comp to 0 and return home ----
r.set_comp(0);
r.move_cubic([0.15, 0.0, 0.20], phi, 3.0);

%% ---- Summary Table ----
fprintf('\n\n======================================\n');
fprintf(' RESULTS SUMMARY\n');
fprintf('======================================\n');
fprintf('%-8s | %-20s | %-20s | %-20s | %-12s\n', ...
    'Comp(°)', 'Pos1 Z offset(mm)', 'Pos2 Z offset(mm)', 'Pos3 Z offset(mm)', 'Avg |error|');
fprintf('%s\n', repmat('-', 1, 85));

avg_errors = zeros(n, 1);
for i = 1:n
    comp  = results(i,1);
    z1    = results(i,4)  * 1000;   % pos1 Z offset in mm
    z2    = results(i,7)  * 1000;   % pos2 Z offset in mm
    z3    = results(i,10) * 1000;   % pos3 Z offset in mm
    avg_z = mean(abs([z1, z2, z3]));
    avg_errors(i) = avg_z;
    fprintf('%-8.1f | %-20.2f | %-20.2f | %-20.2f | %-12.2f\n', ...
        comp, z1, z2, z3, avg_z);
end

% Find best
[~, best_idx] = min(avg_errors);
best_comp = comp_values(best_idx);

fprintf('%s\n', repmat('-', 1, 85));
fprintf('\n>> RECOMMENDED compensation: %.1f degrees\n', best_comp);
fprintf('   Set S.shoulder_comp = %.1f * (pi/180) in RobotDriver_sync\n', best_comp);
fprintf('   or call r.set_comp(%.1f) at the start of your task script\n\n', best_comp);

%% ---- Plot ----
figure('Color','w','Name','Shoulder Compensation Sweep');
hold on; grid on;
plot(comp_values, results(:,4)*1000,  '-o', 'LineWidth', 2, 'DisplayName', 'Pos1 Z (close)');
plot(comp_values, results(:,7)*1000,  '-s', 'LineWidth', 2, 'DisplayName', 'Pos2 Z (mid)');
plot(comp_values, results(:,10)*1000, '-^', 'LineWidth', 2, 'DisplayName', 'Pos3 Z (extended)');
yline(0, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Zero error');
xline(best_comp, 'r--', 'LineWidth', 1.5, 'DisplayName', sprintf('Best: %.1f°', best_comp));
xlabel('Shoulder Compensation (degrees)');
ylabel('Z Offset (mm)');
title('Shoulder Compensation Test  |  phi = -pi/3');
legend('Location', 'best');


% ================================================================
% IK REACHABILITY CHECK (standalone, no driver needed)
% ================================================================
function [th, isReachable] = ik_check(p, phi, params)
    x = p(1); y = p(2); z = p(3);
    h = params.h; L1 = params.L1; L2 = params.L2;
    Le = params.Le; delta = params.delta; deltaSign = params.deltaSign;
    th1 = atan2(y, x);
    r   = hypot(x, y);
    rw  = r - Le*cos(phi);
    zw  = z - Le*sin(phi);
    zp  = zw - h;
    c3  = (rw^2 + zp^2 - L1^2 - L2^2) / (2*L1*L2);
    if abs(c3) > 1
        th = [NaN NaN NaN NaN]; isReachable = false; return;
    end
    s3      = -sqrt(max(0, 1 - c3^2));
    th3     = atan2(s3, c3);
    beta    = atan2(zp, rw);
    gamma   = atan2(L2*sin(th3), L1 + L2*cos(th3));
    th2_eff = beta - gamma;
    th2     = th2_eff - deltaSign*delta;
    th4     = phi - th2_eff - th3;
    th = [th1 th2 th3 th4];
    isReachable = true;
end
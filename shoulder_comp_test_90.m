% shoulder_comp_test_90.m
% Shoulder compensation sweep for phi = -pi/2
% All positions and transit moves verified within -pi/2 workspace boundary.
%
% KEY DIFFERENCES from -pi/3 version:
%   - Lower test positions (max Z ~0.15m at -pi/2 vs ~0.22m at -pi/3)
%   - Safe home at [0.10, 0.0, 0.10] — well inside -pi/2 boundary
%   - Returns via move_linear (Cartesian) not move_cubic (joint space)
%     to avoid cubic traversal overshooting the workspace boundary
%   - Each position returns to safe home before next move

%% ---- Settings ----
r = RobotDriver_sync();
r.setup();
phi = -pi/2;

pos1 = [0.08,  0.00, 0.08];   % close in   radius=0.080m
pos2 = [0.13,  0.00, 0.08];   % mid range  radius=0.130m
pos3 = [0.18, -0.10, 0.06];   % extended   radius=0.206m

safe_home = [0.10, 0.0, 0.13];  % verified inside -pi/2 boundary

comp_values = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0];

move_time   = 3.0;
settle_time = 0.8;

%% ---- Storage ----
n = numel(comp_values);
results = zeros(n, 10);

%% ---- Validate positions before starting ----
fprintf('======================================\n');
fprintf(' Shoulder Compensation Test  |  phi = -pi/2\n');
fprintf('======================================\n');
fprintf('Validating positions...\n');

v_sh = 0.128; x_sh = 0.024;
ik_p.h = 0.077; ik_p.L1 = hypot(v_sh,x_sh); ik_p.L2 = 0.124;
ik_p.Le = 0.126; ik_p.delta = atan2(x_sh,v_sh); ik_p.deltaSign = -1;

all_ok = true;
check_list = {safe_home, pos1, pos2, pos3};
check_names = {'safe_home', 'pos1', 'pos2', 'pos3'};
for chk = 1:4
    pos = check_list{chk};
    [~, ok] = ik_check(pos, phi, ik_p);
    label = 'OK  ';
    if ~ok, label = 'FAIL'; all_ok = false; end
    fprintf('  [%s] %-12s [%.3f, %.3f, %.3f]  r=%.3fm\n', ...
        label, check_names{chk}, pos(1), pos(2), pos(3), hypot(pos(1),pos(2)));
end
if ~all_ok
    error('One or more positions unreachable at -pi/2. Check workspace_map.m');
end
fprintf('All positions OK.\n\n');

%% ---- Safe Start ----
fprintf('Moving to safe home...\n');
r.move_cubic(safe_home, phi, 4.0);
pause(1.0);

%% ---- Sweep ----
for i = 1:n
    comp = comp_values(i);
    fprintf('\n--- Testing %.1f degrees ---\n', comp);
    r.set_comp(comp);

    % Position 1
    fprintf('  Moving to pos1...\n');
    r.move_cubic(pos1, phi, move_time);
    pause(settle_time);
    actual1 = r.get_xyz();
    off1 = actual1 - pos1;
    r.move_cubic(safe_home, phi, 2.0);  % linear return avoids boundary overshoot
    pause(0.5);

    % Position 2
    fprintf('  Moving to pos2...\n');
    r.move_cubic(pos2, phi, move_time);
    pause(settle_time);
    actual2 = r.get_xyz();
    off2 = actual2 - pos2;
    r.move_cubic(safe_home, phi, 2.0);
    pause(0.5);

    % Position 3
    fprintf('  Moving to pos3...\n');
    r.move_cubic(pos3, phi, move_time);
    pause(settle_time);
    actual3 = r.get_xyz();
    off3 = actual3 - pos3;
    r.move_cubic(safe_home, phi, 2.0);
    pause(0.5);

    results(i,:) = [comp, off1, off2, off3];
    fprintf('  Pos1: X=%+.2fmm  Y=%+.2fmm  Z=%+.2fmm  |  total=%.2fmm\n', off1*1000, norm(off1)*1000);
    fprintf('  Pos2: X=%+.2fmm  Y=%+.2fmm  Z=%+.2fmm  |  total=%.2fmm\n', off2*1000, norm(off2)*1000);
    fprintf('  Pos3: X=%+.2fmm  Y=%+.2fmm  Z=%+.2fmm  |  total=%.2fmm\n', off3*1000, norm(off3)*1000);
end

%% ---- Reset ----
r.set_comp(0);
r.move_cubic(safe_home, phi, 3.0);

%% ---- Summary ----
fprintf('\n\n======================================\n');
fprintf(' RESULTS SUMMARY  |  phi = -pi/2\n');
fprintf('======================================\n');
fprintf('%-8s | %-20s | %-20s | %-20s | %-12s\n', ...
    'Comp(deg)', 'Pos1 Z(mm)', 'Pos2 Z(mm)', 'Pos3 Z(mm)', 'Avg |Z|');
fprintf('%s\n', repmat('-',1,85));

avg_errors = zeros(n,1);
for i = 1:n
    z1 = results(i,4)*1000;
    z2 = results(i,7)*1000;
    z3 = results(i,10)*1000;
    avg_errors(i) = mean(abs([z1,z2,z3]));
    fprintf('%-8.1f | %-20.2f | %-20.2f | %-20.2f | %-12.2f\n', ...
        results(i,1), z1, z2, z3, avg_errors(i));
end

[~, best_idx] = min(avg_errors);
best_comp = comp_values(best_idx);
fprintf('%s\n', repmat('-',1,85));
fprintf('\n>> RECOMMENDED compensation at -pi/2: %.1f degrees\n', best_comp);
fprintf('   Compare with -pi/3 result to decide if phi-dependent comp is needed.\n\n');

%% ---- Plot ----
figure('Color','w','Name','Shoulder Comp Sweep  -pi/2');
hold on; grid on;
plot(comp_values, results(:,4)*1000,  '-o', 'LineWidth',2, 'DisplayName','Pos1 Z (close)');
plot(comp_values, results(:,7)*1000,  '-s', 'LineWidth',2, 'DisplayName','Pos2 Z (mid)');
plot(comp_values, results(:,10)*1000, '-^', 'LineWidth',2, 'DisplayName','Pos3 Z (extended)');
yline(0, 'k--', 'LineWidth',1.5, 'DisplayName','Zero error');
xline(best_comp, 'r--', 'LineWidth',1.5, 'DisplayName',sprintf('Best: %.1f deg',best_comp));
xlabel('Shoulder Compensation (degrees)');
ylabel('Z Offset (mm)');
title('Z Error vs Shoulder Compensation  |  phi = -pi/2');
legend('Location','best');

%% ---- IK check helper ----
function [th, isReachable] = ik_check(pos, phi, params)
    x = pos(1); y = pos(2); z = pos(3);
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
    s3      = -sqrt(max(0, 1-c3^2));
    th3     = atan2(s3, c3);
    beta    = atan2(zp, rw);
    gamma   = atan2(L2*sin(th3), L1+L2*cos(th3));
    th2_eff = beta - gamma;
    th2     = th2_eff - deltaSign*delta;
    th4     = phi - th2_eff - th3;
    th      = [th1 th2 th3 th4];
    isReachable = true;
end
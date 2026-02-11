function robot_ik_move_slow(S, p, phi, duration_s)
% p: 3x1 jaw-centre target (m)
% phi: pitch constraint (rad)
% duration_s: move time

assert(all(size(p)==[3 1]));

% ---- Build params to match your model ----
params = build_ik_params();

% ---- Solve IK (try elbow-down then up) ----
ok = false;
for elbowSign = [-1, +1]
    [th, reachable] = ik_pos_only_between_jaws(p, phi, elbowSign, params);
    if reachable
        ok = true;
        break;
    end
end
assert(ok, "IK unreachable for p=[%.3f %.3f %.3f]", p(1),p(2),p(3));

% ---- Map model th -> "relative q" around your chosen th0 ----
% This is your sim convention:
th0 = [0,  pi/2 + params.delta,  -pi/2,  0];
q_target_rel = th - th0;  % relative to your defined home

% ---- Safety: limit how much we move in one go ----
maxStep = deg2rad(10); % per joint per command (start conservative)
q_target_rel = max(min(q_target_rel, maxStep), -maxStep);

% ---- Execute as relative move ----
robot_fk_slow_relative(S, q_target_rel, duration_s);

end

function params = build_ik_params()
v_sh = 0.128; x_sh = 0.024;

params.h  = 0.077;
params.L1 = hypot(v_sh, x_sh);
params.L2 = 0.124;
params.delta = atan2(x_sh, v_sh);
params.deltaSign = -1;

L3 = 0.126;
Lg = 0.205/2;
params.Le = L3 + Lg;
end

% -- Setup ---
%clear all;
r = VirtualRobotDriver();
r.setup();
r.set_view([-0.1, 0.4], [-0.3, 0.3], [-0.1, 0.4], 35, 20);

% --- The Function Call ---
r.move_cubic([0.22, 0.0, 0.22], 0);
do_slalom_task(r);

% --- Cleanup ---
disp('Mission Complete. Returning to standby...');


function do_slalom_task(robot)
    disp('--- Starting Slalom Task ---');
    phi_transit = -pi/4;

    % =========================================================
    % DEMO DAY CALIBRATION 
    % =========================================================
    % For each value: command arm above position, inch down until
    % centred, read r.get_xyz() and fill in here.
    %
    % gate axis: 'X' = arm travels through gate in X direction
    %            'Y' = arm travels through gate in Y direction
    %            Look at which way the gate posts face:
    %              posts on left/right sides  -> travel in Y
    %              posts on front/back sides  -> travel in X
    %
    % gate dir:  +1 or -1, the sign of travel direction through gate
    %              e.g. 'Y' dir +1 = travelling in +Y direction
    %                   'X' dir -1 = travelling in -X direction

    tool_pick_xyz   = [0.067, -0.171, 0.054]; % <-- MEASURE ON DEMO DAY

g(1).centre = [0.125, -0.125];  g(1).axis = 'Y';  g(1).dir = +1;
g(2).centre = [0.200,  0.050];  g(2).axis = 'Y';  g(2).dir = +1;
g(3).centre = [0.200,  0.050];  g(3).axis = 'Y';  g(3).dir = +1;
g(4).centre = [0.200,  0.150];  g(4).axis = 'X';  g(4).dir = -1;
    
    % =========================================================
    % END OF DEMO DAY CALIBRATION BLOCK
    % =========================================================

    % Settings
    gate_z  = 0.06;   % cruising height: must clear board AND fit under gate crossbar
    ap      = 0.05;   % 50mm approach/exit distance either side of gate centre
    max_spd = 0.06;   % max Cartesian speed through gates (m/s)
    min_T   = 1.0;    % minimum move time (s)

    % =========================================================
    % STEP 1: OPEN GRIPPER
    % =========================================================
    robot.gripper(true);
    pause(1.0);

    % =========================================================
    % STEP 2: TRANSIT TO HOVER ABOVE TOOL
    % =========================================================
    robot.move_cubic([tool_pick_xyz(1), tool_pick_xyz(2), 0.15], phi_transit);
    pause(0.3);

    % =========================================================
    % STEP 3: DESCEND AND PICK
    % =========================================================
    robot.move_linear(tool_pick_xyz, phi_transit, 2.0);
    robot.gripper(false);
    pause(1.0);

    % =========================================================
    % STEP 4: LIFT TO GATE CRUISING HEIGHT
    % =========================================================
    robot.move_linear([tool_pick_xyz(1), tool_pick_xyz(2), gate_z], phi_transit, 1.5);
    pause(0.5);  % critical settle before entering slalom

    % =========================================================
    % BUILD SLALOM PATH FROM GATE DEFINITIONS
    % =========================================================
    % For each gate, generates: approach -> centre -> exit
    % based on its axis and direction of travel.
    % Consecutive gates are connected with axis-aligned corner
    % waypoints (X move then Y move) to prevent diagonal paths
    % near gate posts.

    n_gates = numel(g);

    % Generate 3 waypoints per gate
    gate_wps = cell(n_gates, 1);
    for i = 1:n_gates
        gate_wps{i} = make_gate_waypoints(g(i), ap, gate_z);
    end

    % Assemble full path with corners between gates
    slalom_path = gate_wps{1};
    for i = 2:n_gates
        exit_wp  = gate_wps{i-1}(end, :);
        entry_wp = gate_wps{i}(1, :);
        corners  = make_corner(exit_wp, entry_wp, g(i), gate_z, ap);
        slalom_path = [slalom_path; corners; gate_wps{i}]; %#ok<AGROW>
    end

    fprintf('\n  Path: %d waypoints across %d gates\n\n', size(slalom_path,1), n_gates);

    % =========================================================
    % EXECUTE SLALOM WITH DISTANCE-SCALED TIMING
    % =========================================================
    p_prev = [tool_pick_xyz(1), tool_pick_xyz(2), gate_z];

    for i = 1:size(slalom_path, 1)
        d = norm(slalom_path(i,:) - p_prev);
        T = max(min_T, d / max_spd);
        fprintf('  WP %2d: [%5.3f, %6.3f, %5.3f]  dist=%4.1fcm  T=%.1fs\n', ...
            i, slalom_path(i,1), slalom_path(i,2), slalom_path(i,3), d*100, T);
        robot.move_linear(slalom_path(i,:), phi_transit, T);
        pause(0.2);
        p_prev = slalom_path(i,:);
    end

    % =========================================================
    % DROP TOOL AND RETREAT
    % =========================================================
    disp('Releasing tool...');
    robot.gripper(true);
    pause(1.0);

    robot.move_linear([slalom_path(end,1), slalom_path(end,2), 0.18], phi_transit, 2.0);
    pause(0.3);
    robot.move_cubic([0.2, 0.0, 0.20], phi_transit);

    disp('--- Slalom Task Complete ---');
end


% =========================================================
% HELPER: Generate approach->centre->exit for one gate
% =========================================================
function wps = make_gate_waypoints(gate, ap, gate_z)
    cx = gate.centre(1);
    cy = gate.centre(2);
    d  = gate.dir;

    if strcmp(gate.axis, 'Y')
        % Posts face left/right — travel in Y direction
        wps = [
            cx, cy - d*ap, gate_z;   % approach
            cx, cy,        gate_z;   % centre
            cx, cy + d*ap, gate_z;   % exit
        ];
    else
        % Posts face forward/back — travel in X direction
        wps = [
            cx - d*ap, cy, gate_z;   % approach
            cx,        cy, gate_z;   % centre
            cx + d*ap, cy, gate_z;   % exit
        ];
    end
end


% =========================================================
% HELPER: Axis-aligned corner between two waypoints
% =========================================================
% Generates intermediate waypoints so the arm always arrives
% at each gate approach travelling along the gate's travel axis.
%
% For axis=Y gate: arm must arrive in Y direction
%   - If from_wp is on the gate centre Y line: move Y first to
%     approach_y (safe side), then X to gate_x
%   - Otherwise: move X first to gate_x, then Y to approach_y
%
% For axis=X gate: mirror logic with X and Y swapped
%
% Duplicate waypoints are filtered out automatically.
% This logic is general for any gate position, orientation, or direction.
function corners = make_corner(from_wp, to_wp, dest_gate, gate_z, ap)
    tol = 1e-6;
    pts = {};

    if strcmp(dest_gate.axis, 'Y')
        gate_x    = dest_gate.centre(1);
        gate_y    = dest_gate.centre(2);
        d         = dest_gate.dir;
        approach_y = gate_y - d*ap;

        need_x = abs(gate_x    - from_wp(1)) > tol;
        need_y = abs(approach_y - from_wp(2)) > tol;

        if ~need_x && ~need_y
            corners = zeros(0,3);
            return;
        end

        % If from_wp Y is on the gate centre line, moving X first would
        % put us on the gate centre — move Y first instead
        from_on_gate_line = abs(from_wp(2) - gate_y) < tol;

        if from_on_gate_line && need_x && need_y
            % Y first to approach_y, then X to gate_x
            p1 = [from_wp(1), approach_y, gate_z];
            p2 = [gate_x,     approach_y, gate_z];
            if ~pts_equal(p1, from_wp) && ~pts_equal(p1, to_wp)
                pts{end+1} = p1;
            end
            if ~pts_equal(p2, to_wp) && (isempty(pts) || ~pts_equal(p2, pts{end}))
                pts{end+1} = p2;
            end
        else
            % X first to gate_x (safe — from_y is not on gate line)
            % then Y to approach_y
            if need_x
                p1 = [gate_x, from_wp(2), gate_z];
                if ~pts_equal(p1, from_wp) && ~pts_equal(p1, to_wp)
                    pts{end+1} = p1;
                end
            end
            if need_y
                p2 = [gate_x, approach_y, gate_z];
                if ~pts_equal(p2, to_wp) && (isempty(pts) || ~pts_equal(p2, pts{end}))
                    pts{end+1} = p2;
                end
            end
        end

    else  % axis = 'X'
        gate_x    = dest_gate.centre(1);
        gate_y    = dest_gate.centre(2);
        d         = dest_gate.dir;
        approach_x = gate_x - d*ap;

        need_x = abs(approach_x - from_wp(1)) > tol;
        need_y = abs(gate_y     - from_wp(2)) > tol;

        if ~need_x && ~need_y
            corners = zeros(0,3);
            return;
        end

        from_on_gate_line = abs(from_wp(1) - gate_x) < tol;

        if from_on_gate_line && need_x && need_y
            p1 = [approach_x, from_wp(2), gate_z];
            p2 = [approach_x, gate_y,     gate_z];
            if ~pts_equal(p1, from_wp) && ~pts_equal(p1, to_wp)
                pts{end+1} = p1;
            end
            if ~pts_equal(p2, to_wp) && (isempty(pts) || ~pts_equal(p2, pts{end}))
                pts{end+1} = p2;
            end
        else
            if need_y
                p1 = [from_wp(1), gate_y, gate_z];
                if ~pts_equal(p1, from_wp) && ~pts_equal(p1, to_wp)
                    pts{end+1} = p1;
                end
            end
            if need_x
                p2 = [approach_x, gate_y, gate_z];
                if ~pts_equal(p2, to_wp) && (isempty(pts) || ~pts_equal(p2, pts{end}))
                    pts{end+1} = p2;
                end
            end
        end
    end

    if isempty(pts)
        corners = zeros(0,3);
    else
        corners = cell2mat(cellfun(@(p) p, pts, 'UniformOutput', false)');
    end
end


% =========================================================
% HELPER: Check if two 3D points are equal within tolerance
% =========================================================
function eq = pts_equal(a, b)
    tol = 1e-6;
    eq = all(abs(a - b) < tol);
end
if ~exist('robot','var') || ~isfield(robot,'read')
    robot = RobotDriver_sync();
    robot.setup();
    pause(1);
else
    fprintf('[CONNECT] Using existing robot connection.\n');
end

do_slalom_task(robot);




function do_slalom_task(robot)

    phi_transit = -1.13;
    tool_pick_xyz = [-0.045, -0.19, 0.0781];
    g(1).centre = [0.0943, -0.1081];  g(1).axis = 'Y';  g(1).dir = +1;  g(1).z = 0.092;
    g(2).centre = [0.2163, -0.0415];  g(2).axis = 'Y';  g(2).dir = +1;  g(2).z = 0.131;
    g(3).centre = [0.161,    0.0481];  g(3).axis = 'Y';  g(3).dir = +1;  g(3).z = 0.116;
    g(4).centre = [0.0882,  0.0952];  g(4).axis = 'X';  g(4).dir = -1;  g(4).z = 0.09;
    
    
    ap      = 0.025;
    max_spd = 0.22;
    min_T   = 0.15;

    robot.gripper(true);
    pause(1.0);

    % move slightly above tool
    robot.move_cubic([tool_pick_xyz(1), tool_pick_xyz(2), 0.15], phi_transit);
    pause(0.3);

    % grip tool
    robot.move_linear(tool_pick_xyz, phi_transit, 0.7);
    robot.gripper(false);
    pause(0.2);

    % lift tool above and move to start pos
    robot.move_linear([tool_pick_xyz(1), tool_pick_xyz(2), 0.15], phi_transit, 0.7);

    
    
    robot.move_linear([g(1).centre(1), tool_pick_xyz(2), g(1).z], phi_transit, 0.7);
    
    n_gates = numel(g);

    gate_wps = cell(n_gates, 1);
    for i = 1:n_gates
        gate_wps{i} = make_gate_waypoints(g(i), ap, g(i).z);
    end

    slalom_path = gate_wps{1};
    for i = 2:n_gates
        exit_wp  = gate_wps{i-1}(end, :);
        entry_wp = gate_wps{i}(1, :);
        corners  = make_corner(exit_wp, entry_wp, g(i), g(i).z, ap);
        slalom_path = [slalom_path; corners; gate_wps{i}];
    end

    fprintf('\n  Path: %d waypoints across %d gates\n\n', size(slalom_path,1), n_gates);

    % slalom waypoints
    p_prev = [g(1).centre(1), tool_pick_xyz(2), g(1).z];

    for i = 1:size(slalom_path, 1)
        d = norm(slalom_path(i,:) - p_prev);
        T = max(min_T, d / max_spd);
        fprintf('  WP %2d: [%5.3f, %6.3f, %5.3f]  dist=%4.1fcm  T=%.1fs\n', ...
            i, slalom_path(i,1), slalom_path(i,2), slalom_path(i,3), d*100, T);
        robot.move_linear(slalom_path(i,:), phi_transit, T);
        %pause(0.05);  paues not needed
        p_prev = slalom_path(i,:);
    end

   
    robot.gripper(true);
    pause(1.0);
end


% make waypoitn function
function wps = make_gate_waypoints(gate, ap, gate_z)
    cx = gate.centre(1);
    cy = gate.centre(2);
    d  = gate.dir;

    if strcmp(gate.axis, 'Y')
        wps = [cx, cy + d*ap, gate_z];  % ues exits
    else
        wps = [cx + d*ap, cy, gate_z];  % exit only
    end
end




function corners = make_corner(from_wp, to_wp, dest_gate, gate_z, ap)
    gate_x = dest_gate.centre(1);
    gate_y = dest_gate.centre(2);

    if strcmp(dest_gate.axis, 'Y')
        % align X 
        c = [gate_x, from_wp(2), gate_z];
        if pts_equal(c, from_wp) || pts_equal(c, to_wp)
            corners = zeros(0,3);
        else
            corners = c;
        end
    else
        % align Y
        c = [from_wp(1), gate_y, gate_z];
        if pts_equal(c, from_wp) || pts_equal(c, to_wp)
            corners = zeros(0,3);
        else
            corners = c;
        end
    end
end


function eq = pts_equal(a, b)
    tol = 1e-6;
    eq = all(abs(a - b) < tol);
end
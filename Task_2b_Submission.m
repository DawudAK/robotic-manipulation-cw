if ~exist('robot','var') || ~isfield(robot,'read')
    robot = RobotDriver_sync();
    robot.setup();
    pause(1);
else
    fprintf('[CONNECT] Using existing robot connection.\n');
end

robot.torque(true);
robot.gripper(true);


box1_dest     = [-0.0029, 0.1271, 0.0487];
box1_start_v1 = [0.068, -0.1804, 0.0506];
box2_start    = [0.1591, -0.1601, 0.0631];
box2_dest     = [0.1371,  0.1392, 0.065];
high_transit  = [0.1232,  0.0,    0.1433];

box_1_rotate_end = [0.0662, -0.1815,  0.055];
box2_end         = [0.1565, -0.1579, 0.052];

box1_dest_60  = [-0.00033, 0.13,  0.0465];
box2_dest_60  = [ 0.1312,  0.1356, 0.0477];

stack_pos1    = [0.076,  0.0012, 0.0608];
stack_pos2    = [0.075, 0.0016, 0.080];


rotate_block(robot, box1_start_v1, box_1_rotate_end);
rotate_block(robot, box1_start_v1, box_1_rotate_end);
rotate_block(robot, box1_start_v1, box_1_rotate_end);
rotate_block(robot, box2_start,    box2_end);
rotate_block(robot, box2_start,    box2_end);


do_pick_and_place_v2(robot, box1_start_v1, box1_dest_60, -pi/3, high_transit);
robot.move_cubic(high_transit, -pi/3);
do_pick_and_place_v2(robot, box2_start, box2_dest_60, -pi/3, high_transit);

do_bridge_task(robot);

% Stacking
do_pick_and_place_v2(robot, box1_dest, stack_pos1, -pi/2);
do_pick_and_place_v2(robot, box2_dest, stack_pos2, -pi/2);


function rotate_block(r, pick_xyz, drop_xyz)
    phi_pick = -60 * (pi/180);
    phi_drop =  0  * (pi/180);

    hover = pick_xyz;
    hover(3) = pick_xyz(3) + 0.0729;

    r.gripper(true);
    %pause(0.5);
    r.move_cubic(hover, phi_pick);
    r.move_linear(pick_xyz, phi_pick, 0.5);  % was 5.0
    r.gripper(false);
    %pause(0.5);                               % was 1.0

    r.move_linear([pick_xyz(1), pick_xyz(2), pick_xyz(3)+0.07], phi_pick, 1.0);
    r.move_cubic(hover, phi_drop);
    r.move_linear(drop_xyz, phi_drop, 1.0);
    r.gripper(true);
    %pause(0.5);                               % was 1.0
    r.move_linear(hover, phi_drop, 1.0);
end

function do_bridge_task(robot)
    phi_flat = 0;
    phi_prep = -pi/2;

    robot.gripper(true);
    %pause(0.5);                               % was 1.0

    robot.move_cubic([0.165, 0.0, 0.15], phi_flat, 1.5);

    bridge_height = 0.066;
    robot.move_linear([0.165, 0.0, bridge_height], phi_flat, 0.8);   % was 1.0
    robot.move_cubic([0.1974,  0.0, bridge_height],  phi_flat, 1.0);

    robot.gripper(false);
    %pause(0.2);                               % was 1.0

    friction_height = bridge_height + 0.004;
    robot.move_linear([0.1974, 0.0, friction_height], phi_flat, 1.5); % was 2.0
    %pause(0.2);                               % was 1.0

    robot.move_cubic([0.16, 0.0, friction_height], phi_flat, 1.5);   % was 2.0
    %pause(0.2);                               % was 1.0

    robot.move_linear([0.16, 0.0, 0.18], phi_flat, 1.5);             % was 2.0
    %pause(0.5);                               % was 1.0

    robot.move_cubic([0.010, 0.0, 0.180], phi_prep, 2.0);            % was 2.5
    %pause(0.3);                               % was 0.5

    robot.move_cubic([0.1, 0.0009, 0.039], phi_prep, 1.0);
    %pause(0.5);                               % was 1.0

    %robot.move_linear([0.0966, -0.0011, 0.0423], phi_prep, 1.0);

    robot.gripper(true);
    pause(0.7);                               % was 1.0

    robot.move_cubic([0.0739, 0.0006, 0.1000], phi_prep);
end

function do_pick_and_place_v2(robot, pick_xyz, place_xyz, phi, via_xyz)
    hover_offset = 0.03;

    hover_pick        = pick_xyz;
    hover_pick(3)     = hover_pick(3)  + hover_offset;
    hover_place       = place_xyz;
    hover_place(3)    = hover_place(3) + hover_offset;

    robot.gripper(true);
    %pause(0.2);                               % was 0.5

    robot.move_cubic(hover_pick, phi);
    robot.move_linear(pick_xyz, phi, 0.5);

    robot.gripper(false);
    %pause(0.2);                               % was 1.0

    robot.move_linear(hover_pick, phi, 0.8);  % was 1.0

    if nargin >= 5 && ~isempty(via_xyz)
        robot.move_cubic(via_xyz, phi, 1.5);  % was 2.0
    end

    robot.move_cubic(hover_place, phi);
    robot.move_linear(place_xyz, phi, 0.8);   % was 1.0
    %pause(0.3);                               % was 0.5

    robot.gripper(true);
    %pause(0.1);                               % was 0.5

    robot.move_linear(hover_place, phi, 0.8); % was 1.0
end
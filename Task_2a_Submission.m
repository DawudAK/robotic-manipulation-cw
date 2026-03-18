if ~exist('r', 'var')
    disp('First time running: Connecting to robot...');
    r = RobotDriver_sync();
    r.setup();
else
    disp('Robot is already connected! Skipping setup.');
end

r.torque(1)
r.gripper(true);


% # put one cm above actual recorded z for safety
box1_start = [0.0671,  -0.1712, 0.0541];
box1_dest  = [-0.0019, 0.1228, 0.0483]; 

box2_start = [0.166,  -0.1619, 0.0592];
box2_dest  = [0.1371, 0.1392, 0.0576]; 

box3_start = [0.198, 0.0, 0.0477];
box3_dest  = [0.08, 0, 0.035]; 


do_pick_and_place(r, box1_start, box1_dest);
do_pick_and_place(r, box2_start, box2_dest);
do_pick_and_place(r, box3_start, box3_dest);

function do_pick_and_place(robot, pick_xyz, place_xyz)
    hover_offset = 0.08; % 8cm hover
    phi = -pi/3;             
   
    hover_pick = pick_xyz;
    hover_pick(3) = hover_pick(3) + hover_offset; 
    
    hover_place = place_xyz;
    hover_place(3) = hover_place(3) + hover_offset;
    
    % Move to hover
    robot.move_cubic(hover_pick, phi);
    
    %
    robot.move_linear(pick_xyz, phi, 0.5);

   
    robot.gripper(false); 
    
    robot.move_cubic(hover_pick, phi, 1.0);

    % go to next position
    robot.move_cubic(hover_place, phi);

    robot.move_linear(place_xyz, phi, 0.8);

    robot.gripper(true);
    robot.move_linear(hover_place, phi, 0.8);
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




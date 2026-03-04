%if ~exist('r', 'var')
 %   disp('First time running: Connecting to robot...');
  %  r = RobotDriver_sync();
   % r.setup();
%else
 %   disp('Robot is already connected! Skipping setup.');
%end


r = RobotDriver_sync();
r.setup();

r.torque(true);
r.gripper(true);
% # put one cm above actual recorded z for safety
%box1_start = [0.0671,  -0.1712, 0.0541];
%box1_dest  = [-0.0019, 0.1228, 0.0483]; 
box1_dest  = [-0.0029, 0.1271, 0.0487]; % alternatice to fix task 2b

box1_start_v1 = [0.072,-0.1849,0.0506];

box2_start = [0.1671,  -0.1641, 0.0671];
box2_end = [0.0637,  -0.1704, 0.0573];

box2_dest  = [0.1371, 0.1392, 0.068];



high_transit = [0.1232, 0, 0.1433];

%Rotating Distances
box1_start_v1;
box_1_rotate_end = [0.0692,-0.1845,0.055];
box2_start;
box2_end = [0.1592,-0.1611,0.0422]


rotate_block(r,box1_start_v1,box_1_rotate_end)
rotate_block(r,box1_start_v1,box_1_rotate_end)
rotate_block(r,box1_start_v1,box_1_rotate_end)
rotate_block(r,box2_start,box2_end)
rotate_block(r,box2_start,box2_end)

%r.move_cubic([0.22,0,0.22],0)

%disp('--- Starting Task A: Pick and Place ---');box2_

% Make sure gripper is open before we start doing anything
box1_dest_60 = [-0.0050, 0.1196, 0.0459];
box2_dest_60 = [0.133, 0.1431, 0.06];

do_pick_and_place_v2(r, box1_start, box1_dest_60, -pi/3, high_transit);
%do_pick_and_place(r, box2_start, box2_dest,-pi/3);
%r.move_cubic([0.15, 0.0, 0.2], -pi/3);

r.move_cubic(high_transit,-pi/3);
do_pick_and_place_v2(r, box2_start, box2_dest_60, -pi/3, high_transit);


do_bridge_task(r);


%do_pick_and_place_v2(r, box1_dest, [0.0845, 0.0040, 0.0608],-pi/3);
%do_pick_and_place_v2(r, box2_dest, [0.0845, 0.0040, 0.0858],-pi/3);

%do_pick_and_place_v2(r, box1_dest, [0.075, 0.0028, 0.0608],-pi/2);

do_pick_and_place_v2(r, box1_dest, [0.0718, 0.0035, 0.0608],-pi/2);
do_pick_and_place_v2(r, box2_dest, [0.0734, 0.0028, 0.080],-pi/2);



%rotate_block(r, pick_xyz, drop_xyz) 
% Final Return to Home
disp('Mission Complete. Returning to standby...');

%r.move_cubic([0.12,0,0.12],-pi/2)
% ==========================================
% 4. THE REUSABLE FUNCTION (The "Engine")
% ==========================================
% MATLAB requires local functions to be at the very bottom of the script.
function rotate_block(r, pick_xyz, drop_xyz)

    phi_pick = -60 * (pi/180);
    phi_drop =  0  * (pi/180);
    
    hover_15cm    = pick_xyz;  
    hover_15cm(3) = pick_xyz(3) + 0.0729;
    
    r.gripper(true);
    pause(1)
    disp('Hover Above Block Position')
    r.move_cubic(hover_15cm,phi_pick);
    
    disp('Lower to Block Position')
    r.move_linear(pick_xyz,phi_pick,5);
    get_robot_current_xyz(r, pick_xyz, 'At pick');
    r.gripper(false); % grab the block
    pause(1)
    %------------------------
    
    r.move_linear([pick_xyz(1),pick_xyz(2),pick_xyz(3)+0.07],phi_pick);

    % go back up to hover pos and flip
    r.move_cubic(hover_15cm,phi_drop)

    r.move_linear(drop_xyz,phi_drop,1)
    get_robot_current_xyz(r, drop_xyz, 'At drop');
    r.gripper(true)
    pause(1)

    r.move_linear(hover_15cm,phi_drop)
    
end
function do_pick_and_place(robot, pick_xyz, place_xyz,phi)
    % --- Settings for this move ---
    hover_offset = 0.03; % Hover 8cm above the target             
    speed = 2;         % Seconds to take for big moves
    
    z_offset = 0.01; % to account for gravity making it be further than wanted
    pick_xyz(3)  = pick_xyz(3) + z_offset;
    place_xyz(3) = place_xyz(3) + z_offset;
    
    
    % Get hover positions which are directly above actual locations
    hover_pick = pick_xyz;
    hover_pick(3) = hover_pick(3) + hover_offset; 
    
    hover_place = place_xyz;
    hover_place(3) = hover_place(3) + hover_offset;
    
    % Move to hover position to prepare for picking the cube
    robot.move_cubic(hover_pick, phi);
    get_robot_current_xyz(robot, hover_pick, 'Step 1: Hover over Pick');
    
    % Step 2: Go straight down (Precision)
    robot.move_linear(pick_xyz, phi, 1.0);
    get_robot_current_xyz(robot, pick_xyz, 'Step 2: Down to Pick');
    %r.get_xyz()
    
    % Step 3: Grip the box and wait for physical jaws to close
    robot.gripper(false);
    pause(1.0); 
    
    % Step 4: Lift straight up to Clearance Plane
    robot.move_linear(hover_pick, phi, 1.0);
    get_robot_current_xyz(robot, hover_pick, 'Step 4: Lift to Clearance');
    %r.get_xyz()

    robot.move_cubic([0.22,0,0.22], phi);
    % Step 5: Arc over to Hover above the destination (Fast/Smooth)
    robot.move_cubic(hover_place, phi);
    get_robot_current_xyz(robot, hover_place, 'Step 5: Hover over Place');
    %r.get_xyz()
    
    % Step 6: Go straight down to the table
    robot.move_linear(place_xyz, phi, 1.0);
    pause(1.0); 
    
    get_robot_current_xyz(robot, place_xyz, 'Step 6: Down to Place');
    % Step 7: Release the box and wait for jaws to open
    robot.gripper(true);
    pause(1.0); 
    
    % Step 8: Lift straight back up so we don't knock the box over
    robot.move_linear(hover_place, phi, 1.0);
    get_robot_current_xyz(robot, hover_place, 'Step 8: Retreat Lift');
    %r.get_xyz()
end
function actual_xyz = get_robot_current_xyz(robot, target_xyz, step_name)
    % Reads the actual robot position and compares it to the target
    
    actual_xyz = robot.get_xyz();
    offset = actual_xyz - target_xyz;
    
    fprintf('--- %s ---\n', step_name);
    fprintf(' Target: [%.4f, %.4f, %.4f]\n', target_xyz(1), target_xyz(2), target_xyz(3));
    fprintf(' Actual: [%.4f, %.4f, %.4f]\n', actual_xyz(1), actual_xyz(2), actual_xyz(3));
    fprintf(' Offset: [%.4f, %.4f, %.4f] (meters)\n\n', offset(1), offset(2), offset(3));
end

function do_bridge_task(robot) 
    phi_flat = 0;       % Horizontal wrist for bridge work
    phi_prep = -pi/2;   % Angled wrist for the final placement
    
    % --- Ensure Open ---
    robot.gripper(true);

    pause(1.0);
    
    % ======================================
    % PHASE 1: THE EXTRACTION
    % ======================================
    % Step 1: High Approach (Transit from Box 2)
    disp('Bridge Pick: High Approach...');
    robot.move_cubic([0.165, 0.0, 0.15], phi_flat, 1.5);
    
    bridge_height = 0.067
    % Step 2: Drop Down Outside Bridge
    disp('Bridge Pick: Dropping to entrance...');
    robot.move_linear([0.165, 0.0, bridge_height ], phi_flat, 1.0);
    
    % Step 3: Slide In!!!!!!!!!!!!!!!!!!
    disp('Bridge Pick: Sliding in...');
    robot.move_cubic([0.21, 0.0, bridge_height ], phi_flat, 1.0); % Try push further to get better grabbing
    
    % Step 4: Grip
    disp('Bridge Pick: Grasping...');
    robot.gripper(false);
    pause(1.0);
    
    % Step 5: Holder Clearance Lift (Anti-Drag)
    disp('Bridge Pick: Clearing table friction...'); % TUNE THIS
    friction_height = bridge_height + 0.004 % go up by one cm? , if 
    robot.move_linear([0.195, 0.0, friction_height], phi_flat, 2); % need to change this one to slightly higher to clear holder
    % steadily increase from 0.195
    pause(1);
    % Step 6: Slide Out
    disp('Bridge Pick: Sliding out...');
    %robot.move_cubic([0.165, 0.0, 0.070], phi_flat, 2.0);
    robot.move_cubic([0.16, 0.0, friction_height], phi_flat, 2.0);
    pause(1);
    % slide up
    %robot.move_cubic([0.165, 0.0, 0.1], phi_flat, 2.0);
    robot.move_linear([0.16, 0.0, 0.18], phi_flat, 2.0); % move to a higher distance before getting to hover preparation - 0.15?
    pause(1);
    % Step 7: Safe Hover Prep
    disp('Bridge Pick: Sweeping to safe hover prep...');
    robot.move_cubic([0.010, 0.0, 0.180], phi_prep, 2.5);
    pause(0.5);
    
    % ======================================
    % PHASE 2: THE PLACEMENT
    % ======================================
    % Step 8: Arc to Destination Hover
    % Move directly above your final target coordinate (Z + ~6cm clearance)
    disp('Hovering over target...');
    robot.move_cubic([0.0944, 0.0032, 0.0758], phi_prep, 1.0);
    
    pause(1);
    % Step 9: Plunge to Target
    % Drop linearly straight down to your exact coordinates
    disp('Lowering into holder...');
    robot.move_linear([0.0944, 0.0032, 0.0358], phi_prep) % change to linear as its more safe?, go even lower for better placement
    
    % Step 10: Release
    disp('Releasing...');
    robot.gripper(true);
    pause(1.0);
    
    % Step 11: Retreat
    % Pull straight back up so the jaws don't clip the block
    disp('Bridge Place: Retreating...');
    robot.move_cubic([0.0739, 0.0006, 0.1000], phi_prep);

    disp('--- Bridge Task Complete! ---');
end

function do_pick_and_place_v2(robot, pick_xyz, place_xyz, phi, via_xyz)
    % --- Settings for this move ---
    hover_offset = 0.03; % Safely lowered to 3cm to prevent "Too Short" IK errors
    % For 90 degrees, stacking at the center position , the max hover
    % offset is 0.055!!!!! , use 0.03 to avoid being close to error
    
    % Get hover positions which are directly above actual locations
    hover_pick = pick_xyz;
    hover_pick(3) = hover_pick(3) + hover_offset; 
    
    hover_place = place_xyz;
    hover_place(3) = hover_place(3) + hover_offset;
    robot.gripper(true);
    pause(0.5)
    % Step 1: Move to hover position to prepare for picking the cube
    robot.move_cubic(hover_pick, phi);
    get_robot_current_xyz(robot, hover_pick, 'Step 1: Hover over Pick');
    
    % Step 2: Go straight down (Precision)
    robot.move_linear(pick_xyz, phi, 1.0);
    get_robot_current_xyz(robot, pick_xyz, 'Step 2: Down to Pick');
    
    % Step 3: Grip the box and wait for physical jaws to close
    robot.gripper(false);
    pause(1.0); 
    
    % Step 4: Lift straight up to Clearance Plane
    robot.move_linear(hover_pick, phi, 1.0);
    get_robot_current_xyz(robot, hover_pick, 'Step 4: Lift to Clearance');
    
    % ========================================================
    % THE UPGRADE: DYNAMIC OBSTACLE AVOIDANCE (Via Point)
    % Only execute this safe transit move if a via_xyz was actually provided!
    if nargin >= 5 && ~isempty(via_xyz)
        disp('Routing via safe transit waypoint to avoid obstacles...');
        robot.move_cubic(via_xyz, phi,2);
    end
    % ========================================================
    
    % Step 5: Arc over to Hover above the destination (Fast/Smooth)
    robot.move_cubic(hover_place, phi);
    get_robot_current_xyz(robot, hover_place, 'Step 5: Hover over Place');
    
    % Step 6: Go straight down to the table
    robot.move_linear(place_xyz, phi, 0.5);
    pause(0.5); % Small pause to let the arm settle before dropping
    get_robot_current_xyz(robot, place_xyz, 'Step 6: Down to Place');
    
    % Step 7: Release the box and wait for jaws to open
    robot.gripper(true);
    pause(0.5); 
    
    % Step 8: Lift straight back up so we don't knock the box over
    robot.move_linear(hover_place, phi, 1.0);
    get_robot_current_xyz(robot, hover_place, 'Step 8: Retreat Lift');
end


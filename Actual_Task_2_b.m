%if ~exist('r', 'var')
 %   disp('First time running: Connecting to robot...');
  %  r = RobotDriver_sync();
   % r.setup();
%else
 %   disp('Robot is already connected! Skipping setup.');
%end


%r = RobotDriver_sync();
%r.setup();

r.torque(true)
% # put one cm above actual recorded z for safety
box1_start = [0.0671,  -0.1712, 0.0541];
box1_dest  = [-0.0019, 0.1228, 0.0483]; 

box2_start = [0.1671,  -0.1641, 0.0671];
box2_end = [0.1562,  -0.1538, 0.0593];

box2_dest  = [0.1371, 0.1392, 0.0576]; 



%r.gripper(true);
%r.move_cubic([0.005, 0, 0.32], -pi/3);
rotate_block(r,box2_start,box2_end)

%disp('--- Starting Task A: Pick and Place ---');

% Make sure gripper is open before we start doing anything

%do_pick_and_place(r, box1_start, box1_dest);
%do_pick_and_place(r, box2_start, box2_dest);
%do_bridge_task(r);


%rotate_block(r, pick_xyz, drop_xyz) 
% Final Return to Home
disp('Mission Complete. Returning to standby...');

r.move_cubic([0.12,0,0.12],-pi/2)
% ==========================================
% 4. THE REUSABLE FUNCTION (The "Engine")
% ==========================================
% MATLAB requires local functions to be at the very bottom of the script.
function rotate_block(r, pick_xyz, drop_xyz)

    phi_pick = -60 * (pi/180);
    phi_drop =  0  * (pi/180);

    hover_z_high  = 0.20;
    hover_15cm    = pick_xyz;  hover_15cm(3) = pick_xyz(3) + 0.15;
    hover_pick    = pick_xyz;  hover_pick(3) = hover_z_high;
    hover_drop    = drop_xyz;  hover_drop(3) = hover_z_high;

    % --- Open gripper ---
    disp('[Rotate] Opening gripper...');
    r.gripper(true);
    pause(1.5);

    % --- Slow initial move to an intermediate safe point first ---
    % Prevents the arm lurching from whatever position it starts in
    disp('[Rotate] Slow initial move to safe start...');
    safe_start    = hover_pick;
    safe_start(1) = 0.15;  safe_start(2) = -0.10;  % pull back in XY first
    r.move_cubic(safe_start, phi_pick, 6.0);  % slow: 6s
    pause(0.5);  % let arm fully settle before next command

    % --- Now arc to hover above pick ---
    disp('[Rotate] Transit to hover above pick...');
    r.move_cubic(hover_pick, phi_pick, 4.0);
    pause(0.3);

    % --- 15cm pre-hover ---
    disp('[Rotate] Pre-hover 15cm above pick...');
    r.move_linear(hover_15cm, phi_pick, 2.0);
    pause(0.3);
    get_robot_current_xyz(r, hover_15cm, 'Pre-hover');

    % --- Final descent ---
    disp('[Rotate] Descending to pick...');
    r.move_linear(pick_xyz, phi_pick, 3.0);
    get_robot_current_xyz(r, pick_xyz, 'At pick');

    % --- Grip ---
    disp('[Rotate] Closing gripper...');
    r.gripper(false);
    pause(2.0);

    % --- Lift in two stages, pause between to let arm settle ---
    disp('[Rotate] Lifting to 15cm...');
    r.move_linear(hover_15cm, phi_pick, 2.0);
    pause(0.3);

    disp('[Rotate] Lifting to full clearance...');
    r.move_linear(hover_pick, phi_pick, 2.5);
    pause(0.5);  % critical pause here - arm must be fully stopped before flip transit

    % --- Move to flip zone slowly ---
    disp('[Rotate] Moving to flip zone...');
    r.move_cubic([0.1461, -0.1280, 0.1788], -pi/3, 4.0);  % slower: 4s
    pause(0.5);  % settle before rotation

    % --- Rotate in place, slowly ---
    disp('[Rotate] Flipping block...');
    r.move_cubic([0.1461, -0.1280, 0.1788], phi_drop, 3.0);  % slower: 3s
    pause(0.3);

    % --- Arc to drop hover ---
    disp('[Rotate] Moving to drop hover...');
    r.move_cubic(hover_drop, phi_drop, 3.0);
    pause(0.3);

    % --- Descend to drop ---
    disp('[Rotate] Descending to drop...');
    r.move_linear(drop_xyz, phi_drop, 3.0);
    get_robot_current_xyz(r, drop_xyz, 'At drop');

    % --- Release ---
    disp('[Rotate] Releasing...');
    r.gripper(true);
    pause(1.5);

    % --- Retreat ---
    disp('[Rotate] Retreating...');
    hover_retreat    = drop_xyz;
    hover_retreat(3) = drop_xyz(3) + 0.10;
    r.move_linear(hover_retreat, phi_drop, 2.0);

    disp('[Rotate] Complete.');
end

function do_pick_and_place(robot, pick_xyz, place_xyz)
    % --- Settings for this move ---
    hover_offset = 0.08; % Hover 8cm above the target
    phi = -pi/3;             
    speed = 2;         % Seconds to take for big moves
    
    z_offset = 0.00; % to account for gravity making it be further than wanted
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
    robot.move_cubic(hover_pick, phi, 1.0);
    get_robot_current_xyz(robot, hover_pick, 'Step 4: Lift to Clearance');
    %r.get_xyz()
    % Step 5: Arc over to Hover above the destination (Fast/Smooth)
    robot.move_cubic(hover_place, phi);
    get_robot_current_xyz(robot, hover_place, 'Step 5: Hover over Place');
    %r.get_xyz()
    
    % Step 6: Go straight down to the table
    robot.move_linear(place_xyz, phi, 1.0);
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
    phi_prep = -pi/3;   % Angled wrist for the final placement
    
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
    robot.move_cubic([0.198, 0.0, bridge_height ], phi_flat, 1.0); % Try push further to get better grabbing
    
    % Step 4: Grip
    disp('Bridge Pick: Grasping...');
    robot.gripper(false);
    pause(1.0);
    
    % Step 5: Holder Clearance Lift (Anti-Drag)
    disp('Bridge Pick: Clearing table friction...'); % TUNE THIS
    friction_height = bridge_height + 0.01 % go up by one cm? , if 
    robot.move_linear([0.195, 0.0, friction_height], phi_flat, 2); % need to change this one to slightly higher to clear holder
    % steadily increase from 0.195

    % Step 6: Slide Out
    disp('Bridge Pick: Sliding out...');
    %robot.move_cubic([0.165, 0.0, 0.070], phi_flat, 2.0);
    robot.move_linear([0.165, 0.0, friction_height], phi_flat, 2.0);
    % slide up
    %robot.move_cubic([0.165, 0.0, 0.1], phi_flat, 2.0);
    robot.move_linear([0.165, 0.0, 0.18], phi_flat, 2.0); % move to a higher distance before getting to hover preparation - 0.15?
    
    % Step 7: Safe Hover Prep
    disp('Bridge Pick: Sweeping to safe hover prep...');
    robot.move_linear([0.010, 0.0, 0.180], phi_prep, 2.5);

    
    % ======================================
    % PHASE 2: THE PLACEMENT
    % ======================================
    % Step 8: Arc to Destination Hover
    % Move directly above your final target coordinate (Z + ~6cm clearance)
    disp('Hovering over target...');
    robot.move_cubic([0.0739, 0.0006, 0.1000], phi_prep, 1.0);
    
    % Step 9: Plunge to Target
    % Drop linearly straight down to your exact coordinates
    disp('Lowering into holder...');
    robot.move_linear([0.08, 0, 0.035], phi_prep) % change to linear as its more safe?, go even lower for better placement
    
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



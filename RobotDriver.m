function funcs = RobotDriver()
    % RobotDriver: A robust, safe interface for OpenManipulator-X
    % --------------------------------------------------------------------
    % EXPOSED COMMANDS (The "Menu"):
    %   robot.setup()        -> Connects, sets smooth profiles, enables torque
    %   robot.move(q)        -> Moves joints (q in Radians relative to Home)
    %   robot.read()         -> Returns current q (Radians relative to Home)
    %   robot.torque(bool)   -> true=ON (Stiff), false=OFF (Limp/Calibration)
    %   robot.gripper(bool)  -> true=OPEN, false=CLOSE
    %   robot.raw_ticks()    -> Returns raw [t1, t2, t3, t4] (For Calibration)
    %   robot.close()        -> Disconnects safely
    % --------------------------------------------------------------------
    
    % --- EXPOSED FUNCTIONS MAPPING ---
    funcs.setup     = @setup_robot;
    funcs.move      = @move_robot;
    funcs.read      = @read_robot;
    funcs.torque    = @set_torque;
    funcs.gripper   = @set_gripper;
    funcs.raw_ticks = @read_raw_ticks;
    funcs.close     = @close_robot;
    funcs.move_ik   = @move_to_xyz;
    funcs.move_cubic = @move_joint_cubic;
    % ====================================================================
    %                  USER CONFIGURATION (EDIT THIS!)
    % ====================================================================
    S = struct();
    S.DeviceName = 'COM9';       % <--- CHECK YOUR PORT
    S.BaudRate   = 1000000;
    S.JointIDs   = [11, 12, 13, 14];
    S.GripperID  = 15;
    
    % --- CALIBRATION DATA ---
    % 1. Run 'r.torque(0)' and move robot to perfect L-Shape.
    % 2. Run 'r.raw_ticks()' to get these numbers.
    % 3. Paste them here:
    S.Home_Ticks = [2048, 2048, 2048, 2048]; 
    
    % --- DIRECTION CORRECTION ---
    % Set to -1 if a joint moves opposite to your simulation.
    % usually: [1, 1, 1, 1] or sometimes [1, -1, 1, 1]
    S.Dirs       = [1, -1, -1, -1]; 
    v_sh = 0.128; x_sh = 0.024;
    S.params.h  = 0.077;
    S.params.L1 = hypot(v_sh, x_sh);
    S.params.L2 = 0.124;
    S.params.Le = 0.1025; % Your grasp center
    S.params.delta = atan2(x_sh, v_sh);
    S.params.deltaSign = -1;

    S.X_LIM = [0.001, 0.273];
    S.Y_LIM = [-0.239, 0.239];
    S.Z_LIM = [0.060, 0.325];


    % ====================================================================
    %                  INTERNAL CONSTANTS (DO NOT EDIT)
    % ====================================================================
    % Dynamixel Protocol 2.0 Registers (X-Series)
    ADDR_TORQUE_ENABLE      = 64;
    ADDR_GOAL_POSITION      = 116;
    ADDR_PRESENT_POSITION   = 132;
    ADDR_OPERATING_MODE     = 11;
    ADDR_PROFILE_VELOCITY   = 112;
    ADDR_PROFILE_ACCEL      = 108;
    
    % Motion Profile (Safety Limits)
    PROFILE_VEL = 15;  % Lower is slower/safer
    PROFILE_ACC = 7.5;  % Lower is smoother
    
    % Internal State
    port_num = 0;
    lib_name = '';

    % ====================================================================
    %                          SETUP
    % ====================================================================
    function success = setup_robot()
        success = false;
        % Load Library
        if strcmp(computer, 'PCWIN64'), lib_name = 'dxl_x64_c';
        elseif strcmp(computer, 'GLNXA64'), lib_name = 'libdxl_x64_c';
        elseif strcmp(computer, 'MACI64'), lib_name = 'libdxl_mac_c';
        end
        if ~libisloaded(lib_name)
            [~, ~] = loadlibrary(lib_name, 'dynamixel_sdk.h', ...
                'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
        end
        
        % Open Port
        port_num = portHandler(S.DeviceName);
        packetHandler();
        if ~openPort(port_num)
            fprintf('[ERROR] Failed to open port %s\n', S.DeviceName); return;
        end
        if ~setBaudRate(port_num, S.BaudRate)
            fprintf('[ERROR] Failed to set baudrate\n'); return;
        end
        
        % Configure Motors
        fprintf('Configuring Motors (ID: %s)...\n', num2str(S.JointIDs));
        all_ids = [S.JointIDs, S.GripperID];
        
        for id = all_ids
            % 1. Disable Torque to change settings
            write1ByteTxRx(port_num, 2.0, id, ADDR_TORQUE_ENABLE, 0);
            
            % 2. Set Operating Mode to Position Control (3)
            write1ByteTxRx(port_num, 2.0, id, ADDR_OPERATING_MODE, 3);
            
            % 3. Set Smooth Motion Profiles
            write4ByteTxRx(port_num, 2.0, id, ADDR_PROFILE_VELOCITY, PROFILE_VEL);
            write4ByteTxRx(port_num, 2.0, id, ADDR_PROFILE_ACCEL, PROFILE_ACC);
            
            % 4. Enable Torque
            write1ByteTxRx(port_num, 2.0, id, ADDR_TORQUE_ENABLE, 1);
        end
        fprintf('Robot Ready. Torque ON.\n');
        success = true;
    end

    % ====================================================================
    %                          MOVE (Write)
    % ====================================================================
    function move_robot(q_rad)
        % q_rad: [q1, q2, q3, q4] in radians relative to home
        if length(q_rad) < 4, error('Input must be 4 angles (rad)'); end
        
        for i = 1:4
            % 1. Convert Rad -> Ticks
            ticks_offset = (q_rad(i) * 2048 / pi) * S.Dirs(i);
            
            % 2. Add to Home
            goal_pos = S.Home_Ticks(i) + ticks_offset;
            
            % 3. Clamp Safety (0-4095)
            goal_pos = max(0, min(4095, round(goal_pos)));
            
            % 4. Write
            write4ByteTxRx(port_num, 2.0, S.JointIDs(i), ADDR_GOAL_POSITION, goal_pos);
        end
    end

    % ====================================================================
    %                    Inverse Kinematics Movement
    % ====================================================================
    function success = move_to_xyz(target_p, phi)
        success = false;
        
        % 1. Use the params stored in S
        p = target_p;
        p(1) = max(S.X_LIM(1), min(S.X_LIM(2), p(1)));
        p(2) = max(S.Y_LIM(1), min(S.Y_LIM(2), p(2)));
        p(3) = max(S.Z_LIM(1), min(S.Z_LIM(2), p(3)));
        
        % 2. Solve IK (Uses S.params directly)
        [th_ik, ok, ~] = ik_pos_only_between_jaws(p, phi, -1, S.params);
        
        if ok
            % 3. Calculate Relative q
            th_home = [0, (pi/2 + S.params.delta), -pi/2, 0];
            q_input = th_ik - th_home;
            
            % 4. Move
            move_robot(q_input);
            success = true;
        end
    end

     % ====================================================================
    %                          Cubic Motion
    % ====================================================================
    function success = move_joint_cubic(target_p, phi, T_total)
        % Moves to XYZ using Joint-Space Cubic Interpolation
        % T_total: Time in seconds for the move
        success = false;
        N = 50; % Number of steps. 50 is usually plenty for 2 seconds.
        dt = T_total / N;

        % 1. Get Current Position (Start)
        q_start = read_robot(); % Relative radians
        th_home = [0, (pi/2 + S.params.delta), -pi/2, 0];
        th_start = q_start + th_home; % Convert to Absolute Math angles

        % 2. Solve IK for Goal (End)
        % Clamp target first
        p = target_p;
        p(1) = max(S.X_LIM(1), min(S.X_LIM(2), p(1)));
        p(2) = max(S.Y_LIM(1), min(S.Y_LIM(2), p(2)));
        p(3) = max(S.Z_LIM(1), min(S.Z_LIM(2), p(3)));
        
        [th_goal, ok, ~] = ik_pos_only_between_jaws(p, phi, -1, S.params);
        
        if ~ok
            warning('Cubic move failed: Goal unreachable.');
            return;
        end

        % 3. Loop through Cubic Interpolation
        fprintf('Starting Cubic Move (%.1fs)...\n', T_total);
        tic;
        for k = 1:N
            tau = (k-1)/(N-1);
            s = 3*tau^2 - 2*tau^3; % Cubic scaling factor
            
            % Interpolate Absolute angles
            th_interp = (1-s)*th_start + s*th_goal;
            
            % Convert back to Relative for the motors
            q_cmd = th_interp - th_home;
            
            % Send command
            move_robot(q_cmd);
            
            % Maintain timing
            while toc < k*dt
                % wait
            end
        end
        fprintf('Move Complete.\n');
        success = true;
    end
    % ====================================================================
    %                          READ (Feedback)
    % ====================================================================
    function q = read_robot()
        % Returns [q1, q2, q3, q4] in radians relative to home
        q = zeros(1,4);
        for i = 1:4
            pos = read4ByteTxRx(port_num, 2.0, S.JointIDs(i), ADDR_PRESENT_POSITION);
            ticks = typecast(uint32(pos), 'int32'); % Handle potential negative
            
            diff = double(ticks) - S.Home_Ticks(i);
            q(i) = (diff * pi / 2048) * S.Dirs(i);
        end
    end

    % ====================================================================
    %                    READ RAW TICKS (For Calibration)
    % ====================================================================
    function ticks = read_raw_ticks()
        % Returns raw [t1, t2, t3, t4] (0-4095)
        ticks = zeros(1,4);
        for i = 1:4
            pos = read4ByteTxRx(port_num, 2.0, S.JointIDs(i), ADDR_PRESENT_POSITION);
            ticks(i) = typecast(uint32(pos), 'int32');
        end
    end

    % ====================================================================
    %                          UTILITIES
    % ====================================================================
    function set_torque(enable)
        val = 0; 
        if enable, val = 1; end
        all_ids = [S.JointIDs, S.GripperID];
        for id = all_ids
            write1ByteTxRx(port_num, 2.0, id, ADDR_TORQUE_ENABLE, val);
        end
        if enable, disp('Torque ON (Stiff)'); else, disp('Torque OFF (Limp)'); end
    end

    function set_gripper(isOpen)
        % Open/Close Gripper
        % NOTE: Tune these values for your specific gripper!
        OPEN_POS = 2500;   % <--- TUNE ME
        CLOSE_POS = 1500;  % <--- TUNE ME
        
        goal = CLOSE_POS;
        if isOpen, goal = OPEN_POS; end
        
        write4ByteTxRx(port_num, 2.0, S.GripperID, ADDR_GOAL_POSITION, goal);
    end

    function close_robot()
        if port_num
            closePort(port_num);
        end
        if libisloaded(lib_name)
            unloadlibrary(lib_name);
        end
        fprintf('Robot Disconnected.\n');
    end
end
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

    funcs.get_xyz = @get_xyz;
    funcs.move_ik   = @move_to_xyz;
    funcs.move_cubic = @move_joint_cubic;
    funcs.move_linear= @move_linear;
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
    S.params.Le = 0.126; % Your grasp center
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

        % --- NEW: TRANSMIT ALL IN ONE BURST ---
        %groupSyncWriteTxPacket(S.group_write_num);
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
        success = false;
        
        % 1. Get Current Angles
        q_start = read_robot(); 
        th_home = [0, (pi/2 + S.params.delta), -pi/2, 0];
        th_start = q_start + th_home; 
        
        % 2. Auto-Timing Brain
        if nargin < 3 || isempty(T_total)
            current_p = get_xyz();
            distance = norm(target_p - current_p);
            cruise_speed = 0.1; % 10 cm/s
            T_total = max(0.5, distance / cruise_speed);
        end
        
        % 3. Dynamic Slicing (~20ms per step)
        dt_target = 0.02; % 50 Hz
        N = max(10, round(T_total / dt_target)); 
        dt = T_total / (N-1);
        % 4. Solve IK for Goal
        p = target_p;
        p(1) = max(S.X_LIM(1), min(S.X_LIM(2), p(1)));
        p(2) = max(S.Y_LIM(1), min(S.Y_LIM(2), p(2)));
        p(3) = max(S.Z_LIM(1), min(S.Z_LIM(2), p(3)));
        
        [th_goal, ok, ~] = ik_pos_only_between_jaws(p, phi, -1, S.params);
        if ~ok
            warning('Cubic move failed: Goal unreachable.'); return;
        end
        
        % 5. Execute Move
        fprintf('Curved Move to [%.2f, %.2f, %.2f] (%.1fs)...\n', p(1), p(2), p(3), T_total);
        startTime = tic;
        for k = 1:N
            tau = (k-1)/(N-1);
            s = 3*tau^2 - 2*tau^3; 
            
            th_interp = (1-s)*th_start + s*th_goal;
            move_robot(th_interp - th_home);
            
            while toc(startTime) < (k-1)*dt; end % Precision wait
        end
        success = true;
    end
    % ====================================================================
    %             Cartesian Linear Motion (Straight & Precise)
    % ====================================================================
    function success = move_linear(target_p, phi, T_total)
        success = false;
        
        % 1. Get Current Position
        current_p = get_xyz();
        
        % 2. Auto-Timing Brain (Slightly slower cruise speed for precision)
        if nargin < 3 || isempty(T_total)
            distance = norm(target_p - current_p);
            cruise_speed = 0.10; % 10 cm/s for straight lines
            T_total = max(0.5, distance / cruise_speed);
        end
        
        % 3. Dynamic Slicing
        dt_target = 0.02; 
        N = max(10, round(T_total / dt_target)); 
        dt = T_total / (N-1); 
        
        % 4. Execute Cartesian Move
        th_home = [0, (pi/2 + S.params.delta), -pi/2, 0];
        fprintf('Linear Move to [%.2f, %.2f, %.2f] (%.1fs)...\n', target_p(1), target_p(2), target_p(3), T_total);
        
        startTime = tic;
        for k = 1:N
            tau = (k-1)/(N-1);
            s = 3*tau^2 - 2*tau^3; 
            
            % Interpolate XYZ distance directly
            p_interp = (1-s)*current_p + s*target_p;
            
            % Solve IK for this specific millimeter
            [th_step, ok, ~] = ik_pos_only_between_jaws(p_interp, phi, -1, S.params);
            if ~ok
                warning('Linear move failed: Path broken at step %d', k); return;
            end
            
            move_robot(th_step - th_home);
            
            while toc(startTime) < (k-1)*dt; end % Precision wait
        end
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
    %                    READ CURRENT POSITION
    % ====================================================================
    function p = get_xyz()
        % Reads current angles and uses FK to return [X, Y, Z]
        q_current = read_robot();
        th_home = [0, (pi/2 + S.params.delta), -pi/2, 0];
        th_eff = q_current + th_home;
        
        % Forward Kinematics
        th_eff(2) = th_eff(2) + S.params.deltaSign * S.params.delta;
        r = S.params.L1*cos(th_eff(2)) + S.params.L2*cos(th_eff(2)+th_eff(3)) + S.params.Le*cos(th_eff(2)+th_eff(3)+th_eff(4));
        z = S.params.h + S.params.L1*sin(th_eff(2)) + S.params.L2*sin(th_eff(2)+th_eff(3)) + S.params.Le*sin(th_eff(2)+th_eff(3)+th_eff(4));
        x = r * cos(th_eff(1));
        y = r * sin(th_eff(1));
        
        p = [x, y, z];
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
        OPEN_POS = 4095;   % <--- TUNE ME 
        CLOSE_POS = 0;  % <--- TUNE ME
        
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
function funcs = RobotDriver_sync();
  
    % ================= EXPOSED FUNCTIONS =================
    
    funcs.setup       = @setup_robot;
    funcs.move        = @move_robot;
    funcs.read        = @read_robot;
    funcs.torque      = @set_torque;
    funcs.gripper     = @set_gripper;
    funcs.raw_ticks   = @read_raw_ticks;
    funcs.close       = @close_robot;
    funcs.get_xyz     = @get_xyz;
    funcs.move_ik     = @move_to_xyz;
    funcs.move_cubic  = @move_joint_cubic;
    funcs.move_linear = @move_linear;
    funcs.torque_joint =@set_torque_joint;
    funcs.set_comp = @set_shoulder_comp;
    
    % Setting up the device / library
    S.DeviceName = 'COM9';       
    S.BaudRate   = 1000000;
    S.JointIDs   = [11, 12, 13, 14];
    S.GripperID  = 15;
    S.Home_Ticks = [2051, 2048, 2048, 2048];
    S.Dirs       = [1, -1, -1, -1];


    % FK parameters
    v_sh = 0.128; x_sh = 0.024;
    S.params.h  = 0.077;
    S.params.L1 = hypot(v_sh, x_sh);
    S.params.L2 = 0.124;
    S.params.Le = 0.126;
    S.params.delta = atan2(x_sh, v_sh);
    S.params.deltaSign = -1;
    S.X_LIM = [-0.10, 0.273];
    S.Y_LIM = [-0.239, 0.239];
    S.Z_LIM = [0, 0.325]; % measure in downward position, what the actual limit is as limits change based on orientation
    S.shoulder_comp = 1.5 * (pi/180);
    funcs.params = S.params;
    ADDR_TORQUE_ENABLE    = 64;
    ADDR_GOAL_POSITION    = 116;
    ADDR_PRESENT_POSITION = 132;
    ADDR_OPERATING_MODE   = 11;
    ADDR_PROFILE_VELOCITY = 112;
    ADDR_PROFILE_ACCEL    = 108;
    
    PROFILE_VEL = 0;
    PROFILE_ACC = 0;

    port_num = 0;
    lib_name = '';

   % Setup
    function success = setup_robot()
        success = false;
        if strcmp(computer, 'PCWIN64'), lib_name = 'dxl_x64_c';
        elseif strcmp(computer, 'GLNXA64'), lib_name = 'libdxl_x64_c';
        elseif strcmp(computer, 'MACI64'), lib_name = 'libdxl_mac_c';
        end
        
        if libisloaded(lib_name), unloadlibrary(lib_name); end
        
        loadlibrary(lib_name, 'dynamixel_sdk.h', ...
            'addheader', 'port_handler.h', ...
            'addheader', 'packet_handler.h', ...
            'addheader', 'group_sync_write.m'); % needed to allow sync
            
        port_num = portHandler(S.DeviceName);
        packetHandler();
        
        if ~openPort(port_num)
            fprintf('[ERROR] Failed to open port %s\n', S.DeviceName); 
            return; 
        end
        if ~setBaudRate(port_num, S.BaudRate)
            fprintf('[ERROR] Failed to set baudrate\n'); 
            return; 
        end
        
        all_ids = [S.JointIDs, S.GripperID];
        for id = all_ids
            write1ByteTxRx(port_num, 2.0, id, ADDR_TORQUE_ENABLE, 0);
            write1ByteTxRx(port_num, 2.0, id, ADDR_OPERATING_MODE, 3);
            write4ByteTxRx(port_num, 2.0, id, ADDR_PROFILE_VELOCITY, PROFILE_VEL);
            write4ByteTxRx(port_num, 2.0, id, ADDR_PROFILE_ACCEL, PROFILE_ACC);
            write1ByteTxRx(port_num, 2.0, id, ADDR_TORQUE_ENABLE, 1);
        end
        
        disp('Robot Ready (Hybrid Sync Write Mode Enabled)');
        success = true;
    end


    function set_shoulder_comp(deg)
    S.shoulder_comp = deg * (pi/180);
    fprintf('[Driver] Shoulder comp set to %.1f degrees\n', deg);
    end
    % sync write
    function move_robot(q_rad)

    if numel(q_rad) < 4
        error('Input must be 4 joint angles');
    end
    q_rad(2) = q_rad(2) + S.shoulder_comp;

    temp_write_num = groupSyncWrite(port_num, 2.0, ADDR_GOAL_POSITION, 4);

    for i = 1:4
        ticks_offset = (q_rad(i) * 2048 / pi) * S.Dirs(i);
        goal = S.Home_Ticks(i) + ticks_offset;
        goal = max(0, min(4095, round(goal)));

        groupSyncWriteAddParam(temp_write_num, ...
                               S.JointIDs(i), ...
                               goal, ...
                               4);
    end

    groupSyncWriteTxPacket(temp_write_num);
    groupSyncWriteClearParam(temp_write_num);

end

  
    % Reading function that converts ticks to angles in degrees
    function q = read_robot()
        q = zeros(1, 4);
        for i = 1:4
            pos = read4ByteTxRx(port_num, 2.0, S.JointIDs(i), ADDR_PRESENT_POSITION);
            ticks = typecast(uint32(pos), 'int32');
            
            diff = double(ticks) - S.Home_Ticks(i);
            q(i) = (diff * pi / 2048) * S.Dirs(i);
        end
    end
    
    % first implementation of inverse to check if it works - works!
    function success = move_to_xyz(p, phi)
        success = false;
        p(1) = max(S.X_LIM(1), min(S.X_LIM(2), p(1)));
        p(2) = max(S.Y_LIM(1), min(S.Y_LIM(2), p(2)));
        p(3) = max(S.Z_LIM(1), min(S.Z_LIM(2), p(3)));
        
        [th, ok] = ik_pos_only_between_jaws(p, phi, -1, S.params);
        if ok
            th_home = [0, pi/2+S.params.delta, -pi/2, 0];
            move_robot(th - th_home);
            success = true;
        end
    end

    % Trajectory Planning, cubic polynmoial for smooth movement that ends
    % at same time, only calls ik once
    function success = move_joint_cubic(target_p, phi, T_total)
        success = false;
        q_start = read_robot();
        th_home = [0, pi/2+S.params.delta, -pi/2, 0];
        th_start = q_start + th_home;
        
        if nargin < 3 || isempty(T_total)
            current_p = get_xyz();
            d = norm(target_p - current_p);
            T_total = max(0.8, d/0.15); % if not specifying a time for movement, min 1s for movement , or at 10 cm/s 
        end
        
        dt_target = 0.02; % change to 0.02? 50Hz
        N = max(10, round(T_total/dt_target));
        dt = T_total/(N-1);
        
        p = target_p;
        p(1) = max(S.X_LIM(1), min(S.X_LIM(2), p(1)));
        p(2) = max(S.Y_LIM(1), min(S.Y_LIM(2), p(2)));
        p(3) = max(S.Z_LIM(1), min(S.Z_LIM(2), p(3)));
        
        [th_goal, ok] = ik_pos_only_between_jaws(p, phi, -1, S.params);
        if ~ok, 
            phi_deg = phi * (180/pi);
    
    % 2. Print the expanded warning in RED
            fprintf(2, '\n!!! KINEMATIC ERROR !!!\n');
            fprintf(2, 'Target Position: [%.3f, %.3f, %.3f] meters\n', p(1), p(2), p(3));
            fprintf(2, 'Requested Angle: %.1f degrees\n', phi_deg);
    
    % 3. The Safety Freeze
            disp('Check the robot. Press ANY KEY to kill the script safely.');
            pause; 
    
    % 4. The "Software E-Stop"
            error('Script terminated: Impossible geometry detected.'); 
        end
        
        
        t0 = tic;
        for k = 1:N
            tau = (k-1)/(N-1);
            s = 3*tau^2 - 2*tau^3;
            th = (1-s)*th_start + s*th_goal;
            move_robot(th - th_home);
            while toc(t0) < (k-1)*dt; end
        end
        success = true;
    end

    % linear movement for straight paths which calls upon ik during the
    % loop
    function success = move_linear(target_p, phi, T_total)
        success = false;
        p0 = get_xyz();
        
        if nargin < 3 || isempty(T_total)
            d = norm(target_p - p0);
            T_total = max(0.3, d/0.15);
        end
        
        dt_target = 0.02;
        N = max(10, round(T_total/dt_target));
        dt = T_total/(N-1);
        th_home = [0, pi/2+S.params.delta, -pi/2, 0];
        
        t0 = tic;
        for k = 1:N
            tau = (k-1)/(N-1);
            s = 3*tau^2 - 2*tau^3;
            p = (1-s)*p0 + s*target_p;
            
            [th, ok] = ik_pos_only_between_jaws(p, phi, -1, S.params);
            if ~ok, 
            phi_deg = phi * (180/pi);
    
    % Check for IK errors of any unreachable points
            fprintf(2, '\n!!! KINEMATIC ERROR !!!\n');
            fprintf(2, 'Target Position: [%.3f, %.3f, %.3f] meters\n', p(1), p(2), p(3));
            fprintf(2, 'Requested Angle: %.1f degrees\n', phi_deg);
            disp('Check the robot. Press ANY KEY to kill the script safely.');
            pause; 
    
    % press control c to pause/end scrpt
            error('Script terminated: Impossible geometry detected.'); 
        end
            
            move_robot(th - th_home);
            while toc(t0) < (k-1)*dt; end
        end
        success = true;
    end

    % fk to get measure of current position in x,yz
    function p = get_xyz()
        q = read_robot();
        th_home = [0, pi/2+S.params.delta, -pi/2, 0];
        th = q + th_home;
        th(2) = th(2) + S.params.deltaSign*S.params.delta;
        
        r = S.params.L1*cos(th(2)) + S.params.L2*cos(th(2)+th(3)) + S.params.Le*cos(th(2)+th(3)+th(4));
        z = S.params.h + S.params.L1*sin(th(2)) + S.params.L2*sin(th(2)+th(3)) + S.params.Le*sin(th(2)+th(3)+th(4));
        x = r*cos(th(1));
        y = r*sin(th(1));
        
        p = [x, y, z];
    end

    % gets encoder values for joints
    function ticks = read_raw_ticks()
        ids = [S.JointIDs, S.GripperID];
        ticks = zeros(1, 5);
        for i = 1:5
            pos = read4ByteTxRx(port_num, 2.0, ids(i), ADDR_PRESENT_POSITION);
            ticks(i) = typecast(uint32(pos), 'int32');
        end
    end

    % turns the torque on with set_torque(1)
    function set_torque(enable)
        val = double(enable);
        ids = [S.JointIDs, S.GripperID];
        for id = ids
            write1ByteTxRx(port_num, 2.0, id, ADDR_TORQUE_ENABLE, val);
        end
    end
    function set_torque_joint(id,enable)
        val = double(enable);
        write1ByteTxRx(port_num, 2.0, id, ADDR_TORQUE_ENABLE, val);
    end

    % open/close gripper
    function set_gripper(isOpen)
        OPEN = 2000; CLOSE = 2400;
        goal = CLOSE;
        if isOpen, goal = OPEN; end
        
        write1ByteTxRx(port_num, 2.0, S.GripperID, 64, 1);
        write4ByteTxRx(port_num, 2.0, S.GripperID, ADDR_GOAL_POSITION, goal);
        pause(0.05);
    end

    % 
    function close_robot()
        if port_num, closePort(port_num); end
        if libisloaded(lib_name), unloadlibrary(lib_name); end
        disp('Robot Disconnected');
    end
end
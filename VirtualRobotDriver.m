function funcs = VirtualRobotDriver()
% VirtualRobotDriver  –  Drop-in simulation replacement for RobotDriver_sync
%
% USAGE
%   robot = VirtualRobotDriver();   % instead of robot = RobotDriver_sync();
%   robot.setup();
%   robot.move_ik([0.2, 0, 0.2], 0);
%   ...
%
% IDENTICAL API
%   robot.setup()            – initialises sim, opens 3-D figure
%   robot.move(q_rad)        – jump joints to target angles
%   robot.read()             – return current joint angles [1x4]
%   robot.torque(enable)     – no-op (printed to console)
%   robot.gripper(isOpen)    – animates gripper open / close
%   robot.raw_ticks()        – synthetic tick values [1x5]
%   robot.close()            – close figure, print disconnect
%   robot.get_xyz()          – FK → current [x,y,z]
%   robot.move_ik(p,phi)     – IK then instant move
%   robot.move_cubic(p,phi,T)– smooth joint-space cubic trajectory
%   robot.move_linear(p,phi,T)– smooth Cartesian linear trajectory
%   robot.torque_joint(id,en)– no-op (printed to console)
%
% All trajectory methods update the 3-D visualisation in real time.

    % ---- Expose API (identical names to RobotDriver_sync) ----
    funcs.setup        = @setup_robot;
    funcs.move         = @move_robot;
    funcs.read         = @read_robot;
    funcs.torque       = @set_torque;
    funcs.gripper      = @set_gripper;
    funcs.raw_ticks    = @read_raw_ticks;
    funcs.close        = @close_robot;
    funcs.get_xyz      = @get_xyz;
    funcs.move_ik      = @move_to_xyz;
    funcs.move_cubic   = @move_joint_cubic;
    funcs.move_linear  = @move_linear;
    funcs.torque_joint = @set_torque_joint;
    funcs.set_view     = @set_view;
    funcs.set_comp = @set_shoulder_comp;

    % ---- Robot geometry (mirrors RobotDriver_sync) ----
    v_sh = 0.128; x_sh = 0.024;
    S.params.h        = 0.077;
    S.params.L1       = hypot(v_sh, x_sh);
    S.params.L2       = 0.124;
    S.params.Le       = 0.126;
    S.params.delta    = atan2(x_sh, v_sh);
    S.params.deltaSign = -1;
    L3 = 0.126; Lg = 0;
    S.L3 = L3; S.Lg = Lg;

    S.Home_Ticks = [2048, 2048, 2048, 2048];
    S.Dirs       = [1, -1, -1, -1];
    S.GripperID  = 15;

    S.X_LIM = [-0.005, 0.273];
    S.Y_LIM = [-0.239, 0.239];
    S.Z_LIM = [0.03,  0.325];
    VS.shoulder_comp = 0.0;

    % ---- Virtual state ----
    th_home = [0, pi/2 + S.params.delta, -pi/2, 0];
    VS.q       = zeros(1,4);          % joint offsets from home (radians)
    VS.th      = th_home;             % absolute joint angles
    VS.gripper_open = true;
    VS.fig     = [];
    VS.ax      = [];
    VS.hLinks  = [];
    VS.hGrip   = [];
    VS.hTrace  = [];
    VS.traceXYZ = zeros(3,0);
    VS.ready   = false;
    VS.session_t0    = [];            % session-wide timer, started in setup()
    VS.traj_log.time = [];            % running trajectory log
    VS.traj_log.xyz  = [];
    VS.traj_log.q    = [];

    % =========================================================
    %  PUBLIC FUNCTIONS
    % =========================================================

    function success = setup_robot()
        fprintf('[VirtualRobot] Initialising simulation...\n');
        VS.q  = zeros(1,4);
        VS.th = th_home;
        VS.gripper_open = true;
        VS.traceXYZ = zeros(3,0);
        VS.traj_log = struct('time',[],'xyz',[],'q',[]);
        VS.session_t0 = tic;   % <-- single session-wide clock

        VS.fig = figure('Color',[0.07 0.07 0.10], ...
                        'Name','Virtual Robot Simulation', ...
                        'NumberTitle','off', ...
                        'MenuBar','none', ...
                        'ToolBar','figure');
        VS.ax = axes(VS.fig);
        hold(VS.ax,'on');
        grid(VS.ax,'on');
        axis(VS.ax,'equal');
        view(VS.ax, 35, 20);

        % Dark theme axes
        set(VS.ax, 'Color',      [0.07 0.07 0.10], ...
                   'XColor',     [0.55 0.75 1.00], ...
                   'YColor',     [0.55 0.75 1.00], ...
                   'ZColor',     [0.55 0.75 1.00], ...
                   'GridColor',  [0.25 0.30 0.40], ...
                   'GridAlpha',  0.4, ...
                   'FontSize',   10);
        xlabel(VS.ax,'X (m)','Color',[0.55 0.75 1.00]);
        ylabel(VS.ax,'Y (m)','Color',[0.55 0.75 1.00]);
        zlabel(VS.ax,'Z (m)','Color',[0.55 0.75 1.00]);
        title(VS.ax,'Virtual Robot Driver  –  Live Simulation', ...
              'Color',[0.85 0.92 1.00],'FontSize',13,'FontWeight','bold');

        % Zoom to actual workspace limits for clarity
        pad = 0.02;
        xlim(VS.ax, [S.X_LIM(1)-pad, S.X_LIM(2)+pad]);
        ylim(VS.ax, [S.Y_LIM(1)-pad, S.Y_LIM(2)+pad]);
        zlim(VS.ax, [0,              S.Z_LIM(2)+pad]);

        % Workspace boundary box (faint)
        draw_workspace_box();

        % Origin indicator
        plot3(VS.ax,0,0,S.params.h,'w+','MarkerSize',10,'LineWidth',2);

        % Trace line
        VS.hTrace = plot3(VS.ax, NaN,NaN,NaN, '-', ...
                          'Color',[0.30 0.85 0.60], ...
                          'LineWidth',1.5);

        % Draw robot at home
        VS.hLinks = [];
        VS.hGrip  = [];
        redraw_robot();

        VS.ready = true;
        fprintf('[VirtualRobot] Ready.  Figure open for visualisation.\n');
        success = true;
    end
    function set_shoulder_comp(deg)
        VS.shoulder_comp = deg * (pi/180);
        fprintf('[VirtualRobot] Shoulder comp set to %.1f degrees\n', deg);
    end
    % ----------------------------------------------------------
    function move_robot(q_rad)
        if numel(q_rad) < 4
            error('[VirtualRobot] move() needs 4 joint angles');
        end
        q_rad(2) = q_rad(2) + VS.shoulder_comp;
        VS.q  = q_rad(1:4);
        VS.th = VS.q + th_home;
        if VS.ready, redraw_robot(); end
    end

    % ----------------------------------------------------------
    function q = read_robot()
        q = VS.q;
    end

    % ----------------------------------------------------------
    function set_torque(enable)
        if enable
            fprintf('[VirtualRobot] Torque ENABLED (all joints)\n');
        else
            fprintf('[VirtualRobot] Torque DISABLED (all joints)\n');
        end
    end

    function set_torque_joint(id, enable)
        fprintf('[VirtualRobot] Torque joint %d -> %d\n', id, enable);
    end

    % ----------------------------------------------------------
    function set_gripper(isOpen)
        VS.gripper_open = isOpen;
        if VS.ready, redraw_robot(); end
        if isOpen
            fprintf('[VirtualRobot] Gripper OPEN\n');
        else
            fprintf('[VirtualRobot] Gripper CLOSED\n');
        end
        pause(0.05);
    end

    % ----------------------------------------------------------
    function ticks = read_raw_ticks()
        ticks = zeros(1,5);
        for i = 1:4
            ticks_offset = (VS.q(i) * 2048 / pi) * S.Dirs(i);
            ticks(i) = round(S.Home_Ticks(i) + ticks_offset);
        end
        % Gripper: open=1995, close=2387
        if VS.gripper_open, ticks(5) = 1995; else, ticks(5) = 2387; end
    end

    % ----------------------------------------------------------
    function close_robot()
        if ~isempty(VS.fig) && isvalid(VS.fig)
            close(VS.fig);        % close sim window first
        end
        plot_diagnostics();       % then open clean diagnostics
        VS.ready = false;
        fprintf('[VirtualRobot] Disconnected.\n');
    end

    % ----------------------------------------------------------
    function p = get_xyz()
        p = fk_xyz(VS.th, S.params);
    end

    % ----------------------------------------------------------
    function success = move_to_xyz(p, phi)
        success = false;
        p(1) = max(S.X_LIM(1), min(S.X_LIM(2), p(1)));
        p(2) = max(S.Y_LIM(1), min(S.Y_LIM(2), p(2)));
        p(3) = max(S.Z_LIM(1), min(S.Z_LIM(2), p(3)));

        [th, ok] = ik_pos_only_between_jaws(p, phi, -1, S.params);
        if ok
            move_robot(th - th_home);
            success = true;
        else
            fprintf('[VirtualRobot] IK unreachable for [%.3f %.3f %.3f]\n', p(1),p(2),p(3));
        end
    end

    % ----------------------------------------------------------
    function success = move_joint_cubic(target_p, phi, T_total)
        success = false;
        q_start = read_robot();
        th_home = [0, pi/2+S.params.delta, -pi/2, 0];
        th_start = q_start + th_home;
        
        if nargin < 3 || isempty(T_total)
            current_p = get_xyz();
            d = norm(target_p - current_p);
            T_total = max(1, d/0.05); % if not specifying a time for movement, min 1s for movement , or at 10 cm/s 
        end
        
        dt_target = 0.1; % change to 0.02? 50Hz
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

    % ----------------------------------------------------------
    function success = move_linear(target_p, phi, T_total)
        success = false;
        p0 = get_xyz();
        
        if nargin < 3 || isempty(T_total)
            d = norm(target_p - p0);
            T_total = max(1, d/0.10);
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
            
            move_robot(th - th_home);
            while toc(t0) < (k-1)*dt; end
        end
        success = true;
    end
    % ----------------------------------------------------------
    function set_view(xl, yl, zl, az, el)
        % Set camera to a custom view for zooming in on a specific region.
        % xl/yl/zl : [min max] axis limits (m). Pass [] to keep current.
        % az/el    : azimuth and elevation angles (deg). Pass [] for defaults.
        %
        % Example – zoom to bridge workspace:
        %   r.set_view([0.00 0.25], [-0.05 0.05], [0.03 0.20], 0, 10);
        if nargin < 4 || isempty(az), az = 35; end
        if nargin < 5 || isempty(el), el = 20; end
        if ~isempty(VS.ax) && isvalid(VS.ax)
            if ~isempty(xl), xlim(VS.ax, xl); end
            if ~isempty(yl), ylim(VS.ax, yl); end
            if ~isempty(zl), zlim(VS.ax, zl); end
            view(VS.ax, az, el);
            drawnow limitrate;
        end
    end

    % =========================================================
    %  PRIVATE HELPERS
    % =========================================================

    function log_state(t)
        VS.traj_log.time(end+1)   = t;
        VS.traj_log.xyz(:,end+1)  = get_xyz()';
        VS.traj_log.q(:,end+1)    = VS.q';
    end

    % ------ Forward kinematics for a single position ----------
    function p = fk_xyz(th, params)
        th2 = th(2) + params.deltaSign * params.delta;
        r = params.L1*cos(th2) + params.L2*cos(th2+th(3)) + params.Le*cos(th2+th(3)+th(4));
        z = params.h + params.L1*sin(th2) + params.L2*sin(th2+th(3)) + params.Le*sin(th2+th(3)+th(4));
        p = [r*cos(th(1)), r*sin(th(1)), z];
    end

    % ------ Full DH chain for visualisation ------------------
    function [dummy_Ts, Ps] = fk_chain(th) %#ok<STOUT>
        th_eff = th;
        th_eff(2) = th_eff(2) + S.params.deltaSign * S.params.delta;
        DH = [0,           pi/2, S.params.h,  th_eff(1);
              S.params.L1, 0,    0,           th_eff(2);
              S.params.L2, 0,    0,           th_eff(3);
              S.L3,        0,    0,           th_eff(4);
              S.Lg,        0,    0,           0        ];

        Ps = zeros(size(DH,1)+1, 3);
        T  = eye(4);
        dummy_Ts = T; % unused but required for output declaration
        Ps(1,:) = T(1:3,4)';
        for i = 1:size(DH,1)
            a  = DH(i,1); al = DH(i,2); d = DH(i,3); thi = DH(i,4);
            Ti = [cos(thi), -sin(thi)*cos(al),  sin(thi)*sin(al), a*cos(thi);
                  sin(thi),  cos(thi)*cos(al), -cos(thi)*sin(al), a*sin(thi);
                  0,         sin(al),           cos(al),          d;
                  0,         0,                 0,                1];
            T = T * Ti;
            Ps(i+1,:) = T(1:3,4)';
        end
    end

    % ------ Draw / update robot ------------------------------
    function redraw_robot()
        if isempty(VS.fig) || ~isvalid(VS.fig), return; end

        [dummy_Ts, Ps] = fk_chain(VS.th); %#ok<ASGLU>

        % Gripper offset along end-effector direction
        g_spread = 0.018;
        ee_dir   = (Ps(end,:) - Ps(end-1,:));
        if norm(ee_dir) > 0, ee_dir = ee_dir / norm(ee_dir); end
        perp = cross(ee_dir, [0 0 1]);
        if norm(perp) < 1e-6, perp = cross(ee_dir, [1 0 0]); end
        perp = perp / norm(perp);
        if VS.gripper_open, spread = g_spread; else, spread = 0.005; end
        grip_L = Ps(end,:) + perp*spread;
        grip_R = Ps(end,:) - perp*spread;

        % Link colours (shoulder→wrist gradient)
        link_colors = {[0.95 0.40 0.20],  ... % link 1 – orange-red
                       [0.95 0.70 0.10],  ... % link 2 – amber
                       [0.30 0.75 0.95],  ... % link 3 – cyan
                       [0.55 0.40 0.95],  ... % link 4 – violet
                       [0.55 0.40 0.95]}; ... % ee stub

        nLinks = size(Ps,1)-1;

        if isempty(VS.hLinks) || ~isvalid(VS.hLinks(1))
            VS.hLinks = gobjects(nLinks, 1);
            for i = 1:nLinks
                VS.hLinks(i) = plot3(VS.ax, ...
                    [Ps(i,1) Ps(i+1,1)], [Ps(i,2) Ps(i+1,2)], [Ps(i,3) Ps(i+1,3)], ...
                    '-o', 'LineWidth', 5, ...
                    'Color',      link_colors{min(i,end)}, ...
                    'MarkerSize', 6, ...
                    'MarkerFaceColor', [0.85 0.85 0.85]);
            end
            % Joint spheres at base
            plot3(VS.ax, 0, 0, 0, 'w.', 'MarkerSize', 18);

            % Gripper jaws
            VS.hGrip(1) = plot3(VS.ax, [Ps(end,1) grip_L(1)], ...
                                        [Ps(end,2) grip_L(2)], ...
                                        [Ps(end,3) grip_L(3)], ...
                                 '-', 'Color',[0.90 0.90 0.90],'LineWidth',4);
            VS.hGrip(2) = plot3(VS.ax, [Ps(end,1) grip_R(1)], ...
                                        [Ps(end,2) grip_R(2)], ...
                                        [Ps(end,3) grip_R(3)], ...
                                 '-', 'Color',[0.90 0.90 0.90],'LineWidth',4);
        else
            for i = 1:nLinks
                set(VS.hLinks(i), ...
                    'XData', [Ps(i,1) Ps(i+1,1)], ...
                    'YData', [Ps(i,2) Ps(i+1,2)], ...
                    'ZData', [Ps(i,3) Ps(i+1,3)]);
            end
            set(VS.hGrip(1), 'XData',[Ps(end,1) grip_L(1)], ...
                              'YData',[Ps(end,2) grip_L(2)], ...
                              'ZData',[Ps(end,3) grip_L(3)]);
            set(VS.hGrip(2), 'XData',[Ps(end,1) grip_R(1)], ...
                              'YData',[Ps(end,2) grip_R(2)], ...
                              'ZData',[Ps(end,3) grip_R(3)]);
        end

        % Update trace
        VS.traceXYZ(:,end+1) = Ps(end,:)';
        set(VS.hTrace, 'XData', VS.traceXYZ(1,:), ...
                       'YData', VS.traceXYZ(2,:), ...
                       'ZData', VS.traceXYZ(3,:));

        drawnow limitrate;
    end

    % ------ Faint workspace boundary box --------------------
    function draw_workspace_box()
        xl = S.X_LIM; yl = S.Y_LIM; zl = S.Z_LIM;
        c  = [0.25 0.30 0.40];
        lw = 0.8;
        corners = [xl(1) yl(1) zl(1); xl(2) yl(1) zl(1);
                   xl(2) yl(2) zl(1); xl(1) yl(2) zl(1); xl(1) yl(1) zl(1)];
        plot3(VS.ax, corners(:,1), corners(:,2), corners(:,3), '--','Color',c,'LineWidth',lw);
        corners2 = corners; corners2(:,3) = zl(2);
        plot3(VS.ax, corners2(:,1), corners2(:,2), corners2(:,3), '--','Color',c,'LineWidth',lw);
        for i = 1:4
            plot3(VS.ax,[corners(i,1) corners2(i,1)], ...
                        [corners(i,2) corners2(i,2)], ...
                        [corners(i,3) corners2(i,3)], '--','Color',c,'LineWidth',lw);
        end
        text(VS.ax, xl(2), yl(2), zl(2), '  workspace', ...
             'Color',[0.35 0.45 0.60],'FontSize',8);
    end

    % ------ Diagnostic plots on close -----------------------
    function plot_diagnostics()
        if isempty(VS.traj_log.time), return; end

        t   = VS.traj_log.time;
        xyz = VS.traj_log.xyz;
        q   = VS.traj_log.q;

        fig_d = figure('Color',[0.07 0.07 0.10], ...
                       'Name','VirtualRobot – Diagnostics', ...
                       'NumberTitle','off');

        ax_bg  = [0.10 0.10 0.14];
        ax_fg  = [0.55 0.75 1.00];
        ax_gc  = [0.20 0.25 0.35];

        % -- XYZ trajectory --
        subplot(3,2,[1 2]);
        ax1 = gca;
        set(ax1,'Color',ax_bg,'XColor',ax_fg,'YColor',ax_fg,'GridColor',ax_gc);
        hold on; grid on;
        cols = {[0.95 0.40 0.20],[0.30 0.85 0.60],[0.80 0.60 0.95]};
        labs = {'X','Y','Z'};
        for i=1:3
            plot(ax1, t, xyz(i,:), '-','LineWidth',2,'Color',cols{i},'DisplayName',labs{i});
        end
        legend(ax1,'TextColor',[0.85 0.85 0.85],'EdgeColor',[0.25 0.30 0.40]);
        title(ax1,'End-Effector XYZ','Color',[0.85 0.92 1.00],'FontSize',11);
        ylabel(ax1,'Position (m)','Color',ax_fg);

        % -- Joint angles --
        subplot(3,2,[3 4]);
        ax2 = gca;
        set(ax2,'Color',ax_bg,'XColor',ax_fg,'YColor',ax_fg,'GridColor',ax_gc);
        hold on; grid on;
        jcols = {[0.95 0.70 0.10],[0.30 0.75 0.95],[0.55 0.40 0.95],[0.95 0.40 0.60]};
        for j=1:4
            plot(ax2, t, rad2deg(q(j,:)), '-','LineWidth',2,'Color',jcols{j},'DisplayName',sprintf('J%d',j));
        end
        legend(ax2,'TextColor',[0.85 0.85 0.85],'EdgeColor',[0.25 0.30 0.40]);
        title(ax2,'Joint Angles','Color',[0.85 0.92 1.00],'FontSize',11);
        ylabel(ax2,'Angle (deg)','Color',ax_fg);

        % -- Joint velocities --
        subplot(3,2,[5 6]);
        ax3 = gca;
        set(ax3,'Color',ax_bg,'XColor',ax_fg,'YColor',ax_fg,'GridColor',ax_gc);
        hold on; grid on;
        if numel(t) > 1
            dt_v  = diff(t);
            dq    = diff(q, 1, 2);
            vel   = dq ./ dt_v;
            t_v   = t(1:end-1);
            for j=1:4
                plot(ax3, t_v, rad2deg(vel(j,:)), '-','LineWidth',1.5,'Color',jcols{j},'DisplayName',sprintf('J%d',j));
            end
        end
        legend(ax3,'TextColor',[0.85 0.85 0.85],'EdgeColor',[0.25 0.30 0.40]);
        title(ax3,'Joint Velocities','Color',[0.85 0.92 1.00],'FontSize',11);
        ylabel(ax3,'Velocity (deg/s)','Color',ax_fg);
        xlabel(ax3,'Time (s)','Color',ax_fg);

        sgtitle(fig_d,'Virtual Robot  –  Session Diagnostics', ...
                'Color',[0.85 0.92 1.00],'FontSize',13,'FontWeight','bold');
    end

end  % VirtualRobotDriver


% ================================================================
%  IK  (copy here so the driver is self-contained)
%  Identical to ik_pos_only_between_jaws used by RobotDriver_sync
% ================================================================
function [th, isReachable, debug] = ik_pos_only_between_jaws(p, phi, elbowSign, params)
    x = p(1); y = p(2); z = p(3);
    h = params.h; L1 = params.L1; L2 = params.L2;
    Le = params.Le; delta = params.delta; deltaSign = params.deltaSign;

    th1 = atan2(y, x);
    r   = hypot(x, y);
    rw  = r  - Le*cos(phi);
    zw  = z  - Le*sin(phi);
    zp  = zw - h;

    c3 = (rw^2 + zp^2 - L1^2 - L2^2) / (2*L1*L2);
    debug = struct('r',r,'rw',rw,'zw',zw,'zp',zp,'c3',c3);

    if abs(c3) > 1
        th = [NaN NaN NaN NaN]; isReachable = false; return;
    end

    s3      = elbowSign * sqrt(max(0, 1 - c3^2));
    th3     = atan2(s3, c3);
    beta    = atan2(zp, rw);
    gamma   = atan2(L2*sin(th3), L1 + L2*cos(th3));
    th2_eff = beta - gamma;
    th2     = th2_eff - deltaSign*delta;
    th4     = phi - th2_eff - th3;

    th = [th1 th2 th3 th4];
    isReachable = true;
    debug.th1 = th1; debug.th2 = th2;
    debug.th3 = th3; debug.th4 = th4;
end
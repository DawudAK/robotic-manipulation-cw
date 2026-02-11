function task1d_square_trace
% Task 1d: Trace 10x10 cm squares in XY, XZ, YZ planes using analytic IK
% - Re-runs IK at each waypoint
% - Animates stickman robot via FK
% - Draws: (i) reference square outlines (3 colours), (ii) traced path so far (black)
% - Records MP4 video
%
% REQUIREMENTS ON PATH:
%   - ik_pos_only_between_jaws.m   (analytic IK function)
%   - fk_chain_from_DH.m           (your DH FK helper returning [Ts, Ps])
%   - plot_frames.m                (your frame drawing helper)

clear; clc; close all;

%% ---------------- Parameters (MATCH YOUR FK/IK MODEL) ----------------
% Shoulder kink spec
v_sh = 0.128;
x_sh = 0.024;

params.h  = 0.077;
params.L1 = hypot(v_sh, x_sh);
params.L2 = 0.124;
params.delta = atan2(x_sh, v_sh);
params.deltaSign = -1;              % you found -1 matches your modelling

% Wrist->gripper body and jaw-centre offset
L3 = 0.126;
Lg = 0.205/2;                       % approx jaw-centre offset (between jaws)
params.Le = L3 + Lg;                % wrist -> jaw-centre effective

% Home offsets (q=0 home pose: up then forward, gripper forward)
th0 = [0,  pi/2 + params.delta,  -pi/2,  0];

% End-effector pitch constraint for position-only IK:
% phi = 0 means gripper "forward/horizontal" in the (r,z) plane.
phi = 0;

% Square size + animation pacing
side = 0.10;         % 10 cm
Nedge = 60;          % points per edge (smoothness)
pause_dt = 0.05;     % visible animation pacing (seconds). Increase if still too fast.

%% ---------------- Choose square centres (REACHABLE) ----------------
% If any corner becomes unreachable, reduce radius (x) or lower z.
cXY = [0.18; 0.00; 0.22];   % XY plane square centre, fixed z
cXZ = [0.20; 0.00; 0.22];   % XZ plane square centre, fixed y (slightly shifted for clarity)
cYZ = [0.16; 0.00; 0.22];   % YZ plane square centre, fixed x (slightly shifted for clarity)

%% ---------------- Build squares (waypoints) ----------------
P_XY = make_square_points('XY', cXY, side, Nedge);
P_XZ = make_square_points('XZ', cXZ, side, Nedge);
P_YZ = make_square_points('YZ', cYZ, side, Nedge);

%% ---------------- Figure ----------------
fig = figure('Color','w','Name','Task 1d: IK Square Tracing');
ax = axes(fig); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal'); view(ax,3);
xlabel(ax,'X (m)'); ylabel(ax,'Y (m)'); zlabel(ax,'Z (m)');
xlim(ax,[-0.05 0.30]); ylim(ax,[-0.20 0.20]); zlim(ax,[0.00 0.35]);

% Draw reference square outlines (different colours)
colXY = [0 0.45 0.74];       % blue-ish
colXZ = [0.85 0.33 0.10];    % orange-ish
colYZ = [0.47 0.67 0.19];    % green-ish
draw_polyline(ax, P_XY, colXY, 2.5);
draw_polyline(ax, P_XZ, colXZ, 2.5);
draw_polyline(ax, P_YZ, colYZ, 2.5);

legend(ax, {'XY square','XZ square','YZ square'}, 'Location','northwest');

%% ---------------- Video Writer ----------------
recordVideo = true;
if recordVideo
    v = VideoWriter('task1d_square_trace.mp4','MPEG-4');
    v.FrameRate = 30;
    open(v);
end

%% ---------------- Animate each plane ----------------
% Continuity: pick elbow-up/down per point based on smallest joint change.
th_prev = [];

% Robot graphics handles
hLinks = gobjects(0);
hFramesGroup = gobjects(1);

% Trace XY
disp('Tracing XY square...');
[th_prev, hLinks, hFramesGroup] = trace_path(P_XY, 'XY', phi, params, th0, ax, fig, ...
    hLinks, hFramesGroup, th_prev, pause_dt, recordVideo, colXY);

% Trace XZ
disp('Tracing XZ square...');
[th_prev, hLinks, hFramesGroup] = trace_path(P_XZ, 'XZ', phi, params, th0, ax, fig, ...
    hLinks, hFramesGroup, th_prev, pause_dt, recordVideo, colXZ);

% Trace YZ
disp('Tracing YZ square...');
[th_prev, hLinks, hFramesGroup] = trace_path(P_YZ, 'YZ', phi, params, th0, ax, fig, ...
    hLinks, hFramesGroup, th_prev, pause_dt, recordVideo, colYZ);

%% ---------------- Close video ----------------
if recordVideo
    close(v);
    disp('Saved: task1d_square_trace.mp4');
end

%% ===================== NESTED FUNCTIONS =====================

    function P = make_square_points(plane, c, s, nEdge)
        % Returns 3xN points forming a closed square polyline.
        half = s/2;

        switch upper(plane)
            case 'XY'
                z = c(3);
                corners = [c(1)-half, c(2)-half, z;
                           c(1)+half, c(2)-half, z;
                           c(1)+half, c(2)+half, z;
                           c(1)-half, c(2)+half, z;
                           c(1)-half, c(2)-half, z]';
            case 'XZ'
                y = c(2);
                corners = [c(1)-half, y, c(3)-half;
                           c(1)+half, y, c(3)-half;
                           c(1)+half, y, c(3)+half;
                           c(1)-half, y, c(3)+half;
                           c(1)-half, y, c(3)-half]';
            case 'YZ'
                x = c(1);
                corners = [x, c(2)-half, c(3)-half;
                           x, c(2)+half, c(3)-half;
                           x, c(2)+half, c(3)+half;
                           x, c(2)-half, c(3)+half;
                           x, c(2)-half, c(3)-half]';
            otherwise
                error('Unknown plane: %s', plane);
        end

        % Interpolate each edge
        P = [];
        for i = 1:4
            p0 = corners(:,i);
            p1 = corners(:,i+1);
            t  = linspace(0,1,nEdge);
            Pi = p0*(1-t) + p1*t;
            P  = [P, Pi]; %#ok<AGROW>
        end
    end

    function draw_polyline(axh, P, col, lw)
        line(axh, P(1,:), P(2,:), P(3,:), 'LineWidth', lw, 'Color', col);
    end

    function [th_out, hL, hFG] = trace_path(P, label, phi_local, params_local, th0_local, axh, figh, ...
                                            hL, hFG, th_prev_local, dt, recVid, labelColor)

        % Create (or reset) the live "traced path so far" line for this plane
        traceXYZ = nan(3,0);
        hTrace = plot3(axh, NaN, NaN, NaN, 'k-', 'LineWidth', 2); % traced path (black)

        % Try both elbow branches each step and pick closest to previous solution
        elbowChoices = [-1, +1];

        % We'll initialise link objects on the first iteration and then update them (smooth animation)
        linksInitialised = false;

        for k = 1:size(P,2)
            p = P(:,k);

            best.ok = false;
            best.th = [];
            best.cost = Inf;

            for es = elbowChoices
                [th_try, ok] = ik_pos_only_between_jaws(p, phi_local, es, params_local);
                if ~ok, continue; end

                if isempty(th_prev_local)
                    cost = 0;
                else
                   
                    d = atan2(sin(th_try - th_prev_local), cos(th_try - th_prev_local));

                    cost = norm(d);
                end

                if cost < best.cost
                    best.ok = true;
                    best.th = th_try;
                    best.cost = cost;
                end
            end

            if ~best.ok
                error('Unreachable waypoint in %s at k=%d: [%.3f %.3f %.3f]', label, k, p(1), p(2), p(3));
            end

            th_prev_local = best.th;

            % FK to jaw-centre (includes kink modelling and jaw offset link)
            th_eff = th_prev_local;
            th_eff(2) = th_eff(2) + params_local.deltaSign*params_local.delta;

            DH = [ 0,               pi/2, params_local.h, th_eff(1);
                   params_local.L1, 0,    0,             th_eff(2);
                   params_local.L2, 0,    0,             th_eff(3);
                   L3,              0,    0,             th_eff(4);
                   Lg,              0,    0,             0        ];

            [Ts, Ps] = fk_chain_from_DH(DH);

            % Update trace so far (jaw-centre position)
            p_ee = Ps(end,:).';
            traceXYZ(:,end+1) = p_ee;
            set(hTrace, 'XData', traceXYZ(1,:), ...
                        'YData', traceXYZ(2,:), ...
                        'ZData', traceXYZ(3,:));

            % Initialise link objects once, then update their data (smooth & visible)
            if ~linksInitialised
                % Clear any old handles from previous plane
                if ~isempty(hL), delete(hL(isgraphics(hL))); end
                if isgraphics(hFG), delete(hFG); end

                nSeg = size(Ps,1)-1;
                hL = gobjects(nSeg,1);
                for i = 1:nSeg
                    hL(i) = line(axh, ...
                        [Ps(i,1) Ps(i+1,1)], ...
                        [Ps(i,2) Ps(i+1,2)], ...
                        [Ps(i,3) Ps(i+1,3)], ...
                        'LineWidth', 5);
                end

                % Frames group (optional marks)
                hF = plot_frames(Ts, 0.03);
                hFG = hggroup;
                set(hF(:),'Parent',hFG);

                linksInitialised = true;
            else
                % Update existing links
                for i = 1:numel(hL)
                    set(hL(i), 'XData', [Ps(i,1) Ps(i+1,1)], ...
                               'YData', [Ps(i,2) Ps(i+1,2)], ...
                               'ZData', [Ps(i,3) Ps(i+1,3)]);
                end

                % Re-draw frames (simpler than updating each quiver line)
                if isgraphics(hFG), delete(hFG); end
                hF = plot_frames(Ts, 0.03);
                hFG = hggroup;
                set(hF(:),'Parent',hFG);
            end

            % Update title
            q = th_prev_local - th0_local;
            title(axh, sprintf('Tracing %s | q=[%.2f %.2f %.2f %.2f] rad', label, q(1),q(2),q(3),q(4)));

            % Optional: label current plane in the plot (small text)
            if k == 1
                text(axh, p_ee(1), p_ee(2), p_ee(3)+0.03, sprintf('%s trace', label), ...
                    'Color', labelColor, 'FontWeight','bold');
            end

            % FORCE visible animation
            drawnow limitrate;
            pause(dt);

            % Record video
            if recVid
                writeVideo(v, getframe(figh));
            end
        end

        th_out = th_prev_local;
    end

end

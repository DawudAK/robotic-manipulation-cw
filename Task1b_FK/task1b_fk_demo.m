function task1b_fk_demo
% Task 1b: Forward kinematics simulation using line() objects
% Self-contained (no Robotics Toolbox). Requires helper files:
%   - dh_T.m
%   - fk_chain_from_DH.m
%   - plot_frames.m

clear; clc; close all;

%% --- Geometry constants (metres) ---
h  = 0.077;                 % base height
L2 = 0.124;                 % elbow->wrist
L3 = 0.126;          % wrist -> gripper body (spec)
Lg = 0.205/2;        % gripper body -> jaw centre (approx, between jaws)
Le = L3 + Lg;         % wrist -> jaw-centre effective tool length
                 % jaw-centre offset (editable live)

% Shoulder "kink" offset (from appendix)
v_sh = 0.128;               % vertical component
x_sh = 0.024;               % horizontal offset component

% Option B: Pythagoras + direction effect
L1    = hypot(v_sh, x_sh);  % ~= 0.1302 (effective shoulder->elbow)
delta = atan2(x_sh, v_sh);  % ~= 0.185 rad (~10.6 deg)

% If the tilt points the wrong way in your plot, flip this to -1:
deltaSign = -1;

%% --- Angle offsets for servo zero (hardware "home" pose) ---
% q = 0 means robot is at th0.
th0 = [0,  pi/2, -pi/2, 0]; % [th1_0 th2_0 th3_0 th4_0] rad

%% --- Figure setup ---
fig = figure('Name','Task 1b FK Simulation','Color','w');
ax = axes(fig);
grid(ax,'on'); axis(ax,'equal'); view(ax,3);
xlabel(ax,'X (m)'); ylabel(ax,'Y (m)'); zlabel(ax,'Z (m)');
xlim(ax,[-0.25 0.25]); ylim(ax,[-0.25 0.25]); zlim(ax,[0 0.35]);
hold(ax,'on');

%% --- Sliders for relative joint angles q (rad) ---
s = struct();
s.th1 = uicontrol('Style','slider','Min',-pi,'Max',pi,'Value',0, ...
   'Units','normalized','Position',[0.15 0.02 0.18 0.03]);
s.th2 = uicontrol('Style','slider','Min',-pi,'Max',pi,'Value',0, ...
   'Units','normalized','Position',[0.35 0.02 0.18 0.03]);
s.th3 = uicontrol('Style','slider','Min',-pi,'Max',pi,'Value',0, ...
   'Units','normalized','Position',[0.55 0.02 0.18 0.03]);
s.th4 = uicontrol('Style','slider','Min',-pi,'Max',pi,'Value',0, ...
   'Units','normalized','Position',[0.75 0.02 0.18 0.03]);

uicontrol('Style','text','String','q1 (rad)','Units','normalized', ...
   'Position',[0.15 0.05 0.18 0.02],'BackgroundColor','w');
uicontrol('Style','text','String','q2 (rad)','Units','normalized', ...
   'Position',[0.35 0.05 0.18 0.02],'BackgroundColor','w');
uicontrol('Style','text','String','q3 (rad)','Units','normalized', ...
   'Position',[0.55 0.05 0.18 0.02],'BackgroundColor','w');
uicontrol('Style','text','String','q4 (rad)','Units','normalized', ...
   'Position',[0.75 0.05 0.18 0.02],'BackgroundColor','w');

%% --- Typed input boxes (preferred over sliders) ---
e = struct();
e.q1 = uicontrol('Style','edit','String','0', 'Units','normalized', ...
   'Position',[0.15 0.00 0.18 0.02], 'BackgroundColor','w');
e.q2 = uicontrol('Style','edit','String','0', 'Units','normalized', ...
   'Position',[0.35 0.00 0.18 0.02], 'BackgroundColor','w');
e.q3 = uicontrol('Style','edit','String','0', 'Units','normalized', ...
   'Position',[0.55 0.00 0.18 0.02], 'BackgroundColor','w');
e.q4 = uicontrol('Style','edit','String','0', 'Units','normalized', ...
   'Position',[0.75 0.00 0.18 0.02], 'BackgroundColor','w');

set(e.q1,'Callback',@(~,~)update_plot());
set(e.q2,'Callback',@(~,~)update_plot());
set(e.q3,'Callback',@(~,~)update_plot());
set(e.q4,'Callback',@(~,~)update_plot());

% Editable Lg (jaw-centre offset)
uicontrol('Style','text','String','Lg (m)','Units','normalized', ...
   'Position',[0.02 0.05 0.10 0.02],'BackgroundColor','w');
e.Lg = uicontrol('Style','edit','String',num2str(Lg,'%.4f'), 'Units','normalized', ...
   'Position',[0.02 0.02 0.10 0.03], 'BackgroundColor','w');
set(e.Lg,'Callback',@(~,~)update_plot());

% Home button
uicontrol('Style','pushbutton','String','Home (q=0)','Units','normalized', ...
   'Position',[0.02 0.00 0.10 0.02], 'Callback', @(~,~)go_home());

% Info box for delta/L1
uicontrol('Style','text', ...
    'String',sprintf('L1=%.4f m, delta=%.4f rad (%.1f deg)', L1, delta, rad2deg(delta)), ...
    'Units','normalized','Position',[0.02 0.085 0.40 0.02], ...
    'BackgroundColor','w','HorizontalAlignment','left');

%% --- Plot handles ---
hLinks  = gobjects(0);
hFrames = gobjects(1);

drawnow;

%% --- Attach slider callbacks ---
set(s.th1,'Callback',@(~,~)update_plot());
set(s.th2,'Callback',@(~,~)update_plot());
set(s.th3,'Callback',@(~,~)update_plot());
set(s.th4,'Callback',@(~,~)update_plot());

% Initial draw
update_plot();

%% --- Automated FK sanity check: sweep each joint independently ---
doSweep = true;
if doSweep
   pause(0.5);
   sweepAngles = linspace(-pi/3, pi/3, 60);
   for a = sweepAngles, set_q([a 0 0 0]); update_plot(); pause(0.01); end
   for a = sweepAngles, set_q([0 a 0 0]); update_plot(); pause(0.01); end
   for a = sweepAngles, set_q([0 0 a 0]); update_plot(); pause(0.01); end
   for a = sweepAngles, set_q([0 0 0 a]); update_plot(); pause(0.01); end
end

%% --- Optional: record a video of FK demo ---
recordVideo = true;
if recordVideo
   v = VideoWriter('task1b_fk_demo.mp4','MPEG-4');
   v.FrameRate = 15;
   open(v);
   sweepAngles = linspace(-pi/3, pi/3, 240);
   for a = sweepAngles
       set_q([a 0 0 0]);
       update_plot();
       writeVideo(v, getframe(fig));
   end
   close(v);
   disp('Saved task1b_fk_demo.mp4');
end

%% ---------------- Nested helpers ----------------
   function go_home()
       set_q([0 0 0 0]);
       update_plot();
   end

   function set_q(q)
       % Sync sliders + text boxes to the given relative angles q
       s.th1.Value = q(1); s.th2.Value = q(2); s.th3.Value = q(3); s.th4.Value = q(4);
       e.q1.String = num2str(q(1),'%.4f');
       e.q2.String = num2str(q(2),'%.4f');
       e.q3.String = num2str(q(3),'%.4f');
       e.q4.String = num2str(q(4),'%.4f');
   end

   function [q, usedText] = read_q()
       % Prefer typed values. If any are invalid, fall back to sliders.
       q_txt = [str2double(e.q1.String), str2double(e.q2.String), ...
                str2double(e.q3.String), str2double(e.q4.String)];
       usedText = all(isfinite(q_txt));
       if usedText
           q = q_txt;
       else
           q = [s.th1.Value, s.th2.Value, s.th3.Value, s.th4.Value];
       end
   end

   function update_plot()
       % Update editable Lg if typed correctly
       Lg_new = str2double(e.Lg.String);
       if isfinite(Lg_new)
           Lg = Lg_new;
       else
           e.Lg.String = num2str(Lg,'%.4f');
       end

       % Read relative joint angles q, then convert to actual th
       [q, usedText] = read_q();
       th = q + th0;

       % Keep UI in sync
       if usedText
           s.th1.Value = q(1); s.th2.Value = q(2); s.th3.Value = q(3); s.th4.Value = q(4);
       else
           e.q1.String = num2str(q(1),'%.4f');
           e.q2.String = num2str(q(2),'%.4f');
           e.q3.String = num2str(q(3),'%.4f');
           e.q4.String = num2str(q(4),'%.4f');
       end

       % ----- Direction effect for the 0.024 m offset (Option B) -----
       % Apply fixed tilt delta to the shoulder angle used in FK.
       th_eff = th;
       th_eff(2) = th_eff(2) + deltaSign * delta;

       % Clean standard DH (Craig): [a alpha d theta]
       DH = [ 0,   pi/2, h,  th_eff(1);
              L1,  0,    0,  th_eff(2);
              L2,  0,    0,  th_eff(3);
              L3,  0,    0,  th_eff(4);
              Lg,  0,    0,  0   ];

       [Ts, Ps] = fk_chain_from_DH(DH);

       % Clear old graphics
       delete(hLinks(isgraphics(hLinks)));
       if isgraphics(hFrames), delete(hFrames); end

       % Draw stickman links (line objects)
       nSeg = size(Ps,1)-1;
       hLinks = gobjects(nSeg,1);
       for i = 1:nSeg
           hLinks(i) = line(ax, ...
               [Ps(i,1) Ps(i+1,1)], ...
               [Ps(i,2) Ps(i+1,2)], ...
               [Ps(i,3) Ps(i+1,3)], ...
               'LineWidth', 5);
       end

       % Draw frames (optional marks)
       hF = plot_frames(Ts, 0.03);
       hFrames = hggroup;
       set(hF(:),'Parent',hFrames);

       title(ax, sprintf(['q = [%.2f %.2f %.2f %.2f] rad | th = q+th0 = [%.2f %.2f %.2f %.2f] rad | ' ...
                          'delta = %+.3f rad -> th2_eff = %.2f rad'], ...
           q(1),q(2),q(3),q(4), th(1),th(2),th(3),th(4), deltaSign*delta, th_eff(2)));

       drawnow;
   end
end



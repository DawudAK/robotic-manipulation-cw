function omx_smooth_control_demo()
% DEMO ENTRY POINT (run this)
% - Setup on COM9 with IDs 11-14 (arm) and 15 (gripper)
% - Smooth IK move to a Cartesian point (jaw-centre) using *delta-from-current* correction
%
% Run:
%   omx_smooth_control_demo

clc; clear; close all;

% ---- Setup ----
S = robot_setup_omx("COM5", 1000000, [11 12 13 14], 15);

robot_move_joints_smooth_rel(S, deg2rad([10, 5, 15, 10]), 2.0);
% ---- Example 2: smooth Cartesian IK move (jaw-centre) ----
% {
% try
%     % Use a reachable target (avoid x=y=0; avoid negative x unless you intend it)
%     p_goal = [0.00; 0.00; 0.00];
%     phi    = 0;
%     robot_move_cartesian_ik_smooth(S, p_goal, phi, 3.0);
% catch ME
%     disp("IK move failed. Error:");
%     disp(ME.message);
% end
 
% ---- Shutdown ----
robot_shutdown_omx(S);
end

%% =====================================================================
%%  SETUP / SHUTDOWN
%% =====================================================================
function S = robot_setup_omx(comPort, baudrate, joint_ids, grip_id)
% Safe setup: Position mode, gentle profile, torque enable.

S = struct();
S.DEVICENAME = string(comPort);
S.BAUDRATE   = double(baudrate);
S.PROTOCOL   = 2.0;

S.JOINT_IDS  = joint_ids(:).'; % ensure row
S.GRIP_ID    = grip_id;

% Gentle motion profiles (tune later)
S.profile_accel = 15;
S.profile_vel   = 40;

% Control table (X-series, Protocol 2.0)
S.ADDR_OPERATING_MODE   = 11;
S.ADDR_TORQUE_ENABLE    = 64;
S.ADDR_PROFILE_ACCEL    = 108;
S.ADDR_PROFILE_VEL      = 112;
S.ADDR_GOAL_POSITION    = 116;
S.ADDR_PRESENT_POSITION = 132;

S.OPMODE_POSITION = 3;

% Ticks <-> rad
S.TICKS_PER_REV = 4096;
S.TWO_PI = 2*pi;

% Joint direction signs (your calibrated values)
S.dir = [ +1 -1 -1 -1 ];

% Load SDK library
S.lib_name = load_dxl_library();

% Open port
S.port_num = portHandler(char(S.DEVICENAME));
packetHandler();

assert(openPort(S.port_num), "Failed to open port %s", S.DEVICENAME);
assert(setBaudRate(S.port_num, S.BAUDRATE), "Failed to set baudrate %d", S.BAUDRATE);

disp("Port open: " + S.DEVICENAME + " @ " + num2str(S.BAUDRATE));

% Configure IDs (arm only for now)
ids = S.JOINT_IDS;

% Torque OFF to change mode
for id = ids
    write1ByteTxRx(S.port_num, S.PROTOCOL, id, S.ADDR_TORQUE_ENABLE, 0);
end

% Position mode + profiles
for id = ids
    write1ByteTxRx(S.port_num, S.PROTOCOL, id, S.ADDR_OPERATING_MODE, S.OPMODE_POSITION);
    write4ByteTxRx(S.port_num, S.PROTOCOL, id, S.ADDR_PROFILE_ACCEL, S.profile_accel);
    write4ByteTxRx(S.port_num, S.PROTOCOL, id, S.ADDR_PROFILE_VEL,   S.profile_vel);
end

% Torque ON
for id = ids
    write1ByteTxRx(S.port_num, S.PROTOCOL, id, S.ADDR_TORQUE_ENABLE, 1);
end

disp("Position mode set, profile applied, torque enabled.");

% Capture "software home" = current ticks (treat as q=0 reference)
S.home_ticks = read_joint_ticks(S);
disp("Software home captured (present ticks, treated as q=0).");
disp(S.home_ticks);
end

function robot_shutdown_omx(S)
ids = S.JOINT_IDS;
for id = ids
    write1ByteTxRx(S.port_num, S.PROTOCOL, id, S.ADDR_TORQUE_ENABLE, 0);
end
closePort(S.port_num);
disp("Torque disabled, port closed.");
end

function lib_name = load_dxl_library()
lib_name = "";
if strcmp(computer, 'PCWIN')
    lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
    lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
    lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
    lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
    lib_name = 'libdxl_mac_c';
end

if ~libisloaded(lib_name)
    loadlibrary(lib_name, 'dynamixel_sdk.h', ...
        'addheader', 'port_handler.h', ...
        'addheader', 'packet_handler.h');
end
end

%% =====================================================================
%%  LOW-LEVEL READ/WRITE + CONVERSIONS
%% =====================================================================
function ticks = read_joint_ticks(S)
ticks = zeros(1,4,'uint32');
for i = 1:4
    id = S.JOINT_IDS(i);
    ticks(i) = uint32(read4ByteTxRx(S.port_num, S.PROTOCOL, id, S.ADDR_PRESENT_POSITION));
end
end

function write_joint_ticks(S, ticks)
for i = 1:4
    id = S.JOINT_IDS(i);
    write4ByteTxRx(S.port_num, S.PROTOCOL, id, S.ADDR_GOAL_POSITION, uint32(ticks(i)));
end
end

function dt = rad_to_ticks_delta(S, q_rad)
% q_rad: 1x4 rad
dt = int32(round( (q_rad .* S.dir) * (S.TICKS_PER_REV / S.TWO_PI) ));
end

function q_rad = ticks_delta_to_rad(S, dticks)
% dticks: int32 1x4 (ticks - home_ticks)
q_rad = (double(dticks) ./ (S.TICKS_PER_REV / S.TWO_PI)) ./ S.dir;
end

%% =====================================================================
%%  SMOOTH JOINT MOTION (FASTEST IN MIDDLE)
%% =====================================================================
function robot_move_joints_smooth_rel(S, q_rel, duration_s)
assert(numel(q_rel)==4, "q_rel must be 1x4 (rad)");

ticks0 = read_joint_ticks(S);
dticks = rad_to_ticks_delta(S, q_rel);
ticksT = uint32(double(ticks0) + double(dticks));

send_ticks_trajectory(S, ticks0, ticksT, duration_s);
end

function send_ticks_trajectory(S, ticks0, ticksT, duration_s)
Hz = 25;
N  = max(8, round(duration_s * Hz));

for k = 1:N
    t = (k-1)/(N-1);
    s = 3*t^2 - 2*t^3;  % cubic ease-in/ease-out

    ticksK = uint32(round( (1-s)*double(ticks0) + s*double(ticksT) ));
    write_joint_ticks(S, ticksK);

    pause(1/Hz);
end
end

%% =====================================================================
%%  CARTESIAN IK MOVE (JAW-CENTRE) -> RELATIVE JOINT MOVE (FIXED)
%% =====================================================================
function robot_move_cartesian_ik_smooth(S, p, phi, duration_s)
assert(all(size(p)==[3 1]), "p must be 3x1");

params = build_ik_params();

% --- Solve IK (try elbow-down then up) ---
ok = false;
for elbowSign = [-1, +1]
    [th, reachable] = ik_pos_only_between_jaws(p, phi, elbowSign, params);
    if reachable
        ok = true;
        break;
    end
end
assert(ok, "IK unreachable for p=[%.3f %.3f %.3f]", p(1),p(2),p(3));

% Model home offsets
th0 = [0,  pi/2 + params.delta,  -pi/2,  0];

% IK target in "model q" coordinates (absolute in model sense)
q_abs = th - th0;

% --- NEW: compute current q from present ticks relative to software home ---
ticks_now = read_joint_ticks(S);
dticks_now = int32(ticks_now) - int32(S.home_ticks);
q_now = ticks_delta_to_rad(S, dticks_now);

% --- Move by the difference (this is the critical fix) ---
dq = q_abs - q_now;

% Safety clamp (start conservative)
maxStep = deg2rad(12);
dq = max(min(dq, maxStep), -maxStep);

% Execute as relative smooth move
robot_move_joints_smooth_rel(S, dq, duration_s);
end

function params = build_ik_params()
v_sh = 0.128; x_sh = 0.024;

params.h  = 0.077;
params.L1 = hypot(v_sh, x_sh);
params.L2 = 0.124;

params.delta = atan2(x_sh, v_sh);
params.deltaSign = -1;

L3 = 0.126;
Lg = 0.205/2;
params.Le = L3 + Lg;
end
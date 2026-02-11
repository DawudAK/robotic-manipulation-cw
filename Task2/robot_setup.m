function S = robot_setup()
% OpenMANIPULATOR-X (Dynamixel SDK, MATLAB)
% Safe setup: Position mode, low profile velocity/accel, torque enable
% Joint IDs: 12 13 14 15

clc;

%% ---- EDIT THESE ----
S.DEVICENAME = 'COM5';          
S.BAUDRATE   = 1000000;         
S.PROTOCOL   = 2.0;

S.JOINT_IDS  = [12 13 14 15];   % joint1..4
S.USE_GRIPPER = true;          % start with arm only for safety
S.GRIP_ID    = 16;              % change if you know it; ignored if USE_GRIPPER=false

% Very gentle motion profiles
S.profile_accel = 30;
S.profile_vel   = 60;

%% ---- Control table (XM/X-series, Protocol 2.0) ----
S.ADDR_OPERATING_MODE   = 11;
S.ADDR_TORQUE_ENABLE    = 64;
S.ADDR_PROFILE_ACCEL    = 108;
S.ADDR_PROFILE_VEL      = 112;
S.ADDR_GOAL_POSITION    = 116;
S.ADDR_PRESENT_POSITION = 132;

S.OPMODE_POSITION = 3;

%% ---- Ticks <-> rad ----
S.TICKS_PER_REV = 4096;
S.TWO_PI = 2*pi;

%% ---- Load SDK ----
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
S.lib_name = lib_name;

%% ---- Open port ----
S.port_num = portHandler(S.DEVICENAME);
packetHandler();

assert(openPort(S.port_num), "Failed to open port");
assert(setBaudRate(S.port_num, S.BAUDRATE), "Failed to set baudrate");

disp("Port open.");

%% ---- IDs we will configure ----
ids = S.JOINT_IDS;
if S.USE_GRIPPER
    ids = [ids, S.GRIP_ID];
end

% Torque off to change mode
for id = ids
    write1ByteTxRx(S.port_num, S.PROTOCOL, id, S.ADDR_TORQUE_ENABLE, 0);
end

% Position mode + slow profiles
for id = ids
    write1ByteTxRx(S.port_num, S.PROTOCOL, id, S.ADDR_OPERATING_MODE, S.OPMODE_POSITION);
    write4ByteTxRx(S.port_num, S.PROTOCOL, id, S.ADDR_PROFILE_ACCEL, S.profile_accel);
    write4ByteTxRx(S.port_num, S.PROTOCOL, id, S.ADDR_PROFILE_VEL,   S.profile_vel);
end

% Torque on
for id = ids
    write1ByteTxRx(S.port_num, S.PROTOCOL, id, S.ADDR_TORQUE_ENABLE, 1);
end

disp("Position mode set, slow profile applied, torque enabled.");

%% ---- Capture "software home" (current pose) ----
S.home_ticks = read_joint_ticks(S);
disp("Software home captured (current pose).");

end

%% ===== helper =====
function ticks = read_joint_ticks(S)
ticks = zeros(1,4,'uint32');
for i = 1:4
    id = S.JOINT_IDS(i);
    ticks(i) = uint32(read4ByteTxRx(S.port_num, S.PROTOCOL, id, S.ADDR_PRESENT_POSITION));
end
end


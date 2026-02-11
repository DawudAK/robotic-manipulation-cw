function robot_shutdown(S)
ids = S.JOINT_IDS;
if isfield(S,'USE_GRIPPER') && S.USE_GRIPPER
    ids = [ids, S.GRIP_ID];
end

for id = ids
    write1ByteTxRx(S.port_num, S.PROTOCOL, id, S.ADDR_TORQUE_ENABLE, 0);
end
closePort(S.port_num);
disp("Torque disabled, port closed.");
end

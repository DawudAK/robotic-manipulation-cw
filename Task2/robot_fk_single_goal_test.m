function robot_fk_single_goal_test(S, q_rel)
% Sends ONE smooth goal instead of many small ones

ticks0 = read_joint_ticks(S);
dticks = rad_to_ticks_delta(S, q_rel);
ticksT = uint32(double(ticks0) + double(dticks));

write_joint_ticks(S, ticksT);
disp('Goal sent. Let the robot move...');
end

%% helpers
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
    write4ByteTxRx(S.port_num, S.PROTOCOL, id, S.ADDR_GOAL_POSITION, ticks(i));
end
end

function dt = rad_to_ticks_delta(S, q)
dt = int32(round(q * (S.TICKS_PER_REV / S.TWO_PI)));
end

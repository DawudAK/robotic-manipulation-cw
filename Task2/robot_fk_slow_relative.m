function robot_fk_slow_relative(S, q_rel, duration_s)
% q_rel: 1x4 radians (small at first!)
% duration_s: total time for move

assert(numel(q_rel)==4);

% Read current ticks as start
ticks0 = read_joint_ticks(S);

% Convert relative radians -> relative ticks
dticks = rad_to_ticks_delta(S, q_rel);

% Target ticks (relative move)
ticksT = uint32(double(ticks0) + double(dticks));

% Interpolate in ticks (slow & smooth)
Hz = 25;
N  = max(10, round(duration_s*Hz));
for k = 1:N
    t = (k-1)/(N-1);
    ticksK = uint32(round( (1-t)*double(ticks0) + t*double(ticksT) ));
    write_joint_ticks(S, ticksK);
    drawnow;
    pause(1/Hz);
end
end

%% ===== helpers =====
function ticks = read_joint_ticks(S)
ticks = zeros(1,4,'uint32');
for i = 1:4
    id = S.JOINT_IDS(i);
    ticks(i) = uint32(read4ByteTxRx(S.port_num, S.PROTOCOL, id, 132));
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

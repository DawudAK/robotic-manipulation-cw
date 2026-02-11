function dxl_jog_one(S, id, delta_deg)
% Jog ONE servo by a small amount (deg) and return to start.
% Uses present position as baseline, so it's safe.

delta = deg2rad(delta_deg);
ticks0 = uint32(read4ByteTxRx(S.port_num, S.PROTOCOL, id, S.ADDR_PRESENT_POSITION));
dticks = int32(round(delta * (S.TICKS_PER_REV / S.TWO_PI)));

ticksT = uint32(double(ticks0) + double(dticks));

write4ByteTxRx(S.port_num, S.PROTOCOL, id, S.ADDR_GOAL_POSITION, ticksT);
pause(2.0);
write4ByteTxRx(S.port_num, S.PROTOCOL, id, S.ADDR_GOAL_POSITION, ticks0);
pause(2.0);

fprintf("Jogged ID %d by %+g deg and returned.\n", id, delta_deg);
end

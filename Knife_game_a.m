if ~exist('robot','var') || ~isfield(robot,'read')
    robot = RobotDriver_sync();
    robot.setup();
    pause(1);
else
    fprintf('[CONNECT] Using existing robot connection.\n');
end

Z_LIFT = 0.105;
PHI    = -1.3;
DT     = 0.015;

params.h         = 0.077;
params.L1        = hypot(0.128, 0.024);
params.L2        = 0.124;
params.Le        = 0.126;
params.delta     = atan2(0.024, 0.128);
params.deltaSign = -1;
th_home          = [0, pi/2 + params.delta, -pi/2, 0];

z_stab = 0.07;

GAP_POS = [...
    0.063,   0.083,   z_stab;   % gap 1 - outside thumb   
    0.1056,  0.048,   0.05;     % gap 2 - thumb index
    0.1822,  0.047,   0.065;    % gap 3 - index middle
    0.182,   0.007,   0.065;    % gap 4 - middle ring
    0.182,  -0.0275,  z_stab;   % gap 5 - ring pinky
    0.109,  -0.0534,  0.052];   % gap 6 - outside pinky  

robot.torque(1)
N_GAPS = size(GAP_POS, 1);

seg_dists    = sqrt(sum(diff(GAP_POS).^2, 2));
mean_spacing = mean(seg_dists);

classic_seq = [];
for t = 2:N_GAPS
    classic_seq = [classic_seq, 1, t];
end
for t = (N_GAPS-1):-1:1
    classic_seq = [classic_seq, N_GAPS, t];
end

rounds(1).label     = 'Round 1 - Slow';
rounds(1).speed_cms = 7;
rounds(1).seq       = classic_seq;
rounds(1).n_seq     = 2;
rounds(1).z_offset  = 0.000;   % no correction at slow speed

rounds(2).label     = 'Round 2 - Medium';
rounds(2).speed_cms = 8;
rounds(2).seq       = classic_seq;
rounds(2).n_seq     = 2;
rounds(2).z_offset  = 0.007;   % +3mm at medium speed

rounds(3).label     = 'Round 3 - Fast';
rounds(3).speed_cms = 11;
rounds(3).seq       = classic_seq;
rounds(3).n_seq     = 2;
rounds(3).z_offset  = 0.01;   % +6mm at fast speed - tune if still stabbing low

% go to start pos just outside thumb
robot.move_cubic(GAP_POS(1,:), PHI, 2.0);
pause(0.5);

for r = 1:numel(rounds)

    seq       = rounds(r).seq;
    N_SEQ     = numel(seq);
    seq_speed = rounds(r).speed_cms / (mean_spacing * 100);  % [steps/s]
    N_SEQ_RND = rounds(r).n_seq;

    % add variable offset with speed coz the offset worsens at higher speed
    GAP_POS_R      = GAP_POS;
    GAP_POS_R(:,3) = GAP_POS(:,3) + rounds(r).z_offset;

    fprintf('\n========================================\n');
    fprintf(' %s  |  %.0f cm/s  |  %d sequence(s)\n', ...
            rounds(r).label, rounds(r).speed_cms, N_SEQ_RND);
    fprintf('========================================\n');
    fprintf('Press ENTER to start...\n');
    pause();

    t_total        = tic;
    t0             = tic;
    t_prev         = 0;
    seq_idx        = 1.0;
    tick           = 0;
    sequences_done = 0;

    while sequences_done < N_SEQ_RND

        t         = toc(t0);
        actual_dt = t - t_prev;
        t_prev    = t;
        tick      = tick + 1;

        seq_idx = seq_idx + seq_speed * actual_dt;
        if seq_idx > N_SEQ
            sequences_done = sequences_done + 1;

            if sequences_done < N_SEQ_RND
                % Sequence ends at gap 1 so just reset and go again
                t0      = tic;
                t_prev  = 0;
                seq_idx = 1.0;
                continue;
            else
                break;
            end
        end

        i1       = max(1, min(N_SEQ, floor(seq_idx)));
        i2       = max(1, min(N_SEQ, ceil(seq_idx)));
        frac     = seq_idx - floor(seq_idx);
        target_p = (1-frac)*GAP_POS_R(seq(i1),:) + frac*GAP_POS_R(seq(i2),:);

        % Z lift: 0 at gap positions, peaks midway between gaps
        target_p(3) = target_p(3) + Z_LIFT * abs(sin(pi * seq_idx));

        [th, ok] = ik_pos_only_between_jaws(target_p, PHI, -1, params);
        if ok
            robot.move(th - th_home);
        end

        elapsed = toc(t0) - t;
        if elapsed < DT; pause(DT - elapsed); end
    end

    fprintf('[ROUND %d DONE]  time=%.2fs\n', r, toc(t_total));

    if r < numel(rounds)
        pause(0.5);
    end
end

robot.move_cubic(GAP_POS(1,:), PHI, 2.0);
pause(0.5);
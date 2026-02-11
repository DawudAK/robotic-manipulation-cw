function h = plot_frames(Ts, axis_len)
% Plot coordinate frames (RGB) for each transform in Ts.
% axis_len is the length of drawn axes.

if nargin < 2, axis_len = 0.03; end
hold on;

N = size(Ts,3);
h = gobjects(N,3);

for i = 1:N
    T = Ts(:,:,i);
    p = T(1:3,4);
    R = T(1:3,1:3);

    x = p + axis_len * R(:,1);
    y = p + axis_len * R(:,2);
    z = p + axis_len * R(:,3);

    h(i,1) = line([p(1) x(1)], [p(2) x(2)], [p(3) x(3)], 'LineWidth', 2); % X (red default)
    h(i,2) = line([p(1) y(1)], [p(2) y(2)], [p(3) y(3)], 'LineWidth', 2); % Y
    h(i,3) = line([p(1) z(1)], [p(2) z(2)], [p(3) z(3)], 'LineWidth', 2); % Z
end

% MATLAB default colours vary; set explicitly to match class (RGB):
set(h(:,1), 'Color', [1 0 0]); % X red
set(h(:,2), 'Color', [0 0.6 0]); % Y green-ish
set(h(:,3), 'Color', [0 0 1]); % Z blue
end

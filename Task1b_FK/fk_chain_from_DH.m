function [Ts, Ps] = fk_chain_from_DH(DH)
% DH is Nx4: [a alpha d theta]
% Returns:
%   Ts: 4x4x(N+1) transforms from base to each frame (T0_0 ... T0_N)
%   Ps: (N+1)x3 positions of each frame origin in base coords

N = size(DH,1);
Ts = zeros(4,4,N+1);
Ps = zeros(N+1,3);

T = eye(4);
Ts(:,:,1) = T;
Ps(1,:) = [0 0 0];

for i = 1:N
    a = DH(i,1); alpha = DH(i,2); d = DH(i,3); theta = DH(i,4);
    T = T * dh_T(a, alpha, d, theta);
    Ts(:,:,i+1) = T;
    Ps(i+1,:) = T(1:3,4).';
end

end

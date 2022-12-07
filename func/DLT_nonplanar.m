% DLT_nonplanar: estimate the initial value of the camera parameters 
%
% Input:
%          P_n  :endpoints of nonplanar Lines;
%          p    :2D point
% Output:
%          K    :intrinsic parameters
%          R    :rotation matrix
%          t    :translation vector
%
% This code follows the algorithm given by 
% [1] "Line-based Event Camera Calibration"
%
% Zibin Liu, Banglei Guan, Yang Shang
%
% This work was developed at National University of Defense Technology, 
% Hunan Provincial Key Laboratory of Image Measurement and Vision Navigation.

function [K, R, t] = DLT_nonplanar(p, P_n)

%% The Number of Points
xSize  = size(p);
XSize  = size(P_n);
noPnts = length(p);

%% image points
if (xSize(2) == 2)
    p = p';
elseif (xSize(1) == 3)
    p = p(1:2,:);
elseif (xSize(2) == 3)
    p = p(:,1:2)';
end

%% object points
if (XSize(2) == 3)
    P_n = P_n';
elseif (XSize(1) == 4)
    P_n = P_n(1:3,:);
elseif (XSize(2) == 4)
    P_n = P_n(:,1:3)';
end


%% centroids of the points
centroid1 = mean(p(1:2,:)')';
centroid2 = mean(P_n(1:3,:)')';

%% Shift the origin of the points to the centroid
p(1,:) = p(1,:) - centroid1(1);
p(2,:) = p(2,:) - centroid1(2);

P_n(1,:) = P_n(1,:) - centroid2(1);
P_n(2,:) = P_n(2,:) - centroid2(2);
P_n(3,:) = P_n(3,:) - centroid2(3);


averagedist1 = mean(sqrt(p(1,:).^2 + p(2,:).^2));
averagedist2 = mean(sqrt(P_n(1,:).^2 + P_n(2,:).^2 + P_n(3,:).^2));

scale1 = sqrt(2)/averagedist1;
scale2 = sqrt(3)/averagedist2;

p(1:2,:) = scale1*p(1:2,:);
P_n(1:3,:) = scale2*P_n(1:3,:);

%% similarity transform 1
T1 = [scale1         0  -scale1*centroid1(1)
           0    scale1  -scale1*centroid1(2)
           0         0                     1];

%% similarity transform 2
T2 = [scale2       0      0   -scale2*centroid2(1)
           0  scale2      0   -scale2*centroid2(2)
           0       0  scale2  -scale2*centroid2(3)
           0       0       0                     1];


p1=p(:,1:2:end);
p2=p(:,2:2:end);
P1_w=P_n(:,1:2:end);
P2_w=P_n(:,2:2:end);

n = length(p1);	
nl = getProjNorm(p1,p2);

M1 = kron([1 1 1], [nl nl]');
M2 = kron([P1_w P2_w]', [1 1 1]);
M = [M1 .* M2 [nl nl]'];
D = ones(2*n,1);
MTM = M' * diag(D) * M;
[XV XD] = xeig(MTM);

P_n = XV(:,1);	
P0 = reshape(P_n,3,4);

%% Denormalization
P = inv(T1)*P0*T2;

[K, R] = rqGivens(P(1:3, 1:3));
K(1,2)=0;
%% ensure that the diagonal is positive
if K(3, 3) < 0
    K = -K;
    R = -R;
end
if K(2, 2) < 0
    S = [1  0  0 
         0 -1  0
         0  0  1];
    K = K * S;
    R = S * R;
end
if K(1, 1) < 0
    S = [-1  0  0 
          0  1  0
          0  0  1];
    K = K * S;
    R = S * R;
end

%% ensure R determinant == 1
t = linsolve(K, P(:, 4));

if det(R) < 0
    R = -R;
    t = -t;
end

K = K ./ K(3, 3);


end
% DLT_planar: estimate the initial value of the camera parameters 
%
% Input:
%          P_p  :endpoints of planar Lines;
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

function [K R t] =DLT_planar(P_p,p)

    H = solveH(P_p,p);
    cx=640/2;cy=480/2;

    [fx, fy] = vision.internal.calibration.computeFocalLength(H, cx, cy);
    K=[fx,0,cx;0,fy,cy;0,0,1];

    Ainv = inv(K);
    h1 = H(:, 1);
    h2 = H(:, 2);
    h3 = H(:, 3);
    lambda = 1 / norm(Ainv * h1); %#ok
    
    % 3D rotation matrix
    r1 = lambda * Ainv * h1; %#ok
    r2 = lambda * Ainv * h2; %#ok
    r3 = cross(r1, r2);
    R = [r1,r2,r3];
          
    % translation vector
    t = lambda * Ainv * h3;  %#ok
    
    if t(3) < 0
    t = -t;
    R = [-R(:,1) -R(:,2) R(:,3)];
    end

%estimate f using zhang's method
    w = abs(-(H(3,1)*H(3,2)/(H(1,1)*H(1,2)+H(2,1)*H(2,2))));
    f = sqrt(1/w);


end
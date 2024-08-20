% gendata_simulate : Generate random test data
%
% Input:
%          a    :distortion coefficients
%          b    :image noises
%          npts :number of line endpoints
%         
% Output:
%          P_p  :endpoints of planar Lines;
%          P_n  :endpoints of nonplanar Lines;
%          p    :2D point
%          pt   :undistorted 2D point
%          K    :intrinsic parameters
%          R    :rotation matrix
%          t    :translation vector
%
% This code follows the algorithm given by 
% [1] "Line-based Event Camera Calibration"
%
%
% This work was developed at National University of Defense Technology, 
% Hunan Provincial Key Laboratory of Image Measurement and Vision Navigation.

function [P_p,P_n,p,pt,K,R,t] = gendata_both(a,b,npts)

w = 640;         % image size
h = 480;
cx = w/2;        % optical center (assumed to be image center)
cy = h/2;
cxy = [cx,cy]';
fx = 400;         % focal length
fy = 400;
K = [fx,0,cx;0,fy,cy;0,0,1];


u_d = w*rand(1, npts);
v_d = h*rand(1, npts) ;
pts2d_u = [u_d
           v_d
           ones(1,npts)];
[pts2d_d] = GetDistortedPtFromIdealImgPt_Brown(pts2d_u,K, a);

noisy_pts2d_d = pts2d_d(1:2,:) + b*rand(2,npts);

min_depth = 5;
max_depth = 10;
d = min_depth + (max_depth - min_depth)*rand(1, npts);

R = randR();
t = min_depth/2 * rand(3,1);
%Generate planar Lines
pts3d_p = inv(K*[R(1:3,1:2) t]) * (d.*pts2d_u);
pts3d_p= [pts3d_p(1,:)./pts3d_p(3,:); pts3d_p(2,:)./pts3d_p(3,:);ones(1,npts)];
%Generate nonplanar Lines
pts3d_n = R' * (d.*(K\pts2d_u) - t);
pts3d_n = [pts3d_n;ones(1,npts)];

p=noisy_pts2d_d;
pt=pts2d_u;
P_p= pts3d_p;
P_n= pts3d_n;


end


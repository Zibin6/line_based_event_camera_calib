
function [P_p,P_n,p,pt,K,R,t] = gendata_w(b,npts,w)

wide = 640;         % image size
h = 480;
cx = 320;        % optical center (assumed to be image center)
cy = 240;
cxy = [cx,cy]';
fx = 400;         % focal length
fy = 400;
K = [fx,0,cx;0,fy,cy;0,0,1];


u_d = wide*rand(1, npts);
v_d = h*rand(1, npts) ;
pts2d_u = [u_d
           v_d
           ones(1,npts)];
[pts2d_d] = GetDistortedPtFromIdealImgPt_Brown(pts2d_u,K, [0.1,0.1]);

sigma = b;
noisy_pts2d_d = pts2d_d(1:2,:) + sigma*rand(2,npts);

min_depth = 5;
max_depth = 10;
d = min_depth + (max_depth - min_depth)*rand(1, npts);

R0 = randR();
t0 = min_depth/2 * rand(3,1);

xita=(1+sign(randn(1))*w)*0.573;
detR=[cos(xita) sin(xita)  0;
     -sin(xita) cos(xita)  0;
          0        0       1];
R=detR*R0;
detT=[0.01*(1+sign(randn(1))*w);
      0.01*(1+sign(randn(1))*w);
      0];
t=detR*t0+detT;
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

xita=0.573;
detR=[cos(xita) sin(xita)  0;
     -sin(xita) cos(xita)  0;
          0        0       1];
R=detR*R0;
detT=[0.01;
      0.01;
      0];
t=detR*t0+detT;



end


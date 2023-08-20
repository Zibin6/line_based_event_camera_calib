% DEMO : Algorithm testing
%
% Input:
%          P         :endpoints of Lines;
%          p         :2D point
%          K         :intrinsic parameters
%          coe       :distortion coefficient
%          R         :rotation matrix
%          t         :translation vector
%
% Output:
%          K_optim   :optimized intrinsic parameters
%          RT_optim  :optimized rotation matrix and translation vector
%          coe_optim :optimized distortion coefficient

% This code follows the algorithm given by 
% [1] "Line-based Event Camera Calibration"
%
%
%
% This work was developed at National University of Defense Technology, 
% Hunan Provincial Key Laboratory of Image Measurement and Vision Navigation.

function [ K_optim, RT_optim, coe_optim] =Optim_Lines_All( P, p,K, coe, R,T)

    mx = 12;
    bkmx = 10;
    bbkmx =10;

    x_init = zeros(mx, 1);

    x_init(1) = K(1, 1);
    x_init(2) = K(2, 2);
    x_init(3) = K(1, 3);
    x_init(4) = K(2, 3);


    ang_init = rodrigues(R);
    x_init(5:7) = ang_init;
    x_init(8:10) = T(1:3,1);
    x_init(11)= coe(1);
    x_init(12)= coe(2);

    options = optimset;
    options.Algorithm = 'levenberg-marquardt';
    options.MaxIter = 1000;
    options.MaxFunEvals = 20000;
    options.TolFun = 1e-8;
    options.TolX = 1e-8;

    [ x_optim,resnorm, residual, exitflag, output] = ...
        lsqnonlin(@(x) ObjFunReprojErrLines( x, p, P, 2, 32 , 1e-10), x_init, [], [], options);

    K_optim = K;
    K_optim(1, 1) = x_optim(1);
    K_optim(2, 2) = x_optim(2);
    K_optim(1, 3) = x_optim(3);
    K_optim(2, 3) = x_optim(4);


    RT_optim = eye(4,4);
    RT_optim(1:3,1:3) = rodrigues([x_optim(5) x_optim(6) x_optim(7)]);
    RT_optim(1:3,4) = [x_optim(8); x_optim(9); x_optim(10)];

    coe_optim(1)=x_optim(11);
    coe_optim(2)=x_optim(12);
 

end


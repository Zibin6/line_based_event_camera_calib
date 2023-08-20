% This work was developed at National University of Defense Technology, 
% Hunan Provincial Key Laboratory of Image Measurement and Vision Navigation.

function [ K_optim, R_optim,T_optim, coe_optim, residual,exitflag,output, resnorm] = ...
    Optim_Lines_All_planar_eventcamera( PW, imgPts, n_PtperLine, K, coe, R,T, k0_flag, k1_flag, k2_flag, k3_flag, k4_flag, undist_max_iter, undist_eps)

   num=size(imgPts,3);
   
   mx = 5+num*6;
 
    x_init = zeros(mx, 1);

    x_init(num*6+1) = K(1,1);     
    x_init(num*6+2) = K(2,2);
    x_init(num*6+3) = K(1,3);     
    x_init(num*6+4) = K(2,3);

    for i=1:num
        ang_init = rodrigues(R(:,:,i));
        x_init(6*i-5:6*i-3) = ang_init;
        x_init(6*i-2:6*i) = T(1:3,1,i);
    end

    
    x_init(num*6+5)= coe(1);
  
  



    options = optimset;
    options.Algorithm = 'levenberg-marquardt';
    options.MaxIter = 1000;
    options.MaxFunEvals = 20000;
    options.TolFun = 1e-8;
    options.TolX = 1e-8;
   

    [ x_optim,resnorm, residual, exitflag, output] = ...
        lsqnonlin(@(x) ObjFunReprojErrLines_planar_eventcamera( x, imgPts, PW, n_PtperLine, k0_flag, k1_flag, k2_flag, k3_flag, k4_flag,  undist_max_iter, undist_eps), x_init, [], [], options);


    K_optim = zeros(3,3);
    K_optim(1, 1) = x_optim(num*6+1);
    K_optim(2, 2) = x_optim(num*6+2);
    K_optim(1, 3) = x_optim(num*6+3);
    K_optim(2, 3) = x_optim(num*6+4);
    K_optim(3, 3) = 1;
    for i=1:num
       R_optim(1:3,1:3,i) = rodrigues([x_optim(6*i-5) x_optim(6*i-4) x_optim(6*i-3)]);
       T_optim(1:3,1,i) = [x_optim(6*i-2); x_optim(6*i-1); x_optim(6*i)];
      
    end

    coe_optim(1)=x_optim(num*6+5);
   

end


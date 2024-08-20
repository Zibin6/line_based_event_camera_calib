% This work was developed at National University of Defense Technology, 
% Hunan Provincial Key Laboratory of Image Measurement and Vision Navigation.
function [coe_ts] = GetCoeByLines_Brown(realImgP, P,  inPara,R ,t, k0_flag, k1_flag, k2_flag, k3_flag, k4_flag)

    coe_ts = zeros(5,1);
    
if size(P,1)==3 %&& P(3,1)==1 
   
    pp=inPara*[R(1:3,1:2) t]*P;
 else

  pp=inPara*[R t]*P;
end

  pp= [pp(1,:)./pp(3,:); pp(2,:)./pp(3,:);ones(1,size(P,2))];

imgLines_re=getProjNorm(pp(:,1:2:end),pp(:,2:2:end));
imgLines_re=imgLines_re';

    mL = size(imgLines_re,1);
    nP = size(realImgP,2);
    
    nCoe = 0;
    if k0_flag == true
        nCoe = nCoe+1;
    end

    if k1_flag == true
        nCoe = nCoe+1;
    end

    if k2_flag == true
        nCoe = nCoe+1;
    end

    if k3_flag == true
        nCoe = nCoe+1;
    end

    if k4_flag == true
        nCoe = nCoe+1;
    end
    
    
    Fx = inPara(1, 1);
    Fy = inPara(2, 2);
    Cx = inPara(1, 3);
    Cy = inPara(2, 3);
    realImgP = realImgP(1:2,:);
    
    [m,n] = size(realImgP);
    Cxy = ones(m,n);
    Cxy(1,:) = Cx;
    Cxy(2,:) = Cy;
    
    imgPd = [1/Fx 0;0 1/Fy ]*(pp(1:2,:) - Cxy);
    
        
    
    A = zeros(nP,nCoe);
    B = zeros(nP,1);
    
    for i = 1:mL
        
         sidx = 2*i-1;
         eidx = 2*i;
        a = imgLines_re(i,1);
        b = imgLines_re(i,2);
        c = imgLines_re(i,3);

        for j = sidx:eidx
            xd = imgPd(1,j);
            yd = imgPd(2,j);
            r2 = xd^2 +yd^2;
            r4 = r2^2;
            r6 = r2^3;
          
            radial_coe = Fx * a * xd + Fy * b * yd;
            

            countCoe = 0;
            if k0_flag == true
                countCoe = countCoe + 1;
                A(j, countCoe) =  radial_coe * r2;
            end

            if k1_flag == true
                countCoe = countCoe + 1;
                A(j, countCoe) = radial_coe * r4;
            end

            if k2_flag == true
                countCoe = countCoe + 1;
                A(j, countCoe) = 2 * Fx * a * xd * yd  + Fy * b * (2 * yd^2 + r2);
            end

            if k3_flag == true
                countCoe = countCoe + 1;
                A(j, countCoe) = 2 * Fy * b * xd * yd + Fx * a * (2 * xd^2 + r2);
            end

            if k4_flag == true
                countCoe = countCoe + 1;
                A(j, countCoe) = radial_coe * r6;
            end

            B(j, 1) = a * realImgP(1,j) + b * realImgP(2,j) + c;
        end
    end
    
    

   x = inv(A'*A)*A'*B;

    tmpCoe = 0;


    if k0_flag == true
        tmpCoe = tmpCoe+1;
        coe_ts(tmpCoe) = x(tmpCoe);
    end

    if k1_flag == true
        tmpCoe = tmpCoe+1;
        coe_ts(tmpCoe) = x(tmpCoe);
    end

    if k2_flag == true
        tmpCoe = tmpCoe+1;
        coe_ts(tmpCoe) = x(tmpCoe);
    end

    if k3_flag == true
        tmpCoe = tmpCoe+1;
        coe_ts(tmpCoe) = x(tmpCoe);
    end

    if k4_flag == true
        tmpCoe = tmpCoe+1;
        coe_ts(tmpCoe) = x(tmpCoe);
    end
    
end
    
    
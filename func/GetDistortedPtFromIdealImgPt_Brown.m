% This work was developed at National University of Defense Technology, 
% Hunan Provincial Key Laboratory of Image Measurement and Vision Navigation.

function [realImgPt, ptDeviation] = GetDistortedPtFromIdealImgPt_Brown(idealImgPt, inPara, kc)

    
    num = size(idealImgPt,2);
    
    fX = inPara(1, 1);
    fY = inPara(2, 2);
    cX = inPara(1, 3);
    cY = inPara(2, 3);
 
    k0 = kc(1);
    k1 = kc(2);
    

    ptDeviation = zeros(3,num);
    realImgPt = zeros(3,num);
    realImgPt(3, :) = 1;
    
    for i = 1:num;
        xd = (idealImgPt(1,i) - cX) / fX;
        yd = (idealImgPt(2,i) - cY) / fY;

        r2 = xd^2 + yd^2;
        r4 = r2^2;
   
        ptDeviation(1,i) = ((k0 * r2 + k1 * r4 ) * xd ) * fX;

        ptDeviation(2,i) = ((k0 * r2 + k1 * r4 ) * yd ) * fY;

        realImgPt(1,i) = idealImgPt(1,i) + ptDeviation(1,i);
        realImgPt(2,i) = idealImgPt(2,i) + ptDeviation(2,i);
    end;
    
    
    
    
    
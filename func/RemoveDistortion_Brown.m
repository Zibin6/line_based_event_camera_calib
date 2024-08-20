% This work was developed at National University of Defense Technology, 
% Hunan Provincial Key Laboratory of Image Measurement and Vision Navigation.
function [ idealImgPt ] = RemoveDistortion_Brown( realImgPt, inPara, distortion,  maxIteration, eps )

    
    ptNum = size(realImgPt, 2);
    
    F(1) = inPara(1, 1);
    F(2) = inPara(2, 2);
    C(1) = inPara(1, 3);
    C(2) = inPara(2, 3);
    
    for i = 1:ptNum
        idealTmpPt = realImgPt(:, i);
        realTmpPt = realImgPt(:, i);
        oldIdealTmpPt = idealTmpPt;
        
        for j = 1:maxIteration
            [tmp ptDeviation(:, j)] = GetDistortedPtFromIdealImgPt_Brown(idealTmpPt, inPara, distortion);
            idealTmpPt = realTmpPt - ptDeviation(:, j);
    
            residue(:, j) = idealTmpPt - oldIdealTmpPt;
            oldIdealTmpPt = idealTmpPt;
            
            if ((abs(residue(1, j)) < eps) && (abs(residue(2, j)) < eps))
                break;
            end;
           
        end
        
    
        idealImgPt(:, i) = idealTmpPt;
    end
    
    end
    
    
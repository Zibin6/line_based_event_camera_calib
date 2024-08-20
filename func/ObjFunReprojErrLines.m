% This work was developed at National University of Defense Technology, 
% Hunan Provincial Key Laboratory of Image Measurement and Vision Navigation.
function ErrorReproj = ObjFunReprojErrLines( x, imgPts, wrdLines,n_PtperLine, undist_max_iter, undist_eps)

imgPts(3, :) = 1;
mx =10;
K = zeros(3, 3);
K(1, 1) = x(1);
K(2, 2) = x(2);
K(1, 3) = x(3);
K(2, 3) = x(4);
K(3, 3) = 1;

RT = eye(3,4);
RT(1:3,1:3) = rodrigues([x(5) x(6) x(7)]);
RT(1:3,4) = [x(8); x(9); x(10)];

coe(1) = x(11);
coe(2) = x(12);


n = size(imgPts, 2); 

mL =n/2;


 if size(wrdLines,1)==3 && wrdLines(3,1)==1
    pp=K*[RT(1:3,1:2) RT(1:3,4)]*wrdLines;
    pp= [pp(1,:)./pp(3,:); pp(2,:)./pp(3,:);ones(1,n)];
 elseif size(wrdLines,1)==4
  pp=K*RT*wrdLines;
  pp= [pp(1,:)./pp(3,:); pp(2,:)./pp(3,:);ones(1,n)];
 else
    error("wrong") ;

 end


imgLines_re=getProjNorm(pp(:,1:2:end),pp(:,2:2:end));
imgLines_re=imgLines_re';
[idealImgPt] = RemoveDistortion_Brown( imgPts, K, coe,undist_max_iter,undist_eps );

ErrorReproj = zeros(n, 1);

for i = 1:mL
    a = imgLines_re(i,1);
    b = imgLines_re(i,2);
    c = imgLines_re(i,3);
    
    normL = sqrt(a^2 + b^2);

    for j=n_PtperLine*(i-1)+1:n_PtperLine*(i)
        ErrorReproj(j,1) = abs(a*idealImgPt(1,j) + b*idealImgPt(2,j) + c)/ normL;
    end
end

end

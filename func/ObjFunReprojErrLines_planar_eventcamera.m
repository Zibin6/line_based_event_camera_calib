% This work was developed at National University of Defense Technology, 
% Hunan Provincial Key Laboratory of Image Measurement and Vision Navigation.

function ErrorReprojjj = ObjFunReprojErrLines_planar_eventcamera( x, imgPts, wrdLines,n_PtperLine, k0_flag, k1_flag, k2_flag, k3_flag, k4_flag,  undist_max_iter, undist_eps)


 imgPts(3,:, :) = 1;


 num=size(imgPts,3);
   
 mx = 5+num*6;
 
for i=1:num
       
    RT(1:3,1:3,i) = rodrigues([x(6*i-5) x(6*i-4) x(6*i-3)]);
    RT(1:3,4,i) = [x(6*i-2); x(6*i-1); x(6*i)];
end
 K= zeros(3,3);
 K(1, 1) = x(num*6+1);
 K(2, 2) = x(num*6+2);
 K(1, 3) = x(num*6+3);
 K(2, 3) = x(num*6+4);
 K(3, 3) = 1;

coe(1)=x(num*6+5);



for ii=1:num
n = size(imgPts, 2); 
% ...........
     if size(wrdLines,1)==3 && wrdLines(3,1)==1
     pp=K*[RT(1:3,1:2,ii) RT(1:3,4,ii)]*wrdLines;
   
     elseif size(wrdLines,1)==4
 
     pp=K*RT(1:3,1:4,ii)*wrdLines;

     else
     error("wrong") ;

     end
% ...........

pp= [pp(1,:)./pp(3,:); pp(2,:)./pp(3,:);ones(1,n)];

imgLines_re=getProjNorm(pp(:,1:2:end),pp(:,2:2:end));
imgLines_re=imgLines_re';

[idealImgPt] = RemoveDistortion_Brown( imgPts(:,:,ii), K, coe,  undist_max_iter,undist_eps );
ErrorReprojj = zeros(num,1);
ErrorReproj  = zeros(n, 1);

for i = 1:n/2
    a = imgLines_re(i,1);
    b = imgLines_re(i,2);
    c = imgLines_re(i,3);
    
    normL = sqrt(a^2 + b^2);

    for j=2*i-1:2*(i)
        ErrorReproj(j,1) = abs(a*idealImgPt(1,j) + b*idealImgPt(2,j) + c)/ normL;
    end

end

ErrorReprojj(ii,1)=sum(ErrorReproj);

end
 ErrorReprojjj=sum( ErrorReprojj);
end

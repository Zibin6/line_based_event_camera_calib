% norm of a projection plane of a 2D line二维直线投影平面的范数
function nl = getProjNorm(p1,p2)

	n = size(p1,2);
    if size(p1,1)==2
	    d1 = [p1; ones(1,n)];
	    d2 = [p2; ones(1,n)];
    else
        d1 =p1;
        d2 =p2;
    end
	nl = cross(d1,d2,1);
%     每一列归一化
  	nl = xnorm(nl);

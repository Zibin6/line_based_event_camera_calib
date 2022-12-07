function H = solveH(x1, x2)

[r1,c1] = size(x1);
[r2,c2] = size(x2);

%% centroids of the points
centroid1 = mean(x1(1:2,:)')';
centroid2 = mean(x2(1:2,:)')';

%% Shift the origin of the points to the centroid
x1(1,:) = x1(1,:) - centroid1(1);
x1(2,:) = x1(2,:) - centroid1(2);

x2(1,:) = x2(1,:) - centroid2(1);
x2(2,:) = x2(2,:) - centroid2(2);


averagedist1 = mean(sqrt(x1(1,:).^2 + x1(2,:).^2));
averagedist2 = mean(sqrt(x2(1,:).^2 + x2(2,:).^2));

scale1 = sqrt(2)/averagedist1;
scale2 = sqrt(2)/averagedist2;

x1(1:2,:) = scale1*x1(1:2,:);
x2(1:2,:) = scale2*x2(1:2,:);

%% similarity transform 1
T1 = [scale1    0       -scale1*centroid1(1)
      0         scale1  -scale1*centroid1(2)
      0         0       1      ];

%% similarity transform 2
T2 = [scale2    0       -scale2*centroid2(1)
      0         scale2  -scale2*centroid2(2)
      0         0       1      ];

 if (c1 == c2)
   
    p1=x2(:,1:2:end);
    p2=x2(:,2:2:end);
    P1_w=x1(:,1:2:end);
    P2_w=x1(:,2:2:end);
    n = length(p1);
    nl = getProjNorm(p1,p2);
	M1 = kron([1 1 1], [nl nl]');
	M2 = kron([P1_w P2_w]', [1 1 1]);
	M = [M1 .* M2 ];
    D = ones(2*n,1);
	MTM = M' * diag(D) * M;
	[XV XD] = xeig(MTM);

	X = XV(:,1);
  
    H = reshape(X,3,3);
end

%% Denormalization
 H = inv(T2)*H*T1;

end
function [eR et efx efy ecx ecy ek1 ek2 ] = err_both(K,R,t,K_e0,R_e0,t_e0,k_e0)

eR= cal_rotation_err(R_e0, R);
et = norm(t - t_e0) / norm(t);
efx= abs(K_e0(1,1) - K(1,1)) / K(1,1);
efy =abs(K_e0(2,2) - K(2,2)) / K(2,2);
ecx= abs(K_e0(1,3) - K(1,3)) / K(1,3);
ecy= abs(K_e0(2,3) - K(2,3)) / K(2,3);
k=[0.1,0.1];
ek1 = abs(k(1) - k_e0(1));
ek2 = abs(k(2) - k_e0(2));


end


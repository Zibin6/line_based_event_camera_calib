function R = randR()

    theta_x = 2*rand(1)-1;
    theta_y = 2*rand(1)-1;
    theta_z = 2*rand(1)-1;
    r_x = [1,0,0; 0, cos(theta_x), - sin(theta_x); 0, sin(theta_x), cos(theta_x) ];
    r_y = [cos(theta_y), 0, sin(theta_y); 0, 1, 0; - sin(theta_y), 0, cos(theta_y)];
    r_z = [cos(theta_z), - sin(theta_z), 0; sin(theta_z), cos(theta_z), 0; 0, 0, 1];
    R = r_z * r_y * r_x;
end

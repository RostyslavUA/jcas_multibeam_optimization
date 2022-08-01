function a = steering_vector(theta_angle, M)
d_vec = 0:M-1;
theta_radian = pi/180*theta_angle;
phase = -pi * d_vec' * sin(theta_radian); 
a = exp(1i*phase);
end


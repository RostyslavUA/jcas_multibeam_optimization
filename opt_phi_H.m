function phi_opt = opt_phi_H(rho, H, w_c, w_s)
a_1 = abs(w_c'*(H'*H)*w_s);
a_2 = abs(w_c'*w_s);
alpha_1 = angle(w_c'*(H'*H)*w_s);
alpha_2 = angle(w_c'*w_s);
P = sqrt(rho*(1-rho));
L = -4*P^2*abs(a_1)*abs(a_2)*sin(alpha_1-alpha_2);
X_1 = -2*P*abs(a_1)*cos(alpha_1)+ ...
       2*P*abs(a_2)*(rho*(norm(H*w_c))^2+(1-rho)*(norm(H*w_s))^2)*cos(alpha_2);
X_2 = -2*P*abs(a_1)*sin(alpha_1)+ ...
       2*P*abs(a_2)*(rho*(norm(H*w_c))^2+(1-rho)*(norm(H*w_s))^2)*sin(alpha_2);

mu_0 = asin(L/sqrt(X_1^2+X_2^2));
upsilon = atan(X_2/X_1);
l = -3:3;
len = length(l);
if X_1 >= 0
    phi = (pi + mu_0 - upsilon)*ones(1,len) + 2*l*pi;
else
    phi = (mu_0 - upsilon)*ones(1,len) + 2*l*pi;
end
ind_opt = find(-pi<=phi & phi<pi);
phi_opt = phi(ind_opt);
end


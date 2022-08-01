function phi_opt = opt_phi_AoD(rho, a, w_c, w_s)
a_1 = abs(w_c'*w_s);
a_2 = abs(w_c'*conj(a));
a_3 = abs(a.'*w_s);
alpha_1 = angle(w_c'*w_s);
alpha_2 = angle(w_c'*conj(a));
alpha_3 = angle(a.'*w_s);
P = sqrt(rho*(1-rho));
L = -4*P^2*a_1*a_2*a_3*sin(alpha_2+alpha_3-alpha_1);
X_1 = -2*P*a_2*a_3*cos(alpha_2+alpha_3)+ ...
       2*P*a_1*(rho*a_2^2+(1-rho)*a_3^2)*cos(alpha_1);
X_2 = -2*P*a_2*a_3*sin(alpha_2+alpha_3)+ ...
       2*P*a_1*(rho*a_2^2+(1-rho)*a_3^2)*sin(alpha_1);

mu_0 = asin(L/sqrt(X_1^2+X_2^2));
upsilon = atan(X_2/X_1);
l = -3:3;
len = length(l);
if X_1 > 0
    phi = (pi + mu_0 - upsilon)*ones(1,len) + 2*l*pi;
elseif X_1 < 0
    phi = (mu_0 - upsilon)*ones(1,len) + 2*l*pi;
end
ind_opt = find(-pi<=phi & phi<pi);
phi_opt = phi(ind_opt);
end

function [v_min,q] = IGSS_Q(a_1, a_2, L_max, epsilon_0, c, w)
% IGSS_Q Algorithm
M = length(w);
K = length(c);
rho = (sqrt(5)-1)/2;
% initialization
l = 0;
a1_l = a_1;
a2_l = a_2;
d_l = a2_l - a1_l;
x1_l = a1_l + (1-rho)*d_l;
x2_l = a1_l + rho*d_l;
while (l <= L_max) && (abs(d_l)>epsilon_0) 
    Err_a1 = cal_error(a1_l, w, c);
    Err_a2 = cal_error(a2_l, w, c);
    Err_x1 = cal_error(x1_l, w, c);
    Err_x2 = cal_error(x2_l, w, c);
    [~, I_min] = min([Err_a1, Err_x1, Err_x2, Err_a2]);
    if ismember(I_min, [1, 2]) == 1
        a2_l = x2_l;
        x2_l = x1_l;     
        x1_l = a1_l + rand(1)*abs(a1_l-x2_l);
    elseif ismember(I_min, [3, 4]) == 1
        a1_l = x1_l;
        x1_l = x2_l;        
        x2_l = x1_l + rand(1)*abs(x1_l-a2_l);
    end
    d_l = a2_l - a1_l;
    l = l + 1;
end
Err_x1 = cal_error(x1_l, w, c);
Err_x2 = cal_error(x2_l, w, c);
[~, ind_x] = min([Err_x1, Err_x2]);
if ind_x == 1
    v_min = x1_l;
else
    v_min = x2_l;
end
q = zeros(M, 1);
for i = 1:M
    diff = abs(v_min*w(i)*ones(1, K) - c); 
    diff = diff.^2;
    [~, min_ind] = min(diff);
    q(i) = c(min_ind);
end
end


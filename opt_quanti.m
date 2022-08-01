%% optimization
L = 8; % 11 multipaths
theta_t_LOS = 0; % AoD of LOS 
theta_r_LOS = 0; % AoA of LOS 
theta_t_NLOS = theta_t_LOS + -7 + 14*rand(L-1, 1); % AoDs of NLOS 
theta_r_NLOS = theta_t_NLOS; % AoAs of NLOS
theta_t = [theta_t_LOS; theta_t_NLOS];
theta_r = [theta_r_LOS; theta_r_NLOS];

d_LOS = 100; % distance of LOS
d_NLOS = 100 + 200*rand(L-1, 1); % distance of NLOS
d = [d_LOS; d_NLOS];
tao_l = d/(3*10^8); % propagation delay

v_A = 0; % speed of A node
v_o = 10*rand(L-1, 1); % speed of the obstacles
v = [v_A; v_A*ones(L-1, 1)+v_o];

f_c = 3*10^10; % carrier frequency
f_D = (v*f_c)/(3*10^8); % Doppler frequency

b_LOS = sqrt(1/2) * (1 + 1i); % amplitude of complex value
b_NLOS = sqrt(0.1/2) * (ones(L-1, 1) + 1i*ones(L-1, 1));
b = [b_LOS; b_NLOS];

% Channel coefficients
M = 12; % array dimension
K_c = 16;
K_s = 12;
H = zeros(M, M);
for i = 1:L
    a_t = steering_vector(theta_t(i), M);   
    a_r = steering_vector(theta_r(i), M);
    H = H + b(i)*exp(1i*2*pi*f_D(i)*tao_l(i))*(1/2)*a_r*a_t.';
end

theta_sense = -10;   % pointing direction of sensing beam
theta_DoA = theta_t_LOS;  
a_c = steering_vector(theta_DoA, M);
w_t_c = 1/sqrt(M)*a_c;  % norm = 1 
a_s = steering_vector(theta_sense, M);
w_t_s = 1/sqrt(M)*a_s; % norm = 1
rho = 0.5;
phi_angle = -180:0.1:179.9;
phi = pi/180*phi_angle;
w_t = zeros(M, length(phi));
P_r = zeros(length(phi), 1);
P_AoD = zeros(length(phi), 1);
for i = 1:length(phi)
    w_t(:,i) = sqrt(rho)*w_t_c + sqrt(1-rho)*exp(1i*phi(i))*w_t_s;
    w_t(:,i) = w_t(:,i)/norm(w_t(:,i));
    P_r(i) = 1/(real(w_t_c'*H'*H*w_t_c))*real(w_t(:,i)'*H'*H*w_t(:,i));  % received signal power
    P_AoD(i) = 1/(norm(a_c.'*w_t_c)^2)*norm(a_c.'*w_t(:,i))^2;  % power at dominating AoD
end
[pk_r, loc_r] = findpeaks(P_r);  % find max point
[pk_AoD, loc_AoD] = findpeaks(P_AoD); % find max point

phi_opt = opt_phi_H(rho, H, w_t_c, w_t_s); % opt_phi_H func get optimal phi
phi_opt_angle = 180/pi*phi_opt;
w_t_opt1 = sqrt(rho)*w_t_c + sqrt(1-rho)*exp(1i*phi_opt)*w_t_s;
% w_t_opt1 = w_t_opt1/norm(w_t_opt1);
Pr_opt1 = 1/(norm(H*w_t_c)^2)*real(w_t_opt1'*H'*H*w_t_opt1);

phi_opt_tilde = opt_phi_AoD(rho, a_c, w_t_c, w_t_s); % opt_phi_AoD func get optimal phi
phi_opt_angle_tilde = 180/pi*phi_opt_tilde;
w_t_opt2 = sqrt(rho)*w_t_c + sqrt(1-rho)*exp(1i*phi_opt_tilde)*w_t_s;
w_t_opt2 = w_t_opt2/norm(w_t_opt2);
Paod_opt2 = 1/(norm(a_c.'*w_t_c)^2)*norm(a_c.'*w_t_opt2)^2;

W = w_t_opt2;  % used in plotting fig.9

%% fig.4
figure(1);
plot(phi_angle, P_r, 'r');
hold on 
scatter(loc_r*0.1-180, pk_r, 'bs');
scatter(phi_opt_angle, Pr_opt1, 'rx');
plot(phi_angle, P_AoD, 'b--');
scatter(loc_AoD*0.1-180, pk_AoD, 'mo');
scatter(phi_opt_angle_tilde, Paod_opt2, 'b*');
scatter(loc_AoD*0.1-180, P_r(loc_AoD),'b^')
line([loc_AoD*0.1-180, loc_AoD*0.1-180], [0, pk_r+0.2], 'linestyle','--', 'Color','b');
ylabel('Normalized Power');
xlabel('\phi(Degrees)')
legend('Power at Rx(H-Knowm)', 'for \phi_{opt} at Rx', 'Maximum at Rx(H-Knowm)',...
       'Power at dominating AoD', 'for \phi_{opt} at dominating AoD',...
       'Maximum at dominating AoD', 'Power at Rx(acieved by \phi_{opt})');
hold off

% fig.5
% rho_hat = 0.1:0.1:0.9; 
% P_H = zeros(length(rho_hat), 100);
% P_DoA = zeros(length(rho_hat), 100);
% for iter = 1:100
%     % channel coefficients
%     theta_t_NLOS = -7 + 14*rand(L-1, 1); % AoDs of NLOS 
%     theta_r_NLOS = theta_t_NLOS; % AoAs of NLOS
%     theta_t = [theta_t_LOS; theta_t_NLOS];
%     theta_r = [theta_r_LOS; theta_r_NLOS];
%     
%     d_LOS = 100; % distance of LOS
%     d_NLOS = 100 + 200*rand(L-1, 1); % distance of NLOS
%     d = [d_LOS; d_NLOS];
%     tao_l = d/(3*10^8); % propagation delay
% 
%     v_A = 0; % speed of A node
%     v_o = 10*rand(L-1, 1); % speed of the obstacles
%     v = [v_A; v_A*ones(L-1, 1)+v_o];
% 
%     f_c = 3*10^10; % carrier frequency
%     f_D = (v*f_c)/(3*10^8); % Doppler frequency
% 
%     H = zeros(M, M);
%     for i = 1:L
%         a_t = steering_vector(theta_t(i), M);
%         a_r = steering_vector(theta_r(i), M);
%         H = H + b(i)*exp(1i*2*pi*f_D(i)*tao_l(i))*(1/2)*a_r*a_t.';
%     end
% 
%      
%     for i = 1:length(rho_hat)
%         phi_opt = opt_phi_H(rho_hat(i), H, w_t_c, w_t_s);
%         phi_opt_angle = 180/pi*phi_opt;
%         w_t_opt1 = sqrt(rho_hat(i))*w_t_c + sqrt(1-rho_hat(i))*exp(1i*phi_opt)*w_t_s;
%         w_t_opt1 = w_t_opt1/norm(w_t_opt1);
%         P_H(i, iter) = 1/(norm(H*w_t_c)^2)*real(w_t_opt1'*H'*H*w_t_opt1);
%         phi_opt_tilde = opt_phi_AoD(rho_hat(i), a_c, w_t_c, w_t_s);
%         phi_opt_angle_tilde = 180/pi*phi_opt_tilde;
%         w_t_opt2 = sqrt(rho_hat(i))*w_t_c + sqrt(1-rho_hat(i))*exp(1i*phi_opt_tilde)*w_t_s;
%         w_t_opt2 = w_t_opt2/norm(w_t_opt2);
%         P_DoA(i, iter) = 1/(norm(a_c.'*w_t_c)^2)*norm(a_c.'*w_t_opt2)^2; 
%     end
% end
% figure(2);
% plot(rho_hat, mean(P_H, 2), 'r');
% hold on 
% plot(rho_hat, mean(P_DoA, 2), 'b--');
% ylabel('mean Normalized signal power at Rx');
% xlabel('\rho')
% xlim([0 1])
% legend('H-known', 'AoD-known')
% hold off

% %% quantization
% % joint quantization using combined quantization codebooks
% b = 2:5; % bit setting 2-5
% w_PS1 = zeros(M, length(b)); % for 1-PS
% w_c1 = zeros(M, length(b)); % for 2-PS with codebook c1
% w_c2 = zeros(M, length(b)); % for 2-PS with codebook c2
% w_igss = zeros(M, length(b)); % for IGSS-Q (codebook c2)
% w_igss1 = zeros(M, length(b)); % for IGSS-Q (codebook c1)
% % initial inputs of IGSS-Q algorithm
% a_1 = 1;
% a_2 = 100;
% L_max = 1000;
% epsilon_0 = 0.01;
% % single phase shifter
% W_PS1 = W/norm(W);
% for i = 1:length(b)
%     w_phs = FBND(W_PS1, 2^b(i));
%     w_PS1(:, i) = 1/sqrt(M)*exp(1i*2*pi*w_phs/2^b(i)); 
% end
% % 2 bit
% [w_c1(:, 1), w_c2(:, 1), c_1_b2, c_2_b2] = BF_quantize(W, b(1));
% [~,q_2] = IGSS_Q(a_1, a_2, L_max, epsilon_0, c_2_b2, W);
% w_igss(:, 1) = q_2/norm(q_2);
% [~,q_2] = IGSS_Q(a_1, a_2, L_max, epsilon_0, c_1_b2, W);
% w_igss1(:, 1) = q_2/norm(q_2);
% % 3 bit
% [w_c1(:, 2), w_c2(:, 2), c_1_b3, c_2_b3] = BF_quantize(W, b(2));
% [~,q_3] = IGSS_Q(a_1, a_2, L_max, epsilon_0, c_2_b3, W);
% w_igss(:, 2) = q_3/norm(q_3);
% [~,q_3] = IGSS_Q(a_1, a_2, L_max, epsilon_0, c_1_b3, W);
% w_igss1(:, 2) = q_3/norm(q_3);
% % 4 bit
% [w_c1(:, 3), w_c2(:, 3), c_1_b4, c_2_b4] = BF_quantize(W, b(3));
% [~,q_4] = IGSS_Q(a_1, a_2, L_max, epsilon_0, c_2_b4, W);
% w_igss(:, 3) = q_4/norm(q_4);
% [~,q_4] = IGSS_Q(a_1, a_2, L_max, epsilon_0, c_1_b4, W);
% w_igss1(:, 3) = q_4/norm(q_4);
% % 5 bit
% [w_c1(:, 4), w_c2(:, 4), c_1_b5, c_2_b5] = BF_quantize(W, b(4));
% [~,q_5] = IGSS_Q(a_1, a_2, L_max, epsilon_0, c_2_b5, W);
% w_igss(:, 4) = q_5/norm(q_5);
% [~,q_5] = IGSS_Q(a_1, a_2, L_max, epsilon_0, c_1_b5, W);
% w_igss1(:, 4) = q_5/norm(q_5);
% 
% angle = -60:0.1:59.9;
% % radiation patterns
% r = zeros(length(angle),1); % radiation patterns without quantization
% r_1 = zeros(length(angle),length(b)); % radiation patterns 1-PS
% r_2 = zeros(length(angle),length(b)); % radiation patterns 2-PS codebook c1
% r_3 = zeros(length(angle),length(b)); % radiation patterns 2-PS codebook c2
% r_4 = zeros(length(angle),length(b)); % radiation patterns IGSS-Q for 2-PS with codebook c2
% r_5 = zeros(length(angle),length(b)); % radiation patterns IGSS-Q for 2-PS with codebook c1
% for j = 1:length(b)
%     for i = 1:length(angle)
%         a = steering_vector(angle(i), M);
%         r_1(i, j) = a.'* w_PS1(:, j);
%         r_2(i, j) = a.'* w_c1(:, j);
%         r_3(i, j) = a.'* w_c2(:, j);
%         r_4(i, j) = a.'* w_igss(:, j);
%         r_5(i, j) = a.'* w_igss1(:, j);
%     end
% end
% 
% for i = 1:length(angle)
%     a = steering_vector(angle(i), M);
%     r(i) = a.'* W;
% end

% %figures
% %constellations
% figure(1);
% subplot(2,2,1)
% scatter(real(c_1_b2),imag(c_1_b2),'b.');
% hold on
% scatter(real(c_2_b2),imag(c_2_b2),'r.');
% xlabel('real part');
% ylabel('imagianry part');
% legend('codebook c_1', 'codebook c_2');
% title('b = 2')
% hold off
% subplot(2,2,2)
% scatter(real(c_1_b3),imag(c_1_b3),'b.');
% hold on
% scatter(real(c_2_b3),imag(c_2_b3),'r.');
% xlabel('real part');
% ylabel('imagianry part');
% legend('codebook c_1', 'codebook c_2');
% title('b = 3')
% hold off
% subplot(2,2,3)
% scatter(real(c_1_b4),imag(c_1_b4),'b.');
% hold on
% scatter(real(c_2_b4),imag(c_2_b4),'r.');
% xlabel('real part');
% ylabel('imagianry part');
% legend('codebook c_1', 'codebook c_2');
% title('b = 4')
% hold off
% subplot(2,2,4)
% scatter(real(c_1_b5),imag(c_1_b5),'b.');
% hold on
% scatter(real(c_2_b5),imag(c_2_b5),'r.');
% xlabel('real part');
% ylabel('imagianry part');
% legend('codebook c_1', 'codebook c_2');
% title('b = 5')
% hold off

% BF radiation pattern
% figure(1);
% plot(angle, 10*log10(abs(r)),'r-');
% hold on
% plot(angle, 10*log10(abs(r_1(:, 1))), 'b--');
% % plot(angle, 10*log10(abs(r_1(:, 2))), 'c-.');
% plot(angle, 10*log10(abs(r_1(:, 3))),'-.','color',[0.19, 0.5, 0.08],'Marker','p','MarkerIndices',1:20:1200);
% plot(angle, 10*log10(abs(r_1(:, 4))), '--','color',[0.5, 0, 0.8],'Marker','o','MarkerIndices',1:20:1200);
% xlabel('Scanning direction in Degrees');
% ylabel('BF radiation pattern(dB)');
% % legend('Without quantization','bit=2','bit=3','bit=4','bit=5');
% legend('Without quantization','bit=2','bit=4','bit=5');
% title('FBND quantization for 1-PS')
% ylim([-25, 10]);
% grid on
% hold off

% figure(2);
% plot(angle, 10*log10(abs(r)),'r-');
% hold on
% plot(angle, 10*log10(abs(r_2(:, 1))), 'b--');
% % plot(angle, 10*log10(abs(r_2(:, 2))), 'c-.');
% plot(angle, 10*log10(abs(r_2(:, 3))), '--','color',[0.5, 0, 0.8],'Marker','p','MarkerIndices',1:20:1200);
% % plot(angle, 10*log10(abs(r_2(:, 4))), '--','color',[0.5, 0, 0.8],'Marker','o','MarkerIndices',1:20:1200);
% xlabel('Scanning direction in Degrees');
% ylabel('BF radiation pattern(dB)');
% % legend('Without quantization','bit=2','bit=3','bit=4','bit=5');
% legend('Without quantization','bit=2','bit=4');
% title('Joint quantization for 2-PS with Codebook C_1')
% ylim([-25, 10]);
% grid on
% hold off

% figure(3);
% plot(angle, 10*log10(abs(r)),'r-');
% hold on
% plot(angle, 10*log10(abs(r_3(:, 1))), 'b--');
% % plot(angle, 10*log10(abs(r_3(:, 2))), 'c-.');
% plot(angle, 10*log10(abs(r_3(:, 3))), '--','color',[0.5, 0, 0.8],'Marker','p','MarkerIndices',1:20:1200);
% % plot(angle, 10*log10(abs(r_3(:, 4))), '--','color',[0.5, 0, 0.8],'Marker','o','MarkerIndices',1:20:1200);
% xlabel('Scanning direction in Degrees');
% ylabel('BF radiation pattern(dB)');
% % legend('Without quantization','bit=2','bit=3','bit=4','bit=5');
% legend('Without quantization','bit=2','bit=4');
% title('Joint quantization for 2-PS with Codebook C_2')
% ylim([-25, 10]);
% grid on
% hold off

% figure(2);
% plot(angle, 10*log10(abs(r)),'r-');
% hold on
% plot(angle, 10*log10(abs(r_5(:, 1))), 'b--');
% % plot(angle, 10*log10(abs(r_4(:, 2))), 'c-.');
% plot(angle, 10*log10(abs(r_5(:, 3))), '-.','color',[0.19, 0.5, 0.08],'Marker','p','MarkerIndices',1:20:1200);
% plot(angle, 10*log10(abs(r_5(:, 4))), '--','color',[0.5, 0, 0.8],'Marker','o','MarkerIndices',1:20:1200);
% xlabel('Scanning direction in Degrees');
% ylabel('BF radiation pattern(dB)');
% % legend('Without quantization','bit=2','bit=3','bit=4','bit=5');
% legend('Without quantization','bit=2','bit=4','bit=5');
% title('IGSS-Q for 2-PS with Codebook C_1')
% ylim([-25, 10]);
% grid on
% hold off

% figure(3);
% plot(angle, 10*log10(abs(r)),'r-');
% hold on
% plot(angle, 10*log10(abs(r_4(:, 1))), 'b--');
% % plot(angle, 10*log10(abs(r_4(:, 2))), 'c-.');
% plot(angle, 10*log10(abs(r_4(:, 3))),'-.','color',[0.19, 0.5, 0.08],'Marker','p','MarkerIndices',1:20:1200);
% plot(angle, 10*log10(abs(r_4(:, 4))), '--','color',[0.5, 0, 0.8],'Marker','o','MarkerIndices',1:20:1200);
% xlabel('Scanning direction in Degrees');
% ylabel('BF radiation pattern(dB)');
% % legend('Without quantization','bit=2','bit=3','bit=4','bit=5');
% legend('Without quantization','bit=2','bit=4','bit=5');
% title('IGSS-Q for 2-PS with Codebook C_2')
% ylim([-25, 10]);
% grid on
% hold off

% %%
% b = 4; % bit setting 2-5
% delta_beta = 2*pi*2^(-b);
% phi = 0:0.01:delta_beta/2;
% phi = [phi, delta_beta/2];
% r_hat = zeros(length(angle), 1); 
% MSE_r = zeros(length(phi), 1);
% for i = 1:length(phi)
%     [cb] = creat_codebook(phi(i),b,b);
%     [~,q] = IGSS_Q(a_1, a_2, L_max, epsilon_0, cb, W);
%     w_igss = q/norm(q);
%     for j = 1:length(angle)
%         a = steering_vector(angle(j), M);
%         r_hat(j) = a.'* w_igss;
%     end
%     MSE_r(i) = norm(r_hat - r)^2/length(angle);
% end
% [~,ind_mse] = min(MSE_r);
% [cb_best] = creat_codebook(phi(ind_mse),b,b);
% [~,q] = IGSS_Q(a_1, a_2, L_max, epsilon_0, cb_best, W);
% w_igss = q/norm(q);
% for j = 1:length(angle)
%      a = steering_vector(angle(j), M);
%      r_hat(j) = a.'* w_igss;
% end
% 
% figure(4);
% plot(phi, MSE_r,'r-');
% xlabel('\phi')
% ylabel('Mean Square Errer')
% grid on 
% 
% figure(5);
% plot(angle, 10*log10(abs(r)),'r-');
% hold on 
% plot(angle, 10*log10(abs(r_hat)), '-.','color',[0.19, 0.5, 0.08],'Marker','p','MarkerIndices',1:20:1200);
% xlabel('Scanning direction in Degrees');
% ylabel('BF radiation pattern(dB)');
% legend('Without quantization','bit=4');
% ylim([-25, 10]);
% grid on
% hold off

%% 
scan_angle = -12.5:2.5:14.9;
power = zeros(length(scan_angle),1);
for i = 1:length(scan_angle)
    a_hat = steering_vector(scan_angle(i), M);
    w_hat = 1/sqrt(M)*a_hat;
    power(i) = (norm(w_t_opt2'*H'*w_hat))^2; 
end
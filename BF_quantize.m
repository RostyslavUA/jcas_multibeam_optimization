function [w_hat_1, w_hat_2, c_1_hat, c_2_hat] = BF_quantize(w, b)
%input: W: BF vector
%       b: the number of quantization bits
%output:w_hat_1: for codebook 1
%       w_hat_2: for codebook 2
M = length(w);
delta = 2*pi*2^(-b); % quantization step
% normalization factor
h_1 = sqrt(2+2^(2-b))*sqrt(M);
h_2 = sqrt(2*M);
% creat two codebooks with phi=0 and phi=delta/2
c_1_hat = creat_codebook(0,b,b);
c_2_hat = creat_codebook(delta/2,b,b);
% normalization
c_1 = c_1_hat/h_1;
c_2 = c_2_hat/h_2;
w_hat_1 = zeros(M, 1);
w_hat_2 = zeros(M, 1);
for i = 1:M
    diff_1 = abs(w(i)*ones(1, length(c_1))-c_1);
    diff_1 = diff_1.^2;
    diff_2 = abs(w(i)*ones(1, length(c_2))-c_2);
    diff_2 = diff_2.^2;
    [~, ind_1] = min(diff_1);
    [~, ind_2] = min(diff_2);
    w_hat_1(i) = c_1(ind_1);
    w_hat_2(i) = c_2(ind_2);
end
end


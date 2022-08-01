function e = cal_error(v, w, c)
% calculate the quantization error
% input: c: codebook
M = length(w);
K = length(c);
q = zeros(M, 1);
for i = 1:M
    diff = abs(v*w(i)*ones(1, K) - c); 
    diff = diff.^2;
    [~, min_ind] = min(diff);
    q(i) = c(min_ind);
end
e = real(sum(v*w-q));
end


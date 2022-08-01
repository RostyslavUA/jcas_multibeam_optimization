function g = FBND(x, M)
% FBND
eta = exp(1i*2*pi/M);
arg = angle(x)*M/(2*pi);
g = round(arg);
[~, u] = sort(g-arg);
p = conj(x).*eta.^g;
v = [sum(p); p(u)*(eta-1)];
[~, best] = max(abs(cumsum(v)));
g(u(1:best-1)) = g(u(1:best-1)) + 1;
g = mod(g-g(1), M);
end


function W_dd = displacePattern(W, delta, M)
% Function generates the BF settings associated with the displaced pattern.
% The initial BF vector W gets displaced by delta equivalent directions 
% (-1...1) that was generated by an array having M elements

% Phase-shifting vector
D_delta = diag(exp(1i*pi*(1:M)*delta));
% BF settings associated with the displaced pattern
W_dd = D_delta*W; % Non-conjugated, as BF vector is conjugated by 
% convention e.g., when computing BF values i.e., w'*A
end%function
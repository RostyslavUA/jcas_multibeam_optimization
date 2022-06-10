function W0 = twoStepILS(iter_nr, alpha, V_pattern, W0, PM, PdM)
% Two-step Iterative Least-Squares (ILS)
% The function iteratively optimizes the beamforming matrix W0 based on the
% initial (PM) and the desried (PdM) pattern magnitudes. alpha contains the
% indices of the angles that we want to approximate, V_pattern is the
% array's response and iter_nr is the maximal number of iterations.
for i = 1:iter_nr
    % Step 2 - Determining Theta
    V = V_pattern(:, alpha);
    % Step 3 - Condition of convergence
    % Set of indicies that aren't optimal yet
    K = find(abs(PM(:, alpha)-PdM(:, alpha)) > 0.001);
    fprintf("Iteration %i, Total difference is %f \n", i, ...
        sum(abs(PM(:, alpha)-PdM(:, alpha))));
    if isempty(K)
        break;
    end%if
    % Step 4 - Update Weight Vector
    % Iterative Least-Squares
    while (1)
        % Replacing V_pattern by V
        PdP = W0'*V*inv(diag(PdM(:, alpha))); % Replace inv by pinv -> very 
        % long computation time
        PdP0 = PdP./abs(PdP);
        W1 = inv(V*V')*V*diag(PdM(:, alpha))*PdP0';
        if norm(W1-W0)^2 < 10^(-4)
            break;
        else
            W0 = W1;
        end%if
    end%while
    W0 = W1;
    PM = abs(W0'*V_pattern);
end%for
end%function
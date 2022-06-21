function [PdM, P_refGen, W0] = generateDesPattern(theta, mainLobeAngle, ...
    V_pattern)
% Functions generates a pattern magnitue (PdM) with the main lobe pointing 
% into mainLobeAngle direction (in degrees) and with the side lobe level 
% being almost 0. Setting SLL to exctly 0 yields division-by-zero error 
% during optimization. PdM is obtained by setting the side lobes of 
% P_refGen to  zero. In turn, P_refGen is computed from W0, which is a
% beamforming  vector obtained during Capon's initialization. All three 
% variables are returned.

% Initial weight vector initialization using Capon's beamforming method
% Find the index correponding to the closest avaialbe angle, the main lobe 
% shall point to
%[~, mainLobeDirIdx] = min(abs(theta-mainLobeAngle*pi/180));
[~, mainLobeDirIdx] = min(abs(theta-mainLobeAngle));
% Initial weight vector using Capon's beamforming method
Ruu = V_pattern(:, mainLobeDirIdx)*V_pattern(:, mainLobeDirIdx)' ... 
    + (rand(1)/1000)^2*eye(size(V_pattern, 1));
W0 = (inv(Ruu)*V_pattern(:, mainLobeDirIdx))/ ...
    (V_pattern(:, mainLobeDirIdx)'*inv(Ruu)*V_pattern(:, mainLobeDirIdx));
% Step 1 of ILS - Initialization
P_refGen = abs(W0'*V_pattern); % For reference pattern generation
% Get the desired magnitue pattern without sidelobes
nullIdcs = findNulls(P_refGen);  % Find two closest-to-the-main-lobe nulls
PdM = P_refGen;
PdM(1:nullIdcs(1)) = 0; % Changing from 1e-15
PdM(nullIdcs(2):end) = 0;
end%function
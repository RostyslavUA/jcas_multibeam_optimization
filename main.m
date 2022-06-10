close all;
clear;
clc;
%% Variable initialization
theta = (-90:0.1:90-0.1)*pi/180;
% Desired directions of the lobes
desRange = (-60:1:60)*pi/180;
lambda = 2; % Wavelength

M_s = 12; % Number of senisng array elemetns
M_c = 16; % Number of communicating array elements
% Sensing & communication steering vectors
V_pattern_s = generateSteeringVector(theta, M_s, lambda);
V_pattern_c = generateSteeringVector(theta, M_c, lambda);
% Desired mainlobe directions for sensing and communicating beams
% Test different directions
desDirs_s = [-54.3, -37.8, -24.4, -12.3, 10.8, 22.8, 35.9, 51.9];
desDirs_c = 0;
% Container to save the sensing & communicating optimized patterns
Ws_s = zeros(M_s, size(desDirs_s, 2));
Ws_c = zeros(M_c, size(desDirs_c, 2));
%% Optimization of the sensing & communicating beams
alpha = find(ismember(theta, desRange));
for i = 1:size(desDirs_s, 2)
    % Genrate a desired pattern and select the angles to be approximated
    [PdM, P_refGen, W0] = generateDesPattern(theta, desDirs_s(i), ...
        V_pattern_s);
    % Input to ILS
    P_init = ones(size(theta)); % For plotting
    PM = P_init; % Optimization variable
    % Optimization
    Ws_s(:, i) = twoStepILS(100, alpha, V_pattern_s, W0, PM, PdM);
end%for
for i = 1:size(desDirs_c, 2)
    [PdM, P_refGen, W0] = generateDesPattern(theta, desDirs_c(i), ...
        V_pattern_c);
    P_init = ones(size(theta));
    PM = P_init;
    Ws_c(:, i) = twoStepILS(100, alpha, V_pattern_c, W0, PM, PdM);
end%for
%% Plot the beams coming from the optimized beamforming vectors
figure;
hold on
for i = 1:size(desDirs_s, 2)
    plot(theta, 20*log10(abs(Ws_s(:, i)'*V_pattern_s)));
end%for
for i = 1:size(desDirs_c, 2)
    plot(theta, 20*log10(abs(Ws_c(:, i)'*V_pattern_c)), '--');
end%for
%% Combination of the sensing and communicating beams
ro = 0.1; % Communication-sensing trade-off parameter
% Method 1
% Normalize beams
Ws_c = Ws_c./vecnorm(Ws_c, 2, 2);
Ws_s = Ws_s./vecnorm(Ws_s, 2, 2);
% Combined beam
WtV = zeros(size(desDirs_s, 2), size(theta, 2));
for i = 1:size(WtV, 1)
    WtV(i, :) = sqrt(ro)*Ws_c'*V_pattern_c ...
        + sqrt(1-ro)*Ws_s(:, i)'*V_pattern_s;
end%for
%% Plot a combined beam
plot(theta*180/pi, 20*log10(abs(WtV(2, :))))
text(4, 10, '\leftarrowCommunication Beam')
text(-53, 10, 'Sensing Beam\rightarrow')
xlabel("\theta in rad")
ylabel("|A(\theta)|, dB")
xlim([-70, 70])
ylim([-30, 20])
%% Plot the combined beams
figure;
hold on;
for i = 1:size(WtV, 1)
    plot(theta, 20*log10(abs(WtV(i, :))));
end%for
xlabel("\theta in degrees")
ylabel("|A(\theta)|, dB")
xlim([-pi/2, pi/2])
ylim([-30, 20])

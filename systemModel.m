M = 16; % Number of array elements
phi = -180:1:180; % Phase shift between communication & sensing beams(deg)
theta = -180:1:180; % Space around antenna
desDir = [-54.3, -37.8, -24.4, -12.3, 10.8, 22.8, 35.9, 51.9]; % Desired
% pointing directions (deg)
bmwc = 2*asin(1.2/16); % 3 dB beamwidth of a communication beam (rad)
bmws = 2*asin(1.2/12); % 3 dB beamwidth of a sensing beam (rad)
ro = 0.5;
% Array response
a = zeros(1, size(desDir, 2));
for i = 1:M
    a(i, :) = exp(i*1i*pi*sin(desDir)); % Eq. 1
end
% Channel
L = 10; % Number of multipath channels
v = 10; % Assumed speed of an object (m/s)
fd = v/physconst('LightSpeed'); % Doppler shift (Hz). If works correctly, 
% consider adding a variation to make it more realistic
d = 250; % Assumed distance to the object (m)
tau = d/physconst('LightSpeed'); % Delay (s) If works correctly, 
% consider adding a variation to make it more realistic
b = randi(2, L, 1) + 1i*randi(2, L, 1); % Amplitude

doi = 10.8; % Direction of interest
H = zeros(M, M);
for i=1:L
    H = H + b(i)*exp(1i*2*pi*fd)*(a(:, desDir==doi)*a(:, desDir==doi)'); % Eq.2
end

wtc = randi(2, M, 1)-1; % Tx beamformer (BF) matrix for communication
wts = randi(2, M, 1)-1; % Tx BF matrix for sensing
wt = sqrt(ro)*wtc + sqrt(1-ro)*exp(1i*phi).*wts; % Combined Tx BF matrix. Eq. 5
wr = randi(2, M, 1)-1; % Rx BF matrix

rxPow = wr'*H*wt;
plot(phi, abs(rxPow)/max(abs(rxPow)));
xlabel('\phi, Degrees')
ylabel('Normalized Power')

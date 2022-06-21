function V_pattern = generateSteeringVector(theta, M, lambda)
% DESCTIPTION TO BE UPDATED
% Function generates the steering vector from the array reponse vector. By 
% computation of array response vector f, it is assumed that M antenna
% elements are equally spaced at the interval of half of the wavelength. 
% Moreover, planar wavefront and narrow-band beamforming models are 
% assumed.
%f = zeros(1, size(theta, 2)); % Array response
V_pattern = zeros(1, size(theta, 2)); % Steering vector
x = lambda/2; % Half-wavelength spacing
for i = 1:M
    %f(i, :) = exp((i-1)*1i*pi*sin(theta));
    %V_pattern(i, :) = f(i, :).*exp(1i*2*pi/lambda*x*(i-1)*sin(theta));
    V_pattern(i, :) = exp(1i*2*pi/lambda*x*(i-1)*sin(theta));
end%for
end%function
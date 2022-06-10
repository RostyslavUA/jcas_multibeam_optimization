function V_pattern = generateSteeringVector(theta, M, lambda)
% Function generates the steering vector from the array reponse vector. By 
% computation of array response vector f, it is assumed that M antenna
% elements are equally spaced at the interval of half of the wavelength. 
% Moreover, planar wavefront and narrow-band beamforming model are assumed.
f = zeros(1, size(theta, 2)); % Array response
V_pattern = zeros(1, size(theta, 2)); % Steering vector
for i = 1:M
    f(i, :) = exp((i-1)*1i*pi*sin(theta));
    V_pattern(i, :) = f(i, :).*exp(1i*2*pi*i/M*sin(theta)/lambda);
end%for
end%function
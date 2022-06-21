function Aq = generateQuantizedArrResponse(M, eqDir)
% Function generates a quanitzed array response. Quantization refers to the 
% phase delay term sin(theta), where the function's value (as opposed to 
% its argument) is quantized and hence becomes equispaced. This then allows
% to use a single phase-shifting vector to displace the reference 
% pattern along the equivalent directions.

% Array response matrix with quantized, equally spaced element values
Aq = zeros(M, size(eqDir, 2)); 
for m = 1:M
    for q = 1:size(eqDir, 2)
        Aq(m, q) = exp(1i*pi*(m-1)*eqDir(q));
    end%for
end%for
end%function
function [c_unique] = creat_codebook(phi,b_1,b_2)
delta_1 = 2*pi*2^(-b_1);
delta_2 = 2*pi*2^(-b_2);
B_1 = [];
B_2 = [];
for i = 0:(2^b_1-1)
    B_1 = [B_1, i*delta_1];
end
for i = 0:(2^b_2-1)
    B_2 = [B_2, phi+i*delta_2];
end
c = [];
for i = 1:length(B_1)
    for j = 1:length(B_2)
        c_new = exp(1i*B_1(i))+exp(1i*B_2(j));
        if abs(real(c_new)) <= 10e-7
            c_new_real = 0;
        else
            c_new_real = real(c_new);
        end
        if abs(imag(c_new)) <= 10e-7
            c_new_imag = 0;
        else
            c_new_imag = imag(c_new);
        end
        c_new = c_new_real + 1i*c_new_imag;
        c = [c, c_new];
    end
end
c_unique = unique(c);
c_unique(c_unique==0) = [];
end


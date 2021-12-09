function [H, x, num, den] = lsf(n,N,u,y)
H = zeros(N,n);
for i = 1:n
    for k = 1:N
        H(k,i) = -y(k)* (1i * u(k))^i;
    end
end

for i = 0:(n-1)
    for k = 1:N
       H(k,i+n+1) = (1i * u(k))^i;
    end
end

x = inv(H' * H) * H' * y;


num = fliplr(x(n+1:end).');
den = fliplr([1, x(1:n).']);
end


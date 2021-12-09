function x_dot = pblm5_nonlin(x,w,G,M)
    arguments
        x (4,1)
        w = 10e-6 * randn;
        G = 6.674e-11;
        M = 5.98e24;
    end
    x_dot = x; % to get same array type
    x_dot(1) = x(2);
    x_dot(2) = x(1) * x(4)^2 - (G*M)/(x(1)^2) + w;
    x_dot(3) = x(4);
    x_dot(4) = -2*x(4)*x(2)/x(1);
end
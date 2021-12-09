close all


v = linspace(0,30);
eta = -0.001778 * v.^2 + 0.05333 * v;

r = 5;
A = pi * (r/2)^2;
rho = 1.27;
mph2metric = 0.447;
p = (1/2)* rho * A * (v * mph2metric).^3 .*eta


figure()
plot (v,eta,'b-');
% xlabel('Speed (m/s)')
% ylabel('Efficiency (%)')

figure()
plot(v,p)


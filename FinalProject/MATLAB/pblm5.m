function [] = pblm5()
% Problem 5 Scipt/Function
% MECH 6325 - Final Probject
G_const = 6.674e-11;
M_const = 5.98e24;

% Part a ---------------------------
disp('Part a ----------------------------')
syms r r_dot theta theta_dot w G M
x = [r; r_dot; theta; theta_dot];
x_dot_sym = pblm5_nonlin(x,w,G,M)

% Part b --------------------------
disp('Part b ---------------------------')
eq = 0 == subs(x_dot_sym(2),w,0)
theta_dot_sym = solve(eq,theta_dot)

% Part c ---------------------------
disp('Part c ----------------------------')
syms r_0 omega_0 T
x_0 = [r_0; 0; omega_0 * T; omega_0];
omega_static = subs(theta_dot_sym,[G, M, r],[G_const, M_const, r_0]);

A = subs(jacobian(x_dot_sym,x), x, x_0)
L = jacobian(x_dot_sym,w)


r_0_const = 6.57e6;
omega_0_const = double(subs(omega_static(1), r_0,r_0_const))
v_const = omega_0_const * r_0_const % This checks orbital speed... should be around 7-8 km/s
A_const = double(subs(A,[omega_0, r_0, G, M],...
                        [omega_0_const, r_0_const, G_const, M_const]))
eig_A = eig(A_const)
tau = 2*pi / (imag(eig_A(2)))
min_int_step = 0.1 * tau


% Part d ------------------------------------
disp('Part d ---------------------------------')
T = 5;
sensorT = 60;
T_max = 3 * 60 * 60;
H = [1,0,0,0;
     0,0,1,0];
R = diag([100,0.1]);

x_0 = [r_0_const; 0; 0; 1.1*omega_0_const];
Q_c = 10e-6;
x_hat_0 = x_0;
P_0 = zeros(4);



[X,Xt,Y,X_hat,P,K,sensor_t] = KalmanFilter_Hybrid_nonlinsys(@pblm5_nonlin,...
                                            A_const,0,L,H,...
                                            x_0, Q_c, R,...
                                            x_hat_0, P_0,...
                                            T,T_max,sensorT);

figure()
sgtitle('Linearized Kalman Filter')
subplot(2,2,1);
hold on
plot(Xt/3600,X(1,:))
plot(sensor_t/3600,X_hat(1,:))
hold off
title('Radial Position')
xlabel('Time (Hr)')
ylabel('(m)')

subplot(2,2,2);
hold on
plot(Xt/3600,X(2,:))
plot(sensor_t/3600,X_hat(2,:))
hold off
title('Radial Velocity')
xlabel('Time (Hr)')
ylabel('(m/s)')

subplot(2,2,3);
hold on
plot(Xt/3600,X(3,:))
plot(sensor_t/3600,X_hat(3,:))
hold off
title('Angular Position')
xlabel('Time (Hr)')
ylabel('(radians)')

subplot(2,2,4);
hold on
plot(Xt/3600,X(4,:))
plot(sensor_t/3600,X_hat(4,:))
hold off
title('Angular Velocity')
xlabel('Time (Hr)')
ylabel('(rad/s)')

saveas(gcf,[pwd '/fig/pblm5_results_hybrid_lin.png'])

test_t = 0:sensorT:T_max;
test_x = interp1(Xt',X',test_t)';
test_x_hat = interp1(sensor_t',X_hat',test_t)';

x_error = test_x_hat - test_x;

figure()
plot(test_t/3600,x_error(1,:));
title(['Linearized Kalman Filter', newline,...
        'Radial Position Estimate Error'])
xlabel('Time (Hr)')
ylabel('(m)')

saveas(gcf,[pwd '/fig/pblm5_est_error_hybrid_lin.png'])

disp(['The performance of the linear Kalman Filter is reduced by the ',...
      'lack of ', newline, 'linearity in the actual system dynamics.',...
      ' This may be improved with faster', newline, 'measurments',...
      ' and Kalman Filter updates; however this is not gaurenteed.']);





% Part e ---------------------------------------------------------
disp('Part e --------------------------------')
[X,Xt,Y,X_hat,P,K,sensor_t] = KalmanFilter_Extended_Hybrid(@pblm5_nonlin,...
                                            A_const,0,L,H,...
                                            x_0, Q_c, R,...
                                            x_hat_0, P_0,...
                                            T,T_max,sensorT);

figure()
sgtitle('Extended Kalman Filter')
subplot(2,2,1);
hold on
plot(Xt/3600,X(1,:))
plot(sensor_t/3600,X_hat(1,:))
hold off
title('Radial Position')
xlabel('Time (Hr)')
ylabel('(m)')

subplot(2,2,2);
hold on
plot(Xt/3600,X(2,:))
plot(sensor_t/3600,X_hat(2,:))
hold off
title('Radial Velocity')
xlabel('Time (Hr)')
ylabel('(m/s)')

subplot(2,2,3);
hold on
plot(Xt/3600,X(3,:))
plot(sensor_t/3600,X_hat(3,:))
hold off
title('Angular Position')
xlabel('Time (Hr)')
ylabel('(radians)')

subplot(2,2,4);
hold on
plot(Xt/3600,X(4,:))
plot(sensor_t/3600,X_hat(4,:))
hold off
title('Angular Velocity')
xlabel('Time (Hr)')
ylabel('(rad/s)')

saveas(gcf,[pwd '/fig/pblm5_results_hybrid_EKF.png'])

test_t = 0:sensorT:T_max;
test_x = interp1(Xt',X',test_t)';
test_x_hat = interp1(sensor_t',X_hat',test_t)';

x_error = test_x_hat - test_x;

figure()
plot(test_t/3600,x_error(1,:));
title(['Extended Kalman Filter', newline,...
        'Radial Position Estimate Error'])
xlabel('Time (Hr)')
ylabel('(m)')

saveas(gcf,[pwd '/fig/pblm5_est_error_hybrid_EKF.png'])

disp(['As is clear between the two estimate error plots, the Extended ',...
      'Kalman Filter ', newline, 'has a lot less error then the Linear one.',...
      'This is very evident from the order ', newline,...
      'of magnitude on each of the plots.'])

assignin('base','X',X)
  
  

end
function [] = pblm2()
% Problem 2 Scipt/Function
% MECH 6325 - Final Probject

% Parameters
R = 100;
L = 1;
C = 1;

% CT System Definitions
A = [-2/(R*C), 1/C;
     -1/L, 0];
B = [1/(R*C);
     1/L];
C = eye(2);
Q_c = 3^2;

T = 0.1;
T_max = 2;
N = floor(T_max/T);

disp('Discretized System:')
[F,G,H,Q,~] = DiscretizeSystem(A,B,C,Q_c,0,T)
% L = G

n = size(F,1);
x_0 = zeros(n,1);
R = eye(2);

x_hat_0 = x_0;
P_0 = 0.1 * eye(n);

[X,Y,~,X_hat_post,P_pre,P_post,~] = ...
    KalmanFilter_Sequential(F,G,G,H,x_0,Q,R,x_hat_0,P_0,N,n);

%Plotting
t = T:T:T_max;
% Part a
figure()
hold on
plot(t,reshape(P_pre(1,1,:),[],1))
plot(t,reshape(P_post(1,1,:,1),[],1))
plot(t,reshape(P_post(1,1,:,2),[],1))
hold off
title('Capacitor Voltage Estimation Variance')
legend('P_{pre}','P_{post}_1','P_{post}_2')
xlabel('Time (s)')
ylabel('Variance')
saveas(gcf,[pwd '/fig/pblm2_est_var.png'])

% Part b
figure()
hold on
plot(t,X(1,:))
plot(t,X_hat_post(1,:))
plot(t,Y(1,:))
hold off
title('Capacitor Voltage Comparison')
legend('Actual','Estimated','Measured')
xlabel('Time (s)')
ylabel('Voltage (V)')
saveas(gcf,[pwd '/fig/pblm2_comparison.png'])

disp('Standard diviations of errors')
Y_error_std = std(Y(1,:)-X(1,:))
X_hat_error_std = std(X_hat_post(1,:)-X(1,:))

end
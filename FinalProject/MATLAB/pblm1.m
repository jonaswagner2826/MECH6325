function [] = pblm1()
% Problem 1 Scipt/Function
% MECH 6325 - Final Probject

% System Definition
F = [1/2, 2;
     0, 0];
G = [0; 1];
f_0 = 250;
L = [0; 1];
H = [1, 0];

x_0 = [650; 250];
Q = 10;
R = 10;

x_hat_0 = [600; 200];
P_0 = [500, 0;
       0, 200];

% Part a - KF Simulation (10 steps) -------------------------------------
n = size(F,1);
T = 1;
T_max = 10;
N = floor(T_max/T);
U = f_0 * ones(N+1,1);

[X,~,~,X_hat,P,K] = KalmanFilter_DT(F,G,L,H,x_0,Q,R,x_hat_0,P_0,N,n,U);


figure('position',[0,0,3*300,2*300])
sgtitle('Modeling Method 1')

subplot(2,2,1)
hold on
plot(X(1,:))
plot(X_hat(1,:))
hold off
title('Wombat Population')
legend('Actual','Estimated')

subplot(2,2,2)
hold on
plot(X(2,:))
plot(X_hat(2,:))
hold off
title('Food Supply')
legend('Actual','Estimated')

subplot(2,2,3)
hold on
plot(sqrt(reshape(P(1,1,:),[],1)))
plot(sqrt(reshape(P(2,2,:),[],1)))
hold off
title('Standard Deviation')
legend('p_k','f_k')

subplot(2,2,4)
hold on
plot(K(1,:))
plot(K(2,:))
hold off
title('Gain Matrix')
legend('k_1','k_2')

saveas(gcf,[pwd '/fig/pblm1_results_method1.png'])


% Part b - P_inf comparison ---------------------------------------------
disp('If using the following F, there is a descrepancy for state 2')
F = [1/2, 2;
     0, 0]
P_minus_inf = idare(F',H',Q,R)
k_inf = P_minus_inf*H'/(H*P_minus_inf*H' + R)
P_plus_inf = (eye(2)-k_inf*H)*P_minus_inf
P_plus_inf_std = sqrt(P_plus_inf)

% Part c - Simulation (1000 steps)
T_max = 1000;
N = floor(T_max/T);
U = f_0 * ones(N+1,1);
[X1,~,~,X_hat1,P1,~] = KalmanFilter_DT(F,G,L,H,x_0,Q,R,x_hat_0,P_0,N,n,U);

figure()
hold on
plot(sqrt(reshape(P1(1,1,:),[],1)))
plot(sqrt(reshape(P1(2,2,:),[],1)))
hold off
title('Standard Deviation')
legend('p_k','f_k')
saveas(gcf,[pwd '/fig/pblm1_std_method1.png'])

disp(['There is a decrepancy for the secound state.', newline,...
     'This is belived to be becouse of the constant input not = 0,',...
     newline, 'so the steady-state Kalman filter for that state is ',...
     'not as accurate.']);


disp('---------------------------------------------------------------')


disp('The alternative method of simulation:')

F = [1/2, 2;
     0, 1]
G = [0; 1]

n = size(F,1);
T = 1;
T_max = 10;
N = floor(T_max/T);
U = 0 * ones(N+1,1);

[X,~,~,X_hat,P,K] = KalmanFilter_DT(F,G,L,H,x_0,Q,R,x_hat_0,P_0,N,n,U);


figure('position',[0,0,3*300,2*300])
sgtitle('Modeling Method 2')

subplot(2,2,1)
hold on
plot(X(1,:))
plot(X_hat(1,:))
hold off
title('Wombat Population')
legend('Actual','Estimated')

subplot(2,2,2)
hold on
plot(X(2,:))
plot(X_hat(2,:))
hold off
title('Food Supply')
legend('Actual','Estimated')

subplot(2,2,3)
hold on
plot(sqrt(reshape(P(1,1,:),[],1)))
plot(sqrt(reshape(P(2,2,:),[],1)))
hold off
title('Standard Deviation')
legend('p_k','f_k')

subplot(2,2,4)
hold on
plot(K(1,:))
plot(K(2,:))
hold off
title('Gain Matrix')
legend('k_1','k_2')

saveas(gcf,[pwd '/fig/pblm1_results_method2.png'])

P_minus_inf = idare(F',H',L*Q*L',R)
k_inf = P_minus_inf*H'/(H*P_minus_inf*H' + R)
P_plus_inf = (eye(2)-k_inf*H)*P_minus_inf
P_plus_inf_std = sqrt(P_plus_inf)

% x_hat_error_inf = 

T_max = 1000;
N = floor(T_max/T);
U = 0 * ones(N+1,1);
[X2,~,~,X_hat2,P2,~] = KalmanFilter_DT(F,G,L,H,x_0,Q,R,x_hat_0,P_0,N,n,U);

figure()
hold on
plot(sqrt(reshape(P2(1,1,:),[],1)))
plot(sqrt(reshape(P2(2,2,:),[],1)))
hold off
title('Standard Deviation')
legend('p_k','f_k')
saveas(gcf,[pwd '/fig/pblm1_std_method2.png'])


disp('The alternative method has a descrepancy for the food term as well')

end


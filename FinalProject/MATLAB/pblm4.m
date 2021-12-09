function [] = pblm4()
% Problem 4 Scipt/Function
% MECH 6325 - Final Probject

% Parameters
T = 0.5;
T_max = 5;

omega = 6;
zeta = 0.16;

% System Definition
A = [0,         1;
     -omega^2,  -2*zeta*omega];
B = [0;1];
C = eye(2);
Q_c = 0.01;

sys = ss(A,B,C,0)


% Part a - Discretization
[F,G,~,Q,~] = DiscretizeSystem(A,B,C,Q_c,0,T);
H = [1,0];
Q
L=G;
R = 1e-4;


dt_sys = ss(F,G,H,0,T)

% Part b - Simulation
x_0 = [1;1];
x_hat_0 = x_0;
P_0 = diag([1e-5,1e-2]);


[X,Xt,~,~,~,X_hat,P,~,hat_t] = KalmanFilter_DT_SDS(sys,F,0,L,H,...
                                            x_0,Q,R,x_hat_0,P_0,T,T_max);



figure('position',[0,0,3*300,2*300])
sgtitle('Sampled Data System Response')

subplot(2,2,1)
hold on
plot(hat_t(1:end-1),reshape(P(1,1,1:end-1),[],1))
hold off
title('Estimation Error Variance (State 1)')
xlabel('Time (s)')

subplot(2,2,2)
hold on
plot(hat_t(1:end-1),reshape(P(2,2,1:end-1),[],1))
hold off
title('Estimation Error Variance (State 2)')
xlabel('Time (s)')

subplot(2,2,3)
hold on
plot(Xt,X(1,:))
plot(hat_t,X_hat(1,:))
hold off
title('State 1')
legend('Actual','Estimate')
xlabel('Time (s)')

subplot(2,2,4)
hold on
plot(Xt,X(2,:))
plot(hat_t,X_hat(2,:))
hold off
title('State 2')
legend('Actual','Estimate')
xlabel('Time (s)')
saveas(gcf,[pwd '/fig/pblm4_results.png'])
end
% MECH 6325 - Homework 44
close all
clear

% Problem 2 b
F = 1;
H = 1;
Q = 1;
R = 1;
M = 0;

syms x_k_1 x_est_k_1 w v

syms x_k(x_k_1, w, v)
x_k(x_k_1,w, v) = H * x_k_1 + w;

syms y
y = x_k + v;

syms K e_k_1
x_est_k = (1- K * H) * F * x_est_k_1 + K * y

e_k = x_k - x_est_k

e_k = (1-K)*e_k_1 + (1-K)*w - K*v

e_k_sqr = expand(e_k^2)

syms P_k_1
P_k = subs(e_k_sqr, [e_k_1^2,   w^2,    v^2,    v*w,    e_k_1*w,    e_k_1*v], ...
                    [P_k_1,     Q,      R,      M,      0,          0]);
syms P_inf
P = solve(P_inf == subs(P_k,P_k_1,P_inf), P_inf);

K_inf = P_inf * H * R^-1;

P_ss = solve(P_inf == subs(P,K,K_inf),P_inf);
P_inf = double(P_ss(3))



% Problem 3 b
clear
F = 1/2;
H = 1;
Psi = 1/2;
Q_w = 1;
R = 1;
Q_zeta = 1;


syms x_k_1 x_est_k_1 w v_k_1 zeta

x_k = H * x_k_1 + w;
v_k = Psi * v_k_1 + zeta;
y = x_k + v_k;

syms K e_k_1

x_est_k = (1- K * H) * F * x_est_k_1 + K * y

e_k = x_k - x_est_k %(1-K)*e_k_1 + (1-K)*w - K*v

e_k = (1-K)*F*e_k_1 + (1-K)*w - K*Psi*v_k_1 - K*zeta

e_k_sqr = expand(e_k^2)

syms P_k_1
P_k = subs(e_k_sqr, [e_k_1^2, w^2, v_k_1^2,      zeta^2, e_k_1*w, e_k_1*v_k_1, e_k_1*zeta, w*v_k_1, w*zeta, v_k_1*zeta], ...
                    [P_k_1,   Q_w, Psi * Q_zeta, Q_zeta, 0,       0,           0,          0,       0,      0])
syms P_inf
P = solve(P_inf == subs(P_k,P_k_1,P_inf), P_inf)

K_inf = P_inf*H*R^(-1);

P_ss = solve(P_inf == subs(P,K,K_inf),P_inf);

P_inf = double(P_ss)

(sqrt(65)-7)/2


% Problem 3 d
F = 1/2;
H =1;
Psi = 1/2;
Q_w = 1;
Q_zeta = 1;
R = Psi * Q_w;

N = 100;
X = zeros(N,1);
Y = zeros(N,1);
V = zeros(N,1);
w = Q_w * randn(N,1);
zeta = Q_zeta * randn(N,1);

x_0 = randn(1);
w_0 = randn(1);
v_0 = randn(1);
zeta_0 = randn(1);
X(1) = F*x_0 + w_0;
V(1) = Psi*v_0 + zeta_0;
Y(1) = H * X(1) + V(1);
for i = 2:N
    X(i) = F * X(i-1) + w(i-1);
    V(i) = Psi * V(i-1) + zeta(i-1);
    Y(i) = H * X(i) + V(i);
end


% KF 1
n = 1;
F = 1/2;
H =1;
Psi = 1/2;
Q = 1;
R = 1;

X1 = zeros(N,1);
X1_est_pri = zeros(N,1);
X1_est_post = zeros(N,1);
P1_pri = zeros(N,1);
P1_post = zeros(N,1);
K1 = zeros(N,1);

x1_0 = 0;
p1_0 = 1;

P1_pri(1) = F*p1_0*F' + Q;
K1(1) = P1_pri(1) * H' * inv(H * P1_pri(1) * H' + R);
X1_est_pri(1) = F*x1_0;
X1_est_post(1) = X1_est_pri(1) + K1(1) * (Y(1) - H * X1_est_pri(1));
P1_post(1) = (eye(n) - K1(1) * H) * P1_pri(1);
for i = 2:N
    P1_pri(i) = F*P1_post(i-1)*F' + Q;
    K1(i) = P1_pri(i) * H' * inv(H * P1_pri(i) * H' + R);
    X1_est_pri(i) = F*X1_est_post(i-1);
    X1_est_post(i) = X1_est_pri(i) + K1(i) * (Y(i) - H * X1_est_pri(i));
    P1_post(i) = (eye(n) - K1(i) * H) * P1_pri(i);
end




% KF 2
n = 2;
F = 1/2 * eye(n);
H = [1 1];
Q = 1;
R = 0;

X2 = zeros(N,2);
X2_est_pri = zeros(N,n);
X2_est_post = zeros(N,n);
P2_pri = zeros(N,n,n);
P2_post = zeros(N,n,n);
K2 = zeros(N,n);

x2_0 = [0; 0];
p2_0 = eye(n);

p2p =F*p2_0*F' + Q;
P2_pri(1,:,:) = p2p;
k2 = p2p * H' * inv(H * p2p * H' + R);
K2(1,:) = k2;
x2p = F*x2_0;
X2_est_pri(1,:) = x2p;
X2_est_post(1,:) = x2p + K2(1) * (Y(1) - H * x2p);
P2_post(1,:,:) = (eye(n) - K2(1) * H) * p2p;
for i = 2:N
    p21 = reshape(P2_post(i-1,:,:),2,2);
    p2p =F*p21*F' + Q;
    P2_pri(i,:,:) = p2p;
    k2 = p2p * H' * inv(H * p2p * H' + R);
    K2(i,:) = k2;
    x21 = reshape(X2_est_post(i-1,:),2,1);
    x2p = F*x21;
    X2_est_pri(i,:) = x2p;
    X2_est_post(i,:) = x2p + K2(i) * (Y(i) - H * x2p);
    P2_post(i,:,:) = (eye(n) - K2(i) * H) * p2p;
end


hold on
plot(X)
plot(X1_est_post)
plot(X2_est_post)
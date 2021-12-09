% MECH 6325 - Homework 3
close all

% Problem 1
F = [0      1/2
    -1/2    2];
G = [0  0]';
L = [1  1]';
q = 1;
Q = L * q * L';

% Part 1a
x_bar_1a = inv(eye(2) - F)*G*0

% Part 1b
P_1b = dlyap(F,Q)


% Problem 2
% Part 2a
A = [-1 0
    0   -1];
L = [5  1]';
q = 1;
Q = L * q * L';

P_2a = lyap(A,Q)

% Part 2b
F = [0.5    0
    0       0.5];
Q = [1  0
    0   1];

P_2b = dlyap(F,Q)

% Problem 3
F = -1/2;
Q = 1;

P_3 = dlyap(F,Q)


% Problem 4
F = [0  1
    0   0];
L = [0  1]';
q = 1;
Q = L * q * L';

H = [1  1];
R = 0;

syms pr11 pr12 pr21 pr22
Pr =  [pr11 pr12; pr21 pr22];

K = Pr * H' * inv(H * Pr * H' + R);

Po = (eye(2) - K * H) * Pr;

Pr = F * Po * F' + Q;

%-----------------


% Problem 6
F = [1  1
    0   1];
Q = eye(2);
H = [1  0];
syms k
R_sym = 2 + (-1)^k;

n = 2;
N = 10;
P_pri = zeros(n,n,N);
P_post = zeros(n,n,N);
K_all = zeros(n,1,N);

P_0 = 10 * eye(1);

for i = 1:N
    R = subs(R_sym,k,i);
    if (i == 1)
        Pr = P_0;
        Po = P_0;
    else
        Pr = P_pri(:,:,i-1);
        Po = P_post(:,:,i-1);
    end
    K = Pr * H' * inv(H * Pr * H' + R);
    Po = (eye(2) - K * H) * Pr;
    Pr = F * Po * F' + Q;
    
    P_pri(:,:,i) = Pr;
    P_post(:,:,i) = Po;
    K_all(:,:,i) = K;
end

P_post

% Problem 7
F = 1;

Q = 4;
R = 1;

P_0 = 100;

% Part 7i
H = 0.5;

n = 1;
N = 200;
P_pri = zeros(n,n,N);
P_post = zeros(n,n,N);
K_all = zeros(n,1,N);

for i = 1:N
    if (i == 1)
        Pr = P_0;
        Po = P_0;
    else
        Pr = P_pri(:,:,i-1);
        Po = P_post(:,:,i-1);
    end
    K = Pr * H' * inv(H * Pr * H' + R);
    Po = (eye(n) - K * H) * Pr;
    Pr = F * Po * F' + Q;
    
    P_pri(:,:,i) = Pr;
    P_post(:,:,i) = Po;
    K_all(:,:,i) = K;
end

figure()
y = reshape(P_post,1,[]);
plot(y)
title("Problem 7 a")



% Part 7i
syms k
H_sym = cos(1 + k/120);

n = 1;
N = 200;
P_pri = zeros(n,n,N);
P_post = zeros(n,n,N);
K_all = zeros(n,1,N);

for i = 1:N
    H = subs(H_sym,k,i);
    if (i == 1)
        Pr = P_0;
        Po = P_0;
    else
        Pr = P_pri(:,:,i-1);
        Po = P_post(:,:,i-1);
    end
    K = Pr * H' * inv(H * Pr * H' + R);
    Po = (eye(n) - K * H) * Pr;
    Pr = F * Po * F' + Q;
    
    P_pri(:,:,i) = Pr;
    P_post(:,:,i) = Po;
    K_all(:,:,i) = K;
end

figure()
y = reshape(P_post,1,[]);
plot(y)
title("Problem 7 b")

%--------------------

% Problem 8
A = -1;
Q = exp(-2);

n = 1;
N = 100;
tMax = 5;
T = tMax / N;
t = linspace(0,tMax,N);
X = zeros(n,n,N);
P = zeros(n,n,N);

x_0 = 1;
P_0 = 1;

for i = 1:N
    if i == 1
        x = x_0;
        p = P_0;
    else
        x = X(i-1);
        p = P(i-1);
    end
    
    x = x + (A * x) * T;
    p = p + (A * p + p * A' + Q) * T;
    
    X(i) = x;
    P(i) = p;
end

x = t;
y1 = reshape(X, 1, []);
y2 = reshape(P, 1, []);

figure()

subplot(1,2,1);
title("Problem 8 - States")
hold on
plot(x,y1)

subplot(1,2,2);
title("Problem 8 - Covariance")
hold on
plot(x,y2)


x_0 = 1;
P_0 = 0;

for i = 1:N
    if i == 1
        x = x_0;
        p = P_0;
    else
        x = X(i-1);
        p = P(i-1);
    end
    
    x = x + (A * x) * T;
    p = p + (A * p + p * A' + Q) * T;
    
    X(i) = x;
    P(i) = p;
end

x = t;
y1 = reshape(X, 1, []);
y2 = reshape(P, 1, []);

subplot(1,2,1);
plot(x,y1)
hold off

subplot(1,2,2);
plot(x,y2)

p_8 = lyap(A,Q)
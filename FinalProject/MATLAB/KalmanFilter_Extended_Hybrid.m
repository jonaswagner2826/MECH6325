function [X,Xt,Y,X_hat,P,K,sensor_t] = KalmanFilter_Extended_Hybrid(...
                                            nonlinsys,A,B,L,H,...
                                            x_0,Q_c,R,...
                                            x_hat_0,P_0,...
                                            T,T_max,sensorT,n,U)                          
arguments
    nonlinsys
    A double
    B double
    L double
    H double
    x_0 double
    Q_c double
    R double
    x_hat_0 double
    P_0 double
    T double = 0.1;
    T_max double = 500;
    sensorT double = 0.5;
    n = size(A,1);
    U = zeros(ceil(T_max/T),1); %This code assuems U = 0... so not needed
end



% Empty Arrays
N = floor(T_max/sensorT);
X = [];
Y = [];
sensor_t = [];
K = [];
X_hat_pre = [];
X_hat_post = [];
P_pre = zeros(n,n,N);
P_post = zeros(n,n,N);


% For Loop Initiation
x = x_0;

p = P_0;
x_hat = x_hat_0;
k = p * H' * inv(R);

for i = 1:N
    % Actual System and Measurment
    t = i * sensorT;
%     x = interp1(Xt,X,t)';

    x_dot = nonlinsys(x);
    x = x + x_dot * sensorT;
    
    y = H * x + R * randn(size(R,2),1);
    
    X = [X x];
    Y = [Y y];
    sensor_t = [sensor_t t];
    
    % Non-lin Time Update
    x_hat_dot = nonlinsys(x_hat,0);
    x_hat = x_hat + x_hat_dot * sensorT;
    p_dot = A * p + p * A' + L*Q_c*L';
    p = p + p_dot * sensorT;
    
    X_hat_pre = [X_hat_pre x_hat];
    P_pre(:,:,i) = p;
    
    % Measurment Update
    k = p*H'*inv(H*p*H' + R);
    x_hat = x_hat + k * (y - H * x_hat);
    p = (eye(n)-k*H)*p*(eye(n)-k*H)' + k*R*k';

    K = [K k];
    X_hat_post = [X_hat_post x_hat];
    P_post(:,:,i) = p;
end


% X = X';
% Xt = Xt';
Xt = sensor_t;

X_hat = X_hat_post;
P = P_post;
end
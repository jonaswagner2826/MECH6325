function [X,Y,X_hat,P,K] = KalmanFilter_CT_Hybrid(A,B,L,H,x_0,Q_c,R,...
                                            x_hat_0,P_0,...
                                            T,T_max,sensorT,n,U)                          
    arguments
        A
        B
        L
        H
        x_0
        Q_c
        R
        x_hat_0
        P_0
        T = 0.1;
        T_max = 500;
        sensorT = 0.5;
        n = size(A,1);
        U = zeros(ceil(T_max/T),1);
    end
    N = floor(T_max/T);
    
    x = x_0;
    y = H * x + R * randn;
    p = P_0;
    x_hat = x_hat_0;
    
    u = U(1);
    k = p * H' * inv(R);

    X = [];
    Y = [];
    
    K = [];
    X_hat = [];
    P = zeros(n,n,N);
    
    t_sensor = 0;

    for i = 1:N
        t = i * T;
        u = U(i);
        x_dot = A * x + B * u + Q_c * randn;
        x = x + x_dot * T;
        if (t - t_sensor >= sensorT)
            y = H * x + R * randn;
            sensorT = t;
        end
        i
        k = p * H' * inv(R*T)
        x_hat_dot  = A * x_hat + B * u + k * (y - H * x_hat)
        x_hat = x_hat_dot * T
        p_dot = -p * H' * inv(R * T) * H * p + p * A' + Q_c
        p = p_dot * T
        
        
        X = [X x];
        Y = [Y y];
        
        K = [K k];
        X_hat = [X_hat x_hat];
        P(:,:,i) = p;
    end
end

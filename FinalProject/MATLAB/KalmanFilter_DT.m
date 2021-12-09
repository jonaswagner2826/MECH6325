function [X,Y,X_hat_pre,X_hat_post,P,K] = KalmanFilter_DT(F,G,L,H,x_0,Q,R,...
                                         x_hat_0,P_0,N,n,U)                          
    arguments
        F
        G
        L
        H
        x_0
        Q
        R
        x_hat_0
        P_0
        N
        n = size(F,1)
        U = zeros(N+1,1);
    end
    
    x = x_0;
    y = H * x + R * randn(size(R,1),1);
    p_pre = P_0;
    p_post = p_pre;

    u = U(1);
    x_hat_pre = x_hat_0;
    k = p_pre * H' * inv(R);
    x_hat_post = x_hat_pre;

    X = [];
    Y = [];

    K = [];
    X_hat_pre= [];
    X_hat_post = [];
    P_pre = zeros(n,n,N-1);
    P_post = zeros(n,n,N-1);

    for i = 1:(N-1)
        u = U(i);
        x = F * x + G * u + L * Q * randn(size(Q,1),1);
        y = H * x + R * randn(size(R,1),1);

        p_pre = F * p_post * F' + L' * Q * L;
        k = p_pre * H' * inv(H * p_pre * H' + R);
        x_hat_pre = F * x_hat_post + G * u;
        x_hat_post = x_hat_pre + k * (y - H * x_hat_pre);
        p_post = (eye(n) - k * H) *p_pre * (eye(n) - k * H)' + k * R * k';

        X = [X x];
        Y = [Y y];

        K = [K k];
        X_hat_pre = [X_hat_pre x_hat_pre];
        X_hat_post = [X_hat_post x_hat_post];
        P_pre(:,:,i) = p_pre;
        P_post(:,:,i) = p_post;
    end
    
    
    X = X;
    Y = Y;
    
    X_hat_pre = X_hat_pre;
    X_hat_post = X_hat_post;
    P = P_post;
    
end

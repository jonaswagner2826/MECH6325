function [X,Xt,Y,W,X_hat_pre,X_hat_post,P,K,hat_t] = KalmanFilter_DT_SDS(sys,F,G,L,H,...
                                        x_0,Q,R,...
                                         x_hat_0,P_0,T,T_max,n,dtdiff,U)                          
    arguments
        sys % CT LTI sys... should have direct output of states
        F
        G
        L %Assumed L = G... if not W or L should be modified...
        H
        x_0
        Q
        R
        x_hat_0
        P_0
        T
        T_max
        n = size(F,1);
        dtdiff = 10;
        U = zeros(dtdiff*floor(T_max/T)+1,1);
    end
    N = floor(T_max/T);
    
    x = x_0;
    y = H * x + R * randn;
    p_pre = P_0;
    p_post = p_pre;

    x_hat_pre = x_hat_0;
    k = p_pre * H' * inv(R);
    x_hat_post = x_hat_pre;
    
    Xt = (T/dtdiff)*(0:(N*dtdiff));
    W = randn(N*dtdiff+1,1);
    X = lsim(sys,U + Q*W,Xt,x_0)';
    Y = [];
    hat_t = [];

    K = [];
    X_hat_pre= [];
    X_hat_post = [];
    P_pre = zeros(n,n,N-1);
    P_post = zeros(n,n,N-1);

    for i = 1:(N-1)
        u = U(i);
        t = i*T;
        x = X(:,dtdiff*i+1);%F * x + G * u + L * Q * randn;
        y = H * x + R * randn;

        p_pre = F * p_post * F' + L' * Q * L;
        k = p_pre * H' * inv(H * p_pre * H' + R);
        x_hat_pre = F * x_hat_post + G * u;
        x_hat_post = x_hat_pre + k * (y - H * x_hat_pre);
        p_post = (eye(n) - k * H) *p_pre * (eye(n) - k * H)' + k * R * k';

        Y = [Y y];
        hat_t = [hat_t t];

        K = [K k];
        X_hat_pre = [X_hat_pre x_hat_pre];
        X_hat_post = [X_hat_post x_hat_post];
        P_pre(:,:,i) = p_pre;
        P_post(:,:,i) = p_post;
    end

    P = P_post;
    
end

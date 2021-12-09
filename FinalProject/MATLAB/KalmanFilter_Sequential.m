function [X,Y,X_hat_pre,X_hat_post,P_pre,P_post,K] = ...
    KalmanFilter_Sequential(F,G,L,H,x_0,Q,R,x_hat_0,P_0,N,n,r,U)                          
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
        N = 100;
        n = size(F,1);
        r = size(H,1)
        U = zeros(N+1,1);
    end
    
    x = x_0;
    y = H * x + R * randn;
    p_pre = P_0;
    p_post = p_pre;
    
    u = U(1);
    x_hat_pre = x_hat_0;
    x_hat_post = x_hat_pre;

    X = [];
    Y = [];
    
    K = zeros(n,r,N);
    X_hat_pre= [];
    X_hat_post = [];
    P_pre = zeros(n,n,N);
    P_post = zeros(n,n,N);

    for i = 1:N
        u = U(i);
        x = F * x + G * u + L * Q * randn(size(Q,1),1);
        y = H * x + R * randn(size(R,1),1);
        
        % Time Update
        p_pre = F * p_post * F' + L' * Q * L;
        x_hat_pre = F * x_hat_post + G * u;
        
        k = zeros(n,r);
        x_hat = zeros(n,r);
        p = zeros(n,n,r);
        
        %Measurement Update
        k(:,1) = p_pre * H(1,:)' * inv(H(1,:) * p_pre * H(1,:)' + R(1,1));
        x_hat(:,1) = x_hat_pre + k(:,1)*(y(1) - H(1,:) * x_hat_pre);
        p(:,:,1) = (eye(n) - k(:,1) * H(1,:))* p_pre;
        
        for j = 2:r
            k(:,j) = p(:,:,j-1) * H(j,:)' * inv(H(j,:) * p(:,:,j-1) * H(j,:)' + R(j,j));
            x_hat(:,j) = x_hat(:,j-1) + k(:,j-1) * (y(j) - H(j,:) * x_hat(:,j-1));
            p(:,:,j) = (eye(n) - k(:,j-1) * H(j,:))* p(:,:,j-1);
        end
        
        x_hat_post = x_hat(:,r);
        p_post = p(:,:,r);
        
        X = [X x];
        Y = [Y y];

        K(:,:,i) = k;
        X_hat_pre = [X_hat_pre x_hat_pre];
        X_hat_post = [X_hat_post x_hat_post];
        P_all(:,:,i,:) = p(:,:,:);
        P_pre(:,:,i) = p_pre;
        P_post(:,:,i) = p_post;
    end
    
    X = X;
    Y = Y;
    
    X_hat_pre = X_hat_pre;
    X_hat_post = X_hat_post;
    P_pre = P_pre;
    P_post = P_all;
end


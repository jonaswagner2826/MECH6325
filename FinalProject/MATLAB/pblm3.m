function [] = pblm3()
% Problem 3 Scipt/Function
% MECH 6325 - Final Probject

C = eye(2);
R = eye(2);
N = 1e3;
T = 1e-2;

% Part a
disp('Part a -----------------')
A = diag([1,2]);
Q = [1,1;
     1,1];
P_0 = eye(2);
P = careInt(A,C,Q,R,P_0,N,T);
P_ss_a = P(:,:,end)

P_ss_a_care = care(A',C',Q,R)

% Part b
disp('Part b -----------------')
A = diag([-1,-1]);
Q = [1,2;
     2,4];
P_0 = eye(2);
P = careInt(A,C,Q,R,P_0,N,T);
P_ss_b = P(:,:,end)

P_ss_b_care = care(A',C',Q,R)


% Part c
disp('Part c -----------------')
A = diag([1,1]);
Q = [1,2;
     2,4];
P_0 = eye(2);
P = careInt(A,C,Q,R,P_0,N,T);
P_ss_c = P(:,:,end)

P_ss_c_care = care(A',C',Q,R)

N = 1e5;

% Part d
disp('Part d -----------------')
A = diag([1,1]);
Q = [1,2;
     2,4];
P_0 = 0;
P = careInt(A,C,Q,R,P_0,N,T);
P_ss_d = P(:,:,end)

P_ss_d_care = care(A',C',Q,R)

K_int = P_ss_d * C' * inv(R)
A_KC_int = A-K_int*C
eig_int = eig(A_KC_int)
disp(['This is an unstable observer, so it does not result in a' ,...
      'steady-state Kalman Filter'])
end


function P = careInt(A,C,Q,R,P_0,N,T)
    arguments
        A
        C
        Q
        R
        P_0
        N = 100;
        T = 0.01;
    end
    p_dot = - P_0 * C' * inv(R) * C * P_0 + A * P_0 + P_0 * A' + Q;
    p = P_0 + p_dot * T;
    P(:,:,1) = p;
    for i = 2:N
        p_dot = - p * C' * inv(R) * C * p + A * p + p * A' + Q;
        p = p + p_dot * T;
        P(:,:,i) = p;
    end
end

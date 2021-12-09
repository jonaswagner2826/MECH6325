function [F,G,H,Q,R] = DiscretizeSystem(A,B,C,Q_c,R_c,T)
    n = size(A,1);
    if B == 0
        B = zeros(n,1);
    end
    if C == 0
        C = zeros(1,n);
    end    
    sys = ss(A,B,C,0);
    dt_sys = c2d(sys,T);
    [F,G,H,~,T] = ssdata(dt_sys);
% Original Code... issues with stability using this method...
%     n = size(A,1);
%     F = exp(A * T);
%     syms tau
%     G = double(F * int(eye(n) - exp(-A * tau),tau,0,T) * B);
%     H = C;
    Q = Q_c * T;
    R = R_c * T;
end
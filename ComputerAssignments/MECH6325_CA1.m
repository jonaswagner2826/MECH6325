clear
close all
clc

%---------------------------------------------------------------------
%Problem 2

% vector fresp contains (complex-valued) frequency responses
% vector w, contains frequencies at which measurements were taken

load G11QA1.mat
fresp=Yo2i(1:end);

%frequency vector (in Hz)
freq=Yo2ix(1:end);

%frequency vector (in rad/sec)
w = 2*pi*freq;

%non-parametric model
G_h = frd(fresp,w);

N = length(w);

y = fresp(1:N);
u = w(1:N);

% (i) Order Guess
n = 2;

[~, ~, num, den] = lsf(n,N,u,y);
G = tf(num,den);

figure()
bode(G_h,w)
hold on
bode(G,w)
title("Problem 2 - N =" + string(n))
hold off

saveas(gcf,"fig/MECH6325_CA1_pblm2_n=" + string(n) + ".png")

% (ii) Higher Orders

for i = (n+1):(n+5)
    n = i;
    [~, ~, num, den] = lsf(n,N,u,y);
    G = tf(num,den);
    
    figure()
    bode(G_h,w)
    hold on
    bode(G,w)
    title("Problem 2 - N =" + string(n))
    hold off
    saveas(gcf,"fig/MECH6325_CA1_pblm2_n=" + string(n) + ".png")
end

% close all


%---------------------------------------------------------------------
%Problem 3

% vector fresp contains (complex-valued) frequency responses
% vector w, contains frequencies at which measurements were taken

load G11QA2.mat
fresp=o2i1(20:1200);%frequency response vector

%frequency vector (in Hz)
freq=o2i1x(20:1200);

%frequency vector (in rad/sec)
w = 2*pi*freq;

%non-parametric model
G_h = frd(fresp,w);

N = length(w);

y = fresp(1:N);
u = w(1:N);

% (i) Order Guess
n = 7;

[~, ~, num, den] = lsf(n,N,u,y);
G = tf(num,den);

figure()
bode(G_h,w)
hold on
bode(G,w)
title("Problem 3 - N =" + string(n))
hold off

saveas(gcf,"fig/MECH6325_CA1_pblm3_n=" + string(n) + ".png")

% (ii) Higher Orders

for i = (n+1):(n+5)
    n = i;
    [~, ~, num, den] = lsf(n,N,u,y);
    G = tf(num,den);
    
    figure()
    bode(G_h,w)
    hold on
    bode(G,w)
    title("Problem 3 - N =" + string(n))
    hold off
    saveas(gcf,"fig/MECH6325_CA1_pblm3_n=" + string(n) + ".png")
end

% close all


% (iv) Subset of data tests

Nmax = N;

for i = 0:4
    for j =0:4
        if i > 0
            N1 = int32(Nmax/i);
        else
            N1 = 1;
        end
        if j > 0
            N2 = Nmax - int32(Nmax/j);
        else
            N2 = Nmax;
        end
        
        if N2 > N1
            N = N2-N1 +1;

            y = fresp(N1:N2);
            u = w(N1:N2);

            n = 7;
            [~, ~, num, den] = lsf(n,N,u,y);
            G = tf(num,den);

            figure()
            bode(G_h,w)
            hold on
            bode(G,w)
            title("Problem 3 (iv) - N =" + string(n) + " - N1(" + string(N1) + "/" + string(Nmax) +") N2(" + string(N2) + "/" + string(Nmax)+")")
            hold off
            saveas(gcf,"fig/MECH6325_CA1_pblm3_iv_n=" + string(n) + "N1(" + string(i) + ")N2(" + string(j) + ")" + ".png")
        end
    end
end

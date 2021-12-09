clear
close all
clc

load G11QA2.mat
fresp=o2i1(20:1200);%frequency response vector

%frequency vector (in Hz)
freq=o2i1x(20:1200);

%frequency vector (in rad/sec)
w = 2*pi*freq;

%non-parametric model
G_h = frd(fresp,w);
bode(G_h,w)
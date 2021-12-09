clear
close all
clc

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
bode(G_h,w)
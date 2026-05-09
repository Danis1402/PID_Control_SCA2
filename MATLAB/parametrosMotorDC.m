clear all
close all
clc
%% Parámetros del motor
Km=0.030766;
Kt=Km;
R=35.75;
L= 0;
J=1.4956e-6;
b=6.4376e-6;
%% Función de transferencia del motor DC
num = [Kt];
den = [L*J, L*b + R*J, R*b + Kt*Km];

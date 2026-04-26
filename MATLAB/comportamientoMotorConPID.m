clear all
close all
clc
%% Cargamos los parámetros
run('parametrosMotorDC.m');

%% Linealizamos el modelo completo (PI + Motor)
[A,B,C,D] = linmod('motorDC');   

%% Obtenemos el sistema en espacio de estados
sys_cl = ss(A,B,C,D);

%% Función de transferencia en lazo cerrado
G_cl = tf(sys_cl)

%% Mostramos la ganancia estática
GE = dcgain(G_cl)

%% Respuesta al escalón
figure(1);
step(6*G_cl);
title('Respuesta al escalón del sistema completo');

%% Parametros del PID 
Kp = 0.06589;                               % Parametros del PID obtenidos a través de PID Tuning 
Ki = 20.07;

%% Función de transferencia del PID
C = pid(Kp, Ki, 0);
G_PI = tf(C)

%% Discretización para el microcontrolador
Ts = 0.001;                                 % Periodo de muestreo    
sysd = c2d(sys_cl, Ts, 'tustin');
G_d = tf(sysd)
pole(G_d)
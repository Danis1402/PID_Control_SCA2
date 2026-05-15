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

%% Hallamos la función de transferencia del motor
G_motor = tf(num, den)

%% Respuesta al escalón
%figure(1);
%step(6*G_motor);
%title('Respuesta al escalón del motor');

%% Respuesta al escalón
%figure(2);
%step(6*G_cl);
%title('Respuesta al escalón del sistema completo');


%% Parametros del PID 
Kp = 0.062787;                              % Parametros del PID obtenidos a través de PID Tuning 
Ki = 1.8219;

%% Función de transferencia del PID
C = pid(Kp, Ki, 0);
G_PI = tf(C)

%% Discretización para el ESP32
Ts = 0.02;                                      % Periodo de muestreo    
Pi_d = c2d(C, Ts, 'zoh');                       % Discretizamos el PID

%% Obtenemos los valores discretos para el ESP32
[num_z, den_z]= tfdata(Pi_d, 'v')
fprintf('Coeficientes para el ESP32:\n');
disp(Pi_d)

%% Método utilizado en SCA1
clear all
close all
clc
%% Funciond de transferencia
%Función linmod
[num,den]=linmod('motorDC')
%Hallamos la función de transferencia con linmod
sys=tf(num,den)
%Comparamos con la reducción de DB
sys2=tf([Kt/(J*R)],[1 ((b/J)+(Km*Kt/(J*R)))])   % Funcion de transferencia del motor en primer orden (ignorar)
%Hallamos la ganancia estática
GE=dcgain(sys)
%Hallamos la respuesta al escalón
step(12*sys)
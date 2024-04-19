close all;
clear all;
clc;

load('practica3_ident.mat');

%h: altura
%u: accion de control
%t: tiempo

figure(1)
plot (t, h)
title("Variacion de la altura");
grid on;
legend ({"Nivel de agua"}, "location", "northeast");
ylabel("h[m]");
xlabel("t[s]");


% La planta tiene la forma 
%
% P(s) = -K / (s + p)
%
% Corresponde a la solucion general
%
% y(t) = A * e^(-p*t) - K/p  
%
%
% y(t = 0) = A - K/P = 0.4586
% y(t -> inf) = -K/p = 0.3747
%
% Entonces:
%
% A = 0.0839


A = 0.0839;
%p = 0.4;
%K = 

p_vec = linspace(0.003,0.002,10);

figure(2)
plot (t, h)
hold on
for i = 1:length(p_vec)
    K = -0.3747*p_vec(i); 
    a = A * exp(-p_vec(i)*t) - K/p_vec(i);
    plot(t,a);
    legendInfo{i} = ['p = ' num2str(p_vec(i))];
   
end
hold off
legend(legendInfo);
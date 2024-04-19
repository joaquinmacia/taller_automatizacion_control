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

%% Estimacion a ojo

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

p_vec = linspace(0.002, 0.003, 10);

figure(2)
plot (t, h)
hold on
for i = 1:length(p_vec)
    K = -0.3747*p_vec(i); 
    a = A * exp(-p_vec(i)*t) - K/p_vec(i);
    plot(t,a)
    title("Estimacion a ojo para distintos p");
    grid on;
    legend ({"Nivel de agua"}, "location", "northeast");
    ylabel("h[m]");
    xlabel("t[s]");
    legendInfo{i} = ['p = ' num2str(p_vec(i))];
    hold on
end
hold off
legend(legendInfo);

%%
% A ojo vemos que el mejor valor es
% p = 0.00233
% K = 0.0087

p = 0.00233;
K = -0.3747*p;
a1 = A * exp(-p*t) - K/p;


figure()
plot (t, a1)
title("Estimacion a ojo, p = 0.00233, K = -0.00087");
grid on;
legend ({"Nivel de agua"}, "location", "northeast");
ylabel("h[m]");
xlabel("t[s]");



%% Estimacion por regresion

% De la misma transferencia 
% 
% P(s) = -K / (s + p)
%
% Podemos ir a la transferencia discreta (con c2d())
%
% Pd(z) = Kd / (z + pd)
%
% En ecuacion en diferencias
%
% h(n + 1) =  pd * h(n) + Kd * u(n)

tam = length(h);

h_n1 = h(2:end) - h(1);
h_n = h - h(1);
u_n = u - u(1);

x = [h_n(1:tam-1) u_n(1:tam-1)];

params = pinv(x) * h_n1;

Kd = params(2);
pd = params(1);

% En la discretizacion de ZOH
%
%
% T = 1, porque muestreo a 1 segundo
%
% pd = exp(-p * T)
% p = -Ln(pd)/T
%
%
% Kd = int(0,T) exp(p * T) * K * dT  
% Kd = - (K / p) * (exp(p*T) - 1)
% K = (Kd * p) / (exp(p*T) - 1)
 
T = 1;
p1 = -log(pd)/T;
K1 = - (Kd * p1) / (exp(p1*T) -1);

%figure()
%a2 = A * exp(-p1*t) - K1/p1;
%plot(t,a2)


s = tf('s');
P = -K1 / (s + p1);


%% Junto las 2 estimaciones

%figure()
hold on
plot (t, h)
plot(t,a1)
step(0.05*P + h(1))
title("Junto las 3 estimaciones")
legend ({"Estimacion por regresion", "Datos", "Estimacion a ojo"}, "location", "northeast");






















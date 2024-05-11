clc;
close all;
clear all;

load('datos_escalon_30.mat');
data = load('datos_escalon_30_congomita.mat');
n = length(data.out.d1);
x = 0:length(data.out.d1)-1;

figure();
plot(x,data.out.d1);
hold on;
plot(x,data.out.d2);
hold on;
plot(x,data.out.d3);
hold on;


syms t s m2 L1 l2 J2 theta(t) phi(t) b2 g;

% Definir la expresión
expr = m2 * L1 * l2 * cos(theta) * diff(phi, t, 2) + diff(theta, t, 2) * J2 - 1/2 * diff(phi, t)^2 * sin(2*theta) * J2 + b2 * diff(theta, t) + g * m2 * l2 * sin(theta);

% Calcular la transformada de Laplace
laplace_expr = laplace(expr, t, s);
 


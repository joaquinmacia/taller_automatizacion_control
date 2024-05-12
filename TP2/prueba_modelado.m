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

%%

s = tf('s');

r = 0.17;   %Largo del brazo [r] = m
%r = 0.19;

l = 0.19;   %Largo del pendulo [l] = m
%l = 0.22;

m = 0.1;   %Masa del pendulo [m] = Kg
%m = 0.03;

%gamma = 0.15;%Coef rozamiento
gamma = 0.25;

g = 9.8;    %Gravedad [g] = m/s^2
w0 = g/l;
Q = w0 * (m*l/gamma);

%Defino la planta modelada

P = ((r/l)*s^2) / (s^2 + (w0/Q)*s + w0^2); 


% Me quedo con la respuesta al escalon de 30grados, que arranca en la
% muestra 500

step_real = data.out.d3(499:end);

f = 100;     %Datos muestreados a 100Hz
duracion = length(step_real)/f; %Duracion del muestreo en segundos

t = (0:1/f:duracion-(1/f));

phi_deg = 30;
phi_rad = deg2rad(phi_deg);

%Calculo rta al escalon de 30 grados
[y_step, t1] = step(phi_rad * P, t);

%Convierto esta respuesta a grados para poder graficarla con la de la
%planta real

y_step_deg = rad2deg(y_step);

figure()
plot(t1,y_step_deg)
hold on
plot(t, -step_real)



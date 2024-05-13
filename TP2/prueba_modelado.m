clc;
close all;
clear all;

%load('datos_escalon_30.mat');
%data = load('datos_escalon_30_congomita.mat');


dato1 = load ('Dato1.mat')

u1 = dato1.out.d1;
phi_medido = dato1.out.d2



%n = length(data.out.d1);
%x = 0:length(data.out.d1)-1;

%figure();
%plot(x,data.out.d1);
%hold on;
%plot(x,data.out.d2);
%hold on;
%plot(x,data.out.d3);
%hold on;

%% Estimacion planta Servo

a = 1000;
b = 175;
c = a;


P_servo = a / (s^2 + b*s + c);

[y_step_servo, t2] = step(phi_deg * P_servo, t);

step_servo_real = data.out.d2(499:end);

duracion = length(step_servo_real)/f; %Duracion del muestreo en segundos

t = (0:1/f:duracion-(1/f));

figure()
plot(t2, y_step_servo)
hold on
plot(t, step_servo_real)



%% Estimacion Planta Pendulo

s = tf('s');

r = 0.1751;   %Largo del brazo [r] = m
%r = 0.19;

l = 0.2053;   %Largo del pendulo [l] = m
%l = 0.22;

m = 0.05;   %Masa del pendulo [m] = Kg
%m = 0.03;

%gamma = 0.15;%Coef rozamiento
gamma = 0.15;

g = 9.8;    %Gravedad [g] = m/s^2
w0 = g/l;
Q = w0 * (m*l/gamma);

%Defino la planta modelada

P_pendulo = ((r/l)*s^2) / (s^2 + (w0/Q)*s + w0^2); 

P_total = minreal(P_servo * P_pendulo);
% Me quedo con la respuesta al escalon de 30grados, que arranca en la
% muestra 500

step_real = data.out.d3(499:end);

f = 100;     %Datos muestreados a 100Hz
duracion = length(step_real)/f; %Duracion del muestreo en segundos

t = (0:1/f:duracion-(1/f));

phi_deg = 30;

%Calculo rta al escalon de 30 grados
[y_step_deg, t1] = step(phi_deg * P_total, t);


figure()
plot(t1,y_step_deg)
hold on
plot(t, -step_real)



clc;
close all;
clear all;

%Cargamos los datos del archivo de mediciones del Simulink
dato1 = load ('Dato1.mat');

%Extraigo los datos
u1 = dato1.out.d1;
phi_medido = dato1.out.d2;
tita_medido = dato1.out.d3;
phi_ref = dato1.out.phi_ref;
t = dato1.out.tout;

%Graficos para verificar que los datos se cargaron bien
figure()
plot(t, phi_ref)
hold on
plot(t, phi_medido-2)

figure()
plot(t, tita_medido)



%% Estimacion planta Servo

s = tf('s');

%syms a b c

a = 800;
b = 60;
c = a;

phi_deg = 30;

%Planta del servo de 2do orden
P_servo = a / (s^2 + b*s + c);


%Recorto la señal del phi_medido para quedarme con una sola respuesta
step_servo_real = phi_medido(757:1005)-2;

%[y_step_servo, t2] = step(phi_deg * P_servo, t(1:length(step_servo_real)));


%figure()
%plot(t2,60 + y_step_servo)
%hold on
%plot(t(1:length(step_servo_real)), step_servo_real)



%% Estimacion Planta Pendulo

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


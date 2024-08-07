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




%% Estimacion planta Servo

s = tf('s');

%syms a b c

a = 292.2;
b = 30.67;
c = a;

phi_deg = 30;

%Planta del servo de 2do orden
P_servo = a / (s^2 + b*s + c);


%Recorto la señal del phi_medido para quedarme con una sola respuesta
step_servo_real = phi_medido(757:1005)-2;

[y_step_servo, t2] = step(phi_deg * P_servo, t(1:length(step_servo_real)));


figure();
plot(t2,60 + y_step_servo);
hold on
plot(t(1:length(step_servo_real)), step_servo_real);
hold off
legend('Servo modelado', 'Servo medido');


%Vectores para estimacion

phi_medido_estim = double(phi_medido(757:1005)-2);
phi_ref_estim = phi_ref(749:997);
tita_medido_estim = double(tita_medido(749:997));

%% Estimacion Planta Pendulo
close all;

%Cargamos los datos del archivo de mediciones del Simulink
dato1 = load ('escalon30.mat');

inicio = 755;%1875;
final = inicio + 300;

%Extraigo los datos
u2 = dato1.out.d1(inicio:final);
phi_medido2 = double(dato1.out.d2(inicio:final) - 2);
phi_medido2(37:end) = 60;
tita_medido2 = double(dato1.out.d3(inicio:final));
phi_ref2 = dato1.out.phi_ref(inicio:final);
t2 = dato1.out.tout(inicio:final) - 14.9800;

figure(8);
plot(t2,phi_medido2);
hold on;
plot(t2,phi_ref2);
%hold on;
%plot(step(-30 * P_servo) + 90);

% Gpendulo = tf([0.95 0 0],[1 b 12.5]);
% figure();
% plot(t2,tita_medido2);
% hold on;
% step(-30 * P_servo * Gpendulo, t2);

% 
% r = 0.1751;   %Largo del brazo [r] = m
% %r = 0.19;
% 
% l = 0.2053;   %Largo del pendulo [l] = m
% %l = 0.22;
% 
% m = 0.05;   %Masa del pendulo [m] = Kg
% %m = 0.03;
% 
% %gamma = 0.15;%Coef rozamiento
% gamma = 0.15;
% 
% g = 9.8;    %Gravedad [g] = m/s^2
% w0 = g/l;
% Q = w0 * (m*l/gamma);
% 
% %Defino la planta modelada
% 
% P_pendulo = ((r/l)*s^2) / (s^2 + (w0/Q)*s + w0^2); 
% 
% P_total = minreal(P_servo * P_pendulo);
% % Me quedo con la respuesta al escalon de 30grados, que arranca en la
% % muestra 500
% 
% %step_real = data.out.d3(499:end);
% 
% f = 100;     %Datos muestreados a 100Hz
% %duracion = length(step_real)/f; %Duracion del muestreo en segundos
% 
% phi_deg = 30;
% 
% %Calculo rta al escalon de 30 grados
% [y_step_deg, t1] = step(phi_deg * P_total, t);
% 
% 
% figure()
% plot(t1,y_step_deg)
% hold on
% plot(t, -step_real)
%%%

% Definir el rango de valores para b
b_values = linspace(0.8, 0.9, 5);  % Ajusta los límites y el número de valores según sea necesario

% Crear una nueva figura
figure();
plot(t2, tita_medido2, 'k', 'LineWidth', 2); % Plotear los datos medidos
hold on;

% Iterar sobre los valores de b y plotear la respuesta del sistema
for b = b_values
    Gpendulo = tf([0.8 0 0], [1 0.9 12.5]);
    step_response = step(-30 * P_servo * Gpendulo, t2 - 0.1);
    plot(t2, step_response, 'DisplayName', ['b = ', num2str(b)]);
end
Gpendulo = tf([0.8 0 0], [1 0.9 12.5]);

%Gpendulo2 = tf1;
% Configurar el gráfico
title('Respuestas del sistema para diferentes valores de b');
xlabel('Tiempo (s)');
ylabel('Respuesta');
legend('show');
grid on;
hold off;

figure(11);
plot(step(-30 * P_servo * Gpendulo,t2 - 0.1));
hold on;
plot(tita_medido2, 'k', 'LineWidth', 2); % Plotear los datos medidos
hold off;

%% Respuesta escalon 30 con tuerca

%Cargamos los datos del archivo de mediciones del Simulink
dato1 = load ('escalon30_con_tuerca.mat');

%Extraigo los datos
u1 = double(dato1.out.d1(1401:1700));
phi_medido = double(dato1.out.d2(1401:1700));
tita_medido_ok = double(dato1.out.d3(1401:1700));
phi_ref = double(dato1.out.phi_ref(1401:1700));
t = double(dato1.out.tout(1:300));
datos_medidos = [t tita_medido_ok];

%Gpendulo = tf([0.7 0 0], [1 2.613 65.51]);
Gpendulo = tf([0.7 0 0], [1 2.613 65]);

%Graficos para verificar que los datos se cargaron bien
figure()
plot(phi_ref);

figure()
plot(t,step(30 * P_servo * Gpendulo,t), 'LineWidth', 2);
hold on;
plot(t(1:end - 7),tita_medido_ok(8:end), 'LineWidth', 2);
legend({'Respuesta planta modelada', 'Respuesta medida'}, 'Location', 'best');
%plot(t,tita_medido_ok, 'LineWidth', 2)


%% Respuesta impulso P
dato1 = load ('Respuesta_impulso_P.mat');
%Extraigo los datos
u1 = double(dato1.out.d1(1140:1350));
phi_medido = double(dato1.out.d2);
tita_medido = double(dato1.out.d3(1140:1350));
phi_ref = double(dato1.out.phi_ref);
t = double(dato1.out.tout(1140:1350));
accion_de_control = [(t-11.39) u1];
data2simulink = [(t-11.39) tita_medido];
%Gpendulo = tf([0.7 0 0], [1 2.613 65.51]);
Gpendulo = tf([0.7 0 0], [1 2.613 65]);

figure()
plot(t - 9.39,tita_medido);

%% Diseño de controlador PI
P = P_servo * Gpendulo;

kp = 0.6;
ki = 5;

C_pi = zpk([-ki/kp],[0],1/kp);

L = P * C_pi;
Ts = 0.01;
Cd_pi = c2d(C_pi,Ts,'tustin'); 
bode(L);

%% Diseño controlador P
kp = 0.6;
C_p = zpk([],[],kp);
Ts = 0.01;
Cd_p = c2d(C_p,Ts,'tustin');
%bode(P * C_p);
%% Diseño de controlador PD
close all;
P = P_servo * Gpendulo;

kp = 0.6;
kd = 0.0001;%0.0001

%C_pd = zpk([-kp/kd],[],kd);

C_pd = zpk([-3],[],0.0631);

L = P * C_pd;
Ts = 0.01;
Cd_pd = c2d(C_pd,Ts,'tustin'); 

figure()
bode(P)

figure()
bode(L)


%figure()
%pzmap(C_pd)

%% Respuesta impulso PD
dato1 = load ('impulso_controladorPD.mat');
%Extraigo los datos
u1 = double(dato1.out.d1(290:550));
phi_medido = double(dato1.out.d2(290:550));
tita_medido = double(dato1.out.d3(290:550));
phi_ref = double(dato1.out.phi_ref(290:550));
t = double(dato1.out.tout(290:550));
accion_de_control = [t-2.9 u1];
data2simulink = [t-2.9 tita_medido];



figure()
plot(t-2.9,u1);




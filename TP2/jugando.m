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


figure()
plot(t2,60 + y_step_servo)
hold on
plot(t(1:length(step_servo_real)), step_servo_real)


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


%% Prueba codigo matlab 

% Datos de ejemplo (estos deberían ser tus datos reales)
t = t2; % Vector de tiempo
response = tita_medido2; % Aquí deberías usar tus datos de respuesta temporal

% Función objetivo para el ajuste de curvas
objective = @(params, t) system_response(params, t);

% Valores iniciales de los parámetros [a, b, c]
initial_params = [1, 1, 1];

% Realizar el ajuste de curvas
options = optimoptions('lsqcurvefit', 'Display', 'off');
estimated_params = lsqcurvefit(objective, initial_params, t, response, [], [], options);

% Mostrar los parámetros estimados
disp('Parámetros estimados:');
disp(['a = ', num2str(estimated_params(1))]);
disp(['b = ', num2str(estimated_params(2))]);
disp(['c = ', num2str(estimated_params(3))]);

% Graficar la respuesta temporal original y la ajustada
fitted_response = system_response(estimated_params, t);

figure;
plot(t, response, 'b', 'LineWidth', 1.5); hold on;
plot(t, fitted_response, 'r--', 'LineWidth', 1.5);
legend('Respuesta original', 'Respuesta ajustada');
xlabel('Tiempo');
ylabel('Respuesta');
title('Ajuste de la función de transferencia');
grid on;


%% Diseño de controlador

P = Gpendulo * P_servo;

bode(P);
k = 0.1;
C = k;
L = P * C;
bode(L);



% Definir la función de transferencia en el dominio del tiempo
function y = system_response(params, t)
    a = params(1);
    b = params(2);
    c = params(3);
    
    % Definir la función de transferencia
    num = [a 0 0];
    den = [1 b c];
    sys = tf(num, den);
    
    % Obtener la respuesta temporal
    [y, ~] = step(sys, t);
end


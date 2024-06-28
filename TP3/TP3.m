clc;
clear all;
close all;

%Defino las matrices de espacio de estados del TP anterior

syms tita tita_p phi phi_p phi_ref;
T = 0.01;

A = [0 1 0 0; -65 -2.613 -204.54 -21.469; 0 0 0 1; 0 0 -292.2 -30.67];

B = [0; 204.54; 0; 292.2];

C = [1 0 0 0; 0 0 1 0];

u = phi_ref;
x = [tita; tita_p; phi; phi_p];
y = C * x;
%x_p = A*x + B*u;


%% Observador de Luenberger completo
% x_k = [tita tita_p phi phi_p] - Observo las 4 variables de estado
% y_k = [tita phi]              - Mido solo los angulos

% Discretizacion ZOH
Ad = eye(4) + A .* T;
Bd = B .* T;
Cd = C;

polos = 2 * [0.2467, 0.2467, 0.2117, 0.2117];

L = place(Ad', Cd', polos);

test = Ad - (L' * Cd);
 
eig(test)

%% Realimentacion de estados

polos2 = [0.90, 0.91, 0.92, 0.93];

K = place(Ad, -Bd, polos2);

test2 = eig(Ad + Bd*K)



%% Matriz de Feedforward

F = pinv(Cd * pinv((eye(4)-(Ad + Bd*K))) * Bd);
F = pinv(Cd * inv(eye(size(Ad)) - (Ad + Bd * K)) * Bd);
%F = pinv(Cd * inv(eye(size(Ad)) - (Ad - Bd * K)) * Bd);

%% Controlador con accion integral

Cdaux = [0,0,1,0];
% Crear la matriz identidad del tamaño apropiado
I = eye(size(Cdaux, 1));

% Crear la matriz grande combinando Ad, Cd y I
A_integral = [
    Ad, zeros(size(Ad, 1), size(I, 1)); % Ad en la esquina superior izquierda y ceros a su derecha
    -Cdaux, I                              % Cd en la parte inferior izquierda y I a su derecha
];

B_integral = [
    Bd;
    zeros(1, 1)
];

polos3 = [polos2 0.945];

H = place(A_integral, -B_integral, polos3);

test3 = eig(A_integral + B_integral * H);

disp(test3)


%% Carga de datos respuesta impulso controlador de estados
clc;
close all;
%Cargamos los datos del archivo de mediciones del Simulink
dato1 = load ('impulso_controlador_estados.mat');

inicio = 930;
fin = 1340;
cte = 9.31;
% Asignar las variables a nombres más cortos si es necesario
% Truncar los datos a partir del índice especificado, convertir a double y crear matrices de dos columnas
accion_control = [double(dato1.out.tout(inicio:fin)) - cte, double(dato1.out.accion_control(inicio:fin))];
phi_estimado = [double(dato1.out.tout(inicio:fin)) - cte, double(dato1.out.phi_estimado(inicio:fin))];
phi_medido = [double(dato1.out.tout(inicio:fin)) - cte, double(dato1.out.phi_medido(inicio:fin))];
phi_p_estimado = [double(dato1.out.tout(inicio:fin)) - cte, double(dato1.out.phi_p_estimado(inicio:fin))];
phi_ref = [double(dato1.out.tout(inicio:fin)) - cte, double(dato1.out.phi_ref(inicio:fin))];
thita_estimado = [double(dato1.out.tout(inicio:fin)) - cte, double(dato1.out.thita_estimado(inicio:fin))];
thita_medido = [double(dato1.out.tout(inicio:fin)) - cte, double(dato1.out.thita_medido(inicio:fin))];
thita_p_estimado = [double(dato1.out.tout(inicio:fin)) - cte, double(dato1.out.thita_p_estimado(inicio:fin))];
thita_p_medido = [double(dato1.out.tout(inicio:fin)) - cte, double(dato1.out.thita_p_medido(inicio:fin))];

plot(thita_estimado(:,2));

%% Cargo los datos de respuesta a escalones del feedfordward

clc;
close all;
%Cargamos los datos del archivo de mediciones del Simulink
dato1 = load ('respuesta_escalon_feedforward_modificado.mat');

inicio = 1;
fin = length(dato1.out.tout);
cte = 0;
% Asignar las variables a nombres más cortos si es necesario
% Truncar los datos a partir del índice especificado, convertir a double y crear matrices de dos columnas
accion_control = [double(dato1.out.tout(inicio:fin)) - cte, double(dato1.out.accion_control(inicio:fin))];
phi_estimado = [double(dato1.out.tout(inicio:fin)) - cte, double(dato1.out.phi_estimado(inicio:fin))];
phi_medido = [double(dato1.out.tout(inicio:fin)) - cte, double(dato1.out.phi_medido(inicio:fin))];
phi_p_estimado = [double(dato1.out.tout(inicio:fin)) - cte, double(dato1.out.phi_p_estimado(inicio:fin))];
phi_ref = [double(dato1.out.tout(inicio:fin)) - cte, double(dato1.out.phi_ref(inicio:fin))];
thita_estimado = [double(dato1.out.tout(inicio:fin)) - cte, double(dato1.out.thita_estimado(inicio:fin))];
thita_medido = [double(dato1.out.tout(inicio:fin)) - cte, double(dato1.out.thita_medido(inicio:fin))];
thita_p_estimado = [double(dato1.out.tout(inicio:fin)) - cte, double(dato1.out.thita_p_estimado(inicio:fin))];
thita_p_medido = [double(dato1.out.tout(inicio:fin)) - cte, double(dato1.out.thita_p_medido(inicio:fin))];

figure();
hold on;
plot(thita_p_medido(:,1),thita_p_medido(:,2));
plot(thita_p_estimado(:,1),thita_p_estimado(:,2));
hold off;

%% Cargo los datos de respuesta a escalones del Integrador

clc;
close all;
%Cargamos los datos del archivo de mediciones del Simulink('escalones_controlador_integral.mat');
dato1 = load('escalones_controlador_integral.mat'); 

inicio = 1;
fin = length(dato1.out.tout);
cte = 0;
% Asignar las variables a nombres más cortos si es necesario
% Truncar los datos a partir del índice especificado, convertir a double y crear matrices de dos columnas
accion_control = [double(dato1.out.tout(inicio:fin)) - cte, double(dato1.out.accion_control(inicio:fin))];
phi_estimado = [double(dato1.out.tout(inicio:fin)) - cte, double(dato1.out.phi_estimado(inicio:fin))];
phi_medido = [double(dato1.out.tout(inicio:fin)) - cte, double(dato1.out.phi_medido(inicio:fin))];
phi_p_estimado = [double(dato1.out.tout(inicio:fin)) - cte, double(dato1.out.phi_p_estimado(inicio:fin))];
phi_ref = [double(dato1.out.tout(inicio:fin)) - cte, double(dato1.out.phi_ref(inicio:fin))];
thita_estimado = [double(dato1.out.tout(inicio:fin)) - cte, double(dato1.out.thita_estimado(inicio:fin))];
thita_medido = [double(dato1.out.tout(inicio:fin)) - cte, double(dato1.out.thita_medido(inicio:fin))];
thita_p_estimado = [double(dato1.out.tout(inicio:fin)) - cte, double(dato1.out.thita_p_estimado(inicio:fin))];
thita_p_medido = [double(dato1.out.tout(inicio:fin)) - cte, double(dato1.out.thita_p_medido(inicio:fin))];

plot(phi_medido(:,1),phi_medido(:,2))
%% Cargo los datos de respuesta impulso del Integrador

clc;
close all;
%Cargamos los datos del archivo de mediciones del Simulink
dato1 = load ('Impulso_controlador_integrador.mat');

inicio = 1050;
fin = 1724;
cte = 10.49;
% Asignar las variables a nombres más cortos si es necesario
% Truncar los datos a partir del índice especificado, convertir a double y crear matrices de dos columnas
accion_control = [double(dato1.out.tout(inicio:fin)) - cte, double(dato1.out.accion_control(inicio:fin))];
phi_estimado = [double(dato1.out.tout(inicio:fin)) - cte, double(dato1.out.phi_estimado(inicio:fin))];
phi_medido = [double(dato1.out.tout(inicio:fin)) - cte, double(dato1.out.phi_medido(inicio:fin))];
phi_p_estimado = [double(dato1.out.tout(inicio:fin)) - cte, double(dato1.out.phi_p_estimado(inicio:fin))];
phi_ref = [double(dato1.out.tout(inicio:fin)) - cte, double(dato1.out.phi_ref(inicio:fin))];
thita_estimado = [double(dato1.out.tout(inicio:fin)) - cte, double(dato1.out.thita_estimado(inicio:fin))];
thita_medido = [double(dato1.out.tout(inicio:fin)) - cte, double(dato1.out.thita_medido(inicio:fin))];
thita_p_estimado = [double(dato1.out.tout(inicio:fin)) - cte, double(dato1.out.thita_p_estimado(inicio:fin))];
thita_p_medido = [double(dato1.out.tout(inicio:fin)) - cte, double(dato1.out.thita_p_medido(inicio:fin))];

plot(thita_medido(:,1),thita_medido(:,2))
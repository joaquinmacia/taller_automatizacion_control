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

polos = [0.2467, 0.2467, 0.2117, 0.2117];

L = place(Ad', Cd', polos);

test = Ad - (L' * Cd);
 
eig(test)

%% Realimentacion de estados

polos2 = [0.90, 0.91, 0.92, 0.93];

K = place(Ad, -Bd, polos2);

test2 = eig(Ad + Bd*K)



%% Matriz de Feedforward


F = pinv(C * pinv((eye(4)-(Ad + Bd*K))) * Bd);
F = pinv(C * pinv(eye(size(Ad)) - (Ad + Bd * K)) * Bd);





clc;
clear all;
close all;


% Datos

l = 0.16;
r2 = 2.625;
g = 9.8;
T = 0.02;
% Ec. dinamica

%tita** = -g/l * tita - k/(m*l^2) * tita*


%% Si Xk = [tita_k, w_k]'

A = [[0 1] ; [-g/l -r2]];

Ad = eye(2) + A .* T;

Cd = [1 0];

polos = [0.5, 0.6];

L = place(Ad', Cd', polos);

test = Ad - (L' .* Cd);

%% Si Xk = [tita_k, w_k, b_k]'

% Agregar una fila de ceros
Aux = [Ad; zeros(1, size(Ad, 2))];
% Agregar una columna de ceros
Aux2 = [Aux, zeros(size(Aux, 1), 1)];

Aux2(3,3) = 1; 

Ad2 = Aux2;

Cd2 = [[1 0 0];[0 1 1]];

polos2 = [0.5, 0.6, 0.98];

L2 = place(Ad2', Cd2', polos2);


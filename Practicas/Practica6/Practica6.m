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


% Si Xk = [tita_k, w_k]'

A = [[0 1] ; [-g/l -r2]];

Ad = eye(2) + A .* T;

Cd = [1 0];

polos = [0.5, 0.6];

L = place(Ad', Cd', polos);



% Si Xk = [tita_k, w_k, b_k]'








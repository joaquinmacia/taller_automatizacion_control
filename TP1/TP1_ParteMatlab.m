clc;
clear all;
close all;

syms L L_h L_l A_h A_l Qi L1 h u A_sal g u0 h0 s;

f = 3 * L1^2 * (Qi - A_sal * u * sqrt(2 * g * h))/(L_l^2 * (3 * h^2 + 6 * L1 * h + 3 * L1^2)) ;
y = h; %Salida del sistema

%Genero las matrices de estados
A = jacobian(f, h);
B = jacobian(f, u);
C = jacobian(y, h);
D = jacobian(y, u);

%Definicion de constantes
g = 9.8;        %En metros sobre segundo cuadrado
L = 0.9;        %En metros
L_h = 0.4;      %En metros
L_l = 0.1;      %En metros
Qi = 0.0001333; %Metros cubicos por segundo
d2 = 10.65e-3;
A_sal = pi * (d2 / 2)^2;
L1 = L_l /(L_h - L_l) * L;

%Puntos de equilibrio
h0 = 0.45;
u0 = Qi / (A_sal * sqrt(2 * g * h0));
h = h0;
u = u0;

A_eval = eval(A);
B_eval = eval(B);
C_eval = eval(C);
D_eval = eval(D);

P = zpk(ss(A_eval,B_eval,C_eval,D_eval));
C = 1;
L = P * C; 
S=1/(1+L);
T=1-S;


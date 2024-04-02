clc;
clear all;
close all;

syms x u;
g = 9.8;        %En metros sobre segundo cuadrado
L = 0.9;        %En metros
L_h = 0.4;      %En metros
L_l = 0.1;      %En metros
A_h = L_h^2;    %En metros cuadrados
A_l = L_l^2;    %En metros cuadrados
Qi = 0.0001333; %Metros cubicos por segundo
gamma = A_h + A_l + sqrt(A_h * A_l);


f = 3 * (Qi - u * sqrt(2 * g * x)) / gamma ;
y = x; %Salida del sistema


%Genero las matrices de estados

A = jacobian(f, x);
B = jacobian(f, u);
C = jacobian(y, x);
D = jacobian(y, u);

%Puntos de equilibrio
xeq = 0.45;
ueq = Qi / sqrt(2 * g * xeq);

x = xeq;
u = ueq;

%Evaluo las matrices en el punto de equilibrio

A = eval(A);
B = eval(B);
C = eval(C);
D = eval(D);

%Veo si es estable (no debe tener autovalores negativos)
%eig(A)

%De una forma:
%[num den] = ss2tf(A, B, C, D);
% = tf(num, den, 1);
%[zeros, poles, gain] = tf2zpk(num, den);


P = zpk(ss(A,B,C,D));





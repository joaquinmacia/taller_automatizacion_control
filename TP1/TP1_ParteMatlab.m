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

%Defino la matriz identidad
I = eye(size(A));

% Calcular la función de transferencia
H = C_eval * inv(s*I - A_eval) * B_eval + D_eval;
P = zpk(ss(A_eval,B_eval,C_eval,D_eval));

%       -0.004233
% P =   -----------
%       (s+0.00237)
 
%Si la planta tiene un tiempo de estabilizacion de alrededor de 8min el
%el tiempo de la planta es tau = 480s siendo la frecuencia natural f= 0.0021.
% w = 0.0132 rad/s

%Ts = 1;
%Pap = zpk([4/Ts], [-4/Ts], -1);

C = zpk([-0.00237], [0], -db2mag(10)); %Este anda

L = P * C; 
S = 1 / (1+L);
T = 1 - S;
PS = 1 / (1 + L);
figure(1);
bode(L);
figure(2);
step(PS);

%Cd = c2d(C, Ts, 'zoh');



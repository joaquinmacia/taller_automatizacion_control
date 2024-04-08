clc;
clear all;
close all;

syms L L_h L_l A_h A_l Qi L1 h u A_sal g u0 h0 s;

L1 = L_l /(L_h - L_l) * L;
f = 3 * L1 * (A_sal * u * sqrt(2 * g * h) - Qi)/(L_l^2 * (3 * h^2 + 6 * L1 * h + 3 * L1^2)) ;
y = h; %Salida del sistema

df_dh = diff(f,h);
df_du = diff(f,u);

%Genero las matrices de estados

A = jacobian(f, h);
B = jacobian(f, u);
C = jacobian(y, h);
D = jacobian(y, u);

%% Buscamos la funcion de transferencia

%%Puntos de equilibrio
h = h0;
u = u0;

%Evaluo las matrices en el punto de equilibrio

A_eval = eval(A);
B_eval = eval(B);
C_eval = eval(C);
D_eval = eval(D);

%Defino la matriz identidad
I = eye(size(A));

% Calcular la función de transferencia
H = C_eval * inv(s*I - A_eval) * B_eval + D_eval;


%% función de transferencia de la planta para distintos puntos de trabajo
h0_vec = [0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80];

%Definicion de constantes
g = 9.8;        %En metros sobre segundo cuadrado
L = 0.9;        %En metros
L_h = 0.4;      %En metros
L_l = 0.1;      %En metros
A_h = L_h^2;    %En metros cuadrados
A_l = L_l^2;    %En metros cuadrados
Qi = 0.0001333; %Metros cubicos por segundo
d2 = 10.65e-3;
A_sal = pi * (d2 / 2)^2;

figure(1);
hold on;

for i = 1:length(h0_vec)
h0 = h0_vec(i);
u0 = Qi / (A_sal * sqrt(2 * g * h0));

h = h0;
u = u0;

%Evaluo las matrices en el punto de equilibrio

A_eval = eval(A);
B_eval = eval(B);
C_eval = eval(C);
D_eval = eval(D);

P = zpk(ss(A_eval,B_eval,C_eval,D_eval));
bode(P);
legendInfo{i} = ['h0 = ' num2str(h0_vec(i))];

end
hold off;
legend(legendInfo);



clc;
clear all;
close all;

syms L L_h L_l A_h A_l Qi L1 h u A_sal g u0 h0 s;

L1 = L_l /(L_h - L_l) * L;
f = 3 * L1^2 * (Qi - A_sal * u * sqrt(2 * g * h))/(L_l^2 * (3 * h^2 + 6 * L1 * h + 3 * L1^2)) ;
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
u = Qi / (A_sal * sqrt(2*g*h0));

%Evaluo las matrices en el punto de equilibrio

A_eval = eval(A);
B_eval = eval(B);
C_eval = eval(C);
D_eval = eval(D);

%Defino la matriz identidad
I = eye(size(A));

% Calcular la función de transferencia
H = C_eval * inv(s*I - A_eval) * B_eval + D_eval;
H_simp = simplify(H);
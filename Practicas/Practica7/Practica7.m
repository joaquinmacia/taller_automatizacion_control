clc;
clear all;
close all;

%load('C2_pid.mat');

s = tf('s');

a = 292.2;
b = 30.67;
c = a;

Ts = 0.01;

P_servo = a / (s^2 + b*s + c);
P_pendulo = tf([0.7 0 0], [1 2.613 65]);
P = minreal(P_servo * P_pendulo);

kp1 = 0.6;
kd1 = 0.0001;
C1_pd = zpk([-kp1/kd1],[],kd1);

T = C1_pd * P_servo / (1 + P_pendulo * C1_pd * P_servo);

Cd1_pd = c2d(C1_pd,Ts,'tustin'); 
%Cd2_pid = c2d(C2_pid,Ts,'tustin'); 




close all
clear all

G = tf(482, [1 4.801 0]);

A = [0 1
    0 -4.801];
B = [0
    482];
C = [1 0];

Tes = 3;
Mp = 0.05;

% Factor de amortiguamiento relativo
far = -log(Mp)/sqrt(pi^2 + log(Mp)^2)

sigma = 4/Tes
wn = sigma/far
fi = acos(far)
% Tiempo de subida
Tr = exp(fi/tan(fi))/wn

% Tr = 1.12
T = 0.25 % Tiempo de muestreo (Tr/4 - Tr/10)

% Muestreo del sistema
FI = eye(2)*T + A*T^2/2 + A^2*T^3/6 + A^3*T^4/24
G = eye(2) + A*FI
H = FI*B

% Servosistema agregando integrador
GN = [G zeros(2, 1)
    -C*G 1]
HN = [H
    -C*H]

% Controlabilidad
CoN = [HN GN*HN GN^2*HN]
det_CoN = det(CoN)
% det_CoN > 2e4 > 0 -> Controlable

% Dinámica deseada
wd = -pi*sigma/log(Mp)
s1 = -sigma + wd*1i
s2 = -sigma - wd*1i
s3 = -5*sigma

% Polos discretos
z1 = exp(T*s1)
z2 = exp(T*s2)
z3 = exp(T*s3)

% Frecuencia de Nyquist
wN = pi/T

% Diseño
KN = acker(GN, HN, [z1 z2 z3])
K2 = KN(1:2)
K1 = -KN(3)


%%
% Diseño del observador


% Observabilidad
Ob = [C
      C*A]

detOb = det(Ob) % detOb = 1 != 0 -> Observable

% Polos del observador
so1 = 2*s1;
so2 = 2*s2;

zo1 = exp(T*so1)
zo2 = exp(T*so2)

% Obs en tiempo discreto
Ke = acker(G', C', [zo1 zo2])'

clear;
clc;
close all;
type = 1;
%% minimum phase case and non-minimum phase case
if type ==1
    sys = minphase;
else
    sys = nonminphase;
end
s=tf('s');
%caculate G(s)
G = sys.C*(s*eye(4)-sys.A)^(-1)*sys.B;
%static decoupling
W2 = eye(2);
W1 = inv(freqresp(G,0));
G_tilde = W2*G*W1;
%bode diagrams of G_tilde
figure(1)
bode(G_tilde)
grid on
%% diagoral controller
%intended values
if type == 1
    wc = 0.1;%minimum phase
else
    wc=0.02;%nonminimum phase
end
phim = pi/3;
%PI controllers
%determine Tij
Ginv0 = inv(freqresp(G,0));
G_tilde = G*Ginv0;
Gt0 = freqresp(G_tilde,0);
[~,phi11] = bode(G_tilde(1,1),wc);
[~,phi22] = bode(G_tilde(2,2),wc);
Ti1 = (1/wc) * tan(phim - pi/2 - (phi11*pi/180));
Ti2 = (1/wc) * tan(phim - pi/2 - (phi22*pi/180));
%determine Ki
L11 = G_tilde(1,1) * (1 + (1/(s*Ti1)));%/Ginv0(1,1);
L22 = G_tilde(2,2) * (1 + (1/(s*Ti2)));%/Ginv0(2,2);
[k1,~] = bode(L11,wc);
[k2,~] = bode(L22,wc);
K1 = 1/k1;
K2 = 1/k2;
f11_tilde = K1*(1 + (1/(s*Ti1)));
f22_tilde = K2*(1 + (1/(s*Ti2)));
f12_tilde= 0;
f21_tilde= 0;
% design F_tilde for G_tilde
F_tilde = [f11_tilde f12_tilde; f21_tilde f22_tilde];
F = inv(freqresp(G,0))*F_tilde;
L = G_tilde*F;
%% Singular values of S and T
% Sensitivity Function
S = minreal(inv(eye(2)+L));
% Complimentary Sensitivity Function 
%T = minreal(inv(eye(2)+L) * L);
T=eye(2)-S;
figure(2)
sigma(S);
hold on;
sigma(T);
grid on;
legend('singular value of S','singular value of T')
%% simulink
sim('closedloop');
figure(3)
plot(uout,'LineWidth',2)
hold on
title('Control Outputs u1 and u2')
legend('u1','u2')
grid on
hold off
figure(4)
plot(yout,'LineWidth',2)
hold on
title('Outputs y1 and y2')
legend('y1','y2')
grid on
hold off
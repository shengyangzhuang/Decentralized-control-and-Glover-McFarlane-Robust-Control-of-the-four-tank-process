clear;
clc;
close all;
type = 2;
%% minimum phase case and non-minimum phase case
if type ==1
    sys = minphase;
else
    sys = nonminphase;
end
s=tf('s');
%caculate G(s)
G = sys.C*(s*eye(4)-sys.A)^(-1)*sys.B;
%% Dynamic decoupling
W2 = eye(2);
%Gtwc = freqresp(G_tilde,0.1); %minimum phase
%Gt0 = freqresp(G_tilde,0);
%rgawc = Gwc.*inv(Gwc)';
%rgat0 = Gt0.*inv(Gt0)';
%investigate each element of the matrix
g11=G(1,1);g12=G(1,2);g21=G(2,1);g22=G(2,2);
%solve RGA of G
G0 = freqresp(G,0);
rga0 = G0.*inv(G0)';
%% minimum phase
%w11=1; w12=-g12/g11; w21=-g21/g22; w22=1;
%% nonminimum phase
w11 = -g22/g21; w12 = 1; w21 = 1; w22 = -g11/g12;
%% solve W1
W1 = [w11 w12; w21 w22];%minimum phase
W1 = minreal(W1*(0.2/(s+0.2))); %non minimum phase
G_tilde=minreal(W2*G*W1);
figure(5)
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
%% 
[~,phi11] = bode(G_tilde(1,1),wc);
[~,phi22] = bode(G_tilde(2,2),wc);
Ti1 = (1/wc) * tan(phim - pi/2 - (phi11*pi/180));
Ti2 = (1/wc) * tan(phim - pi/2 - (phi22*pi/180));
L11 = G_tilde(1,1) * (1 + (1/(s*Ti1)));
L22 = G_tilde(2,2) * (1 + (1/(s*Ti2)));
[k1,~] = bode(L11,wc);
[k2,~] = bode(L22,wc);
K1 = 1/k1;
K2 = 1/k2;
f11_tilde = K1*(1 + (1/(s*Ti1)));
f22_tilde = K2*(1 + (1/(s*Ti2)));
f12_tilde= 0;
f21_tilde= 0;
F_tilde = [f11_tilde f12_tilde; f21_tilde f22_tilde];
%%
F = minreal(W1*F_tilde);
L = minreal(G*F)    ;
%% Singular values of S and T
%Sensitivity Function
S = minreal(inv(eye(2)+L));
% S = feedback(eye(2),L);
% S = inv(eye(2)+L)
% S = feedback(G_tilde,F);
%Complimentary Sensitivity Function 
T = eye(2)-S;
figure(6)
sigma(S);
hold on;
sigma(T);
grid on;
legend('singular value of S','singular value of T')
%% simulink
sim('closedloop');
figure(7)
plot(uout,'LineWidth',2)
hold on
title('Control Outputs u1 and u2')
xlabel('Time(s)')
legend('u1','u2')
grid on
hold off
figure(8)
plot(yout,'LineWidth',2)
hold on
title('Outputs y1 and y2')
xlabel('Time(s)')
legend('y1','y2')
grid on
hold off
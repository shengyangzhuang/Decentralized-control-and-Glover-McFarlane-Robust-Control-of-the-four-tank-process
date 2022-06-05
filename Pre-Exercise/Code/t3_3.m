clear;
clc;
close all;
%% minimum phase case and non-minimum phase case
%sys = minphase;
sys = nonminphase;
s=tf('s');
%caculate G(s)
G = minreal(sys.C/(s*eye(4)-sys.A)*sys.B);
%% minimum phase
% g11=G(1,1);g12=G(1,2);g21=G(2,1);g22=G(2,2);
% w11=1; w12=-g12/g11; w21=-g21/g22; w22=1;
% W1 = [w11 w12; w21 w22];
%% nonminimum phase
g11=G(1,1);g12=G(1,2);g21=G(2,1);g22=G(2,2);
w11 = -g22/g21; w12 = 1; w21 = 1; w22 = -g11/g12;
W1 = [w11 w12; w21 w22]
W1 = minreal(W1*(0.2/(s+0.2)));
%%
%K1 = 1.6271;Ti1=5.7939;K2=1.9551;Ti2=6.2765; % minphase, dynamic
K1 = 1.1320;Ti1=22.3993;K2=0.9438;Ti2=23.4419; % nonminphase, static
%%
f11_tilde = K1*(1 + (1/(s*Ti1)));
f22_tilde = K2*(1 + (1/(s*Ti2)));
f12_tilde= 0;
f21_tilde= 0;
F_tilde = [minreal(f11_tilde) f12_tilde; f21_tilde minreal(f22_tilde)];
%%
L0 = minreal(G*W1*F_tilde);
%figure(1)
%margin(L0(1,1))
%margin(L0(2,2))
%%
alpha = 1.1;
[Fr,gam] = rloop(L0,alpha);
disp(['gam = ', num2str(gam)]);
%Fr_tf = minreal(Fr.C*inv(s*eye(18)-Fr.A)*Fr.B);
%%
%F = W1*F_tilde*Fr_tf;
F =minreal(W1*F_tilde*Fr);
% S = inv(eye(2)+G*F);
% T = eye(2)-S;
% figure(2)
% sigma(S);
% hold on;
% sigma(T);
% grid on;
%%
sim('closedloop')
figure(3)
plot(uout,'LineWidth',2)
hold on
title('Control Outputs u1 and u2')
xlabel('Time(s)')
legend('u1','u2')
grid on
hold off
figure(4)
plot(yout,'LineWidth',2)
hold on
title('Outputs y1 and y2')
xlabel('Time(s)')
legend('y1','y2')
grid on
hold off
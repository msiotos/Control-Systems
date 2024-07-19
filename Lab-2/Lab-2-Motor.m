%Systimata Eleghou 2h ergastiriakh askhsh
%Omada:25

%motor

clear all;
close all;

s = tf('s'); % Laplace variable

%1
figure (1);
Tg = 1.05; Tu = 0.14;
T1 = Tg/exp(1); 
T2 = Tu/(3-exp(1));
Ks = 0.8;
Hs = 0.8/(T1*T2*s^2 + (T1+T2)*s + 1);
step(Hs);
axis([0 7 0 0.9]);
title('Step Response of motor');

%2 & 3

t = 0:0.01:60;
d = +3.75:15:60;
Ps =1500 + 500*pulstran(t, d, @rectpuls, 7.5);

% P Controller 0%

Kp = (0.3*Tg)/(Tu*0.8);
Gp = pidstd(Kp);
Sp = feedback(Gp*Hs, 1);

figure(2);
lsim(Sp, Ps, t);
axis([0 60 1000 2200]);

title('P controller simulation');
LS = lsim(Sp, Ps, t);
buffer = Ps - (LS.');
errorsp = abs(buffer);
errorsp_sqrd = errorsp.^2;
time_weighted_errorsp = t .* errorsp;
ISEp = trapz(t, errorsp_sqrd);
ITAEp = trapz(t, time_weighted_errorsp);

% PI Controller 0%

Kpi = (0.35*Tg)/(Tu*0.8);
TIpi = 1.2*Tg;
Gpi = pidstd(Kpi,TIpi);
Spi = feedback(Gpi*Hs, 1);

figure(3);
lsim(Spi, Ps, t);
axis([0 60 1000 2200]);

title('PI controller simulation');
LS = lsim(Spi, Ps, t);
buffer = Ps - (LS.');
errorspi = abs(buffer);
errorspi_sqrd = errorspi.^2;
time_weighted_errorspi = t .* errorspi;
ISEpi = trapz(t, errorspi_sqrd);
ITAEpi = trapz(t, time_weighted_errorspi);

% PID Controller 0%

Kpid = (0.6*Tg)/(Tu*0.8);
TIpid = Tg;
Tdpid = 0.5*Tu;
Gpid = pidstd(Kpid,TIpid,Tdpid);
Spid = feedback(Gpid*Hs, 1);

figure(4);
lsim(Spid, Ps, t);
axis([0 60 1000 2200]);

title('PID controller simulation');
LS = lsim(Spid, Ps, t);
buffer = Ps - (LS.');
errorspid = abs(buffer);
errorspid_sqrd = errorspid.^2;
time_weighted_errorspid = t .* errorspid;
ISEpid = trapz(t, errorspid_sqrd);
ITAEpid = trapz(t, time_weighted_errorspid);

%4

Tsum = T1+T2;

% P Controller

Kpt = 1/Ks;
Gpt = pidstd(Kpt);
Spt = feedback(Gpt*Hs, 1);

figure(5);
lsim(Spt, Ps, t);
axis([0 60 1000 2200]);

title('P controller simulation');
LS = lsim(Spt, Ps, t);
buffer = Ps - (LS.');
errorspt = abs(buffer);
errorspt_sqrd = errorspt.^2;
time_weighted_errorspt = t .* errorspt;
ISEpt = trapz(t, errorspt_sqrd);
ITAEpt = trapz(t, time_weighted_errorspt);

% PI Controller

Kpit = 0.5/Ks;
TIpit = 0.5*Tsum;
Gpit = pidstd(Kpit,TIpit);
Spit = feedback(Gpit*Hs, 1);

figure(6);
lsim(Spit, Ps, t);
axis([0 60 1000 2200]);

title('PI controller simulation');
LS = lsim(Spit, Ps, t);
buffer = Ps - (LS.');
errorspit = abs(buffer);
errorspit_sqrd = errorspit.^2;
time_weighted_errorspit = t .* errorspit;
ISEpit = trapz(t, errorspit_sqrd);
ITAEpit = trapz(t, time_weighted_errorspit);

% PID Controller

Kpidt = 1/Ks;
TIpidt = 0.66*Tsum; 
Tdpidt = 0.17*Tsum;
Gpidt = pidstd(Kpidt,TIpidt,Tdpidt);
Spidt = feedback(Gpidt*Hs, 1);

figure(7);
lsim(Spidt, Ps, t);
axis([0 60 1000 2200]);

title('PID controller simulation');
LS = lsim(Spidt, Ps, t);
buffer = Ps - (LS.');
errorspidt = abs(buffer);
errorspidt_sqrd = errorspidt.^2;
time_weighted_errorspidt = t .* errorspidt;
ISEpidt = trapz(t, errorspidt_sqrd);
ITAEpidt = trapz(t, time_weighted_errorspidt);



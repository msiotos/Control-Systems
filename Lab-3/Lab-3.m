%Sistimata elegxou
%Omada25
%Ergastirio 3 (Temperature)

clear
close all

s = tf('s');

%1
Ks = 0.75; 
Tg = 50; 
Tu = 5;
T1 = 18.5; 
T2 = 16.65;
Hs = Ks/(T1*T2*s^2 + (T1+T2)*s + 1);
dt = 0.01;
t = 0:dt:200;
Y = 3+0*t;

figure(1);
plot(t, lsim(Hs, Y, t));
hold on;
plot(t, Y, 'r');
hold off;
axis([0 200 0 5]);

%2

t1 = 0:dt:300;
X = 6 - (t1>150);

% P Controller 20%

Kp = (0.7*Tg)/(Ks*Tu);
Cp = pidstd(Kp);
ControllerP = feedback(Cp*Hs, 1);

figure(2);
lsim(ControllerP, X, t1);
axis([0 300 0 7.5]);
title('P controller simulation');

L = lsim(ControllerP, X, t1);

errorsp =abs((L.')- X);
errorsp_sqrd = errorsp .^ 2;
time_weighted_errorsp = t1 .* errorsp;
ISEp = trapz(t1, errorsp_sqrd);
ITAEp = trapz(t1, time_weighted_errorsp);


% PI Controller 20%

Kpi = (0.6*Tg)/(Ks*Tu) ;
Tid = Tg;
Cpi = pidstd(Kpi,Tid);
ControllerPI = feedback(Cpi*Hs, 1);

figure(3);
lsim(ControllerPI, X, t1);
axis([0 300 0 8]);
title('PI controller simulation');

L = lsim(ControllerPI, X, t1);
errorspi = abs((L.')- X);
errorspi_sqrd = errorspi.^2;
time_weighted_errorspi = t1 .* errorspi;
ISEpi = trapz(t1, errorspi_sqrd);
ITAEpi = trapz(t1, time_weighted_errorspi);


%PID Controller 20%

Kpid = (0.95*Tg)/(Ks*Tu); 
Tipid = 1.4*Tg; 
Tdpid = 0.42*Tu;
Cpid = pidstd(Kpid,Tipid,Tdpid);
ControllerPID = feedback(Cpid*Hs, 1);

figure(4);
lsim(ControllerPID, X, t1);
axis([0 300 0 7.5]);
title('PID controller simulation');

L = lsim(ControllerPID, X, t1);
errorspid = abs((L.')- X);
errorspid_sqrd = errorspid.^2;
time_weighted_errorspid = t1 .* errorspid;
ISEpid = trapz(t1, errorspid_sqrd);
ITAEpid = trapz(t1, time_weighted_errorspid);

%4

%PID Controller 0%

t2 = 0:dt:900;
temp = 8 - (t2 < 300)*2 - (t2 > 600)*3;
d = 25:100:900;  
disturbance = pulstran(t2, d, @rectpuls, 50);
X2 = temp + disturbance;

Kpid2 = (0.95*Tg)/(Ks*Tu); 
Tipid2 = 2.2*Tg; 
Tdpid2 = 0.42*Tu;
Cpid2 = pidstd(Kpid2,Tipid2,Tdpid2);
ControllerPID2 = feedback(Cpid2*Hs, 1);

figure(5);
L = lsim(ControllerPID2, X2, t2);
plot(t2, temp,'Color', [167 167 167]/255);
hold on;
plot(t2, disturbance, 'r');
hold on;
plot(t2, L, 'b');
axis([0 900 0 10]);
title('PID controller simulation');

errorspid2 = abs((L.')- temp);
errorspid2_sqrd = errorspid2.^2;
time_weighted_errorspid2 = t2 .* errorspid2;
ISEpid2 = trapz(t2, errorspid2_sqrd);
ITAEpid2 = trapz(t2, time_weighted_errorspid2);

%5

Tsum = T1+T2;

%P Controller

Kptsum = 1/Ks;
Ctsum = pidstd(Kptsum);
ControllerPtsum = feedback(Ctsum*Hs, 1);

figure(6);
lsim(ControllerPtsum, X, t1);
axis([0 300 0 7.5]);
title('P Tsum controller simulation');

L = lsim(ControllerPtsum, X, t1);

errorsptsum =abs((L.')- X);
errorsptsum_sqrd = errorsptsum .^ 2;
time_weighted_errorsptsum = t1 .* errorsptsum;
ISEptsum = trapz(t1, errorsptsum_sqrd);
ITAEptsum = trapz(t1, time_weighted_errorsptsum);

%PI Controller 

Kpitsum = 0.5/Ks; 
Tipitsum = 0.5*Tsum;
Cpitsum = pidstd(Kpitsum,Tipitsum);
ControllerPItsum = feedback(Cpitsum*Hs, 1);

figure(7);
lsim(ControllerPItsum, X, t1);
axis([0 300 0 7.5]);
title('PI Tsum controller simulation');

L = lsim(ControllerPItsum, X, t1);

errorspitsum =abs((L.')- X);
errorspitsum_sqrd = errorspitsum .^ 2;
time_weighted_errorspitsum = t1 .* errorspitsum;
ISEpitsum = trapz(t1, errorspitsum_sqrd);
ITAEpitsum = trapz(t1, time_weighted_errorspitsum);

%PID Controller

Kpidtsum = 1/Ks; 
Tipidtsum = 0.66*Tsum; 
Tdpidtsum = 0.17*Tsum;
Cpidtsum = pidstd(Kpidtsum,Tipidtsum,Tdpidtsum);
ControllerPIDtsum = feedback(Cpidtsum*Hs, 1);

figure(8);
lsim(ControllerPIDtsum, X, t1);
axis([0 300 0 7.5]);
title('PID Tsum controller simulation');

L = lsim(ControllerPIDtsum, X, t1);

errorspidtsum =abs((L.')- X);
errorspidtsum_sqrd = errorspidtsum .^ 2;
time_weighted_errorspidtsum = t1 .* errorspidtsum;
ISEpidtsum = trapz(t1, errorspidtsum_sqrd);
ITAEpidtsum = trapz(t1, time_weighted_errorspidtsum);

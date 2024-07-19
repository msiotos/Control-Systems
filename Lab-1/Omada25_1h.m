close all;
clear all;

%Xwris anadrasi

num1 = [0 1];
den1 = [2 1];
G1 = tf(num1, den1);
num2 = [0 1];
den2 = [2 1];
G2 = tf(num2, den2);
num3 = [0 1];
den3 = [2 1];
G3 = tf(num3, den3);
sys = G1 * G2 * G3;
step(sys);


%Me anadrasi
Kp1 = [1:0.5:9];
t = 0:0.01:300;

num1 = [0 1];
den1 = [2 1];
G1 = tf(num1, den1);
num2 = [0 1];
den2 = [2 1];
G2 = tf(num2, den2);
num3 = [0 1];
den3 = [2 1];
G3 = tf(num3, den3);
G4= G1*G2*G3;

for i = 1:17
figure
pid = pidstd(Kp1(i),Inf,0,Inf);
sys = series(G4,pid);
sys_feedback = feedback(sys,1);
y = step(sys_feedback,t);
stepinfo(sys_feedback);
plot(t,y)
xlabel('t')
ylabel('T(t)')
str = sprintf('Plot of the Step P Response with Kp=%s ', string(Kp1(i)));
title(str)
grid('on');
end;

%P)
Kp1 = 4;
t = 0:0.01:60;
Ti = Inf; 
Td = 0;
N=Inf;
figure
pid= pidstd(Kp1,Ti,Td,N);
sys = series(G4,pid);
sys_feedback = feedback(sys,1);
y=step(sys_feedback,t);
%stepinfo(sys_feedback);
plot(t,y)
xlabel('t')
ylabel('T(t)')
str = sprintf('Plot of the Step P Response with Kpcr=8 Tcri=7.3');
title(str)
grid('on');
%PI)
Kp1 = 3.6;
t = 0:0.01:80;
Ti = 6.20; 
Td = 0;
N=Inf;
figure
pid= pidstd(Kp1,Ti,Td,N);
sys = series(G4,pid);
sys_feedback = feedback(sys,1);
y=step(sys_feedback,t);
%stepinfo(sys_feedback);
plot(t,y)
xlabel('t')
ylabel('T(t)')
str = sprintf('Plot of the Step PI Response with Kpcr=8 Tcri=7.3');
title(str)
grid('on');
%PID)
Kp1 = 4.8;
t = 0:0.01:40;
Ti = 3.65; 
Td = 0.87;
N=Inf;
figure
pid= pidstd(Kp1,Ti,Td,N);
sys = series(G4,pid);
sys_feedback = feedback(sys,1);
y=step(sys_feedback,t);
%stepinfo(sys_feedback);
plot(t,y)
xlabel('t')
ylabel('T(t)')
str = sprintf('Plot of the Step PID Response with Kpcr=8 Tcri=7.3');
title(str)
grid('on');

%CHR
%Xwris Anadrasi
figure
t = 0:0.01:30;
y=step(G4,t);
stepinfo(y);
plot(t,y)
xlabel('t')
ylabel('T(t)')
str = sprintf('Plot of the Step Response');
title(str)
grid('on');

%0-20% yperhpswsh
%P)
Kp1 = [1.18 2.75];
Ti = [Inf Inf]; 
Td= [0 0];
t = 0:0.01:50;
Over=[0 20];
N=Inf;
figure
for i = 1:2
pid= pidstd(Kp1(i),Ti(i),Td(i),N);
sys = series(G4,pid);
sys_feedback = feedback(sys,1);
y=step(sys_feedback,t);
%stepinfo(sys_feedback);
subplot(2,1,i)
plot(t,y)
xlabel('t')
ylabel('T(t)')
str = sprintf('Plot of the Step P Response with Overshoot %s  ', string(Over(i)));
title(str)
grid('on');
end;

%PI)
Kp1 = [1.38 2.36];
Ti = [8.04 6.70]; 
Td= [0 0];
t = 0:0.01:50;
Over=[0 20];
N=Inf;
figure
for i = 1:2
pid= pidstd(Kp1(i),Ti(i),Td(i),N);
sys = series(G4,pid);
sys_feedback = feedback(sys,1);
y=step(sys_feedback,t);
%stepinfo(sys_feedback);
subplot(2,1,i)
plot(t,y)
xlabel('t')
ylabel('T(t)')
str = sprintf('Plot of the Step PI Response with Overshoot %s  ', string(Over(i)));
title(str)
grid('on');
end;

%PID)
Kp1 = [2.36 3.74];
Ti = [6.70 9.38]; 
Td= [0.85 0.80];
t = 0:0.01:50;
Over=[0 20];
N=Inf;
figure
for i = 1:2
pid= pidstd(Kp1(i),Ti(i),Td(i),N);
sys = series(G4,pid);
sys_feedback = feedback(sys,1);
y=step(sys_feedback,t);
%stepinfo(sys_feedback);
subplot(2,1,i)
plot(t,y)
xlabel('t')
ylabel('T(t)')
str = sprintf('Plot of the Step PID Response with Overshoot %s  ', string(Over(i)));
title(str)
grid('on');
end;
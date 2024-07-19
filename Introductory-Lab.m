%Omada 25
%Eisagwgikh Askhsh
figure
a = [1 2];
for i = 1:2
    num = 1; % Numerator is 1
    den = conv([a(i), 1], [a(i), 1]); % Denominator is (a*s + 1)^2, a=1,2
    sys = tf(num, den);
    t = 0:0.01:a(i)*5;
    subplot(numel(a), 1, i);
    step(sys, t);
    xlabel('t');
    ylabel('u(t)');
    title(['Plot of the Step Function for 1/(', num2str(a(i)), 's + 1)^2']);
    grid('on');
end

figure
b = [1 0.1];
c = [5 10];
for i = 1:2
    num = 1; % Numerator is 1
    den = conv([b(i), 1], [c(i), 1]); % Denominator is (b*s + 1)(c*s+1), b=1,0.1 , c=5,10
    sys = tf(num, den);
    t = 0:0.01:c(i)*5;
    subplot(numel(b), 1, i);
    step(sys, t);
    xlabel('t');
    ylabel('u(t)');
    title(['Plot of the Step Function for 1/[(', num2str(b(i)), 's + 1)*(', num2str(c(i)),'s+1)]']);
    grid('on');
end

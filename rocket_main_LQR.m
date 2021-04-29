%
%   MECH468/509 HW5
%   Inverted Pendulum Control
%
clear;
close all;
clc;

% System paramters
rocket_para

% Linearized state-space model
% (A,B,C,D) matrices
% states=[𝑥,𝑥̇,𝑧,𝑧̇,𝜃,𝜃̇
% 𝑢=[𝐹e,𝐹s,𝜑]

rocket_ABCD

sys=ss(A,B,C,D);

figure
step(sys,1);

eig(A);

n = size(A,1);

tf = 120; % final time

%
%   Design parameters (to be adjusted)
%
Q = diag([0.01 0.01 150 30000 0.01 0.01]);% Weight for states
R = diag([0.00002 0.01 1250]);    % Weight for input
% Q = diag([0.01 0.01 500000 1000 0.01 0.01]);% Weight for states
% R = diag([0.1 0.01 1250]);    % Weight for input

% Q = diag([0.1 0.1 1500 0.1 0.1 0.1]);% Weight for states
% R = diag([0.001 0.01 1250]);    % Weight for input


%
%   Infinite horizon LQR optimal control design
%
[K,P,E] = lqr(sys,Q,R);

%
%   Closed-loop system
%
Acl = A-B*K;
x0=[10 0 1000 0 deg2rad(-10) 0]';
t = 0:0.01:tf;
this = diag([1 1 1 1 1 1]);
%u = zeros(length(t),3);
for k=1:length(t)
    x1(k)=this(1,:)*expm(Acl*t(k))*x0;
    x2(k)=this(2,:)*expm(Acl*t(k))*x0;
    x3(k)=this(3,:)*expm(Acl*t(k))*x0; 
    x4(k)=this(4,:)*expm(Acl*t(k))*x0;
    x5(k)=this(5,:)*expm(Acl*t(k))*x0;
    x6(k)=this(6,:)*expm(Acl*t(k))*x0;
    u(:,k)=-K*[x1(k) x2(k) x3(k) x4(k) x5(k) x6(k)]';

end

%
%   Plot
%
figure
subplot(3,1,1), plot(t,u(1,:)/1000), grid on
set(gca,'fontsize',12,'fontweight','bold') % Fontsize
title('Vertical Thrust input','fontsize',12,'fontweight','bold')
ylabel('F_E [kN]','fontsize',12,'fontweight','bold')
xlabel('Time (sec)','fontsize',12,'fontweight','bold')
subplot(3,1,2), plot(t,u(2,:)/1000), grid on
set(gca,'fontsize',12,'fontweight','bold') % Fontsize
title('Horizontal Thrust input','fontsize',12,'fontweight','bold')
ylabel('F_S [kN]','fontsize',12,'fontweight','bold')
xlabel('Time (sec)','fontsize',12,'fontweight','bold')
subplot(3,1,3), plot(t,rad2deg(u(3,:))), grid on
set(gca,'fontsize',12,'fontweight','bold') % Fontsize
title('Nozzle Angle','fontsize',12,'fontweight','bold')
ylabel('\phi [deg]','fontsize',12,'fontweight','bold')
xlabel('Time (sec)','fontsize',12,'fontweight','bold')
% x1 = reshape(x(1,1,:),1,length(t));
% x2 = reshape(x(2,1,:),1,length(t));
figure
subplot(3,2,1), plot(t,x1), grid on
axis([0 120 -6 10])
set(gca,'fontsize',12,'fontweight','bold') % Fontsize
title('Horizontal Distance From Ground','fontsize',12,'fontweight','bold')
ylabel('x [m]','fontsize',12,'fontweight','bold')
xlabel('Time (sec)','fontsize',12,'fontweight','bold')
subplot(3,2,2), plot(t,x2), grid on
axis([0 120 -4 2])
set(gca,'fontsize',12,'fontweight','bold') % Fontsize
title('Horizontal Distance From Ground','fontsize',12,'fontweight','bold')
ylabel('dx/dt [m]','fontsize',12,'fontweight','bold')
xlabel('Time (sec)','fontsize',12,'fontweight','bold')

subplot(3,2,3), plot(t,x3), grid on
axis([0 120 0 1000])
set(gca,'fontsize',12,'fontweight','bold') % Fontsize
title('Vertical Distance From Ground','fontsize',12,'fontweight','bold')
ylabel('z','fontsize',12,'fontweight','bold')
xlabel('Time (sec)','fontsize',12,'fontweight','bold')
subplot(3,2,4), plot(t,x4), grid on
axis([0 120 -60 0])
set(gca,'fontsize',12,'fontweight','bold') % Fontsize
title('Vertical Distance From Ground','fontsize',12,'fontweight','bold')
ylabel('dz/dt','fontsize',12,'fontweight','bold')
xlabel('Time (sec)','fontsize',12,'fontweight','bold')

subplot(3,2,5), plot(t,rad2deg(x5)), grid on
axis([0 120 -10 6])
set(gca,'fontsize',12,'fontweight','bold') % Fontsize
title('Vertical Orientation','fontsize',12,'fontweight','bold')
ylabel('\theta [deg]','fontsize',12,'fontweight','bold')
xlabel('Time (sec)','fontsize',12,'fontweight','bold')
subplot(3,2,6), plot(t,rad2deg(x6)), grid on
axis([0 120 -2 5])
set(gca,'fontsize',12,'fontweight','bold') % Fontsize
title('Vertical Orientation','fontsize',12,'fontweight','bold')
ylabel('d\theta/dt [deg]','fontsize',12,'fontweight','bold')
xlabel('Time (sec)','fontsize',12,'fontweight','bold')

figure
plot(x1,x3)
xlim([-50,50])
ylim([0,1000])
xlabel('X-axis Position (m)','fontsize',12,'fontweight','bold')
ylabel('Altitude (m)','fontsize',12,'fontweight','bold')
title('Vertical Rocket Landing','fontsize',12,'fontweight','bold')


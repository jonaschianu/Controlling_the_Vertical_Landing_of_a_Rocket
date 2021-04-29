% LQG Rocket

clear;
close all;
clc;

% System paramters
rocket_para

% Linearized state-space model
% (A,B,C,D) matrices
% states=[𝑥,𝑥̇,𝑧,𝑧̇,𝜃,𝜃̇
% 𝑢=[𝐹e,𝐹s,𝜑]
inic=[10 0 1000 0 deg2rad(-10) 0]';
rocket_ABCD

sys = ss(A,[B Bw],C,D);
Ts = .001;    % sampling time
sysd = c2d(sys,Ts)

Ad = sysd.a; Bd = sysd.b(:,1:3); Bwd = sysd.b(:,4);
Cd = sysd.c; Dd = sysd.d;

%
%   Covariances
%
Rw = 0.1^2; % disturbance input
Rv = 0.3^2;   % measurement noise


Rv=(0.3^2)*eye(3);

kfinal = 120;

%
%   Simulation of
%   x[k+1] = A x[k] + B u[k] + Bw w[k]
%    y[k]  = C x[k] + v[k]
%
x(:,:,1) = [10 0 1000 0 deg2rad(-10) 0]';
% x0=[50 0 200 0 deg2rad(-30) 0]';
w = randn(1,kfinal)*sqrt(Rw); % torque disturbance
v = randn(1,kfinal)*sqrt(Rv(1,1)); % measurement noise
v=repmat(v,3,1);
% [4,6]*ones(6,1)
% u = .01*[ones(1,kfinal/4) -0.5*ones(1,kfinal/4)...
%     0.5*ones(1,kfinal/4) -0.3*ones(1,kfinal/4)];
u = zeros(3,kfinal);
for k = 1:kfinal
    y(:,k) = Cd*x(:,:,k)+v(:,k);
    x(:,:,k+1) =Ad*x(:,:,k)+Bd*u(:,k)+Bwd*w(k);
end

%
%   Plot of the raw data
%
figure(1)
x1sub = x(1,1,:); x1 = x1sub(:);
x2sub = x(2,1,:); x2 = x2sub(:);
x3sub = x(3,1,:); x3 = x3sub(:);
x4sub = x(4,1,:); x4 = x4sub(:);
x5sub = x(5,1,:); x5 = x5sub(:);
x6sub = x(6,1,:); x6 = x6sub(:);

subplot(3,2,1), plot(x1(1:end-1),'.'), hold on
ylabel('x1[k]','fontsize',12,'fontweight','bold')
subplot(3,2,2), plot(x2(1:end-1),'.'), hold on
ylabel('x2[k]','fontsize',12,'fontweight','bold')
subplot(3,2,3), plot(x3(1:end-1),'.'), hold on
ylabel('x3[k]','fontsize',12,'fontweight','bold')
subplot(3,2,4), plot(x4(1:end-1),'.'), hold on
ylabel('x4[k]','fontsize',12,'fontweight','bold')
subplot(3,2,5), plot(x5(1:end-1),'.'), hold on
ylabel('x5[k]','fontsize',12,'fontweight','bold')
subplot(3,2,6), plot(x6(1:end-1),'.'), hold on
ylabel('x6[k]','fontsize',12,'fontweight','bold')
xlabel('k','fontsize',12,'fontweight','bold')


[Kss,Mss,Pss] = dlqe(Ad,Bwd,Cd,Rw,Rv);

%
%   State estimates
%
xhat_tss(:,:,1) = [10 0 1000 0 deg2rad(-10) 0]'; % xhat(0|-1) (time update)
for k = 1:kfinal
    % steady-state KF
    xhat_mss(:,:,k) = xhat_tss(:,:,k)+Kss*(y(k)-Cd*xhat_tss(:,:,k)); % xhat(k|k) (measurement update)
    xhat_tss(:,:,k+1) = Ad*xhat_mss(:,:,k)+Bd*u(:,k); % xhat(k+1|k) (time update)
end

figure(1)
% theta after correction
xhatm1sssub = xhat_mss(1,1,:); xhatm1ss = xhatm1sssub(:); 
% d(theta)/dt after correction
xhatm2sssub = xhat_mss(2,1,:); xhatm2ss = xhatm2sssub(:); 
% d(theta)/dt after correction
xhatm3sssub = xhat_mss(3,1,:); xhatm3ss = xhatm3sssub(:);
% d(theta)/dt after correction
xhatm4sssub = xhat_mss(4,1,:); xhatm4ss = xhatm4sssub(:);
% d(theta)/dt after correction
xhatm5sssub = xhat_mss(5,1,:); xhatm5ss = xhatm5sssub(:);
% d(theta)/dt after correction
xhatm6sssub = xhat_mss(6,1,:); xhatm6ss = xhatm6sssub(:);
% theta after prediction
xhatt1sssub = xhat_tss(1,1,:); xhatt1ss = xhatt1sssub(:); 
% d(theta)/dt after prediction
xhatt2sssub = xhat_tss(2,1,:); xhatt2ss = xhatt2sssub(:); 
% d(theta)/dt after prediction
xhatt3sssub = xhat_tss(3,1,:); xhatt3ss = xhatt3sssub(:); 
% d(theta)/dt after prediction
xhatt4sssub = xhat_tss(4,1,:); xhatt4ss = xhatt4sssub(:); 
% d(theta)/dt after prediction
xhatt5sssub = xhat_tss(5,1,:); xhatt5ss = xhatt5sssub(:); 
% d(theta)/dt after prediction
xhatt6sssub = xhat_tss(6,1,:); xhatt6ss = xhatt6sssub(:); 

kgrid = 1:kfinal;
subplot(3,2,1), plot(kgrid,xhatm1ss,'r',kgrid,xhatt1ss(1:end-1),'g-.')
%legend('true x_1','correction','prediction')
set(gca,'fontsize',12,'fontweight','bold') % Fontsize
subplot(3,2,2), plot(kgrid,xhatm2ss,'r',kgrid,xhatt2ss(1:end-1),'g-.')
%legend('true x_2','correction','prediction')
set(gca,'fontsize',12,'fontweight','bold') % Fontsize
subplot(3,2,3), plot(kgrid,xhatm3ss,'r',kgrid,xhatt3ss(1:end-1),'g-.')
%legend('true x_2','correction','prediction')
set(gca,'fontsize',12,'fontweight','bold') % Fontsize
subplot(3,2,4), plot(kgrid,xhatm4ss,'r',kgrid,xhatt4ss(1:end-1),'g-.')
%legend('true x_2','correction','prediction')
set(gca,'fontsize',12,'fontweight','bold') % Fontsize
subplot(3,2,5), plot(kgrid,xhatm5ss,'r',kgrid,xhatt5ss(1:end-1),'g-.')
%legend('true x_2','correction','prediction')
set(gca,'fontsize',12,'fontweight','bold') % Fontsize
subplot(3,2,6), plot(kgrid,xhatm6ss,'r',kgrid,xhatt6ss(1:end-1),'g-.')
%legend('true x_2','correction','prediction')
set(gca,'fontsize',12,'fontweight','bold') % Fontsize
legend('true value','correction','prediction')

% DLQR design
%Qlqr =  diag([0.01 0.01 150 30000 0.01 0.01]);% Weight for states
%Rlqr = diag([0.00002 0.01 1250]);   % Weight for input

%  Qlqr = diag([0.1 0.1 1500 0.1 0.1 0.1]);% Weight for states
%  Rlqr = diag([0.001 0.01 1250]);       % Weight for input
 Qlqr = diag([0.01 0.01 150 30000 0.01 0.01]);% Weight for states
 Rlqr = diag([0.00002 0.01 1250]);       % Weight for input
 
% Qlqr = 100*eye(6); Rlqr = 0.1;
[Kdlqr,Pdlqr,Edlqr] = dlqr(Ad,Bd,Qlqr,Rlqr);

x0 = [10 0 1000 0 deg2rad(-10) 0]';
sim('LQGrocket')

data=LQGx1.Data; data = data(1,1,:); datax1 = data(:);
data=LQGx2.Data; data = data(1,1,:); datax2 = data(:);
data=LQGx3.Data; data = data(1,1,:); datax3 = data(:);
data=LQGx4.Data; data = data(1,1,:); datax4 = data(:);
data=LQGx5.Data; data = data(1,1,:); datax5 = data(:);
data=LQGx6.Data; data = data(1,1,:); datax6 = data(:);

figure(2)
subplot(3,2,1)
grid on;
plot(LQGx1.Time,datax1,'r','LineWidth',2), hold on
axis([0 120 -6 10])
grid on
hold off
set(gca,'fontsize',12,'fontweight','bold')
title('Horizontal Distance From Ground','fontsize',12,'fontweight','bold')
ylabel('x [m]','fontsize',12,'fontweight','bold')
xlabel('Time (sec)','fontsize',12,'fontweight','bold')
subplot(3,2,2)
plot(LQGx2.Time,datax2,'r','LineWidth',2), hold on
axis([0 120 -4 2])
grid on
hold off
set(gca,'fontsize',12,'fontweight','bold')
title('Horizontal Velocity','fontsize',12,'fontweight','bold')
ylabel('dx/dt [m]','fontsize',12,'fontweight','bold')
xlabel('Time (sec)','fontsize',12,'fontweight','bold')
subplot(3,2,3)
plot(LQGx3.Time,datax3,'r','LineWidth',2), hold on
axis([0 120 0 1000])
grid on
hold off
set(gca,'fontsize',12,'fontweight','bold')
title('Vertical Distance From Ground','fontsize',12,'fontweight','bold')
ylabel('z','fontsize',12,'fontweight','bold')
xlabel('Time (sec)','fontsize',12,'fontweight','bold')
subplot(3,2,4)
plot(LQGx4.Time,datax4,'r','LineWidth',2), hold on
axis([0 120 -60 0])
grid on
hold off
set(gca,'fontsize',12,'fontweight','bold')
title('Vertical Velocity','fontsize',12,'fontweight','bold')
ylabel('dz/dt','fontsize',12,'fontweight','bold')
xlabel('Time (sec)','fontsize',12,'fontweight','bold')
subplot(3,2,5)
plot(LQGx5.Time,rad2deg(datax5),'r','LineWidth',2), hold on
axis([0 120 -10 6])
grid on
hold off
set(gca,'fontsize',12,'fontweight','bold')
title('Vertical Orientation','fontsize',12,'fontweight','bold')
ylabel('\theta [deg]','fontsize',12,'fontweight','bold')
xlabel('Time (sec)','fontsize',12,'fontweight','bold')
subplot(3,2,6)
plot(LQGx6.Time,rad2deg(datax6),'r','LineWidth',2), hold on
axis([0 120 -2 5])
grid on
hold off
set(gca,'fontsize',12,'fontweight','bold')
title('Angular Velocity','fontsize',12,'fontweight','bold')
ylabel('d\theta/dt [deg]','fontsize',12,'fontweight','bold')
xlabel('Time (sec)','fontsize',12,'fontweight','bold')

input=u1.Data; input = input(1,1,:); input_u1 = input(:);
input=u2.Data; input = input(1,1,:); input_u2 = input(:);
input=u3.Data; input = input(1,1,:); input_u3 = input(:);

figure
subplot(3,1,1), plot(u1.Time,input_u1/1000,'r','LineWidth',1), grid on
set(gca,'fontsize',12,'fontweight','bold') % Fontsize
title('Vertical Thrust input','fontsize',12,'fontweight','bold')
ylabel('F_E [kN]','fontsize',12,'fontweight','bold')
xlabel('Time (sec)','fontsize',12,'fontweight','bold')
subplot(3,1,2), plot(u2.Time,input_u2/1000,'r','LineWidth',1), grid on
set(gca,'fontsize',12,'fontweight','bold') % Fontsize
title('Horizontal Thrust input','fontsize',12,'fontweight','bold')
ylabel('F_S [kN]','fontsize',12,'fontweight','bold')
xlabel('Time (sec)','fontsize',12,'fontweight','bold')
subplot(3,1,3), plot(u3.Time,input_u3,'r','LineWidth',1), grid on
set(gca,'fontsize',12,'fontweight','bold') % Fontsize
title('Nozzle Angle','fontsize',12,'fontweight','bold')
ylabel('\phi [deg]','fontsize',12,'fontweight','bold')
xlabel('Time (sec)','fontsize',12,'fontweight','bold')

figure
plot(datax1,datax3,'r','LineWidth',1)
xlim([-50,50])
ylim([0,1000])
xlabel('X-axis Position (m)','fontsize',12,'fontweight','bold')
ylabel('Altitude (m)','fontsize',12,'fontweight','bold')
title('Vertical Rocket Landing','fontsize',12,'fontweight','bold')

x0 = Simulink.BlockDiagram.getInitialState('LQGrocket')

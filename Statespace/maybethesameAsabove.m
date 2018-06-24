%Roco319 State space control CW
clc
clear all
close all
var1 = 1;
var2 = 0;
var3 = [var1/var2]
%solve
syms x
eqn = sin(x) == 1;
solx = solve(eqn,x)
Mb = 0.595
Mw = 0.031
Jb = 0.0015
r = 0.04
Jw = 0.00000059
Lm = 0.08
L = Lm
Ke = 0.486
Km = 0.317
R = 6.69
b = 0.002
g = 9.81

    alpha =  (2*(R*b - Ke*Km)*(Mb*L^(2) + Mb*r*L+Jb))/ R(2*(Jb*Jw+ Jw * L^(2) *Mb + Jb*Mw*r^(2)+L^(2)*Mb*Mw*R^(2))+Jb*Mb*r^(2))
beta = (-L^(2)*Mb^(2)*g*r^(2))/ (Jb*(2*Jw+Mb*r^(2)+2*Mw*r^(2))+2*Jw*L^(2)*Mb+2*L^(2)*Mb*Mw*r^(2))

gamma = (-2*(Rb-Ke*Km)*(2*Jw+Mb*r^2+2*Mw*r^2+L*Mb*r))/(r*R(2*(Jb*Jw+Jw*L^2*Mb+Jb*Mw*r^2+L^2*Mb*Mw*r^2)+Jb*Mb*r^2))

delta = (L*Mb*g*(2*Jw+Mb*r^2+2*Mw*r^2))/(2*Jb*Jw+2*Jw*L^2*Mb+Jb*Mb*r^2+2*Jb*Mw*r^2+2*L^2*Mb*Mw*r^2)

epsilon = (Km*r)/(R*b-Ke*Km)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%A)
%Write the non-linear equations of motion in state space form
syms  x1 x2 theta1 theta2 u M m b In g L
%Sub to reduce size
s = m*L*sin(theta1);
c = m*L*cos(theta1);
Mt = M + m;
It = In+m*(L^2);
%Rearrange original equations to get x'' and theta'' as subject
%then decouple and reduce order
%Write in state space form where dotXvec = [x1';x2';theta1';theta2'] = [x',x'',theta',theta'']
dotXvec = [x2; 
    (((b*x2)+(s*theta2^2)-u)*It-(s*g*c))/(c^2-Mt*It); 
    theta2; 
    ((u-(b*x2)-(s*theta2^2))*c+(s*g*Mt))/(It*Mt-c^2)]
M = 0.5;
m = 0.5;
b = 0.1;
In = 0.006;
g = 9.8;
L = 0.3;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%B)
%Linearise using jacobian where Xvec = 0 and u = 0 and write as dotXvec = Ax + Bu
% and Y = Cx + Du
A = jacobian(dotXvec, [x1, x2, theta1, theta2]);
B = jacobian(dotXvec, u);
A = subs(A, theta1, 0);
A = subs(A, theta2, 0);
A = subs(A, x1, 0);
A = subs(A, x2, 0);
A = subs(A, u, 0);
B = subs(B, theta1, 0);
B = subs(B, theta2, 0);
B = subs(B, x1, 0);
B = subs(B, x2, 0);
B = subs(B, u, 0);
disp('Symbolic linear A matrix:')
pretty(A)
disp('Symbolic linear B matrix')
pretty(B)
%Y = [x;theta1] = CX + Du
D = [0;0];
C = [1 0 0 0;
     0 0 1 0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%C)
%Sub in parameters and Get controllability and observability
M = 0.5;
m = 0.5;
b = 0.1;
In = 0.006;
g = 9.8;
L = 0.3;
A = double(subs(A))
B = double(subs(B))
A*[x1;x2;theta1;theta2]+B*u
figure(10)
%Simulation of linearised dynamical system
Simulate_linear_dynsys(147/19,-17/96,980/19,-10/19,-0.1,-0.1);
figure(11)
Simulate_linear_dynsys(147/19,-17/96,980/19,-10/19,0.1,0.1);
%Represent as a state space system
sys = ss(A,B,C,0);
%Find controllability matrix
Co = ctrb(sys);
%Rank of Co tells us how controllable our system is, full rank means the
%system is fully controllable
rank(Co);
%Find observablity matrix
Ob = obsv(sys);
%Rank of Ob tells us how observable our system is, full rank means system
%is completely observable
rank(Ob);
%Find eigen values to determine stability
disp('Poles of uncontrolled system:')
eig(A)
%Simulate system
t = 0:0.001:2.5;
r = zeros(size(t));
r(1)=1;
SYS = ss(A,B,C,0);
figure(5)
[y,t,xx]=lsim(SYS,r,t,[0 0 0 0]);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(AX,'FontSize',14);
set(get(AX(1),'Ylabel'),'String','cart position (m)','FontSize',16);
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)','FontSize',16);
title('System with no controller','FontSize',16);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%D)
%Kalman decomposition
%As system Controllability and Observability matrices both have full rank
%intutitively we know that the system is already the result of its Kalman
%decompostion, however we will uses the 'minreal; function to test this.
[MSYS, U] = minreal(sys);
%E)
%Design observer for system and place eigen values at [-40 -41 -42 -43]
%xhat = Axhat + Bu + L(y-Cxhat) = observer
C = [1 0 0 0;0 0 1 0];
p = [-40, -41,-42,-43];
disp('Calculated L gain matrix:')
Lg = place(A',C',p)'
%error equation
Ae = A-Lg*C;
disp('Placed poles of observer:')
eig(A-Lg*C)
Be = [zeros(size(B))];
Ce = [zeros(size(C))];
De = [0];
syso = ss(Ae,Be,Ce,De);
syms xhat1 xhat2 thetahat1 thetahat2 u(t) x1 theta1
xhatVec = [xhat1;xhat2;thetahat1;thetahat2]
observer = A*xhatVec + B*u(t) + Lg*([x1;theta1]-C*xhatVec);
pretty(observer)
% Simulate the error equations
t = 0:0.001:2.5;
rvec = zeros(size(t)); % no input
x0 = [0 0 pi 0];
%x0 =[0 0 pi randn]; % [dtheta(0) theta(0) dx(0) x(0)]
[y,t,x] = lsim(syso, rvec, t, x0);
xhat = xx-x;
figure(1);
subplot(4,1,1);
plot(t,xx(:,1),'k',t,xhat(:,1),'k--');hold on;
ylabel('x_1 (m)','FontSize',16);
xlabel('time (sec)','FontSize',16);
subplot(4,1,2);
plot(t,xx(:,2),'r',t,xhat(:,2),'r--');hold on;
ylabel('x_2 (m/s)','FontSize',16);
xlabel('time (sec)','FontSize',16);
subplot(4,1,3);
plot(t,xx(:,3),'b',t,xhat(:,3),'b--');hold on;
ylabel('theta_1 (rad)','FontSize',16);
xlabel('time (sec)','FontSize',16);
% axis([0 1 -5 5]);
subplot(4,1,4);
plot(t,xx(:,4),'g',t,xhat(:,4),'g--');hold on;
ylabel('theta_2 (rad/s)','FontSize',16);
xlabel('time (sec)','FontSize',16);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%F)
%Design controller for system and place eigen values at [-8+7i, -8-7i,-5+i,-5-i]
P = [-8+7i, -8-7i,-5+i,-5-i];
disp('Calculated K gains of required to place eigen values:')
K = place(A,B,P)
Cx = [1 0 0 0];
Kr = -1/(Cx*((A-(B*K))^-1)*B)
%% xdot = (A-BK)x + BKr*u
syms x1 x2 theta1 theta2 r
feedBackControl = (A-B*K)*[x1;x2;theta1;theta2] + B*Kr*r
disp('Placed poles of feedback system:')
eig(A-B*K)
newA = [(A-(B*K)), B*K;
        zeros(size(A)), (A-(Lg*C))];
newB = [B*Kr;zeros(size(B*Kr))];
newC = [C, zeros(size(C))];
states = {'x' 'x_dot' 'theta' 'theta_dot' 'e1' 'e2' 'e3' 'e4'};
inputs = {'r'};
outputs = {'x' 'theta';};
feedBackSYS = ss(newA,newB,newC, 0,'statename',states,'inputname',inputs,'outputname',outputs);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%G)
%Simulate observer based feedback control
t = 0:0.01:6;
figure(2)
r = 0.25*[ones(size(t));];
[y,t,x]=lsim(feedBackSYS,r,t,[0 0 0 0 0 0 0 0]);
subplot(2,2,1); hold on;
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(AX,'FontSize',14);
set(get(AX(1),'Ylabel'),'String','cart position (m)','FontSize',16);
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)','FontSize',16);
title('Step Response with Observer-Based State-Feedback Control(r = 0.25)','FontSize',16);
r = 0.5*[ones(size(t));];
[y,t,x]=lsim(feedBackSYS,r,t);
subplot(2,2,2); hold on;
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(AX,'FontSize',14);
set(get(AX(1),'Ylabel'),'String','cart position (m)','FontSize',16);
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)','FontSize',16);
title('Step Response with Observer-Based State-Feedback Control(r = 0.5)','FontSize',16);
r = 1*[ones(size(t));];
[y,t,x]=lsim(feedBackSYS,r,t);
subplot(2,2,3); hold on;
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(AX,'FontSize',14);
set(get(AX(1),'Ylabel'),'String','cart position (m)','FontSize',16);
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)','FontSize',16);
title('Step Response with Observer-Based State-Feedback Control(r = 1)','FontSize',16);
r = 200*[ones(size(t));];
[y,t,x]=lsim(feedBackSYS,r,t);
subplot(2,2,4); hold on;
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(AX,'FontSize',14);
set(get(AX(1),'Ylabel'),'String','cart position (m)','FontSize',16);
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)','FontSize',16);
title('Step Response with Observer-Based State-Feedback Control(r = 200)','FontSize',16);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%H)
%Optimal control
C = [1,0,0,0;
    0,0,1,0];
Q = C'*C;
Q(1,1) = 300;
Q(3,3) = 100;
R = 1;
[K,P,E] = lqr(A,B,Q,R);
disp('Calculated K gain for optimised system:')
K
Ace = [(A-B*K)];
Bce = [B];
Cce = [C];
disp('Caculated poles of optimal system(Q(1,1) = 300 and Q(3,3) = 100):')
eig(A-B*K)
Dce = [0;0];
states = {'x' 'x_dot' 'theta' 'theta_dot'};
inputs = {'r'};
outputs = {'x'; 'theta'};
optimalSYS = ss(Ace,Bce,Cce, 0,'statename',states,'inputname',inputs,'outputname',outputs);
t = 0:0.01:6;
r = 0.25*[ones(size(t))];
figure(3)
subplot(2,2,1);
[y,t,x]=lsim(optimalSYS,r,t , [0 0 0 0]);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(AX,'FontSize',14);
set(get(AX(1),'Ylabel'),'String','cart position (m)', 'FontSize',16);
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)', 'FontSize',16);
title('Step Response with Optimal Control(r = 0.25)', 'FontSize',16);
r = 0.5*[ones(size(t))];
figure(3)
subplot(2,2,2);
[y,t,x]=lsim(optimalSYS,r,t, [0 0 0 0]);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(AX,'FontSize',14);
set(get(AX(1),'Ylabel'),'String','cart position (m)', 'FontSize',16);
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)', 'FontSize',16);
title('Step Response with Optimal Control(r = 0.5)', 'FontSize',16);
r = [ones(size(t))];
figure(3)
subplot(2,2,3)
[y,t,x]=lsim(optimalSYS,r,t, [0 0 0 0]);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(AX,'FontSize',14);
set(get(AX(1),'Ylabel'),'String','cart position (m)', 'FontSize',16);
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)', 'FontSize',16);
title('Step Response with Optimal Control(r = 1)', 'FontSize',16);
r = 200*[ones(size(t))];
figure(3)
subplot(2,2,4);
[y,t,x]=lsim(optimalSYS,r,t, [0 0 0 0]);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(AX,'FontSize',14);
set(get(AX(1),'Ylabel'),'String','cart position (m)', 'FontSize',16);
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)', 'FontSize',16);
title('Step Response with Optimal Control(r = 200)', 'FontSize',16);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%H)
%Optimal control
C = [1,0,0,0;
    0,0,1,0];
Q = C'*C;
Q(1,1) = 5000;
Q(3,3) = 100;
R = 1;
[K,P,E] = lqr(A,B,Q,R);
disp('Calculated K gain for optimised system:')
K
Ace = [(A-B*K)];
disp('Caculated poles of optimal system(Q(1,1) = 5000 and Q(3,3) = 100):')
eig(Ace)
Ace*[x1;x2;theta1;theta2];
Bce = [B];
Cce = [C];
Dce = [0;0];
states = {'x' 'x_dot' 'theta' 'theta_dot'};
inputs = {'r'};
outputs = {'x'; 'theta'};
optimalSYS = ss(Ace,Bce,Cce, 0,'statename',states,'inputname',inputs,'outputname',outputs);
t = 0:0.01:6;
r = 0.25*[ones(size(t))];
figure(4)
subplot(2,2,1);
[y,t,x]=lsim(optimalSYS,r,t, [0 0 0 0]);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(AX,'FontSize',14);
set(get(AX(1),'Ylabel'),'String','cart position (m)', 'FontSize',16);
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)', 'FontSize',16);
title('Step Response with Optimal Control(r = 0.25)', 'FontSize',16);
r = 0.5*[ones(size(t))];
figure(4)
subplot(2,2,2);
[y,t,x]=lsim(optimalSYS,r,t, [0 0 0 0]);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(AX,'FontSize',14);
set(get(AX(1),'Ylabel'),'String','cart position (m)', 'FontSize',16);
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)', 'FontSize',16);
title('Step Response with Optimal Control(r = 0.5)', 'FontSize',16);
r = [ones(size(t))];
figure(4)
subplot(2,2,3);
[y,t,x]=lsim(optimalSYS,r,t, [0 0 0 0]);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(AX,'FontSize',14);
set(get(AX(1),'Ylabel'),'String','cart position (m)', 'FontSize',16);
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)', 'FontSize',16);
title('Step Response with Optimal Control(r = 1)', 'FontSize',16);
r = 200*[ones(size(t))];
figure(4)
subplot(2,2,4);
[y,t,x]=lsim(optimalSYS,r,t, [0 0 0 0]);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(AX,'FontSize',14);
set(get(AX(1),'Ylabel'),'String','cart position (m)', 'FontSize',16);
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)', 'FontSize',16);
title('Step Response with Optimal Control(r = 200)', 'FontSize',16);
%H)
%Optimal control
C = [1,0,0,0;
    0,0,1,0];
Q = C'*C;
Q(1,1) = 1;
Q(3,3) = 1;
R = 1;
[K,P,E] = lqr(A,B,Q,R);
disp('Calculated K gain for optimised system:')
K
Ace = [(A-B*K)];
disp('Caculated poles of optimal system(Q(1,1) = 1 and Q(3,3) = 1):')
eig(Ace)
Ace*[x1;x2;theta1;theta2];
Bce = [B];
Cce = [C];
Dce = [0;0];
states = {'x' 'x_dot' 'theta' 'theta_dot'};
inputs = {'r'};
outputs = {'x'; 'theta'};
optimalSYS = ss(Ace,Bce,Cce, 0,'statename',states,'inputname',inputs,'outputname',outputs);
t = 0:0.01:6;
r = 0.25*[ones(size(t))];
figure(8)
subplot(2,2,1);
[y,t,x]=lsim(optimalSYS,r,t, [0 0 0 0]);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(AX,'FontSize',14);
set(get(AX(1),'Ylabel'),'String','cart position (m)', 'FontSize',16);
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)', 'FontSize',16);
title('Step Response with Optimal Control(r = 0.25)', 'FontSize',16);
r = 0.5*[ones(size(t))];
figure(8)
subplot(2,2,2);
[y,t,x]=lsim(optimalSYS,r,t, [0 0 0 0]);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(AX,'FontSize',14);
set(get(AX(1),'Ylabel'),'String','cart position (m)', 'FontSize',16);
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)', 'FontSize',16);
title('Step Response with Optimal Control(r = 0.5)', 'FontSize',16);
r = [ones(size(t))];
figure(8)
subplot(2,2,3);
[y,t,x]=lsim(optimalSYS,r,t, [0 0 0 0]);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(AX,'FontSize',14);
set(get(AX(1),'Ylabel'),'String','cart position (m)', 'FontSize',16);
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)', 'FontSize',16);
title('Step Response with Optimal Control(r = 1)', 'FontSize',16);
r = 200*[ones(size(t))];
figure(8)
subplot(2,2,4);
[y,t,x]=lsim(optimalSYS,r,t, [0 0 0 0]);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(AX,'FontSize',14);
set(get(AX(1),'Ylabel'),'String','cart position (m)', 'FontSize',16);
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)', 'FontSize',16);
title('Step Response with Optimal Control(r = 200)', 'FontSize',16);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
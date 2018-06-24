%http://sebastiannilsson.com/wp-content/uploads/2013/05/Self-balancing-two-wheeled-robot-report.pdf
Mb = 0.595
Mw = 0.031
Jb = 0.0015
r = 0.04
Jw = 0.00000059
Lm = 0.08
Ke = 0.486
Km = 0.317
R = 6.69
b = 0.002
g = 9.81
%L = 1
L = [0.0290 -0.9072; 0.1395 1.9804; 0.9902 29.1690]

Mb*L.^2 
r*L
L2 = L*L'

%alpha = ( 2*(R*b - Ke*Km)*(Mb*L.^2 + Mb*r*L+Jb))/ R*(2*(Jb*Jw+ Jw * L.^2 *Mb + Jb*Mw*r^2+L.^2*Mb*Mw*R^2)+Jb*Mb*r^2)
alpha = ( 2*(R*b - Ke*Km)*(Mb*L2 + (Mb*r*L)+Jb))/ R*(2*(Jb*Jw+ Jw * L2 *Mb + Jb*Mw*r^2+L2*Mb*Mw*R^2)+Jb*Mb*r^2)
%beta = (-L^2*Mb^2*g*r^2)/ (Jb*(2*Jw+Mb*r^2+2*Mw*r^2)+2*Jw*L^2*Mb+2*L^2*Mb*Mw*r^2)
%gammanum = (-2*(R*b-Ke*Km)*(2*Jw+Mb*r^2+2*Mw*r^2+L*Mb*r))
%gammadem = (r*R*(2*(Jb*Jw+Jw*L^2*Mb+Jb*Mw*r^2+L^2*Mb*Mw*r^2)+Jb*Mb*r^2))
%gamma = gammanum/gammadem
%delta = (L*Mb*g*(2*Jw+Mb*r^2+2*Mw*r^2))/(2*Jb*Jw+2*Jw*L^2*Mb+Jb*Mb*r^2+2*Jb*Mw*r^2+2*L^2*Mb*Mw*r^2)
beta = (-L2*Mb^2*g*r^2)/ (Jb*(2*Jw+Mb*r^2+2*Mw*r^2)+2*Jw*L2*Mb+2*L2*Mb*Mw*r^2)

gammanum = (-2*(R*b-Ke*Km)*(2*Jw+Mb*r^2+2*Mw*r^2+L*Mb*r))
gammadem = (r*R*(2*(Jb*Jw+Jw*L2*Mb+Jb*Mw*r^2+L2*Mb*Mw*r^2)+Jb*Mb*r^2))
gamma = gammanum/gammadem
delta = (L*Mb*g*(2*Jw+Mb*r^2+2*Mw*r^2))/(2*Jb*Jw+2*Jw*L2*Mb+Jb*Mb*r^2+2*Jb*Mw*r^2+2*L2*Mb*Mw*r^2)

epsilon = (Km*r)/(R*b-Ke*Km)

A = [0 1 0 0; 0 alpha beta -r*alpha; 0 0 0 1; 0 gamma delta -r*gamma]
B = [0 alpha*epsilon 0 gamma*epsilon]'
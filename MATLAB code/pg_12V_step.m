clc
clear all
close all

mbattery = 0.432;
m_motor = 0.458;
Mw = 0.106; % mass of wheel
Mbody = 2.243-(2*(m_motor+Mw)); % mass of robot(including battery) excluding wheel and motor
Ke = 0.118; %back emf constant
Rm = 5.6; %terminal resistance
wheeldia = 0.100;
rw = wheeldia/2; %radius of wheel
Kb = Ke^2/(Rm*rw); % back emf related something
hcom = 0.111; %cog from the centre of wheel
Ibody = 0.01306; % body inertia
Itotal = Ibody + Mbody*(hcom^2); % total pitch

Jw = 0.5*Mw*(rw^2); % wheel inertia
Jm = 2.66*(10^-5); % rotor inertia
Meq = Mbody + (2*Mw)+ (2*(Jw+Jm)/(rw^2)); % total mass
Km = Ke/Rm ; % motor constant
delta = Meq*Itotal - (Mbody*hcom)^2; 
g = 9.81; % acc due to gravity

A= [0 1 0 0;
    0 ((1/delta)*(2*Kb*Mbody*hcom - (2*Kb*Itotal/rw))) (Mbody*hcom)^2*g/delta 0;
    0 0 0 1;
    0 ((1/delta)*(2*Kb*Meq - (2*Kb*Mbody*hcom/rw))) Meq*Mbody*g*hcom/delta 0];

B = [0;
    ((1/delta)*(-2*Km*Mbody*hcom + (2*Km*Itotal/rw)));
    0;
    ((1/delta)*(-2*Km*Meq + (2*Km*Mbody*hcom/rw)))];
C = [0 0 1 0];
D =0;

sys=ss(A,B,C,D);
systf=tf(sys)

ki =749.905;
kp=0.1 +(27.2684+1.819*ki)/3.1523;
kd=0;

den = [1 0];
num = [kd kp ki];
controller_tf = tf(num,den);
full_tf = systf * controller_tf;
closedloop_tf = feedback(full_tf,1);

stepplot(closedloop_tf)
sisotool(systf)

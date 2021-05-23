clear all
close all
clc

%% Rocket definition
mn_1 = 250000; % [kg]
mn_2 = 50000; % [kg]
m1 = 12000; % [kg]

m0 = m1+mn_1+mn_2; % [kg]

m8_1 = 0.9*mn_1; % [kg]
m8_2 = 0.9*mn_2; % [kg]

C_1 = 2.9518e+03; % [kg]
C_2 = 4.5111e+03; % [kg]

a_1 = 12.5; % [m/s^2]
a_2 = 10; % [m/s^2]

cw_1 = 0.4;
cw_2 = 0.4;
ca_1 = 0;
ca_2 = 0;
% TODO: D berechnen
D = 4;

%% Planet

r0=6371000;
K=3.9658e14;
g0=9.81;
gamma0 = 90 * pi/180; 


%% Konstantenberechnung 
A_1 = D^2 * pi /4;
A_2 = D^2 * pi /4;

F_1 = m0 * a_1; % [N]
F_2 = (m0 - mn_1) * a_2; % [N]

dF_1 = 0.08;
dF_2 = 0;

Itot_1 = m8_1 * C_1;
Itot_2 = m8_2 * C_2;

mp_1 =  F_1/C_1;
mp_2 =  F_2/C_2;

tc_1 = Itot_1 / F_1;
tc_2 = Itot_2 / F_2;

%% Simulation
%% Unterstufe

c = [cw_1 ca_1 A_1 K mp_1 F_1 tc_1 r0 dF_1];
tspan = 0:1:tc_1;
y0 = [0 r0 m0 gamma0 0];

[T1,Y1] = ode15s(@(t,y) Rocket_2DOF(t,y,c), tspan, y0);

v1 = Y1(end,1);
r1 = Y1(end,2);
m1 = Y1(end,3);
gamma1 = Y1(end,4);
angle1 = Y1(end,5);

%% TODO: Zwischenflugphase Stufentrennung
t_sep = 6; % [s]

c = [cw_1 ca_1 A_1 K 0 0 0 r0 0];
tspan = 0:1:t_sep;
y0 = [v1 r1 m0-mn_1 gamma1 angle1];

[Ts,Ys] = ode15s(@(t,y) Rocket_2DOF(t,y,c), tspan, y0);

vs = Ys(end,1);
rs = Ys(end,2);
ms = Ys(end,3);
gammas = Ys(end,4);
angles = Ys(end,5);



%% Oberstufe

c = [cw_2 ca_2 A_2 K mp_2 F_2 tc_2 r0 dF_2];
tspan = [0:1:tc_2];
y1 = [vs rs m0-mn_1 gammas angles];

[T2,Y2] = ode15s(@(t,y) Rocket_2DOF(t,y,c), tspan, y1);


%% Combine Sim-Data

T = [T1; Ts+length(T1); T2+length(T1)+length(Ts)];
Y = [Y1; Ys; Y2];

%% Plot results
% Calculate accelerations as derivatives from velocities
    deltav=diff(Y(:,1));
    deltat=diff(T);
    lt=length(T);
    for j=1 : lt-1
        acc(j)=deltav(j)/deltat(j);
    end
    acc(lt)=acc(lt-1);
    if length (acc) > lt
        la=length(acc);
        acc(lt+1:la)=[];
    end
% plot results  

subplot(5,1,1)
plot(T,acc,'b-')
ylabel ('Acceleration [m/s^2]')
xlabel ('Time [s]')
xline(length(T1)-1, '-black', {'Stage seperation'});
xline(length(T1)-1+length(Ts)-1, '-r', {'Upper stage ignition'});

subplot(5,1,2)
plot (T,Y(:,1),'r-')
ylabel ('Velocity [m/s]')
xlabel ('Time [s]')
xline(length(T1)-1, '-black', {'Stage seperation'});
xline(length(T1)-1+length(Ts)-1, '-r', {'Upper stage ignition'});

subplot(5,1,3)
plot (T,Y(:,2)-r0,'b-')
ylabel ('Altitude [m]')
xlabel ('Time [s]')
xline(length(T1)-1, '-black', {'Stage seperation'});
xline(length(T1)-1+length(Ts)-1, '-r', {'Upper stage ignition'});

subplot(5,1,4)
plot (T,Y(:,4)*180/pi)
ylabel ('gamma [°]')
xlabel ('Time [s]')
xline(length(T1)-1, '-black', {'Stage seperation'});
xline(length(T1)-1+length(Ts)-1, '-r', {'Upper stage ignition'});

subplot(5,1,5)
plot (T,Y(:,3))
ylabel ('Mass [kg]')
xlabel ('Time [s]')
xline(length(T1)-1, '-black', {'Stage seperation'});
xline(length(T1)-1+length(Ts)-1, '-r', {'Upper stage ignition'});





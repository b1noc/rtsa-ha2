clear all
close all
clc



%% Rocket definition
m0 = 1.7819e+05; % [kg]

mn_1 = 1.1772e+05; % [kg]
mn_2 = 5.0474e+04; % [kg]
m1 = 10000; % [kg]

m8_1 = 1.0519e+05; % [kg]
m8_2 = 4.1619e+04; % [kg]

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

Itot_1 = m8_1 * C_1;
Itot_2 = m8_2 * C_2;

mp_1 =  F_1/C_1;
mp_2 =  F_2/C_2;

tc_1 = Itot_1 / F_1;
tc_2 = Itot_2 / F_2;

gammarate = - (90 * pi/180)/(tc_1 + tc_2);

g_1 = gammarate * 0.6;
g_2 = gammarate - g_1;


%% Simulation
%% Unterstufe

c = [cw_1 ca_1 A_1 K mp_1 F_1 tc_1 r0 g_1];
tspan = 0:1:tc_1;
y0 = [0 r0 m0 gamma0 0];

[T_1,Y_1] = ode15s(@(t,y) Rocket_2DOF(t,y,c), tspan, y0);




%% Oberstufe

v1 = Y_1(end,1);
r1 = Y_1(end,2);
m1 = Y_1(end,3);
gamma1 = Y_1(end,4);
angle1 = Y_1(end,5);

% TODO: Zwischenflugphase Stufentrennung

c = [cw_2 ca_2 A_2 K mp_2 F_2 tc_2 r0 g_2];
tspan = [0:1:tc_2];

y1 = [v1 r1 (m0-mn_1) gamma1 angle1];

[T_2,Y_2] = ode15s(@(t,y) Rocket_2DOF(t,y,c), tspan, y1);

T = [T_1; T_2+length(T_1)];
Y = [Y_1; Y_2];



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
xline(length(T_1)-1, '-r', {'Stage Seperation'})

subplot(5,1,2)
plot (T,Y(:,1),'r-')
ylabel ('Velocity [m/s]')
xlabel ('Time [s]')
xline(length(T_1)-1, '-black', {'Stage Seperation'})

subplot(5,1,3)
plot (T,Y(:,2)-r0,'b-')
ylabel ('Altitude [m]')
xlabel ('Time [s]')
xline(length(T_1)-1, '-black', {'Stage Seperation'})

subplot(5,1,4)
plot (T,Y(:,4)*180/pi)
ylabel ('gamma [°]')
xlabel ('Time [s]')
xline(length(T_1)-1, '-black', {'Stage Seperation'})

subplot(5,1,5)
plot (T,Y(:,3))
ylabel ('Mass [kg]')
xlabel ('Time [s]')
xline(length(T_1)-1, '-black', {'Stage Seperation'})





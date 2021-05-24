clc
clear Rocket_2DOF
clear all

%% Rocket definition
mn_1 = 250000; % [kg]
mn_2 = 40000; % [kg]
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

%% Bahnparameter



dt_grav = 3;
t_grav = 24;
alpha_grav = 3.9;

t_oben = [300];
alpha2 = [0 99];


%% Unterstufe
tm = [0 t_grav t_grav+dt_grav tc_1];
alpha = [0 -alpha_grav 0];

vi = 0;
ri = r0;
mi = m0;
gi = gamma0;
ai = 0;

T1 = [];
Y1 = [];

mode = 0;

for i = 1:3
    c = [cw_1 A_1 K mp_1 F_1 tc_1 r0 alpha(i)];
    tspan = [tm(i) tm(i+1)];
    y0 = [vi ri mi gi ai];
    
    [Ti,Yi] = ode15s(@(t,y) Rocket_2DOF(t,y,c), tspan, y0);
    
    vi = Yi(end,1);
    ri = Yi(end,2);
    mi = Yi(end,3);
    gi = Yi(end,4);
    ai = Yi(end,5);
    
    T1 = [T1; Ti];
    Y1 = [Y1; Yi];
end

 v1 = vi;
 r1 = ri;
 %ma1 = Y1(end,3);
 gamma1 = gi;
 angle1 = ai;
 t1 = tc_1;


 
%%  Zwischenflugphase Stufentrennung
t_sep = 6; % [s]

c = [cw_1 A_1 K 0 0 tc_1+t_sep r0 0];
tspan = [t1 t1+t_sep];
y0 = [v1 r1 m0-mn_1 gamma1 angle1];

[Ts,Ys] = ode15s(@(t,y) Rocket_2DOF(t,y,c), tspan, y0);


vs = Ys(end,1);
rs = Ys(end,2);
ms = Ys(end,3);
gammas = Ys(end,4);
angles = Ys(end,5);
ts = t1+t_sep;

%% Oberstufe


tm2 = [ts; t_oben; ts+tc_2];

y1 = [vs rs m0-mn_1 gammas angles];

vi = vs;
ri = rs;
mi = m0-mn_1;
gi = gammas;
ai = angles;

T2 = [];
Y2 = [];

for i = 1:length(tm2)-1
    c = [cw_2 A_2 K mp_2 F_2 ts+tc_2 r0 alpha2(i)];
    tspan = [tm2(i) tm2(i+1)];
    y0 = [vi ri mi gi ai];
    
    [Ti,Yi] = ode15s(@(t,y) Rocket_2DOF(t,y,c), tspan, y0);
    
    vi = Yi(end,1);
    ri = Yi(end,2);
    mi = Yi(end,3);
    gi = Yi(end,4);
    ai = Yi(end,5);
    
    T2 = [T2; Ti];
    Y2 = [Y2; Yi];
end


%% Combine Sim-Data

T = [T1; Ts; T2];
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

subplot(6,1,1)
plot(T,acc,'b-')
ylabel ('Acceleration [m/s^2]')
xlabel ('Time [s]')
xline(tc_1, '-black', {'Stage seperation'});
xline(tc_1+t_sep, '-r', {'Upper stage ignition'});

subplot(6,1,2)
plot (T,Y(:,1),'r-')
ylabel ('Velocity [m/s]')
xlabel ('Time [s]')
xline(tc_1, '-black', {'Stage seperation'});
xline(tc_1+t_sep, '-r', {'Upper stage ignition'});

subplot(6,1,3)
plot (T,Y(:,2)-r0,'b-')
ylabel ('Altitude [m]')
xlabel ('Time [s]')
xline(tc_1, '-black', {'Stage seperation'});
xline(tc_1+t_sep, '-r', {'Upper stage ignition'});

subplot(6,1,4)
plot (T,Y(:,4)*180/pi)
ylim([-90 90])
ylabel ('gamma [°]')
xlabel ('Time [s]')
xline(tc_1, '-black', {'Stage seperation'});
xline(tc_1+t_sep, '-r', {'Upper stage ignition'});

% subplot(6,1,5)
% plot (T,Y(:,3))
% ylabel ('Mass [kg]')
% xlabel ('Time [s]')
% xline(tc_1, '-black', {'Stage seperation'});
% xline(tc_1+t_sep, '-r', {'Upper stage ignition'});

umfang_E = r0 * 2 * pi;
groundtrack = (umfang_E * rad2deg(Y(:,5)))/360 * 10^-3;

subplot(6,1,5)
plot (T,groundtrack)
ylabel ('Ground Track [km]')
xlabel ('Time [s]')
xline(tc_1, '-black', {'Stage seperation'});
xline(tc_1+t_sep, '-r', {'Upper stage ignition'});


% deltav=diff(Y(:,1));
% deltar=diff(Y(:,2));
% deltam=diff(Y(:,3));
% deltag=diff(Y(:,4));
% deltat=diff(T);
% lt=length(T);
% for j=1 : lt-1
%     [~,~,~,rho]=atmosisa(deltar(j)-r0);
%     Fw = 0.5 * rho * deltav(j)^2 * A_1 * cw_1;
%     g = K / deltar(j)^2;
%     alpha(j)= acosd(((deltat(j) + g * sin(deltag(j)))*deltam(j) + Fw)/F_1);
% end
% 
% alpha(lt)=alpha(lt-1);
% 
% if length (alpha) > lt
%     la=length(alpha);
%     alpha(lt+1:la)=[];
% end

global alp gas;

alp = sortrows(alp,1);
gas = sortrows(gas,1);

subplot(6,1,6)
plot(gas(:,1),gas(:,2),'b-');
ylabel ('gamma_{soll} [°]')
xlabel ('Time [s]')
xline(tc_1, '-black', {'Stage seperation'});
xline(tc_1+t_sep, '-r', {'Upper stage ignition'});

subplot(6,1,6)
plot(alp(:,1),alp(:,2),'b-');
ylabel ('alpha [°]')
xlabel ('Time [s]')
xline(tc_1, '-black', {'Stage seperation'});
xline(tc_1+t_sep, '-r', {'Upper stage ignition'});


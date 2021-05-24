function dy = Rocket_2DOF(t,y,c)

global alp gas

r0=6371000;

cw = c(1);
A = c(2);
K = c(3);
mp = c(4);
F0 = c(5);
tc = c(6);
alpha = c(8);




F = F0;
gimbal_max = 50;



%% Atmospheric data at r from ISA atmosphere model (Aerospace Toolbox) 

if (y(2) - r0) < 84000 && (y(2) - r0) > 0
    [T,a,P,rho]=atmoscoesa(y(2)-r0);
else
    P = 0;
    rho = 0;
end

%% Berechnung der variablen (nicht integrierte Variablen)

%TODO
Fs = F0;
g = K / y(2)^2;
Fw = 0.5 * rho * y(1)^2 * A * cw;



tct = 494.2420;
v_vert_soll = (200000 - (y(2)-r0))/(tct-t);

v_vert = y(1) * sin(y(4));

dv_vert = (v_vert_soll-v_vert);
gamma_soll = asin(dv_vert/y(1));

if isreal(gamma_soll)
    rate = (y(4) - gamma_soll);
else
    gamma_soll = 0;
end
    





%% Bahnberechnung

if alpha == 99
    alpha = 0;
   
    if gamma_soll ~0;
        midterm = (K/y(2)^2/y(1) - y(1)/y(2)) ;
        alpha  = - asind((rate + midterm * cos(y(4))) * y(3)* y(1) / Fs);
    end

end




% Set Matrix of results to zero
dy=zeros(5,1);

% 1 Velocity
% 2 Radius
% 3 Mass
% 4 Gamma
% 5 Erdwinkel

if t>tc
    F=0;
    mp=0;
    alpha=0;
end

if alpha > gimbal_max
    alpha = gimbal_max;
elseif alpha < - gimbal_max
    alpha = -gimbal_max;
end

if y(2) >= r0 
    dy(1) = (Fs * cosd(alpha) - Fw)/y(3) - g * sin(y(4));
    dy(2) = y(1) * sin(y(4));
    dy(5) = y(1)/y(2)*cos(y(4));
else
    dy(1) = 0;
    dy(2) = 0;
    dy(5) = 0;
 
end

if (y(2)-r0) > 0.1
   dy(4) = (Fs * sind(alpha)) / (y(3) * y(1)) - (g / y(1) - y(1)/y(2)) * cos(y(4));
else
    dy(4) = 0;
end

dy(3) = -mp;
    


alp(end+1,:) = [t alpha];
gas(end+1,:) = [t rad2deg(gamma_soll)];


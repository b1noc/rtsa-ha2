function dy = Rocket_2DOF(t,y,c)

%global cw ca A K mpunkt_quer F_quer tc alpha lrampe r0

cw = c(1);
ca = c(2);
A = c(3);
K = c(4);
mp = c(5);
F_0 = c(6);
tc = c(7);
r0 = c(8);
dF = c(9);

alpha = 0;

% Atmospheric data at r from ISA atmosphere model (Aerospace Toolbox) 

if (y(2) - r0) < 84000 && (y(2) - r0) > 0
    [T,a,P,rho]=atmoscoesa(y(2)-r0);
else
    P = 0;
    rho = 0;
end


F = F_0 + dF * ((101325 - P)/101325) * F_0;
% Thrust switched on as long as burning time of engine



% Set Matrix of results to zero
dy=zeros(5,1);

% Gravity turn
if (y(2) - r0) > 1000 && (y(2) - r0) < 1500
    alpha = - deg2rad(4);
end

% target orbit trajectory
if (y(2)  - r0) > 80000)
    dh = 200000 - y(2) - r0;
    gamma_opt = - acos(dh/y(1)/tc) +  deg2rad(90);
    alpha = asin( (K/(y(2)^2 * y(1)) - y(1)/y(2)) * cos(gamma_opt) - ((rho*ca*A/2*y(1)*y(1))/y(2)/y(1))) * y(2) * y(1) / F;
end

%{
    
elseif (y(2) - r0) > 20000 && (y(2) - r0) < 30000
    alpha = deg2rad(0);
elseif (y(2) - r0) > 30000 && (y(2) - r0) < 40000
    alpha = deg2rad(0);
elseif (y(2) - r0) > 40000 && (y(2) - r0) < 50000
    alpha = deg2rad(0);
elseif (y(2) - r0) > 50000 && (y(2) - r0) < 80000
    alpha = deg2rad(0);
elseif (y(2) - r0) > 80000 && (y(2) - r0) < 120000
    alpha = deg2rad(0);
elseif (y(2) - r0) > 120000 && (y(2) - r0) < 150000
    alpha = deg2rad(0);
elseif (y(2) - r0) > 150000
    alpha = deg2rad(0);
else
    alpha = 0;
end
%}

% if y(2) > (180000 + r0) 
%      alpha = asin( (K/(y(2)^2 * y(1)) - y(1)/y(2)) * cos(y(4)) - ((rho*ca*A/2*y(1)*y(1))/y(2)/y(1))) * y(2) * y(1) / F;
% end

%if y(2) > (150000 + r0) && y(4) > 0
%     alpha = asin(gammarate + (K/(y(2)^2 * y(1)) - y(1)/y(2)) * cos(y(4)) - ((rho*ca*A/2*y(1)*y(1))/y(2)/y(1))) * y(2) * y(1) / F;
%end



if t>tc
    F=0;
    mp=0;
    alpha=0;
end


    
% Calculate derivatives of velocity, radius and ground angle for alitudes
% above ground
if y(2)>=r0
    dy(1)=(F*cos(alpha)-rho*cw*A/2*y(1)*y(1))/y(3)-K/y(2)/y(2)*sin(y(4));
    dy(2)=y(1)*sin(y(4));
    dy(5)=y(1)/y(2)*cos(y(4));
else
    dy(1)=0;
    dy(2)=0;
    dy(5)=0;
end
% Calculate derivative of flight path angles for altitudes above ramp
% length

% Calculate derivative of mass   
dy(3)=-mp;

if y(2)>=(r0+0.1)
    dy(4)=(F*sin(alpha)+rho*ca*A/2*y(1)*y(1))/y(3)/y(1)-(K/y(2)/y(2)/y(1)-y(1)/y(2))*cos(y(4));
else
    dy(4)=0;  
end



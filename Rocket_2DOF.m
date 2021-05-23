function dy = Rocket_2DOF(t,y,c)

%global cw ca A K mpunkt_quer F_quer tc alpha lrampe r0
persistent t_grav

cw = c(1);
ca = c(2);
A = c(3);
K = c(4);
mp = c(5);
F_0 = c(6);
tc = c(7);
r0 = c(8);
dF = c(9);
mleer = c(10);

alpha = 0;

%TODO: alpha max as property c(10) = 5
alpha_max = 30; % degree
gamma_grav = 85; % degree
alpha_grav = 5;

% Atmospheric data at r from ISA atmosphere model (Aerospace Toolbox) 

if (y(2) - r0) < 84000 && (y(2) - r0) > 0
    [T,a,P,rho]=atmoscoesa(y(2)-r0);
else
    P = 0;
    rho = 0;
end


%F = F_0 + dF * ((101325 - P)/101325) * F_0;
% Thrust switched on as long as burning time of engine
F = F_0;


% Set Matrix of results to zero
dy=zeros(5,1);



% Gravity turn
if (y(2) - r0) > 100  && (y(2) - r0) < 300
    alpha = -.1;
end
    
    
    %&& isempty(t_grav) 
%     turnRate = sind(alpha_grav) * F / (y(3) * y(1)) - (K/(y(2)^2 * y(1))- y(1)/y(2)) * cos(y(4))
%     %dturn = deg2rad(90-gamma_grav)/turnRate
%     alpha = asin(turnRate + (K/(y(2)^2 * y(1))- y(1)/y(2)) * cos(y(4)) * y(2) * y(1) / F)
% end



% if ~isempty(t_grav)  && t_grav(1) < ts && ts < t_grav(2) && (y(2) - r0) < 3000
%   
%     alpha = (alpha_grav);
% end

% target orbit trajectory
%TODO: 80 000



timeToBurnout = (y(3)-mleer) / mp;
dh = 200000 - (y(2)-r0);
v_vert_traj = dh/ (timeToBurnout);

v_vert_traj/y(1);
if (y(2)  - r0) > 80000 && v_vert_traj > 0
    gamma_traj = asind(v_vert_traj/y(1)) ;
    dGamma = rad2deg(y(4)) - gamma_traj;
    alpha = - dGamma ;
%     
% elseif (y(2)  - r0) > 200000 && v_vert_traj < 0 && tc > 200
%     gamma_traj = asind(v_vert_traj/y(1)) 
%     dGamma = rad2deg(y(4)) - gamma_traj
%     alpha = dGamma ;
end



if t>tc
    F=0;
    mp=0;
    alpha=0;
end

if alpha > alpha_max 
   alpha = alpha_max;
elseif alpha < - alpha_max
    alpha = - alpha_max;
end

    
% Calculate derivatives of velocity, radius and ground angle for alitudes
% above ground
if y(2)>=r0
    dy(1)=(F*cosd(alpha)-rho*cw*A/2*y(1)*y(1))/y(3)-K/y(2)/y(2)*sin(y(4));
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
    dy(4)=F*sind(alpha)/y(3)/y(1) - (K/y(2)^2/y(1) - y(1)/y(2) )*cos(y(4));
else
    dy(4)=0;  
end
    



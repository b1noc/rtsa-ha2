function dy = Rocket_2DOF(t,y,c)

%global cw ca A K mpunkt_quer F_quer tc alpha lrampe r0

cw = c(1);
ca = c(2);
A = c(3);
K = c(4);
mp = c(5);
F = c(6);
tc = c(7);
r0 = c(8);
gammarate = -0.0025;


% Thrust switched on as long as burning time of engine
if t>tc
    F=0;
    mp=0;
end

% Atmospheric data at r from ISA atmosphere model (Aerospace Toolbox) 

if (y(2) - r0) < 84000 && (y(2) - r0) > 0
    [T,a,P,rho]=atmoscoesa(y(2)-r0);
else
     rho = 0;
end

% Set Matrix of results to zero
dy=zeros(5,1);

% if y(2) > (r0 + 50000)
%     P = 0;
%     rho = 0;
% end




if y(1) > 100 && y(1) < 200
    rate = 1;
    
elseif y(2) > (180000 + r0) && y(4) > 0
    rate = 20;
%     
% elseif y(2) < 200000 && y(4) < 0
%     rate = -5;
else
    rate = 0;
end

alpha = - rate * pi/180;    
    

% if y(2)>=(r0+0.1)
%     alpha = asin(gammarate + (K/(y(2)^2 * y(1)) - y(1)/y(2)) * cos(y(4)) - ((rho*ca*A/2*y(1)*y(1))/y(2)/y(1))) * y(2) * y(1) / F;
% else
%     alpha = 0;
% end

    
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




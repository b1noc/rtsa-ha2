function dy = ha2_engine(t,y, c)


cw = c(1);
ca = c(2);
A = c(3);
K = c(4);
mp = c(5);
F = c(6);
tc = c(7);
r0 = c(8);

alpha = 0;

% Thrust switched on as long as burning time of engine
if t>tc
    F=0;
    mp=0;
end
% Atmospheric data at r from ISA atmosphere model (Aerospace Toolbox) 
[T,a,P,rho]=atmosisa(y(2)-r0);
% Set Matrix of results to zero
dy=zeros(5,1);
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
% Calculate derivative of mass   
dy(3)=-mp;
% Calculate derivative of flight path angles for altitudes above ramp
% length

dy(4)=(F*sin(alpha)+rho*ca*A/2*y(1)*y(1))/y(3)/y(1)-(K/y(2)/y(2)/y(1)-y(1)/y(2))*cos(y(4));

end







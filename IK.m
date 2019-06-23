function q = IK(x,y,z,a,b,c)
%This function is used for inverse kinematics calculations.

%link lenghts in mm


d1=0.4865;
d2=0.150;
d3=0.475;
d4=0.600;
d6=0.065;


alpha = [ 0,-pi/2, 0, -pi/2, pi/2, -pi/2];
% a = [d2 d3 0 0 0 0];
% d = [d1 0 0 d4 0 d6];

%%%%%%%%%%%%%%%%%Inverse kinematics%%%%%%%%%%%%%%%%%

R06= [cos(a)*cos(b), cos(a)*sin(c)*sin(b)-sin(a)*cos(c), cos(a)*sin(b)*cos(c)+sin(a)*sin(c);
      sin(a)*cos(b), sin(a)*sin(b)*sin(c)+cos(a)*cos(c), sin(a)*sin(b)*cos(c)-cos(a)*sin(c);
      - sin(b),      cos(b)*sin(c),                      cos(b)*cos(c)];


p06=[x y z];
p04=p06-transpose(d6*R06*[0;0;1]);

xc=p04(1);
yc=p04(2);
zc=p04(3);



%Inverse kinematic equations for position
theta1 = atan2(yc,xc);

r = (xc^2 + yc^2)^(1/2);
COS = ((r - d2)^2 + (zc - d1)^2 - d3^2 - d4^2)/(2*d3*d4);

theta3_dash = atan2((-(1 - COS^2)^(1/2)), COS);
theta3 = theta3_dash;
theta33 = abs(theta3_dash) - (90*pi/180);

theta2_dash = (atan2( zc-d1, r-d2) - atan2(d4*sind(theta3_dash*180/pi),d3+d4*cosd(theta3_dash*180/pi))) ;
theta2 = theta2_dash;
theta22 = (90*pi/180) - theta2_dash;

theta5 = atan2(sqrt(1-(sin(theta1)*R06(1,3)- cos(theta1)*R06(2,3))^2),sin(theta1)*R06(1,3) - cos(theta1)*R06(2,3)) -pi;
% theta4 = atan2(-cos(theta1)*sin(theta22+theta33)*R06(1,3)-sin(theta1)*sin(theta22+theta33)*R06(2,3)+cos(theta22+theta33)*R06(3,3),cos(theta1)*cos(theta22+theta33)*R06(1,3)+sin(theta1)*cos(theta22+theta33)*R06(2,3)+sin(theta22+theta33)*R06(3,3));
theta4 = atan2(-cos(theta1)*sin(theta2+theta3)*R06(1,3)-sin(theta1)*sin(theta2+theta3)*R06(2,3)+cos(theta2+theta3)*R06(3,3),cos(theta1)*cos(theta2+theta3)*R06(1,3)+sin(theta1)*cos(theta2+theta3)*R06(2,3)+sin(theta2+theta3)*R06(3,3));
theta6= atan2(sin(theta1)*R06(1,2)-cos(theta1)*R06(2,2),-sin(theta1)*R06(1,1)+cos(theta1)*R06(2,1));

%%Theta 4 изменен

% R30 = [cos(theta1).*cos(theta2+theta3), -cos(theta1).*sin(theta2+theta3), sin(theta1);
%  sin(theta1).*cos(theta2+theta3), -sin(theta1).*sin(theta2+theta3), cos(theta1);
%  -sin(theta2+theta3), -cos(theta2+theta3), 0 ] ;

R03=[ cos(theta1)*cos(theta2)*cos(theta3) - cos(theta1)*sin(theta2)*sin(theta3), - cos(theta1)*cos(theta2)*sin(theta3) - cos(theta1)*cos(theta3)*sin(theta2), -sin(theta1);
 cos(theta2)*cos(theta3)*sin(theta1) - sin(theta1)*sin(theta2)*sin(theta3), - cos(theta2)*sin(theta1)*sin(theta3) - cos(theta3)*sin(theta1)*sin(theta2),  cos(theta1);
     - cos(theta2)*sin(theta3) - cos(theta3)*sin(theta2),         sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3),   0];

 RT03= transpose (R03);
 
R36 = RT03 * R06 ;
g11 = R36 (1,1);
g12 = R36 (1,2);
g23 = R36 (2,3);
g13 = R36 (1,3);
g22 = R36 (2,2);
g33 = R36 (3,3);
g21 = R36 (2,1);

Q5 = atan2 ( sqrt(1-g23^2), g23)-pi/2;
if(Q5 == 0)
    Q5=0.0000001;
%     Q4= 0;
%     Q5= 0;
%     Q6 = atan2 (-g12, g11);
elseif (Q5 == pi)
    Q5=pi+0.0000001;
%     Q4= 0;
%     Q5= 0;
%     Q6 = atan2 (g12,-g11);
else
    Q4 = atan2 (g33/ sin (Q5), - g13/ sin (Q5));
    Q6 = atan2 (g22/ -sin (Q5), g21/ sin (Q5));
end

q(1)=theta1;
q(2)=theta2;
q(3)=theta3;
q(4)=Q4;
q(5)=Q5;
q(6)=Q6;


end

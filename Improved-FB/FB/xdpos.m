function [ RT ] = xdpos( a,b )
c=[1;0;0];
theta(1)=atan2(a(2),a(1))*180/pi;
theta(2)=-atan2(a(3),(a(1)^2+a(2)^2)^0.5)*180/pi;


rt=rotz(theta(1))*roty(theta(2));
o=inv(rt)*a;


theta(1)=atan2(b(2),b(1))*180/pi;
theta(2)=-atan2(b(3),(b(1)^2+b(2)^2)^0.5)*180/pi;
rt1=rotz(theta(1))*roty(theta(2));
y=rt1*o;
RT=rt1*inv(rt);%aµ½bµÄ±ä»»¾ØÕó

end


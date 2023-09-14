function [theta] = position_conversion_angle(en_dafter)
cc=eye(3);
theta=zeros(20,1);
m=size(en_dafter,2)-1;
for i=1:2:(m-1)
 d=inv(cc)*(en_dafter(:,i+2)-en_dafter(:,i));
 

     theta(i)=atan2(d(2),d(1))*180/pi;
     theta(i+1)=atan2(d(3),(d(1)^2+ d(2)^2)^0.5)*180/pi;
     cc=cc*rotz(theta(i))*roty(-theta(i+1));

end
end


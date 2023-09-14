function [En_dafter Theta] = inversekinematics(primepoints,T_edps,Rxb,zhixiang,limit)
m=size(primepoints,2)-1;
en_dafter=primepoints;
len=Rxb.Link_length;
L=zeros(3,m+1);
theta= [0, 0, 0, 0, 0, 0, 0, 0,0,0,0, 0, 0, 0, 0, 0, 0, 0,0,0]';
%%
%ºá¹ö
theta=position_conversion_angle(primepoints);
for i=1:100
if (norm(en_dafter(:,m+1)'-T_edps)>0.01&&i<80)
%    ttt=  norm(en_dafter(:,m+1)'-T_edps)
 T_matrix2 = Forward_Kinematics(theta',Rxb);  
  henggun_Z_zhou_4wei=cell2mat(T_matrix2(m))*[200;0;100;1]-cell2mat(T_matrix2(m))*[200;0;0;1];
  henggun_Z_zhou= henggun_Z_zhou_4wei(1:3,:);
en_dafter=FB1(len,T_edps,en_dafter,henggun_Z_zhou,zhixiang,limit);
theta=position_conversion_angle(en_dafter);
end
end
En_dafter=en_dafter;
theta=position_conversion_angle(en_dafter);
 T_matrix1 = Forward_Kinematics(theta',Rxb);  
ENDPOS1=cell2mat(T_matrix1(m))*[Rxb.Link_length(end);0;0;1]

Theta=theta;
end


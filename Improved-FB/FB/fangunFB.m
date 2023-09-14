    function [en_dafter theta ] = fangunFB(LL,thetaa,T_edps,Rxb ,limit)
%参数

   LL=190;
thetaa=360;
m=size(Rxb.Link_length,2)*2;%m=10个自由度
alpha=thetaa/(m/2-1);%每段对应的圆心角

R=(200^2-LL^2)^0.5/2 /sind(alpha/2) ;
a=zeros(1,m);%臂长0,2,4,6,8
FGen_dafter=zeros(3,m+1);%虚拟关节位置12,34,56,78,910,11
Gen_dafter=zeros(3,m+1);%真实关节起始位置12,34,56,78,910,11
Y0=(-cosd(-90-alpha/2+m/2*alpha)+cosd(-90-alpha/2))*R;%变基线起始点
Z0=(sind(-90-alpha/2+m/2*alpha)-sind(-90-alpha/2))*R;

%T_edps5=T_edps;
FBlen=((LL*m/2)^2+Y0^2+Z0^2)^0.5/m*2;%虚拟关节长度
T_edps5=T_edps;
T_edps0=[0;0;0];
%虚拟臂赋位置初值
%
%虚拟臂关节点赋值
for i=1:(m+1)
    FGen_dafter(1,i)=(fix(i/2)+mod(i,2)-1)*LL;
   FGen_dafter(2,i)=Y0/m*(fix(i/2)+mod(i,2)-1)*2;
   FGen_dafter(3,i)=Z0/m*(fix(i/2)+mod(i,2)-1)*2;
   Gen_dafter(1,i)=(fix(i/2)+mod(i,2)-1)*LL;
   Gen_dafter(2,i)=(-cosd(-90-alpha/2+(fix(i/2)+mod(i,2)-1)*alpha)+cosd(-90-alpha/2))*R;
   Gen_dafter(3,i)=(sind(-90-alpha/2+(fix(i/2)+mod(i,2)-1)*alpha)-sind(-90-alpha/2))*R;
end
FGAen_dafter=FGen_dafter;
%%
for i=1:50
   ra=800;
     path(:,i)=[900 ra*sin((135+i)/180*pi)-1200 ra*cos((135+i)/180*pi)+600];
end

 lamuda=0;
 
for i=1:60
    T_edps5=path(:,i)'+[-R*2/3 0 -R*2/3] ;
     lamuda=0.5*i/49;
    zhixiang=(1-lamuda)*(path(:,i)'-path(:,i+1)')+lamuda*(path(:,1)'-path(:,i+1)');
FGAen_dafter=FGFB(FGAen_dafter,FBlen,T_edps0,T_edps5,zhixiang,limit);%前进后退法求虚拟关节点
en_dafter=JJD(FGen_dafter,FGAen_dafter,Gen_dafter);%解真实关节臂坐标点
theta=position_conversion_angle(en_dafter);% 解角度
%huatu
 clf   
 for j=1:49
 X1=path(:,j)' ; 
 X2=path(:,j+1)';  
 r=140;  
 n=300;  
cyl_color='b';  
 closed=0;  
 cylinder3(X1,X2,r,30,cyl_color,0,0,0.3); 
 hold on
 end
 T_matrix1 = Forward_Kinematics(theta',Rxb);
DrawConfigure(T_matrix1,Rxb);
  view(-45,30)
 pause(0.001);
frames(i)=getframe(gcf);
end
gif(frames);
% view(15,15)
%     T_matrix1 = Forward_Kinematics(theta',Rxb) 
%     DrawConfigure(T_matrix1,Rxb);
% 
%   t_endpos2=cell2mat(T_matrix1(m))*[200;0;0;1];%4维
%   b1=cell2mat(T_matrix1(m))*[Rxb.Link_length(end);0;0;1];
%  b2=cell2mat(T_matrix1(m))*[Rxb.Link_length(end);1;0;1];
% b3=b1-b2;
% fgj=atan(b3(3)/((b3(1)^2+b3(2)^2)^0.5))*180/pi;
% 

end


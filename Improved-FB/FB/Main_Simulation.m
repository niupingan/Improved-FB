clc
clear all
format short
set(0,'defaultfigurecolor','w')
%%参数配置（set(0,'defaultfigurecolor','w')图像背景）
Rxb.Segment_num = 10;%段数+1
Rxb.Segment_Joint_num = 2;      %关节段中小关节的个数
Rxb.Link_length = [ 200 200 200 200 200 200 200 200 200 200];%小关节的长度
Rxb.Diameters = [55 55 55 55 55 55 55 55 55 55 55 55];  %小关节的直径
Rxb.Base_Rot_Mat = eye(4);      %使整个柔性臂按一定角度旋转，先不旋转
Rxb.JGT_Rot_Mat = [rotx(90),[0 0 0]';[0 0 0 1]];%先不旋转
m = Rxb.Segment_Joint_num*Rxb.Segment_num; %%总的关节的个数
Theta_Limit = ones(Rxb.Segment_num*2+1,1)*45;
Theta0 = [0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,0,0,0,0,0,0,0,0]';       %%伸直状态
T_matrix = Forward_Kinematics(Theta0',Rxb);
T_endpos=cell2mat(T_matrix(m))*[Rxb.Link_length(end);0;0;1];%末端位置4维
%DrawConfigure(T_matrix,Rxb);

% Theta0 = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]';%初始角度
Theta0=[-0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
T_matrix = Forward_Kinematics(Theta0',Rxb);
T_endpos=cell2mat(T_matrix(m))*[Rxb.Link_length(end);0;0;1];%四维
%DrawConfigure(T_matrix,Rxb);
en_dpointsprime=ENDPOINTS(T_matrix,T_endpos);%包括m+1末端位置
% T_endpos=[1607.6;349.8;299.8];
%%
%路径规划
limit=800;
for i=1:50
   ra=800;
    path(:,i)=[900 ra*sin((135+i)/180*pi)-1200 ra*cos((135+i)/180*pi)+600];
end
 lamuda=0;
for i=1:49
    T_endpos=path(:,i)' ;
    lamuda=0.1*i/49;

    zhixiang=(1-lamuda)*(path(:,i)'-path(:,i+1)')+lamuda*(path(:,1)'-path(:,i+1)');

  [en_dpointsprime theta]=inversekinematics(en_dpointsprime,T_endpos,Rxb,zhixiang,limit);
T_matrix1 = Forward_Kinematics(theta',Rxb);
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
  DrawConfigure(T_matrix1,Rxb);
  view(-45,30)
 pause(0.001);
frames(i)=getframe(gcf);
end
hold on
% T_matrix1 = Forward_Kinematics(theta',Rxb);
% aaa=[0;0;0;0];
% aaa=cell2mat(T_matrix1(m))*[Rxb.Link_length(end);0;0;1];
% T_endpos=aaa(1:3)
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%henggun_Zzz_zhou_4wei=cell2mat(T_matrix1(m))*[200;0;1;1]-cell2mat(T_matrix1(m))*[200;0;0;1]
%  [en_dpointsprime theta]=inversekinematics1(en_dpointsprime,T_endpos,Rxb,zhixiang,limit);
%  T_matrix1 = Forward_Kinematics(theta',Rxb);
%  DrawConfigure(T_matrix1,Rxb);
 %动态图
%  gif(frames);
% fps = 40;
% myVideo = VideoWriter('test2.avi'); 
% myVideo.FrameRate = fps; 
% open(myVideo); 
% figure
% for k = 1:50
%     frame = getframe(gcf);
%     im = frame2im(frame); 
%     writeVideo(myVideo,im); 
% end
% close(myVideo);




   
 

   
















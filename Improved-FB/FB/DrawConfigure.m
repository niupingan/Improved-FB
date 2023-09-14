                    function  DrawConfigure(mT,RXB_Parameter)
Sg_num=RXB_Parameter.Segment_num;
Sg_Joint_num=RXB_Parameter.Segment_Joint_num;
Radius=RXB_Parameter.Diameters*0.5;
en_dlence=RXB_Parameter.Link_length(end);

m=size(mT,2);
GOAL=cell2mat(mT(m))*[en_dlence;0;0;1];%ª˙–µ±€ƒ©∂À

x_axis = zeros(1,m+1);
y_axis = zeros(1,m+1);
z_axis = zeros(1,m+1);
   x_axis(m+1) =GOAL(1);
    y_axis(m+1) =GOAL(2);
    z_axis(m+1) =GOAL(3);
for i = 1:m
    x_axis(i) =mT{i}(1,4);
    y_axis(i) =mT{i}(2,4);
    z_axis(i) =mT{i}(3,4);
end

% Color1='-rs';
% Color2='-ms';
% Color3='-gs';
% Color4='-ks';
% Color5='-gs';
% Color6='-ys';
% Color7='-rs';

Color1='r';
Color2='g';
Color3='m';
Color4='y';
Color5='g';
Color6='y';
Color7='r';
Color8='g';
Color9='y';
Color10='r';
for segment_num=1:1:Sg_num
 Strat_num=(segment_num)*Sg_Joint_num;
 End_num=segment_num*Sg_Joint_num+1; 

 for i=(Strat_num+1):End_num
      A=[ x_axis(i), y_axis(i), z_axis(i)];
      B=[ x_axis(i-1), y_axis(i-1), z_axis(i-1)];
      if(norm(A-B)>(10^-6))   
      C=(A-B)/norm(A-B);
      X1=B+C*8;
      X2=A-C*8;
     cylinder3(X1,X2,Radius(segment_num),20,eval(['Color',int2str(segment_num)]),1,0,0.3); 
     hold on 
     end
     
 end
%  plot3(x_axis(Strat_num:End_num),y_axis(Strat_num:End_num),z_axis(Strat_num:End_num),eval(['Color',int2str(segment_num)]),'LineWidth',2.5,...
%                 'MarkerEdgeColor','k',...
%                 'MarkerFaceColor','g',...
%                 'MarkerSize',3)
%             hold on
end   

Plot_Coordinate_System(mT{m},100,GOAL);
hold on
xlabel('X÷·(mm)');
ylabel('Y÷·(mm)');
zlabel('Z÷·(mm)');
     
grid on;
axis square
axis equal

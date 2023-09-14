function  Plot_Coordinate_System(T,Aixs_Length,GL)
 nx=T(1:3,1);   
 ny=T(1:3,2);                 
 nz=T(1:3,3); 
 P=GL(1:3); 
 x_axis(1)=P(1);
 y_axis(1)=P(2);
 z_axis(1)=P(3);
 
Color1='-r';
Color2='-g';
Color3='-b';
for axis_num = 1:1:3
 if axis_num == 1
    P_end = Aixs_Length*nx+GL(1:3); 
 elseif  axis_num == 2
    P_end = Aixs_Length*ny+GL(1:3);      
 else
    P_end = Aixs_Length*nz+GL(1:3);       
 end
 
 x_axis(2) = P_end(1);
 y_axis(2) = P_end(2);
 z_axis(2) = P_end(3);

 plot3(x_axis(1:2),y_axis(1:2),z_axis(1:2),eval(['Color',int2str(axis_num)]),'LineWidth',1.5)
 hold on
end
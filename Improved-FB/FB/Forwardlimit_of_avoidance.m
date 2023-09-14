function [En_dafter] = Forwardlimit_of_avoidance(en_dafter,limit)
m=size(en_dafter,2)-1;
 lamda=zeros(1,m);
b=zeros(3,m);
  bichang=200;
 for i=m-2:-2:2
       L1=en_dafter(:,i+3)-en_dafter(:,i+1);
       L2=en_dafter(:,i)-en_dafter(:,i+1);%下边的b(:,m)原本的臂段指向
       oumiga=acos(dot(L2,L1)/(norm(L1)*norm(L2)))*180/pi;
       if(oumiga<180-limit)
           LZ=cross(L2,L1);
           L3=cross(LZ,L2);
            %避极限到的关节点
           en_dafter(:,i)=en_dafter(:,i+1)+bichang*cos(limit*pi/180)*(-L1)/norm(L1)+bichang*sin(limit*pi/180)*(L3)/norm(L3);
           en_dafter(:,i-1)=en_dafter(:,i);
      zhixiang=en_dafter(:,i)-en_dafter(:,i+1);%避极限后的臂段指向
      rt=xdpos(L2,zhixiang);
      if (i>2)
for j=1:i-2
    b(:,j)=en_dafter(:,j)-en_dafter(:,i+2);   
en_dafter(:,j)=rt*b(:,j)+en_dafter(:,i+2);
end
      end
       end     
 end  
 En_dafter=en_dafter;
end


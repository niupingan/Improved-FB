function [en_dafter] = backwardlimit_of_avoidance(en_dafter,limit)
m=size(en_dafter,2)-1;
  lamda=zeros(1,m);
  bichang=200;
b=zeros(3,m);
c=zeros(3,m);
c(:,1)=[1 0 0]';
 for i=3:2:m+1
       L1=en_dafter(:,i)-en_dafter(:,i-1);
       if(i==3)
           L2=[-bichang 0 0]';
       else
       L2=en_dafter(:,i-3)-en_dafter(:,i-1);%下边的b(:,m)
       end
              c(:,i)=-L2;%原来臂段指向
       oumiga=acos(dot(L2,L1)/(norm(L1)*norm(L2)))*180/pi;
       if(oumiga<180-limit)
           LZ=cross(L2,L1);
           L3=cross(LZ,L2);
            %避极限到的关节点
           en_dafter(:,i)=en_dafter(:,i-1)+bichang*cos(limit*pi/180)*(L1)/norm(L1)+bichang*sin(limit*pi/180)*(L3)/norm(L3);
           if (i<m+1)
           en_dafter(:,i+1)=en_dafter(:,i);
           else
                en_dafter(:,i)=en_dafter(:,i);
           end
      zhixiang=en_dafter(:,i)-en_dafter(:,i-1);%避极限后的臂段指向
      rt=xdpos(c(:,i),zhixiang);
     for j=i+2:m+1
      b(:,j)=en_dafter(:,j)-en_dafter(:,i-1);   
      en_dafter(:,j)=rt*b(:,j)+en_dafter(:,i-1);
      end
       end
 end
end






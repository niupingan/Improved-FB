function [en_dafter] = FB(len,T_edps,en_dafter,henggun_Z_zhou,zhixiang,limit)
m=size(en_dafter,2)-1;
lamda=zeros(1,m);
a=zeros(1,m);
b=zeros(3,m);
bichang=200;
for i=1:1:m
    if (mod(i,2)==0)
        a(i)=len(i/2);
    else
      a(i)=0 ; 
    end
end

%%前进
en_dafter(:,m+1)=T_edps;
for i=m:-1:1 
   lamda(i)=a(i)/(norm(en_dafter(1:3,i+1)-en_dafter(:,i))); 
   en_dafter(:,i)=(1-lamda(i))*en_dafter(:,i+1)+ lamda(i)*en_dafter(:,i);
    end

 en_dafter=Forwardlimit_of_avoidance(en_dafter,limit);% 避极限 

%-------------------------------------------------------指向旋转
b(:,m)=en_dafter(:,m)-en_dafter(:,m+1);
    rt=xdpos(b(:,m),zhixiang);
for i=1:m
    b(:,i)=en_dafter(:,i)-en_dafter(:,m+1);   
en_dafter(:,i)=rt*b(:,i)+en_dafter(:,m+1);
end

%------------------------------------------------------旋转2横滚
xuanzhuanzhixiang=en_dafter(:,m)-en_dafter(:,m+1);
danwei_xuanzhuan_zhixiang=xuanzhuanzhixiang/norm(xuanzhuanzhixiang);
rt_xuanzhuan=xdpos(henggun_Z_zhou,[0;-0.5;1]);
%横滚迭代
% for i=1:(m-1)
%      c(:,i)=en_dafter(:,i)-en_dafter(:,m)  
%      d(:,i)=en_dafter(:,m)+danwei_xuanzhuan_zhixiang*dot(danwei_xuanzhuan_zhixiang,c(:,i))
%      en_dafter(:,i)=rt_xuanzhuan*(en_dafter(:,i)-d(:,i))+d(:,i);
%    
% end

%后退
en_dafter(:,1)=[0 0 0];
for i=1:m
     lamda(i)=a(i)/(norm(en_dafter(1:3,i)-en_dafter(:,i+1)));
     en_dafter(:,i+1)=(1-lamda(i))*en_dafter(:,i)+ lamda(i)*en_dafter(:,i+1);
end

    en_dafter=backwardlimit_of_avoidance(en_dafter,limit);% 避极限 
  

end


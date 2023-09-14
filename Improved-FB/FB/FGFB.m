function [ en_dafterL ] = FGFB(en_daf,FBlen,T_edps0,T_edps5,zhixiang,limit  )
m=size(en_daf,2)-1;
lamda=zeros(1,m);
for i=1:1:m
    if (mod(i,2)==0)
        a(i)=FBlen;
    else
      a(i)=0 ; 
    end
end

for i=1:50
    if (norm(en_daf(:,m+1)'-T_edps5)>0.01&&i<50)
        %qianjin
en_daf(:,m+1)=T_edps5;
for i=m:-1:1
    
   lamda(i)=a(i)/(norm(en_daf(1:3,i+1)-en_daf(:,i)));
   en_daf(:,i)=(1-lamda(i))*en_daf(:,i+1)+ lamda(i)*en_daf(:,i);
end
en_daf=Forwardlimit_of_avoidance(en_daf,limit);
%xuanzhuan
b(:,m)=zhixiang;
    rt=xdpos(b(:,m),zhixiang);
for i=1:m
    b(:,i)=en_daf(:,i)-en_daf(:,m+1);   
en_daf(:,i)=rt*b(:,i)+en_daf(:,m+1);
end
%houtui
en_daf(:,1)=T_edps0;
for i=1:m
     lamda(i)=a(i)/(norm(en_daf(1:3,i)-en_daf(:,i+1)));
     en_daf(:,i+1)=(1-lamda(i))*en_daf(:,i)+ lamda(i)*en_daf(:,i+1);

end
 en_daf=backwardlimit_of_avoidance(en_daf,limit);
    end

end
en_dafterL=en_daf;
end



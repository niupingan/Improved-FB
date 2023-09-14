function [EPOINTS] = ENDPOINTS(mT,ENDPOS)
m=size(mT,2);
EPOINTS=zeros(3,m+1);
for i=1:1:m
    EPOINTS(:,i)=mT{i}(1:3,4);
end
EPOINTS(:,m+1)=ENDPOS(1:3);

end


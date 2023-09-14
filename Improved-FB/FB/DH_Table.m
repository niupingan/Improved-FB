%DH-table for segmented manipulator
function DH_Table = DH_Table(theta,Link_num_Of_RXB,L)
Tem_DH_Table= [
    0 90 0 theta % Link_num=2n
    L  -90  0 theta%2n+1
    0 0 0 theta      %    Link_num=1   
                 ];                
type_num=mod(Link_num_Of_RXB,2)+1;
if Link_num_Of_RXB==1
    type_num=3;
end
DH_Table=Tem_DH_Table(type_num,:);
 return
function Ti = Forward_Kinematics(theta,RXB_Parameter)
%�˲��������˶�ѧ

Sg_num=RXB_Parameter.Segment_num;
Sg_Joint_num=RXB_Parameter.Segment_Joint_num;
Link_length=RXB_Parameter.Link_length;
Base_Rotate_Mat=RXB_Parameter.Base_Rot_Mat;
Ti=cell(1,Sg_num*Sg_Joint_num);
m=Sg_num*Sg_Joint_num;
for Segment_num=1:1:Sg_num
    for i=1:1:Sg_Joint_num
        link_num=(Segment_num-1)*Sg_Joint_num+i;%�ӹؽں�
     Theta_num=link_num;
        DH_Para = DH_Table(theta(Theta_num),link_num,Link_length(Segment_num));%D-H������
        Tem_Ti = DH_Mat(DH_Para(1),DH_Para(2),DH_Para(3),DH_Para(4));%��α任����    
      
 if (link_num>1)
            Ti{link_num}=Ti{link_num-1}*Tem_Ti;%�ݳ�
        else
            Ti{link_num}=Base_Rotate_Mat*Tem_Ti;
        end
    end

end

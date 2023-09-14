function [theta1 flag] = InverseKinematics2(RXB_Parameter,T_end,Theta_Primary,Angle_limit,theta4flag)
format long
Segment_num = RXB_Parameter.Segment_num;
theta = Theta_Primary;
T_matrix = Forward_Kinematics(theta',RXB_Parameter);
m = size(T_matrix,2);
dPos_Old = 0;
dA_Old = 0;
count = 1;
flag = 1;
while(1)
    T_matrix = Forward_Kinematics(theta',RXB_Parameter);
    
    dPos = T_end(1:3,4)-T_matrix{1,m}(1:3,4);
    dA = 0.5*(cross(T_matrix{1,m}(1:3,1),T_end(1:3,1))+...
        cross(T_matrix{1,m}(1:3,2),T_end(1:3,2))+...
        cross(T_matrix{1,m}(1:3,3),T_end(1:3,3)));
    dPos_A = [dPos
        dA];
    J1 = Jacob_Mat(T_matrix,RXB_Parameter);
    Tem_theta2 = pinv(J1)*dPos_A;
    theta = Tem_theta2 + theta;
    
    if(theta4flag == 0)
        theta(7) = 0;
        theta(8) = 0;
    end
      
    Over_limit_Num = 0;      
    for theta_i = 1:(Segment_num*2)
        if (abs(theta(theta_i)) > Angle_limit(theta_i)) %%超出了关节角极限了
            if(theta(theta_i) > 0)
                theta(theta_i) = Angle_limit(theta_i);
            else
                theta(theta_i) = -Angle_limit(theta_i);
            end
        else
            Over_limit_Num=1;
        end
    end
      
    if (Over_limit_Num == 0)
        disp('角度大于20度，程序出错')
        flag = 0;
        disp(theta)
        break;
    end
    
    d_p = norm(dPos)
    d_A = norm(dA)
    if(((norm(dPos) < 0.2) && (norm(dA) < 0.01)))%判断是否结束
        flag = 1;
        break;
    end
    
    if(((norm(dPos-dPos_Old) < 0.0001) && (norm(dA-dA_Old) < 0.0001)) || (count == 20000)) % 判断是否有效
        if(((norm(dPos) < 0.5) && (norm(dA) < 0.01))) % 判断是否结束
            flag = 1;
        else
            flag = 0;
        end
        disp('迭代无效，退出')
        break;
    end
    
    dPos_Old = dPos;
    dA_Old = dA;
    count = count + 1;
    hold on
    if count%101=0;
%        plot3(T_matrix{end}(1,4),T_matrix{end}(2,4),T_matrix{end}(3,4),'b.','markersize',10);
    end
end %end while

theta1 = theta;
return
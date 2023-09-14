%DH-tansformation matrics 
function T = DH_Mat(len,alpha,offset,theta)
%degree->radian
theta_t = (theta/180)*pi;
alpha_t = (alpha/180)*pi;
lambda  = cos(alpha_t);
mu      = sin(alpha_t);
%--------------------------------------------------------------------------
T = [cos(theta_t)                -sin(theta_t)              0                   len
     sin(theta_t)*lambda   lambda*cos(theta_t)   -mu       0
     sin(theta_t)*mu   cos(theta_t)*mu    lambda        0                        
     0                           0                            0                   1];
 return
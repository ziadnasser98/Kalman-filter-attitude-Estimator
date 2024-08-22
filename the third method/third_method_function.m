function [xk_plus,Pk_plus,ak,roll,pitch] = third_method(xk_,Pk_,ak_,dt,ca,s,Gyrox,Gyroy,Gyroz,accx,accy,accz,Q,R)

%xk_ the state vector at time step k-1
%Pk_ the covariance matrix at time step k-1
%dt the sampling period 
%xk the state vector at time step k (prior estiamtion)
%xk_plus the state vector at time step k (posterior estimation)
%Pk the covariance matrix at time step k (prior estiamtion)
%Pk_plus the covariance matrix at time step k (posterior estiamtion)

% calculate the F_(k-1) and G_(k-1) matrices for the state equation:
% xk=F_(k-1)*xk_+G_(k-1)*w
g=9.87;
F_k=expm(-dt.*[0 -Gyroz Gyroy;Gyroz 0 -Gyrox;-Gyroy Gyrox 0]);           
H=g*eye(3,3);
G_k=-dt.*[0 -xk_(3) xk_(2);
         xk_(3) 0 -xk_(1);
        -xk_(2) xk_(1) 0];
    

% the prior estimate of the state vector and the state covariance matrix
xk=F_k*xk_';
xk=xk';
Pk=F_k*Pk_*F_k'+G_k*Q*G_k';
y=[accx;accy;accz];
ek=y-ca.*ak_-H*xk';
%calculate Kalman gain
if (s==1)
    segma=(ek*ek'-H*Pk*H')*R^(-1);
    l=max([segma(1,1),segma(2,2),segma(3,3),1]);
    segma=l.*eye(3,3);
    K=Pk*H'*(H*Pk*H'+segma*R)^(-1);
else
    segma=(ek'*ek-trace(H*Pk*H'))/trace(R);
    segma=max(segma,1);
    K=Pk*H'*(H*Pk*H'+segma.*R)^(-1);
end

%the posterior estimate of the state vector and the state covariance matrix
xk_plus = xk'+K*ek;
xk_plus=xk_plus';
Pk_plus=(eye(3,3)-K*H)*Pk*(eye(3,3)-K*H)'+K*R*K';
ak=y-H*xk_plus';
roll=atan2(xk_plus(2),xk_plus(3));
pitch=-asin(xk_plus(1));

end


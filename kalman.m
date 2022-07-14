clc;
clear all;
close all;

H=eye(3);%observation matrix
disp('H Matrix:');
disp(H);

sigma_1=0.25;
sigma_2=0.15;
I=eye(3);

Q=sigma_1*I;  % process error covariance
disp('Q matrix:');
disp(Q);

R=sigma_2*I;
disp('R matrix:'); % measurement error covariance
disp(R);

    
    

a=0.98;
A0=a*I;% state transition matrix
disp('A matrix:');
disp(A0);

b=10^-2;
P0=b*I;       % initial covariance
disp('P0:');
disp(P0);

x_a = 1.3;
x_b = 1.6;
r1 = (x_b-x_a).*rand(10,1) + x_a;

y_a = 1.4;
y_b = 1.8;
r2 = (y_b-y_a).*rand(10,1) + y_a;

z_a = 1.1;
z_b = 1.5;
r3 = (z_b-z_a).*rand(10,1) + z_a;



for j=1:10
    t1(j)=r1(j,1); 
     t2(j)=r2(j,1);
    t3(j)=r3(j,1);
    
    
end;

for k=1:10
    x(1,k)=t1(k);
    x(2,k)=t2(k);
    x(3,k)=t3(k);

end


X0=[1.5;1.7;1.3]; % home position 
X1=[x(1,1);x(2,1);x(3,1)];
X2=[x(1,2);x(2,2);x(3,2)];
X3=[x(1,3);x(2,3);x(3,3)];
X4=[x(1,4);x(2,4);x(3,4)];
X5=[x(1,5);x(2,5);x(3,5)];
X6=[x(1,6);x(2,6);x(3,6)];
X7=[x(1,7);x(2,7);x(3,7)];
X8=[x(1,8);x(2,8);x(3,8)];
X9=[x(1,9);x(2,9);x(3,9)];
X10=[x(1,10);x(2,10);x(3,10)];


% readings from the sensor as far as now generated 

Z1=awgn(X1,30,'measured');
Z2=awgn(X2,30,'measured');
Z3=awgn(X3,30,'measured');
Z4=awgn(X4,30,'measured');
Z5=awgn(X5,30,'measured');
Z6=awgn(X6,30,'measured');
Z7=awgn(X7,30,'measured');
Z8=awgn(X8,30,'measured');
Z9=awgn(X9,30,'measured');
Z10=awgn(X10,30,'measured');

disp('X0 assumed values of X0');
disp(X0);

disp('X1 assumed values of X1 ');
disp(X1);
disp('Z1 generated values of Z1');
disp(Z1);


disp('X2 assumed values of X2');
disp(X2);
disp('Z2 generated values of Z2');
disp(Z2);



disp('X3 assumed values of X3');
disp(X3);
disp('Z3 generated values of Z3');
disp(Z3);

disp('X4 assumed values of X4');
disp(X4);
disp('Z4 generated values of Z4');
disp(Z4);


disp('X5 assumed values of X5');
disp(X5);
disp('Z5 generated values of Z5');
disp(Z5);

disp('X6 assumed values of X6');
disp(X6);
disp('Z6 generated values of Z6');
disp(Z6);


disp('X7 assumed values of X7');
disp(X7);
disp('Z7 generated values of Z7');
disp(Z7);


disp('X8 assumed values of X8');
disp(X8);
disp('Z8 generated values of Z8');
disp(Z8);


disp('X9 assumed values of X9');
disp(X9);
disp('Z9 generated values of Z9');
disp(Z9);


disp('X10 assumed values of X10');
disp(X10);
disp('Z10 generated values of Z10');
disp(Z10);




 for j=1:300
X1P=A0*X0;
disp('X1 value predicted using X0');
disp(X1P);
P1=A0*P0*A0.'+Q;
disp('co variance prediction using previous P');
disp(P1);
Y1=H*Z1-H*X1P;
disp(' innovation ')
disp(Y1);
S1=H*P1*H.'+R;
disp('innovation covariance');
disp(S1);
K1=P1*H.'*inv(S1);
disp('kalman gain');
disp(K1);
XU1=X1P+K1*Y1;
disp('state update');
disp(XU1);
PU1=(I-K1*H)*P1;
disp('co variance update');
disp(PU1);
disp('end of one iteration');

E1=X1-XU1;
disp('error');
disp(E1);


disp('rounded of error');
C1=round(E1,2);
disp(C1);

E_R=[0.5;0.5;0.5];

if(C1 <= E_R);
    X21=XU1;
    A1=A0;
    
   
    disp('for next iteration,values  matched');
    disp(X21);
    disp('total average error:');
    E_T1=mean(C1);
    disp(E_T1);
    
    
else
    J1=X1P/X0;
    A0=J1;
    X0=XU1;
    P0=P1;
    
    disp('values not matched, ne4xt oteration starts');
    disp(X0);
    disp(P0);
    disp(A0);
    


 end
 end
 
 
for r=1:300
X2P=A1*X21;
disp('X1 value predicted using X0');
disp(X2P);
P2=A1*P1*A1.'+Q;
disp('co variance prediction using previous P');
disp(P2);
Y2=H*Z2-H*X2P;
disp(' innovation ')
disp(Y2);
S2=H*P2*H.'+R;
disp('innovation covariance');
disp(S2);
K2=P2*H.'*inv(S2);
disp('kalman gain');
disp(K2);
XU2=X2P+K1*Y2;
disp('state update');
disp(XU2);
PU2=(I-K2*H)*P2;
disp('co variance update');
disp(PU2);
disp('end of one iteration');

E2=X2-XU2;
disp('error');
disp(E2);


disp('rounded of error');
C2=round(E2,2);
disp(C2);

E_R=[0.5;0.5;0.5];

if(C2 <= E_R);
    X22=XU2;
    A2=A1;
    
   
    disp('next iteration,values  matched'); 
    
    disp(X22);
    disp('total average error:');
    E_T2=mean(C2);
    disp(E_T2);
    
    
else
    J2=X2P/X21;
    A1=J2;
    X21=XU2;
    P1=P2;
    disp('values not matched, ne4xt oteration starts');
    disp(X21);
    disp(P1);
    


 end
 end
 
 
for q=1:300
X3P=A2*X22;
disp('X1 value predicted using X0');
disp(X3P);
P3=A2*P2*A2.'+Q;
disp('co variance prediction using previous P');
disp(P3);
Y3=H*Z3-H*X3P;
disp(' innovation ')
disp(Y3);
S3=H*P3*H.'+R;
disp('innovation covariance');
disp(S3);
K3=P3*H.'*inv(S3);
disp('kalman gain');
disp(K3);
XU3=X3P+K3*Y3;
disp('state update');
disp(XU3);
PU3=(I-K3*H)*P3;
disp('co variance update');
disp(PU3);
disp('end of one iteration');

E3=X3-XU3;
disp('error');
disp(E3);


disp('rounded of error');
C3=round(E3,2);
disp(C3);

E_R=[0.5;0.5;0.5];

if(C3 <= E_R);
    X23=XU3;
    A3=A2;
   
    disp('for next iteration,values  matched');
    disp(X23);
    disp('total average error:');
    E_T3=mean(C3);
    disp(E_T3);
    
    
else
    J3=X3P/X22;
    A2=J3;
    X22=XU3;
    P2=P3;
    
    disp('values not matched, ne4xt oteration starts');
    disp(X22);
    disp(P2);
    


 end
 end
 
for q=1:300
X4P=A3*X23;
disp('X1 value predicted using X0');
disp(X4P);
P4=A3*P3*A3.'+Q;
disp('co variance prediction using previous P');
disp(P4);
Y4=H*Z4-H*X4P;
disp(' innovation ')
disp(Y4);
S4=H*P4*H.'+R;
disp('innovation covariance');
disp(S4);
K4=P4*H.'*inv(S4);
disp('kalman gain');
disp(K4);
XU4=X4P+K4*Y4;
disp('state update');
disp(XU4);
PU4=(I-K4*H)*P4;
disp('co variance update');
disp(PU4);
disp('end of one iteration');

E4=X4-XU4; 
disp('error');
disp(E4);


disp('rounded of error');
C4=round(E4,2);
disp(C4);

E_R=[0.5;0.5;0.5];

if(C4 <= E_R);
    X24=XU4;
    A4=A3;
     
   
    disp('for next iteration,values  matched');
    disp(X24);
    disp('total average error:');
    E_T4=mean(C4);
    disp(E_T4);
    
    
else
    J4=X4P/X23;
    A3=J4;
    X23=XU4;
    P3=P4;
    
    disp('values not matched, ne4xt oteration starts');
    disp(X23);
    disp(P3);
    


 end
 end
 
 
for w=1:300
X5P=A4*X24;
disp('X1 value predicted using X0');
disp(X5P);
P5=A4*P4*A4.'+Q;
disp('co variance prediction using previous P');
disp(P5);
Y5=H*Z5-H*X5P;
disp(' innovation ')
disp(Y5);
S5=H*P5*H.'+R;
disp('innovation covariance');
disp(S5);
K5=P5*H.'*inv(S5);
disp('kalman gain');
disp(K5);
XU5=X5P+K5*Y5;
disp('state update');
disp(XU5);
PU5=(I-K5*H)*P5;
disp('co variance update');
disp(PU5);
disp('end of one iteration');

E5=X5-XU5;
disp('error');
disp(E5);


disp('rounded of error');
C5=round(E5,2);
disp(C5);

E_R=[0.5;0.5;0.5];

if(C5 <= E_R);
    X25=XU5;
     A5=A4;
    
   
    disp('for next iteration,values  matched');
    disp(X25);
    disp('total average error:');
    E_T5=mean(C5);
    disp(E_T5);
    
    
else
    J5=X5P/X24;
    A4=J5;
    X24=XU5;
    P4=P5;
    
    disp('values not matched, ne4xt oteration starts');
    disp(X24);
    disp(P4);
    


 end
 end
 
for e=1:300
X6P=A5*X25;
disp('X1 value predicted using X0');
disp(X6P);
P6=A5*P5*A5.'+Q;
disp('co variance prediction using previous P');
disp(P6);
Y6=H*Z6-H*X6P;
disp(' innovation ')
disp(Y6);
S6=H*P6*H.'+R;
disp('innovation covariance');
disp(S6);
K6=P6*H.'*inv(S6);
disp('kalman gain');
disp(K6);
XU6=X6P+K6*Y6;
disp('state update');
disp(XU6);
PU6=(I-K6*H)*P6;
disp('co variance update');
disp(PU6);
disp('end of one iteration');

E6=X6-XU6;
disp('error');
disp(E6);


disp('rounded of error');
C6=round(E6,2);
disp(C6);

E_R=[0.5;0.5;0.5];

if(C6 <= E_R);
    X26=XU6;
     A6=A5;
    
   
    disp('for next iteration,values  matched');
    disp(X26);
    disp('total average error:');
    E_T6=mean(C6);
    disp(E_T6);
    
    
else
    J6=X6P/X25;
    A5=J6;
    X25=XU6;
    P5=P6;
    
    disp('values not matched, ne4xt oteration starts');
    disp(X25);
    disp(P5);
    


 end
 end
 
for r=1:300
X7P=A6*X26;
disp('X1 value predicted using X0');
disp(X7P);
P7=A6*P6*A6.'+Q;
disp('co variance prediction using previous P');
disp(P7);
Y7=H*Z7-H*X7P;
disp(' innovation ')
disp(Y7);
S7=H*P7*H.'+R;
disp('innovation covariance');
disp(S7);
K7=P7*H.'*inv(S7);
disp('kalman gain');
disp(K7);
XU7=X7P+K7*Y7;
disp('state update');
disp(XU7);
PU7=(I-K7*H)*P7;
disp('co variance update');
disp(PU7);
disp('end of one iteration');

E7=X7-XU7;
disp('error');
disp(E7);


disp('rounded of error');
C7=round(E7,2);
disp(C7);

E_R=[0.5;0.5;0.5];

if(C7 <= E_R);
    X27=XU7;
     A7=A6;
    
   
    disp('for next iteration,values  matched');
    disp(X27);
    disp('total average error:');
    E_T7=mean(C7);
    disp(E_T7);
    
    
else
    J7=X7P/X26;
    A6=J7;
    X26=XU7;
    P6=P7;
    
    disp('values not matched, ne4xt oteration starts');
    disp(X26);
    disp(P6);
    


 end
 end
 
 
for t=1:300
X8P=A7*X27;
disp('X1 value predicted using X0');
disp(X8P);
P8=A7*P7*A7.'+Q;
disp('co variance prediction using previous P');
disp(P8);
Y8=H*Z8-H*X8P;
disp(' innovation ')
disp(Y8);
S8=H*P8*H.'+R;
disp('innovation covariance');
disp(S8);
K8=P8*H.'*inv(S8);
disp('kalman gain');
disp(K8);
XU8=X8P+K8*Y8;
disp('state update');
disp(XU8);
PU8=(I-K8*H)*P8;
disp('co variance update');
disp(PU8);
disp('end of one iteration');

E8=X8-XU8;
disp('error');
disp(E8);


disp('rounded of error');
C8=round(E8,2);
disp(C8);

E_R=[0.5;0.5;0.5];

if(C8 <= E_R);
    X28=XU8;
    A8=A7;
    
   
    disp('for next iteration,values  matched');
    disp(X28);
    disp('total average error:');
    E_T8=mean(C8);
    disp(E_T8);
    
    
else
    J8=X8P/X27;
    A7=J8;
    X27=XU8;
    P7=P8;
    
    disp('values not matched, ne4xt oteration starts');
    disp(X27);
    disp(P7);
    


 end
 end
 
for y=1:300
X9P=A8*X28;
disp('X1 value predicted using X0');
disp(X9P);
P9=A8*P8*A8.'+Q;
disp('co variance prediction using previous P');
disp(P9);
Y9=H*Z9-H*X9P;
disp(' innovation ')
disp(Y9);
S9=H*P9*H.'+R;
disp('innovation covariance');
disp(S9);
K9=P9*H.'*inv(S9);
disp('kalman gain');
disp(K9);
XU9=X9P+K9*Y9;
disp('state update');
disp(XU9);
PU9=(I-K9*H)*P9;
disp('co variance update');
disp(PU9);
disp('end of one iteration');

E9=X9-XU9;
disp('error');
disp(E9);


disp('rounded of error');
C9=round(E9,2);
disp(C9);

E_R=[0.5;0.5;0.5];

if(C9 <= E_R);
    X29=XU9;
    A9=A8;
    
   
    disp('for next iteration,values  matched');
    disp(X29);
    disp('total average error:');
    E_T9=mean(C9);
    disp(E_T9);
    
    
else
    J9=X9P/X28;
    A8=J9;
    X28=XU9;
    P8=P9;
    
    disp('values not matched, ne4xt oteration starts');
    disp(X28);
    disp(P8);
    


 end
 end
 
for u=1:300
X10P=A9*X29;
disp('X1 value predicted using X0');
disp(X10P);
P10=A9*P9*A9.'+Q;
disp('co variance prediction using previous P');
disp(P10);
Y10=H*Z10-H*X10P;
disp(' innovation ')
disp(Y10);
S10=H*P10*H.'+R;
disp('innovation covariance');
disp(S10);
K10=P10*H.'*inv(S10);
disp('kalman gain');
disp(K10);
XU10=X10P+K10*Y10;
disp('state update');
disp(XU10);
PU10=(I-K10*H)*P10;
disp('co variance update');
disp(PU10);
disp('end of one iteration');

E10=X10-XU10;
disp('error');
disp(E10);


disp('rounded of error');
C10=round(E10,2);
disp(C10);

E_R=[0.5;0.5;0.5];

if(C10 <= E_R);
    X30=XU10;
    A10=A9;
    
   
    disp('for next iteration,values  matched');
    disp(X30);
    disp('total average error:');
    E_T10=mean(C10);
    disp(E_T10);
    
    
else
    J10=X10P/X29;
    A9=J10;
    X29=XU10;
    P9=P10;
   
    disp('values not matched, ne4xt oteration starts');
    disp(X29);
    disp(P9);
    


 end
 end
 
 
disp('end of one product and final error');
to_er=E_T1+E_T2+E_T3+E_T4+E_T5+E_T6+E_T7+E_T8+E_T9+E_T10;
t_er=mean(to_er);
disp(t_er);





 

 

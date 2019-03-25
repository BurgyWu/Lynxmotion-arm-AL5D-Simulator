clear all
close all
clc
%parameters
l1=6;
l2=15;
l3=15;
l4=3.5;
l5=6.5;
w1=[0;0;1];
w5=[0 1 0]';
w2=[1 0 0]';
w3=w2;
w4=[1 0 0]';
theta1=25.1111*pi/180;
theta2=25.2233*pi/180;
theta3=25.4455*pi/180;
theta4=25.6789*pi/180;
theta5=25.1154*pi/180;
c1=cos(theta1);
s1=sin(theta1);
c2=cos(theta2);
s2=sin(theta2);
c3=cos(theta3);
s3=sin(theta3);
c4=cos(theta4);
s4=sin(theta4);
c5=cos(theta5);
s5=sin(theta5);
c23=cos(theta2+theta3);
s23=sin(theta2+theta3);
c234=cos(theta2+theta3+theta4);
s234=sin(theta2+theta3+theta4);
j1=[0 0 0]';
j2=[0 0 l1]';
j3=[0 l2 l1]';
j4=[0 l2+l3 l1]';
j5=[0 l2+l3 l1+l4+l5]';
%% Forward kinemtatic
gsto=[1 0 0 0;0 1 0 l2+l3+l4+l5;0 0 1 l1;0 0 0 1];

e1=[c1 -s1 0 0; s1 c1 0 0; 0 0 1 0;0 0 0 1];

R1=[c1 -s1 0; s1 c1 0 ; 0 0 1];

e2=[1 0 0 0;0 c2 -s2 l1*s2;0 s2 c2 l1*(1-c2);0 0 0 1];
R2=[1 0 0;0 c2 -s2;0 s2 c2];

e3=[1 0 0 0;0 c3 -s3 l2*(1-c3)+l1*s3;0 s3 c3 -s3*l2+l1*(1-c3);0 0 0 1];
R3=[1 0 0;0 c3 -s3;0 s3 c3];

e4=[1 0 0 0;0 c4 -s4 (l2+l3)*(1-c4)+l1*s4;0 s4 c4 -s4*(l2+l3)+l1*(1-c4);0 0 0 1];
R4=[1 0 0;0 c4 -s4;0 s3 c3];

e5=[c5 0 s5 s5*-l1; 0 1 0 0;-s5 0 c5 l1*(1-c5);0 0 0 1];
R5=[c5 0 s5; 0 1 0;-s5 0 c5];

e6=[1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];
R6=[1 0 0;0 1 0;0 0 1];

Rtheta=R1*R2*R3*R4*R5*R6;
gstheta=e1*e2*e3*e4*e5*e6*gsto;

ptheta=[-s1*(l2*c2+l3*c23+(l4+l5)*c234); c1*(l2*c2+l3*c23+(l4+l5)*c234);l1+l2*s2+l3*s23+l4*s234+l5*s234];

%gstheta=[Rtheta(1,:) 0; Rtheta(2,:) 0;Rtheta(3,:) 0;0 0 0 1]+[0 0 0 -s1*(l2*c2+l3*c23+(l4+l5)*c234);0 0 0 c1*(l2*c2+l3*c23+(l4+l5)*c234);0 0 0 l1+l2*s2+l3*s23+l4*s234+l5*s234;0 0 0 0];

%% IK
R11=Rtheta(1,1);
R12=Rtheta(1,2);
R13=Rtheta(1,3);
R21=Rtheta(2,1);
R22=Rtheta(2,2);
R23=Rtheta(2,3);
R31=Rtheta(3,1);
R32=Rtheta(3,2);
R33=Rtheta(3,3);


invgst0=[1 0 0 0;0 1 0 -l2-l3-l4-l5;0 0 1 -l1;0 0 0 1];
g1=gstheta*invgst0;
%theta1
r4=[0 l2+l3 l1]';
qw=r4;
q=g1*[qw;1];
qx=q(1,1);
qy=q(2,1);
qz=q(3,1);
q1=[0 0 0];
q2=[0 0 1];
qw21=[0 sqrt(qx^2+qy^2) qz]';
qw22=[0 -sqrt(qx^2+qy^2) qz]';
u11=qw21-r4;
u12=qw22-r4;
upj11=u11-w1*transpose(w1)*u11;
upj12=u12-w1*transpose(w1)*u12;
v11=qw21-r4;
v12=qw22-r4;
vpj11=v11-w1*transpose(w1)*v11;
vpj12=v12-w1*transpose(w1)*v12;

theta11=atan2(-sqrt(qx^2+qy^2)*qx,+sqrt(qx^2+qy^2)*qy);
theta12=atan2(sqrt(qx^2+qy^2)*qx,-sqrt(qx^2+qy^2)*qy);
t11=theta11*180/pi
t12=theta12*180/pi

%theta2
r2=[0 0 l1]';
r3=[0 l2 l1]';
u21=qw21-r2;
u22=qw22-r2;
v2=qw-r3;
r23=r2-r3;
r32=r3-r2;

theta011=atan2(-(qz-l1)*l2,l2*sqrt(qx^2+qy^2));
theta012=atan2(-(qz-l1)*l2,-l2*sqrt(qx^2+qy^2));
phi11=acos((norm(u21)^2+norm(r23)^2-norm(v2)^2)/(2*norm(u21)*norm(r23)));
phi12=acos((norm(u22)^2+norm(r23)^2-norm(v2)^2)/(2*norm(u22)*norm(r23)));
theta21=-(theta011+phi11);
theta22=-(theta011-phi11);
t21=theta21*180/pi
t22=(theta22+pi/2)*180/pi

%theta3
theta02=atan2(transpose(w3)*cross(v2,r23),transpose(v2)*r23);
phi21=acos((-norm(u21)^2+norm(r23)^2+norm(v2)^2)/(2*norm(v2)*norm(r23)));
phi22=acos((-norm(u22)^2+norm(r23)^2+norm(v2)^2)/(2*norm(v2)*norm(r23)));
theta31=theta02+phi21;
theta32=theta02-phi21;

t31=theta31*180/pi
t32=theta32*180/pi
% create inverse matrixs for first three joints
c11=cos(theta11);
s11=sin(theta11);
c12=cos(theta12);
s12=sin(theta12);
e11=[c11 -s11 0 0;s11 c11 0 0;0 0 1 0;0 0 0 1];
e12=[c12 -s12 0 0;s12 c12 0 0;0 0 1 0;0 0 0 1];

c21=cos(theta21);
s21=sin(theta21);
c22=cos(theta22);
s22=sin(theta22);
e21=[1 0 0 0;0 c21 -s21 l1*s21;0 s21 c21 l1*(1-c21);0 0 0 1];
e22=[1 0 0 0;0 c22 -s22 l1*s22;0 s22 c22 l1*(1-c22);0 0 0 1];

c31=cos(theta31);
s31=sin(theta31);
c32=cos(theta32);
s32=sin(theta32);
e31=[1 0 0 0;0 c31 -s31 l2*(1-c3)+l1*s31;0 s31 c31 -s31*l2+l1*(1-c31);0 0 0 1];
e32=[1 0 0 0;0 c32 -s32 l2*(1-c3)+l1*s32;0 s32 c32 -s32*l2+l1*(1-c32);0 0 0 1];

%theta4 8 solutions
g21=inv(e31)*inv(e21)*inv(e11)*g1;
g22=inv(e32)*inv(e21)*inv(e11)*g1;
g23=inv(e31)*inv(e22)*inv(e11)*g1;
g24=inv(e32)*inv(e22)*inv(e11)*g1;
g25=inv(e31)*inv(e21)*inv(e12)*g1;
g26=inv(e32)*inv(e21)*inv(e12)*g1;
g27=inv(e31)*inv(e22)*inv(e12)*g1;
g28=inv(e32)*inv(e22)*inv(e12)*g1;

q6=[0 l2+l3+l4 l1]';

qq1=g21*[q6;1];
qx61=qq1(1,1);
qy61=qq1(2,1);
qz61=qq1(3,1);
qq61=[qx61 qy61 qz61]';

v41=qq61-r4;
vpj41=v41-w4'*w4*v41;

qq2=g22*[q6;1];
qx62=qq2(1,1);
qy62=qq2(2,1);
qz62=qq2(3,1);
qq62=[qx62 qy62 qz62]';
v42=qq62-r4;
vpj42=v42-w4'*w4*v42;

qq3=g23*[q6;1];
qx63=qq3(1,1);
qy63=qq3(2,1);
qz63=qq3(3,1);
qq63=[qx63 qy63 qz63]';
v43=qq63-r4;
vpj43=v43-w4'*w4*v43;

qq4=g24*[q6;1];
qx64=qq4(1,1);
qy64=qq4(2,1);
qz64=qq4(4,1);
qq64=[qx64 qy64 qz64]';
v44=qq64-r4;
vpj44=v44-w4'*w4*v44;

qq5=g25*[q6;1];
qx65=qq5(1,1);
qy65=qq5(2,1);
qz65=qq5(4,1);
qq65=[qx65 qy65 qz65]';
v45=qq65-r4;
vpj45=v45-w4'*w4*v45;

qq6=g26*[q6;1];
qx66=qq6(1,1);
qy66=qq6(2,1);
qz66=qq6(4,1);
qq66=[qx66 qy66 qz66]';
v46=qq66-r4;
vpj46=v46-w4'*w4*v46;

qq7=g27*[q6;1];
qx67=qq7(1,1);
qy67=qq7(2,1);
qz67=qq7(4,1);
qq67=[qx67 qy67 qz67]';
v47=qq67-r4;
vpj47=v47-w4'*w4*v47;

qq8=g28*[q6;1];
qx68=qq8(1,1);
qy68=qq8(2,1);
qz68=qq8(4,1);
qq68=[qx68 qy68 qz68]';
u4=q6-r4;
v48=qq68-r4;
upj4=u4-w4'*w4*u4;
vpj48=v48-w4'*w4*v48;

theta41=atan2(w4'*cross(u4,v41),u4'*v41);
theta42=atan2(w4'*cross(u4,v42),u4'*v42);
theta43=-atan2(w4'*cross(u4,v43),u4'*v43);
theta44=atan2(w4'*cross(u4,v44),u4'*v44);
theta45=atan2(w4'*cross(u4,v45),u4'*v45);
theta46=atan2(w4'*cross(u4,v46),u4'*v46);
theta47=atan2(w4'*cross(u4,v47),u4'*v47);
theta48=atan2(w4'*cross(u4,v48),u4'*v48);

t41=theta41*180/pi
t42=theta42*180/pi
t43=theta43*180/pi
t44=theta44*180/pi
t45=theta45*180/pi
t46=theta46*180/pi
t47=theta47*180/pi
t48=theta48*180/pi
%theta5
t5=atan2(gstheta(3,3),gstheta(3,1));
theta5=t5-pi/2;
t5=theta5*180/pi
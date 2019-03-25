%% workspace task 1

close all
clc 
%theta1=0
L1 = 6;L2 = 15;L3 = 15;L4 = 10;%L5 = 6.5;
[q1,q2,q3,q4]=ndgrid(0, 0:5*pi/180:pi, 0:5*pi/180:160*pi/180, -pi/2:5*pi/180:pi/2);
c1 = cos(q1);
s1 = sin(q1);
c2 = cos(q2);
s2 = sin(q2);
c3 = cos(q3);
s3 = sin(q3);
c4 = cos(q4);
s4 = sin(q4);
c23= cos(q2+ q3);
s23= sin(q2+ q3);
c24= cos(q2+ q3+ q4);
s24= sin(q2+ q3+ q4);               
xwork = (L2.*c2 + L3.*c23 - L4.*s24).* c1;
ywork = (L2.*c2 + L3.*c23 - L4*s24).* s1;
zwork = L1+L2.*s2 + L3.*s23 + L4.*c24;
figure(1) 
plot3(xwork(:),ywork(:),zwork(:),'.')
axis([-45 45 -45 45 -30 50])
xlabel('x'); 
ylabel('y');
zlabel('z');
grid on
hold on

%theta2=0
L1 = 6;L2 = 15;L3 = 15;L4 = 10;%L5 = 6.5;
[q1,q2,q3,q4]=ndgrid(-pi/2:5*pi/180:pi/2, 0, 0:5*pi/180:160*pi/180, -pi/2:5*pi/180:pi/2);
c1 = cos(q1);
s1 = sin(q1);
c2 = cos(q2);
s2 = sin(q2);
c3 = cos(q3);
s3 = sin(q3);
c4 = cos(q4);
s4 = sin(q4);
c23= cos(q2+ q3);
s23= sin(q2+ q3);
c24= cos(q2+ q3+ q4);
s24= sin(q2+ q3+ q4);               
xwork = (L2.*c2 + L3.*c23 - L4.*s24).* c1;
ywork = (L2.*c2 + L3.*c23 - L4*s24).* s1;
zwork = L1+L2.*s2 + L3.*s23 + L4.*c24;
figure(2) 
plot3(xwork(:),ywork(:),zwork(:),'.')
axis([-45 45 -45 45 -30 50])
xlabel('x'); 
ylabel('y');
zlabel('z');
grid on
hold on


%theta3=0
L1 = 6;L2 = 15;L3 = 15;L4 = 10;%L5 = 6.5;
[q1,q2,q3,q4]=ndgrid(-pi/2:5*pi/180:pi/2, 0:5*pi/180:pi, 0, -pi/2:5*pi/180:pi/2);
c1 = cos(q1);
s1 = sin(q1);
c2 = cos(q2);
s2 = sin(q2);
c3 = cos(q3);
s3 = sin(q3);
c4 = cos(q4);
s4 = sin(q4);
c23= cos(q2+ q3);
s23= sin(q2+ q3);
c24= cos(q2+ q3+ q4);
s24= sin(q2+ q3+ q4);               
xwork = (L2.*c2 + L3.*c23 - L4.*s24).* c1;
ywork = (L2.*c2 + L3.*c23 - L4*s24).* s1;
zwork = L1+L2.*s2 + L3.*s23 + L4.*c24;
figure(3) 
plot3(xwork(:),ywork(:),zwork(:),'.')
axis([-45 45 -45 45 -30 50])
xlabel('x'); 
ylabel('y');
zlabel('z');
grid on
hold on

%theta4=0
L1 = 6;L2 = 15;L3 = 15;L4 = 10;%L5 = 6.5;
[q1,q2,q3,q4]=ndgrid(-pi/2:5*pi/180:pi/2, 0:5*pi/180:pi, 0:5*pi/180:160*pi/180, 0);
c1 = cos(q1);
s1 = sin(q1);
c2 = cos(q2);
s2 = sin(q2);
c3 = cos(q3);
s3 = sin(q3);
c4 = cos(q4);
s4 = sin(q4);
c23= cos(q2+ q3);
s23= sin(q2+ q3);
c24= cos(q2+ q3+ q4);
s24= sin(q2+ q3+ q4);               
xwork = (L2.*c2 + L3.*c23 - L4.*s24).* c1;
ywork = (L2.*c2 + L3.*c23 - L4*s24).* s1;
zwork = L1+L2.*s2 + L3.*s23 + L4.*c24;
figure(4) 
plot3(xwork(:),ywork(:),zwork(:),'.')
axis([-45 45 -45 45 -30 50])
xlabel('x'); 
ylabel('y');
zlabel('z');
grid on
hold on

%workspace without fix theta
%theta4=0
L1 = 6;L2 = 15;L3 = 15;L4 = 10;%L5 = 6.5;
[q1,q2,q3,q4]=ndgrid(-pi/2:5*pi/180:pi/2, 0:5*pi/180:pi, 0:5*pi/180:160*pi/180, -pi/2:5*pi/180:pi/2);
c1 = cos(q1);
s1 = sin(q1);
c2 = cos(q2);
s2 = sin(q2);
c3 = cos(q3);
s3 = sin(q3);
c4 = cos(q4);
s4 = sin(q4);
c23= cos(q2+ q3);
s23= sin(q2+ q3);
c24= cos(q2+ q3+ q4);
s24= sin(q2+ q3+ q4);               
xwork = (L2.*c2 + L3.*c23 - L4.*s24).* c1;
ywork = (L2.*c2 + L3.*c23 - L4*s24).* s1;
zwork = L1+L2.*s2 + L3.*s23 + L4.*c24;
figure(5) 
plot3(xwork(:),ywork(:),zwork(:),'.')
axis([-45 45 -45 45 -30 50])
xlabel('x'); 
ylabel('y');
zlabel('z');
grid on
hold on

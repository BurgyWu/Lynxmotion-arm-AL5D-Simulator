clc
clear all
L1 = 6;L2 = 15;L3 = 15;L4 = 3.5;L5 = 6.5;
h=-20;%x-athetais of centre point
k=0;%y-athetais of centre point
l=0;%z-athetais of centre point
r=5; %circle center
d=10; % straight distance from point to point in cm
%point1(-20+5*sqrt(2)/2,5*sqrt(2)/2,0) to point to (-20-5*sqrt(2)/2,-5*sqrt(2)/2,0)
tf=4; % time spend for the whole task in s

%gradient of line from point to point
q11=zeros(1,6);
q22=zeros(1,6);
q33=zeros(1,6);
q44=zeros(1,6);
v1=zeros(1,6);
a1=zeros(1,6);
i=0;
for t=0:0.1:0.5;
     i = i +1;
    % position
     theta = 0.5*40/7*pi*t^2/5;
     px = r*cos(theta)-20;
     py = r*sin(theta);
     pz = 0;
     v=40/7*t;
     a=40/7;
    sw24=0;
    cw24=-1;
   %theta1
    theta1 = atan2(py,px);
    if theta1<-pi/2
        theta1 = theta1 + pi;
    elseif theta1>pi/2
        theta1 = theta1 - pi;
    end
    ax=-cos(theta1)*sw24;
    ay=-sin(theta1)*sw24;
    az=cw24;
    %theta3
    D=(((pz-az.*(L4+L5)-L1).^2+(px-ax.*(L4+L5)).^2+(py-ay.*(L4+L5)).^2)-L2^2-L3^2)/(2*L2*L3);
    theta3=atan2(sqrt(1-D.^2),D);
    %theta2
    K=(pz-az.*(L4+L5)-L1).^2+(px-ax.*(L4+L5)).^2+(py-ay.*(L4+L5)).^2;
    beta=atan2((pz-az.*(L4+L5)-L1),(sqrt((px-ax.*(L4+L5)).^2+(py-ay.*(L4+L5)).^2)));
    H=(L2^2+K-L3^2)/(2*L2*sqrt(K));
    phla=atan2(sqrt(1-H^2),H);
    theta2=pi-beta-phla;  %elbow up 
    %theta4
    theta234 = atan2(sw24,cw24);
    theta4=theta234-theta2-theta3;
   
    q1 = theta1;
    q2 = theta2;
    q3 = theta3;
    q4 = theta4;
    %creat matrix for theta,linear velocity and linear acceleration
    q11(i)=q1;
    q22(i)=q2;
    q33(i)=q3;
    q44(i)=q4;
    v1(i)=v;
    a1(i)=a;
    %calculate cos and sin
    c1 = cos(q1);
    c2 = cos(q2);
    c3 = cos(q3);
    c4 = cos(q4);

    c23 = cos(q2 + q3);
    c24 = cos(q2 + q3 + q4);

    s1 = sin(q1);
    s2 = sin(q2);
    s3 = sin(q3);
    s4 = sin(q4);

    s23 = sin(q2 + q3);
    s24 = sin(q2 + q3 + q4);

    %calculate joints' position
    x0 = 0;
    x1 = 0;
    x2 = L2 * c2 * c1;
    x3 = (L2 * c2 + L3 * c23) * c1;
    xe = ( L2 * c2 + L3 * c23 - (L4+L5) * s24 ) * c1;

    y0 = 0;
    y1 = 0;
    y2 = L2 * c2 * s1;
    y3 = (L2 * c2 + L3 * c23) * s1;
    ye = ( L2 * c2 + L3 * c23 - (L4+L5) * s24 ) * s1;

    z0 = 0;
    z1 = L1;
    z2 = L1 + L2 * s2;
    z3 = L1 + L2 * s2 + L3 * s23;
    ze = z3 + (L4+L5) * c24;

    xx = [x0 x1 x2 x3 xe];
    yy = [y0 y1 y2 y3 ye];
    zz = [z0 z1 z2 z3 ze];

    %plot links
    figure(1)
    plot3(xx, yy, zz, 'ko-','Linewidth',2)
    xlabel('x'); 
    ylabel('y');
    zlabel('z');
    axis([-30 5 -15 15 0 30]);
    hold on
    grid on
    %plot points
    figure(2)
    plot3(xe,ye,ze, 'g.')
    xlabel('x'); 
    ylabel('y');
    zlabel('z');
    axis([-30 -10 -10 10 0 20]);
    %text(-15,1.5,0,'ptstart');
    %text(-15,-1.5,0,'ptend');
    hold on
    grid on

    pause(0.1)
  
end

q111=zeros(1,30);
q222=zeros(1,30);
q333=zeros(1,30);
q444=zeros(1,30);
v2=zeros(1,30);
a2=zeros(1,30);
i=0;
for t=0.6:0.1:3.5;
     i = i +1;
    % position
     theta = 0.5*40/7*pi*0.5^2/5+40/7*pi*0.5*(t-0.5)/5;
     px = r*cos(theta)-20;
     py = r*sin(theta);
     pz = 0;
     v=0.5*40/7;
     a=0;
    sw24=0;
    cw24=-1;
   %theta1
    theta1 = atan2(py,px);
    if theta1<-pi/2
        theta1 = theta1 + pi;
    elseif theta1>pi/2
        theta1 = theta1 - pi;
    end
    ax=-cos(theta1)*sw24;
    ay=-sin(theta1)*sw24;
    az=cw24;
    %theta3
    D=(((pz-az.*(L4+L5)-L1).^2+(px-ax.*(L4+L5)).^2+(py-ay.*(L4+L5)).^2)-L2^2-L3^2)/(2*L2*L3);
    theta3=atan2(sqrt(1-D.^2),D);
    %theta2
    K=(pz-az.*(L4+L5)-L1).^2+(px-ax.*(L4+L5)).^2+(py-ay.*(L4+L5)).^2;
    beta=atan2((pz-az.*(L4+L5)-L1),(sqrt((px-ax.*(L4+L5)).^2+(py-ay.*(L4+L5)).^2)));
    H=(L2^2+K-L3^2)/(2*L2*sqrt(K));
    phla=atan2(sqrt(1-H^2),H);
    theta2=pi-beta-phla;  %elbow up 
    %theta4
    theta234 = atan2(sw24,cw24);
    theta4=theta234-theta2-theta3;
   
    q1 = theta1;
    q2 = theta2;
    q3 = theta3;
    q4 = theta4;
    %creat matrix for theta,linear velocity and linear acceleration
    q111(i)=q1;
    q222(i)=q2;
    q333(i)=q3;
    q444(i)=q4;
    v2(i)=v;
    a2(i)=a;

    %calculate cos and sin
    c1 = cos(q1);
    c2 = cos(q2);
    c3 = cos(q3);
    c4 = cos(q4);

    c23 = cos(q2 + q3);
    c24 = cos(q2 + q3 + q4);

    s1 = sin(q1);
    s2 = sin(q2);
    s3 = sin(q3);
    s4 = sin(q4);

    s23 = sin(q2 + q3);
    s24 = sin(q2 + q3 + q4);

    %calculate joints' position
    x0 = 0;
    x1 = 0;
    x2 = L2 * c2 * c1;
    x3 = (L2 * c2 + L3 * c23) * c1;
    xe = ( L2 * c2 + L3 * c23 - (L4+L5) * s24 ) * c1;

    y0 = 0;
    y1 = 0;
    y2 = L2 * c2 * s1;
    y3 = (L2 * c2 + L3 * c23) * s1;
    ye = ( L2 * c2 + L3 * c23 - (L4+L5) * s24 ) * s1;

    z0 = 0;
    z1 = L1;
    z2 = L1 + L2 * s2;
    z3 = L1 + L2 * s2 + L3 * s23;
    ze = z3 + (L4+L5) * c24;

    xx = [x0 x1 x2 x3 xe];
    yy = [y0 y1 y2 y3 ye];
    zz = [z0 z1 z2 z3 ze];

    %plot links
    figure(1)
    plot3(xx, yy, zz, 'ko-','Linewidth',2)
    xlabel('x'); 
    ylabel('y');
    zlabel('z');
    axis([-30 5 -15 15 0 30]);
    hold on
    grid on
    %plot points
    figure(2)
    plot3(xe,ye,ze, 'k.')
    xlabel('x'); 
    ylabel('y');
    zlabel('z');
    axis([-30 -10 -10 10 0 20]);
    text(-14,0,0,'ptstart & end');
    %text(-15,-1.5,0,'ptend');
    hold on

    pause(0.1)
  
end

q1111=zeros(1,5);
q2222=zeros(1,5);
q3333=zeros(1,5);
q4444=zeros(1,5);
v3=zeros(1,5);
a3=zeros(1,5);
i=0;

for t=3.6:0.1:4;
     i = i +1;
    % position
     theta =0.5*40/7*pi*0.5^2/5+40/7*pi*0.5*(3.5-0.5)/5+0.5*40/7*pi*(t-3.5)/5-0.5*40/7*pi*(t-3.5)^2/5;
     px = r*cos(theta)-20;
     py = r*sin(theta);
     pz = 0;
     v=0.5*40/7-40/7*(t-3.5);
     a=-40/7;
    sw24=0;
    cw24=-1;
   %theta1
    theta1 = atan2(py,px);
    if theta1<-pi/2
        theta1 = theta1 + pi;
    elseif theta1>pi/2
        theta1 = theta1 - pi;
    end
    ax=-cos(theta1)*sw24;
    ay=-sin(theta1)*sw24;
    az=cw24;
    %theta3
    D=(((pz-az.*(L4+L5)-L1).^2+(px-ax.*(L4+L5)).^2+(py-ay.*(L4+L5)).^2)-L2^2-L3^2)/(2*L2*L3);
    theta3=atan2(sqrt(1-D.^2),D);
    %theta2
    K=(pz-az.*(L4+L5)-L1).^2+(px-ax.*(L4+L5)).^2+(py-ay.*(L4+L5)).^2;
    beta=atan2((pz-az.*(L4+L5)-L1),(sqrt((px-ax.*(L4+L5)).^2+(py-ay.*(L4+L5)).^2)));
    H=(L2^2+K-L3^2)/(2*L2*sqrt(K));
    phla=atan2(sqrt(1-H^2),H);
    theta2=pi-beta-phla;  %elbow up 
    %theta4
    theta234 = atan2(sw24,cw24);
    theta4=theta234-theta2-theta3;
   
    q1 = theta1;
    q2 = theta2;
    q3 = theta3;
    q4 = theta4;
    %creat matrix for theta
    q1111(i)=q1;
    q2222(i)=q2;
    q3333(i)=q3;
    q4444(i)=q4;
    v3(i)=v;
    a3(i)=a;

    %calculate cos and sin
    c1 = cos(q1);
    c2 = cos(q2);
    c3 = cos(q3);
    c4 = cos(q4);

    c23 = cos(q2 + q3);
    c24 = cos(q2 + q3 + q4);

    s1 = sin(q1);
    s2 = sin(q2);
    s3 = sin(q3);
    s4 = sin(q4);

    s23 = sin(q2 + q3);
    s24 = sin(q2 + q3 + q4);

    %calculate joints' position
    x0 = 0;
    x1 = 0;
    x2 = L2 * c2 * c1;
    x3 = (L2 * c2 + L3 * c23) * c1;
    xe = ( L2 * c2 + L3 * c23 - (L4+L5) * s24 ) * c1;

    y0 = 0;
    y1 = 0;
    y2 = L2 * c2 * s1;
    y3 = (L2 * c2 + L3 * c23) * s1;
    ye = ( L2 * c2 + L3 * c23 - (L4+L5) * s24 ) * s1;

    z0 = 0;
    z1 = L1;
    z2 = L1 + L2 * s2;
    z3 = L1 + L2 * s2 + L3 * s23;
    ze = z3 + (L4+L5) * c24;

    xx = [x0 x1 x2 x3 xe];
    yy = [y0 y1 y2 y3 ye];
    zz = [z0 z1 z2 z3 ze];

    %plot links
    figure(1)
    plot3(xx, yy, zz, 'ko-','Linewidth',2)
   
    xlabel('x'); 
    ylabel('y');
    zlabel('z');
    axis([-30 5 -15 15 0 30]);
    hold on
    grid on
    %plot points
    figure(2)
    plot3(xe,ye,ze, 'r.')
    xlabel('x'); 
    ylabel('y');
    zlabel('z');
    axis([-30 -10 -10 10 0 20]);
    %text(-15,1.5,0,'ptstart');
    %text(-15,-1.5,0,'ptend');
    hold on

    pause(0.1)
  
end
q1all=[q11 q111 q1111];
q2all=[q22 q222 q2222];
q3all=[q33 q333 q3333];
q4all=[q44 q444 q4444];
vall=[v1 v2 v3];
aall=[a1 0 a2 -40/7 a3];
st1=0:0.1:0.5;
st2=0.6:0.1:3.5;
st3=3.6:0.1:4;
st=[st1 0.5 st2 3.5 st3];
%plots for joint position against time and the fit curve
 t=0:0.1:4;
figure (3)
plot(t,q1all,'ko-'); 
hold on
plot(t,q2all,'ms-');
hold on
plot(t,q3all,'rx-'); 
hold on
plot(t,q4all,'bd-');
title('Joint position against time');
legend('Theta1','Theta2','Theta3','Theta4')
xlabel('Time(s)') ; ylabel('Joint Position (radians)')
grid on
%plots for linear speed against time
figure(4)
plot(t,vall);
hold on
grid on
title('Linear speed against time');
xlabel('Time(s)') ; ylabel('Linear speed (cm/s)')

%plots for linear acceleration against time
figure(5)
plot(st,aall);
hold on
grid on
title('Linear acceleration against time');
xlabel('Time(s)') ; ylabel('Acceleration speed (cm/s)')

    
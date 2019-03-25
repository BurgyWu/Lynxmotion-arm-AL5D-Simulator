clc
clear all
L1 = 6;L2 = 15;L3 = 15;L4 = 3.5;L5 = 6.5;
h=-20;%x-axis of centre point
k=0;%y-axis of centre point
l=0;%z-axis of centre point
r=5; %circle center
d=10; % straight distance from point to point in cm

%% point1(-20+5*sqrt(2)/2,5*sqrt(2)/2,0) to point 2 (-20-1.5*sqrt(2)/2,1.5*sqrt(2)/2,0)
[x,y,z] = sphere;
figure(2)
hold on
surf(x-20,y,z+1);
%accelarations segment from P1 to P2  
%where ditance=3.5cm and accelaratons set to be 20 cm/s^2
qa1=zeros(1,3);
qa2=zeros(1,3);
qa3=zeros(1,3);
qa4=zeros(1,3);
v1=zeros(1,3);
a1=zeros(1,3);
i = 0;
for t=0:0.1:0.2;
    i = i +1;
    % position
    px = -20+5*sqrt(2)/2-1/2*20*t^2/sqrt(2);
    py = 5*sqrt(2)/2-1/2*20*t^2/sqrt(2);
    pz = 0;
    v=20*t;
    a=20;
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
    qa1(i)=q1;
    qa2(i)=q2;
    qa3(i)=q3;
    qa4(i)=q4;
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
    plot3(xx, yy, zz, 'ko-','Linewidth',2);
    xlabel('x'); 
    ylabel('y');
    zlabel('z');
    axis([-30 5 -15 15 0 30])
    hold on
    %plot points
    figure(2)
    plot3(xe,ye,ze, 'g.');
    xlabel('x'); 
    ylabel('y');
    zlabel('z');
    axis([-30 -10 -10 10 0 50])
    hold on
    grid on
    
    pause(0.1)
  
  
end
%constant speed segment where speed is 4 cm/s
qa11=zeros(1,7);
qa22=zeros(1,7);
qa33=zeros(1,7);
qa44=zeros(1,7);
v2=zeros(1,7);
a2=zeros(1,7);
i = 0;
for t=0.1:0.1:0.7;
    i = i +1;
    % position
    px = -20+5*sqrt(2)/2-1/2*20*0.2^2/sqrt(2)-4*t/sqrt(2);
    py = 5*sqrt(2)/2-1/2*20*0.2^2/sqrt(2)-4*t/sqrt(2);
    pz = 0;
    v=4;
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
    qa11(i)=q1;
    qa22(i)=q2;
    qa33(i)=q3;
    qa44(i)=q4;
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
    plot3(xx, yy, zz, 'ko-','Linewidth',2);
    xlabel('x'); 
    ylabel('y');
    zlabel('z');
    axis([-30 5 -15 15 0 30])
    hold on
    %plot points
    figure(2)
    plot3(xe,ye,ze, 'k.');
    xlabel('x'); 
    ylabel('y');
    zlabel('z');
    axis([-30 -10 -10 10 0 50])
    hold on
    grid on
    
    pause(0.1)
end
% Decelerations segment where deceleratons is 20 cm/s^2
qa111=zeros(1,1);
qa222=zeros(1,1);
qa333=zeros(1,1);
qa444=zeros(1,1);
v3=zeros(1,1);
a3=zeros(1,1);
i = 0;
for t=0.1:0.1:0.1;
    i = i +1;
    % position
    px = -20+5*sqrt(2)/2-1/2*20*0.2^2/sqrt(2)-4*0.7/sqrt(2)-(4/sqrt(2)*t-1/2*20*t^2/sqrt(2));
    py = 5*sqrt(2)/2-1/2*20*0.2^2/sqrt(2)-4*0.7/sqrt(2)-(4/sqrt(2)*t-1/2*20*t^2/sqrt(2));
    pz = 0;
    v=4-20*t;
    a=-20;
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
    qa111(i)=q1;
    qa222(i)=q2;
    qa333(i)=q3;
    qa444(i)=q4;
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
    plot3(xx, yy, zz, 'ko-','Linewidth',2);
    xlabel('x'); 
    ylabel('y');
    zlabel('z');
    axis([-30 5 -15 15 0 30])
    hold on
    %plot points
    figure(2)
    plot3(xe,ye,ze, 'r.');
    xlabel('x'); 
    ylabel('y');
    zlabel('z');
    axis([-30 -10 -10 10 0 50])
    hold on
    grid on
    
    pause(0.1)
end

%% point2(-20+1.5*sqrt(2)/2,1.5/sqrt(2)) to point3 (-20-1.5*sqrt(2)/2,1.5/sqrt(2))
%accelrations segment
qb1=zeros(1,2);
qb2=zeros(1,2);
qb3=zeros(1,2);
qb4=zeros(1,2);
v4=zeros(1,2);
a4=zeros(1,2);
i = 0;
for t=0.1:0.1:0.2;
    i = i +1;
    % position
    theta=pi/4+2/1.5*t+1/2*5.401389216/1.5*t^2;
    px = 1.5*cos(theta)-20;
    py = 1.5*sin(theta);
    pz = 0;
    v=2+5.401389216*t;
    a=5.401389216;
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
    qb1(i)=q1;
    qb2(i)=q2;
    qb3(i)=q3;
    qb4(i)=q4;
    v4(i)=v;
    a4(i)=a;
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
    plot3(xx, yy, zz, 'ko-','Linewidth',2);
    xlabel('x'); 
    ylabel('y');
    zlabel('z');
    axis([-30 5 -15 15 0 30])
    hold on
    %plot points
    figure(2)
    plot3(xe,ye,ze, 'g.');
    xlabel('x'); 
    ylabel('y');
    zlabel('z');
    axis([-30 -10 -10 10 0 50])
    hold on
    grid on
    
    pause(0.1)
end

%constant speed segment
qb11=zeros(1,12);
qb22=zeros(1,12);
qb33=zeros(1,12);
qb44=zeros(1,12);
v5=zeros(1,12);
a5=zeros(1,12);
i = 0;
for t=0.1:0.1:1.2;
    i = i +1;
    % position
    theta=pi/4+2/1.5*0.2+1/2*5.401389216/1.5*0.2^2+(5.401389216*0.2+2)/1.5*t;
    px = 1.5*cos(theta)-20;
    py = 1.5*sin(theta);
    pz = 0;
    v=2+5.401389216*0.2;
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
    qb11(i)=q1;
    qb22(i)=q2;
    qb33(i)=q3;
    qb44(i)=q4;
    v5(i)=v;
    a5(i)=a;
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
    plot3(xx, yy, zz, 'ko-','Linewidth',2);
    xlabel('x'); 
    ylabel('y');
    zlabel('z');
    axis([-30 5 -15 15 0 30])
    hold on
    %plot points
    figure(2)
    plot3(xe,ye,ze, 'k.');
    xlabel('x'); 
    ylabel('y');
    zlabel('z');
    axis([-30 -10 -10 10 0 50])
    hold on
    grid on
    
    pause(0.1)
end
%deceleration segment
qb111=zeros(1,2);
qb222=zeros(1,2);
qb333=zeros(1,2);
qb444=zeros(1,2);
v6=zeros(1,2);
a6=zeros(1,2);
i = 0;
for t=0.1:0.1:0.2;
    i = i +1;
    % position
    theta=pi/4+2/1.5*0.2+(5.401389216*0.2+2)/1.5*1.2+(2+5.401389216*0.2)/1.5*t;
    px = 1.5*cos(theta)-20;
    py = 1.5*sin(theta);
    pz = 0;
    v=2+5.401389216*0.2-5.401389216*t;
    a=-5.401389216;
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
    qb111(i)=q1;
    qb222(i)=q2;
    qb333(i)=q3;
    qb444(i)=q4;
    v6(i)=v;
    a6(i)=a;
    
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
    plot3(xx, yy, zz, 'ko-','Linewidth',2);
    xlabel('x'); 
    ylabel('y');
    zlabel('z');
    axis([-30 5 -15 15 0 30])
    hold on
    %plot points
    figure(2)
    plot3(xe,ye,ze, 'r.');
    xlabel('x'); 
    ylabel('y');
    zlabel('z');
    axis([-30 -10 -10 10 0 50])
    hold on
    grid on
    
    pause(0.1)
end

%% point3(-20-1.5*sqrt(2)/2,-1.5/sqrt(2)) to point4(-20-5*sqrt(2)/2,-5*sqrt(2)/2)
%accelaration segment
qc1=zeros(1,1);
qc2=zeros(1,1);
qc3=zeros(1,1);
qc4=zeros(1,1);
v7=zeros(1,1);
a7=zeros(1,1);
i = 0;
for t=0.1:0.1:0.1;
    i = i +1;
    % position
    px = -20-1.5*sqrt(2)/2-1/2*20*t^2/sqrt(2)-2*t/sqrt(2);
    py = -1.5*sqrt(2)/2-1/2*20*t^2/sqrt(2)-2*t/sqrt(2);
    pz = 0;
    v=20*0.2;
    a=20;
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
    qc1(i)=q1;
    qc2(i)=q2;
    qc3(i)=q3;
    qc4(i)=q4;
    v7(i)=v;
    a7(i)=a;
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
    plot3(xx, yy, zz, 'ko-','Linewidth',2);
    xlabel('x'); 
    ylabel('y');
    zlabel('z');
    axis([-30 5 -15 15 0 30])
    hold on
    %plot points
    figure(2)
    plot3(xe,ye,ze, 'g.');
    xlabel('x'); 
    ylabel('y');
    zlabel('z');
    axis([-30 -10 -10 10 0 50])
    hold on
    grid on
    
    pause(0.1)
end

%constant speed segment
qc11=zeros(1,7);
qc22=zeros(1,7);
qc33=zeros(1,7);
qc44=zeros(1,7);
v8=zeros(1,7);
at=zeros(1,7);
i = 0;
for t=0.1:0.1:0.7;
    i = i +1;
    % position
    px = -20-1.5*sqrt(2)/2-1/2*20*0.1^2/sqrt(2)-2*0.1/sqrt(2)-4*t/sqrt(2);
    py = -1.5*sqrt(2)/2-1/2*20*0.1^2/sqrt(2)-2*0.1/sqrt(2)-4*t/sqrt(2);
    pz = 0;
    v=4;
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
    qc11(i)=q1;
    qc22(i)=q2;
    qc33(i)=q3;
    qc44(i)=q4;
    v8(i)=v;
    a8(i)=a;
    
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
    plot3(xx, yy, zz, 'ko-','Linewidth',2);
    xlabel('x'); 
    ylabel('y');
    zlabel('z');
    axis([-30 5 -15 15 0 30])
    hold on
    %plot points
    figure(2)
    plot3(xe,ye,ze, 'k.');
    xlabel('x'); 
    ylabel('y');
    zlabel('z');
    axis([-30 -10 -10 10 0 50])
    hold on
    grid on
    
    pause(0.1)
end

%deceleration segment
qc111=zeros(1,2);
qc222=zeros(1,2);
qc333=zeros(1,2);
qc444=zeros(1,2);
v9=zeros(1,2);
a9=zeros(1,2);
i=0;
for t=0.1:0.1:0.2;
    i = i +1;
    % position
    px = -20-1.5*sqrt(2)/2-1/2*20*0.1^2/sqrt(2)-2*0.1/sqrt(2)-4*0.7/sqrt(2)-4*t/sqrt(2)+1/2*20*t^2/sqrt(2);
    py = -1.5*sqrt(2)/2-1/2*20*0.1^2/sqrt(2)-2*0.1/sqrt(2)-4*0.7/sqrt(2)-4*t/sqrt(2)+1/2*20*t^2/sqrt(2);
    pz = 0;
    v=4-t*20;
    a=-20;
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
    qc111(i)=q1;
    qc222(i)=q2;
    qc333(i)=q3;
    qc444(i)=q4;
    v9(i)=v;
    a9(i)=a;
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
    plot3(xx, yy, zz, 'ko-','Linewidth',2);
    xlabel('x'); 
    ylabel('y');
    zlabel('z');
    axis([-30 5 -15 15 0 30])
    grid on
    hold on
    %plot points
    figure(2)
    plot3(xe,ye,ze, 'r.');
    xlabel('x'); 
    ylabel('y');
    zlabel('z');
    axis([-30 -10 -10 10 0 50])
    hold on
    grid on
    
    pause(0.1)
  
  
end

q1all=[qa1 qa11 qa111 qb1 qb11 qb111 qc1 qc11 qc111];
q2all=[qa2 qa22 qa222 qb2 qb22 qb222 qc2 qc22 qc222];
q3all=[qa3 qa33 qa333 qb3 qb33 qb333 qc3 qc33 qc333];
q4all=[qa4 qa44 qa444 qb4 qb44 qb444 qc4 qc44 qc444];
vall=[v1 v2 v3 v4 v5 v6 v7 v8 v9];

aall=[a1 0 a2 -20 a3 5.40139 a4 0 a5 -5.40139 a6 20 a7 0 a8 -20 a9];
st1=0:0.1:0.2;
st2=0.3:0.1:0.9;
st3=1;
st4=1.1:0.1:1.2;
st5=1.3:0.1:2.4;
st6=2.5:0.1:2.6;
st7=2.7:0.1:2.7;
st8=2.8:0.1:3.4;
st9=3.5:0.1:3.6;
st=[st1 0.2 st2 0.9 st3 1 st4 1.2 st5 2.4 st6 2.6 st7 2.7 st8 3.4 st9];
%plots for joint position against time
t=0:0.1:3.6;
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
axis([0 3.6 -2 2.5 0 1]);
%plots for linear speed against time
figure(4)
t=0:0.1:3.6;
plot(t,vall);
hold on
grid on
title('Linear speed against time');
xlabel('Time(s)') ; ylabel('Linear speed (cm/s)')
axis([0 4 0 4.5 0 1]);
%plots for linear acceleration against time
figure(5)
plot(st,aall);
hold on
grid on
title('Linear acceleration against time');
xlabel('Time(s)') ; ylabel('Acceleration speed (cm/s)')
axis([0 3.8 -22 22 0 1]);

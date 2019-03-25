clc
clear all
L1 = 6;L2 = 15;L3 = 15;L4 = 3.5;L5 = 6.5;
h=-20;%x-axis of centre point
k=0;%y-axis of centre point
l=0;%z-axis of centre point
r=5; %circle center
d=10; % straight distance from point to point in cm
%point1(-20+5*sqrt(2)/2,5*sqrt(2)/2,0) to point 2 (-20-5*sqrt(2)/2,-5*sqrt(2)/2,0)
tf=2; % time spend from point to point in s
xi=-20+5*sqrt(2)/2;
xf=-20-5*sqrt(2)/2;
yi=5*sqrt(2)/2;
m=1;%gradient of line from point to point
q11=zeros(1,21);
q22=zeros(1,21);
q33=zeros(1,21);
q44=zeros(1,21);
i = 0;

for t=0:0.1:2;
    i = i +1;
    % position
    px = xi + 3*(xf-xi)/(tf^2)*(t^2) - 2*(xf-xi)/(tf^3)*(t^3);
    py = px + 20;
    pz = 0;
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
    q11(i)=q1;
    q22(i)=q2;
    q33(i)=q3;
    q44(i)=q4;
    
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
    plot3(xe,ye,ze, '.');
    xlabel('x'); 
    ylabel('y');
    zlabel('z');
    axis([-30 -10 -10 10 0 50])
    text(-16,4,0,'ptstart');
    text(-24,-4,0,'ptend');
    hold on
    grid on
    
    pause(0.1)
  
  
end
%plots for Linear speed against time
  t=0:0.1:2;
  vx = 6*(xf-xi)/(tf^2)*t - 3*2*(xf-xi)/(tf^3)*(t.^2);
  vy=vx;
  v=sqrt(vx.^2+vy.^2);
  figure(3)
  plot(t,v);
  grid on
  title('Linear speed against time');
  xlabel('Time(s)') ; ylabel('Linear speed (cm/s)')
%plots for linear acceleration against time

   t=0:0.1:2;
   ax = 6*(xf-xi)/(tf^2) - 12*(xf-xi)/(tf^3).*t;
   ay=ax;
   a=sqrt(ax.^2+ay.^2);
  figure(4)
  plot(t,a);
  grid on
  title('Linear acceleration against time');
  xlabel('Time(s)') ; ylabel('Linear speed (cm/s^2)')
%plots for changing of joints position against time 
t=0:0.1:2;
figure (5)
plot(t,q11,'ko-'); 
hold on
plot(t,q22,'ms-');
hold on
plot(t,q33,'rx-'); 
hold on
plot(t,q44,'bd-');
grid on
title('Joint position against time');
legend('Theta1','Theta2','Theta3','Theta4')
xlabel('Time(s)') ; ylabel('Joint Position (radians)')



  
clc
close all
clear all
L1 = 6;L2 = 15;L3 = 15;L4 = 3.5;L5 = 6.5;
h=-20;%x-axis of centre point
k=0;%y-axis of centre point
l=0;%z-axis of centre point
r=5; %circle center
%angle for each joints for start point to end point
theta10=-0.2115; theta1f=0.1491;
theta20=1.9526; theta2f=2.3390;
theta30=1.9116; theta3f=1.2721;
theta40=-0.7226; theta4f=-0.4695;
tf=1.5; %time for moving for point to point
%% free motion
for t=0:0.1:1.5
    q1 = theta10 + 3*(theta1f-theta10)/(tf^2)*(t^2) - 2*(theta1f-theta10)/(tf^3)*(t^3);
    q2 = theta20 + 3*(theta2f-theta20)/(tf^2)*(t^2) - 2*(theta2f-theta20)/(tf^3)*(t^3);
    q3 = theta30 + 3*(theta3f-theta30)/(tf^2)*(t^2) - 2*(theta3f-theta30)/(tf^3)*(t^3);
    q4 = theta40 + 3*(theta4f-theta40)/(tf^2)*(t^2) - 2*(theta4f-theta40)/(tf^3)*(t^3);
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
    xe = ( L2 * c2 + L3 * c23 - (L4+L5) * s24 ) * c1

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
    axis([-30 5 -15 15 0 30])
    hold on
    grid on
    %plot points
    figure(2)
    plot3(xe,ye,ze, '.')
    xlabel('x'); 
    ylabel('y');
    zlabel('z');
    axis([-30 -10 -10 10 0 50])
    text(-20+5*sqrt(2)/2+0.5,5*sqrt(2)/2,0,'ptstart');
    text(-20-5*sqrt(2)/2+0.5,-5*sqrt(2)/2,0,'ptend');
    hold on
    grid on

    figure(1)
    pause(0.1)
    
end
%plots for changing of angles of each joint against time
    t=0:0.1:1.5;
    q1 = theta10 + 3*(theta1f-theta10)/(tf^2)*(t.^2) - 2*(theta1f-theta10)/(tf^3)*(t.^3);
    q2 = theta20 + 3*(theta2f-theta20)/(tf^2)*(t.^2) - 2*(theta2f-theta20)/(tf^3)*(t.^3);
    q3 = theta30 + 3*(theta3f-theta30)/(tf^2)*(t.^2) - 2*(theta3f-theta30)/(tf^3)*(t.^3);
    q4 = theta40 + 3*(theta4f-theta40)/(tf^2)*(t.^2) - 2*(theta4f-theta40)/(tf^3)*(t.^3);
    figure (3)
    plot(t,q1);
    hold on
    plot(t,q2);
    hold on
    plot(t,q3);
    hold on
    plot(t,q4);
    hold on
    grid on
    title('Joint position against time');
    xlabel('Time(s)') ; ylabel('Joint Position (radians)')
    legend('Theta1','Theta2','Theta3','Theta4');
%plots for Joint speed against time
    t=0:0.1:1.5;
    qv1 = 6*(theta1f-theta10)/(tf^2)*t - 3*2*(theta1f-theta10)/(tf^3)*(t.^2);
    qv2 = 6*(theta2f-theta20)/(tf^2)*t - 3*2*(theta2f-theta20)/(tf^3)*(t.^2);
    qv3 = 6*(theta3f-theta30)/(tf^2)*t - 3*2*(theta3f-theta30)/(tf^3)*(t.^2);
    qv4 = 6*(theta4f-theta40)/(tf^2)*t - 3*2*(theta4f-theta40)/(tf^3)*(t.^2);
    figure (4)
    plot(t,qv1);
    hold on
    plot(t,qv2);
    hold on
    plot(t,qv3);
    hold on
    plot(t,qv4);
    hold on
    grid on
    title('Joint speed against time');
    xlabel('Time(s)') ; ylabel('Joint speed (radian/s)');
    legend('Theta1dot','Theta2dot','Theta3dot','Theta4dot');
 %plots for Joint accelerations against time
    t=0:0.1:1.5;
    qa1 = 6*(theta1f-theta10)/(tf^2) - 12*(theta1f-theta10)/(tf^3).*t;
    qa2 = 6*(theta2f-theta20)/(tf^2) - 12*(theta2f-theta20)/(tf^3).*t;
    qa3 = 6*(theta3f-theta30)/(tf^2) - 12*(theta3f-theta30)/(tf^3).*t;
    qa4 = 6*(theta4f-theta40)/(tf^2) - 12*(theta4f-theta40)/(tf^3).*t;
    figure (5)
    plot(t,qa1);
    hold on
    plot(t,qa2);
    hold on
    plot(t,qa3);
    hold on
    plot(t,qa4);
    hold on
    grid on
    title('Joint accelerations against time');
    xlabel('Time(s)') ; ylabel('Joint Accelerations (radian/s^2)');
    legend('Theta1dotdot','Theta2dotdot','Theta3dotdot','Theta4dotdot');
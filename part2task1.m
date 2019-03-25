clear all
close all
clc

figure (1)
figure (2)


% parameters of links
ra = 170;
L = 130;
rplat = 130;
rbase = 290;
alph = 45;
alpha = alph*pi/180;
xc = 0;
yc = 0;
thetaPB = [pi+pi/6 pi/2 2*pi-pi/6];
xpb = [0 rbase/2*sqrt(3) rbase*sqrt(3)];
ypb = [0 rbase*1.5 0];

xpp = xc + rplat * cos(alpha + thetaPB) - rbase * cos(thetaPB);
ypp = yc + rplat * sin(alpha + thetaPB) - rbase * sin(thetaPB);

e1 = -2 * ypp *ra;
e2 = -2 * xpp * ra;
e3 = xpp.^2 + ypp.^2 + ra^2 - L^2;

t1 = (-e1 + sqrt(e1.^2 + e2.^2 - e3.^2))./(e3 - e2);
t2 = (-e1 - sqrt(e1.^2 + e2.^2 - e3.^2))./(e3 - e2);

theta1 = 2*atan(t1);
theta2 = 2*atan(t2);

cosfa1 = xc + rplat * cos(alpha + thetaPB) - rbase * cos(thetaPB) - ra * cos(theta1);
cosfa2 = xc + rplat * cos(alpha + thetaPB) - rbase * cos(thetaPB) - ra * cos(theta2);

sinfa1 = yc + rplat * sin(alpha + thetaPB) - rbase * sin(thetaPB) - ra * sin(theta1);
sinfa2 = yc + rplat * sin(alpha + thetaPB) - rbase * sin(thetaPB) - ra * sin(theta2);

fa1 = atan2(sinfa1,cosfa1);
fa2 = atan2(sinfa2,cosfa2);

%% plot robot
figure (1)
plot([0 rbase/2*sqrt(3)],[0 rbase*1.5],'b');
hold on
plot([rbase/2*sqrt(3) rbase*sqrt(3)],[rbase*1.5 0],'b');
hold on
plot([0 rbase*sqrt(3)],[0 0],'b');
hold on
grid on

xM = ra * cos(theta1) + xpb;
yM = ra * sin(theta1) + ypb;

xpp = ra * cos(theta1) + L * cos(fa1) + xpb;
ypp = ra * sin(theta1) + L * sin(fa1) + ypb;

for i = 1:3
    plot([xpb(i) xM(i)],[ypb(i) yM(i)],'-og')
    hold on
end

for i = 1:3
    plot([xM(i) xpp(i)],[yM(i) ypp(i)],'-ob')
    hold on
end

plot([xpp(1) xpp(2)],[ypp(1) ypp(2)],'-or')
hold on
plot([xpp(2) xpp(3)],[ypp(2) ypp(3)],'-or')
hold on
plot([xpp(3) xpp(1)],[ypp(3) ypp(1)],'-or')
hold on
axis([-300 600 -300 500])
hold on

%% plot robot
figure (2)
plot([0 rbase/2*sqrt(3)],[0 rbase*1.5],'b');
hold on
plot([rbase/2*sqrt(3) rbase*sqrt(3)],[rbase*1.5 0],'b');
hold on
plot([0 rbase*sqrt(3)],[0 0],'b');
hold on
grid on
xM = ra * cos(theta2) + xpb;
yM = ra * sin(theta2) + ypb;

xpp = ra * cos(theta2) + L * cos(fa2) + xpb;
ypp = ra * sin(theta2) + L * sin(fa2) + ypb;

for i = 1:3
    plot([xpb(i) xM(i)],[ypb(i) yM(i)],'-og')
    hold on
end

for i = 1:3
    plot([xM(i) xpp(i)],[yM(i) ypp(i)],'-ob')
    hold on
end

plot([xpp(1) xpp(2)],[ypp(1) ypp(2)],'-or')
hold on
plot([xpp(2) xpp(3)],[ypp(2) ypp(3)],'-or')
hold on
plot([xpp(3) xpp(1)],[ypp(3) ypp(1)],'-or')
hold on
axis([-300 600 -300 500])
hold on



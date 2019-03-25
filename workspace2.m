clear
close all
clc
ra = 170;
L = 130;
Lmax=ra+L;
Lmin=ra-L;
rplat = 130;
alpha = 60*pi/180;

rbase = 290;
topx=rbase/2*sqrt(3);
topy=rbase*1.5;
botx=rbase*sqrt(3);
boty=0;
plot([0 rbase/2*sqrt(3)],[0 rbase*1.5],'b');
hold on
plot([rbase/2*sqrt(3) rbase*sqrt(3)],[rbase*1.5 0],'b');
hold on
plot([0 rbase*sqrt(3)],[0 0],'b');
axis([-100 600 -100 450 0 1]);
xlabel('X') ; ylabel('Y')
hold on
grid on


xe = 0:600;
ye = 0:600;

for i = 1:1:601
    
    for j = 1:1:601
        
        A1Asqure = (xe(i) - rplat*cos(alpha+30*pi/180))^2 + (ye(j) - rplat*sin(alpha+30*pi/180))^2;
        B1Bsqure = (topx - ( xe(i)-rplat*sin(alpha) ))^2 + (topy-(ye(j)+rplat*cos(alpha)))^2;
        C1Csqure = (botx - (xe(i)+ rplat*cos(30*pi/180-alpha)))^2 + (boty-(ye(j)-rplat*sin(30*pi/180-alpha)))^2;
        
        if A1Asqure>=Lmin^2 && A1Asqure<=Lmax^2  
            
            if B1Bsqure>=Lmin^2 && B1Bsqure<=Lmax^2
                
                if C1Csqure>=Lmin^2 && C1Csqure<=Lmax^2
                    
                    plot(xe(i),ye(j),'.r')
                    hold on
                    
                end
            end
        end
    end
end


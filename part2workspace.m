clear
close all
clc
ra = 170;
L = 130;
Lmax=ra+L;
Lmin=ra-L;
rplat = 130;
alpha = 60;
rbase = 290;

plot([-rbase/2*sqrt(3) 0],[-rbase/2 rbase],'b')
hold on
plot([0 rbase/2*sqrt(3)],[rbase -rbase/2],'b')
hold on
plot([rbase/2*sqrt(3) -rbase/2*sqrt(3)],[-rbase/2 -rbase/2],'b')
hold on
grid on

xe = -300:300;
ye = -200:300;

for i = 1:1:601
    
    for j = 1:1:501
        
        A1Asqure = (xe(i) - rplat*cos((alpha+30)*pi/180) + rbase/2*sqrt(3))^2 + (ye(j) - rplat*sin((alpha+30)*pi/180) + rbase/2)^2;
        B1Bsqure = (xe(i) - rplat*sin(alpha*pi/180))^2 + (rbase-ye(j) - rplat*cos(alpha*pi/180) )^2;
        C1Csqure = (xe(i) + rplat*cos((30-alpha)*pi/180) - rbase/2*sqrt(3))^2 + (ye(j) - rplat*sin((30-alpha)*pi/180) + rbase/2)^2;
        
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



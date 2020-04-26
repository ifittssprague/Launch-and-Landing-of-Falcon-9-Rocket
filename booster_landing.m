function [success, D] = booster_landing()

%% call launch and initialize variables
[mf,x,y,vx,vy,ax,ay,theta,~] = launch_sumulation();
close all

Opt = 2;
xt = x(end)/4;
FT_max = 7000000;
FT = 0;
g = 9.81;

mfb = mf(end)+150000;
mb = 150000;
mdot_f = 1000;

xb = x(end);
yb = y(end);
vxb = vx(end);
vyb = vy(end);
axb = ax(end);
ayb = ay(end);
thetab = theta(end);

t = 0;
dt = 0.1;

%% while loop
j = 1;
sw = 0;

while yb(j,1) > 0

    [F_dx, F_dy] = force_of_air_drag(vxb(j,1),vyb(j,1),yb(j,1));
    F_W = 1600*rand(1) - 800;
    
    F_total_x = FT*cosd(thetab(j,1)) - F_dx*sign(vxb(j,1)) + F_W;
    F_total_y = FT*sind(thetab(j,1)) - (mb + mfb(j,1))*g - F_dy*sign(vyb(j,1));
    
    axb(j,1) = F_total_x/(mb + mfb(j,1));
    ayb(j,1) = F_total_y/(mb + mfb(j,1));
    
    vxb(j+1,1) = vxb(j,1) + axb(j,1)*dt;
    vyb(j+1,1) = vyb(j,1) + ayb(j,1)*dt;

    xb(j+1,1) = xb(j,1) + 0.5*(vxb(j,1) + vxb(j+1,1))*dt;
    yb(j+1,1) = yb(j,1) + 0.5*(vyb(j,1) + vyb(j+1,1))*dt;   
   
    if FT ~= 0
        mfb(j+1,1) = mfb(j,1) - mdot_f*dt;
        if mfb(j+1,1) < 0
            mfb(j+1,1) = 0;
        end
    else
        mfb(j+1,1) = mfb(j,1);
    end
    
    if Opt == 1
        thetab(j+1,1) = thetab(j,1);
    else
        % find theta and FT
        theta_v = 180/pi*atan2(vyb(j+1,1),vxb(j+1,1));
        theta_t = 180/pi*atan2(yb(j+1,1),xb(j+1,1)-xt);


        if vxb(j+1) > -127.5 && sw == 0 % abs(mod(abs(theta_t-theta_v),360) - 180) > 5
            dtheta = thetab(j,1) - theta_t + 180;
            if mfb(j,1) > 0      
                FT = FT_max;
            else
                FT = 0;
            end
        else
            sw = 1;
            if vxb(j+1,1) < -50
                dtheta = theta_t - thetab(j,1);
            else
                dtheta = 90 - thetab(j,1);
            end
            
            if mfb(j,1) > 0
                if vyb(j,1) < -68 && yb(j,1) < 50000
                    FT = FT_max;
                else
                    FT = 0;
                end
            else
                FT = 0;
            end
        end

        if abs(dtheta/dt) > 10
            dtheta = 10*dt*sign(dtheta);
        end

        thetab(j+1,1) = thetab(j,1) + dtheta;
        
        if thetab(j+1,1) >= 180
            thetab(j+1,1) = thetab(j,1);
        end
%         thetab(j+1,1) = max(70,thetab(j+1,1));
    end
          
    t = [t; t(end)+dt];
    
    j = j+1; 
    
    if rem(j,1000)==1 || yb(j,1) < 100
        a = 1;
    end
    
end

vx_f = vxb(end);
vy_f = vyb(end);
x_final = xb(end);
mf_final = mfb(end);
D = abs(xt - x_final);
if D < 500
    success = true;
else
    success = false;
end
plot(xb,yb)
axis equal

figure
plot(t,vxb)
hold on
plot(t,vyb,'r')
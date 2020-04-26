function [mf,x,y,vx,vy,ax,ay,theta,t] = launch_sumulation()

%% Given parameters
FT = 7000000;
mr = 300000;
mf = 400000;
mdot_f = 1000;
dt = 0.1;
g = 9.81;

%% initialize variables
t = 0;
x = 0;
y = 0;
vx = 0;
vy = 0;
ax = 0;
ay = 0;
theta = 90;
j = 1;

%% loop over time
while y(j) < 100000

    [F_dx, F_dy] = force_of_air_drag(vx(j,1),vy(j,1),y(j,1));
    
    F_total_x = FT*cosd(theta(j,1)) - F_dx;
    F_total_y = FT*sind(theta(j,1)) - (mr + mf(j,1))*g - F_dy;
    
    ax(j,1) = F_total_x/(mr + mf(j,1));
    ay(j,1) = F_total_y/(mr + mf(j,1));
    
    vx(j+1,1) = vx(j,1) + ax(j,1)*dt;
    vy(j+1,1) = vy(j,1) + ay(j,1)*dt;

    x(j+1,1) = x(j,1) + 0.5*(vx(j,1) + vx(j+1,1))*dt;
    y(j+1,1) = y(j,1) + 0.5*(vy(j,1) + vy(j+1,1))*dt;   
   
    mf(j+1,1) = mf(j,1) - mdot_f*dt;
    
    if y(j+1,1) > 50000 && theta(j,1) > 55
        theta(j+1,1) = theta(j,1) - 5*dt;  
    else
        theta(j+1,1) = theta(j,1);  
    end
    
    t = [t; t(end)+dt];
    
    j = j+1;
end

%% plot
plot(x,y)
hold on
ind_50 = min(find(y>=50000));
plot(x(ind_50,1),y(ind_50,1),'ro')
axis equal

figure
plot(t(1:end-1,1),ax)
hold on
plot(t(1:end-1,1),ay,'k')
plot(t(ind_50,1),ax(ind_50,1),'ro')
plot(t(ind_50,1),ay(ind_50,1),'ro')


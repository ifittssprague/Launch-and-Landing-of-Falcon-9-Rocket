function [F_dx, F_dy] = force_of_air_drag(vx,vy,h)

% density
if h <= 100000
    rho = -0.000015*h + 1.5;
else
    rho = 0;
end

% areas and Cd
Cd = 0.25;
Ax = 148.29;
Ay = 48.29;

% drag
F_dx = .5 * rho .* Cd .* Ax .* vx.^2;
F_dy = .5 * rho .* Cd .* Ay .* vy.^2;


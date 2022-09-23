y0 = [0.1; 0.1; 0]; % i convert the initial values for x,y into initial values for xp, yp
tspan = [0 18];

[t_out, robot_path] = ode45(@controller, tspan, y0);

s = t_out>=2.5;
xp = robot_path(:, 1);
yp = robot_path(:, 2);
plot(xp(s), yp(s), "-or", xp, yp, "g")
xlabel("x")
ylabel("y")
legend('sprayer path(xp, yp)', 'robot path(x, y)')
% note that the draw in solution_B is not identical to the draw in
% solution_A, why? as the line is shifted by 0.1 meter in some places as
% the x,y positions is shifted by 0.1 meter comparied to xp, yp

function x_dot = controller(t, state_vec)

  state_vec = zeros(3, 1);
  xp = state_vec(1);
  yp = state_vec(2);
  theta = state_vec(3);
  l = 0.1;

  dy_dt_ = pi*cos(t.*pi/10)+(pi/2)*sin(t.*pi/10); %obtained by differentiating y(t)
  dx_dt_ = -2*pi*sin(t.*pi/5)+(pi/2)*cos(t.*pi/10);%obtained by differentiating x(t)

  theta = atan(dy_dt_./dx_dt_);%obtained just by solving equations 1, 2
  v = dx_dt_./cos(theta); %given in equation 1
  
  w_numenator = (0.1*pi+0.2*pi*sin(t.*pi*0.3)+0.2*pi*sin(0.1*pi.*t)+cos(0.1*pi.*t)+0.1*pi*cos(0.3*pi.*t)+0.1*pi*cos(0.1*pi.*t));
  w_denuminator = (11+2*cos(t.*0.2*pi)+2*sin(t.*0.2*pi)-8*cos(t.*0.4*pi)-4*sin(t.*0.3*pi)-4*sin(t.*0.1*pi));
  w = w_numenator./w_denuminator; % obtained by differentiating theta 

  dx_dt = v.*cos(theta);%given in equation 1
  dy_dt = v.*sin(theta);%given in equation 2
  dtheta_dt = w;       %given in equation 3
  
  dxp_dt = dx_dt-l.*dtheta_dt.*sin(theta);%obtained by differentiating equation 4
  dyp_dt = dy_dt+l.*dtheta_dt.*cos(theta);%obtained by differentiating equation 5

  x_dot = [dxp_dt; dyp_dt; dtheta_dt];
end
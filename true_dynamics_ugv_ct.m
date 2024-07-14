function dx_dt = true_dynamics_ugv_ct(t, x, u)
    % continous-time update equation of the nominal dynamics in earth frame.
    % For a double integrator xdot = [xdot;ydot;xddot;yddot]
    x_1 = x(1);
    y = x(2);
    x1_dot = x(3);
    y_dot = x(4);
    
    % Control inputs
    u_x = u(1);  % Acceleration in x direction
    u_y = u(2);  % Acceleration in y direction
    
    % Dynamics equations for 2D double integrator
    x_ddot = u_x;
    y_ddot = u_y;
    
    % State derivatives
    dx_dt(1,1) = x1_dot + 0.1 * randn; 
    dx_dt(2,1) = y_dot; 
    dx_dt(3,1) = x_ddot; 
    dx_dt(4,1) = y_ddot;

end

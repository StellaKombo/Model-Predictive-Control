function dx_dt = true_dynamics_ugv_ct(t, x, u)
    % continous-time update equation of the nominal dynamics in earth frame.
    
    % Extract the current heading angle from the state vector x
    %theta_position = x(3,:);
    % Since we are using a linear MPc we have to use the new state
    % variables

    % Extract the current state variables
    x_val = x(1);
    y_val = x(2);
    xdot_val = x(3);
    ydot_val = x(4);

    % Extract linear and angular velocity from the control input u
    v_val = u(1);
    w_val = u(2);


    % % Extract perturbations (process noise)
    %w_y = perturbations(2);
    % w_xdot = perturbations(3);
    % w_ydot = perturbations(4);

    % Initialize the rate of change of the state vector (dx/dt)
    dx_dt = zeros(size(x));

    % % Update the rate of change of the state vector based on kinematic equations
    dx_dt(1,:) = xdot_val + randn*0.1;
    %dx_dt(2,:) = ydot_val + w_y;
    % dx_dt(3,:) = v_val + w_xdot;
    % dx_dt(4,:) = w_val + w_ydot;

    % Update the rate of change of the state vector based on kinematic equations
    %dx_dt(1,:) = xdot_val;
    dx_dt(2,:) = ydot_val;
    dx_dt(3,:) = v_val;
    dx_dt(4,:) = w_val;

end

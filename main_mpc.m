clear all; clc; close all;
% Main code of the simulation for autonomous motion tracking and model
% discrepancies.
global dt; 
dt = 0.05;

% time of the simulation
tsim_end = 100;
tsim = 0:dt:tsim_end;
t_step = 1;
N_mpc = 1000;
n_horizon = 20;

% provide intialization to your simulator, state, have a initial safety

% Define an initial buffer
r_ego = 0.3; % robot radius
r_map = 0.05;  % define the grid map resolution
r_delta_t = 0.01;
r_s = r_ego + r_delta_t;
r0 = 0.001;

initial_epsilon = 0.005;
epsilon_desired = 0.002;


% provide a warm start path - differential flatness 
% load or run build_global_occupancy.

   %% Step 0: Obtain a local occupancy map centered around the vehicle current position
    % Load the saved global occupancy map
    load('global_occupancy_map.mat');
    
    % Visualize the initial occupancy map
    figure;
    imagesc(occupancy_Map_global)
    title('Global local occupancy map');

    % Define the current position of the vehicle
    curr_x_position = 5.0;
    curr_y_position = 5.0;
    
    % Define the local map dimensions
    local_x_min = -1*(curr_x_position - r_map/2);
    local_x_max = curr_x_position + r_map/2;
    local_y_max = curr_y_position + r_map/2;
    local_y_min = -1*(curr_y_position - r_map/2);
    
    % Create a grid coordinate for the local occupancy map
    x_grid = local_x_min:r_map:local_x_max;
    y_grid = local_y_min:r_map:local_y_max;

    % Initialize the local occupancy map with the relevant portion of the
    % global map
    map_grid = zeros(length(x_grid),length(y_grid));

    % Compute grid coordinates for the local map
    local_x_indices = find(x_grid >= local_x_min & x_grid <= local_x_max);
    local_y_indices = find(y_grid >= local_y_min & y_grid <= local_y_max);

    % Extract the local map from the global map
    local_occupancy_map = occupancy_Map_global(local_x_indices, local_y_indices);

    % Visualize the local occupancy map
    figure;
    imagesc(local_occupancy_map');
    title('Local Occupancy Map without Obstacles');
    
    % Define the obstacles position
    x_obstacle1 = 0.99816;
    y_obstacle1 = 3.873;
    r_obstacle1 = 0.5;
    obstacle_1_width = 0.5;
    obstacle_1_length = 0.2;

    x_obstacle2 = 1.4;
    y_obstacle2 = -1.2;
    obstacle_2_width = 0.2;
    obstacle_2_length = 0.3;
    r_obstacle2 = 0.1;

    x_obstacle3 = 2.8;
    y_obstacle3 = -2.8;
    obstacle_3_width = 0.8;
    obstacle_3_length = 0.1;

    r_val1 = r_obstacle1 + r_ego;
    r_val2 = r_obstacle2 + r_ego;

    obstacle1_xindex = (x_obstacle1 - local_x_min)/r_map;
    obstacle1_yindex = (y_obstacle1 - local_y_min)/r_map;
    obstacle1_xindex_range = (obstacle_1_length + 2*r_ego)/r_map;
    obstacle1_yindex_range = (obstacle_1_width + 2*r_ego)/r_map;
    obstacle2_xindex = (x_obstacle2 - local_x_min)/r_map;
    obstacle2_yindex = (y_obstacle2 - local_y_min)/r_map;
    obstacle2_xindex_range = (obstacle_2_length + 2*r_ego)/r_map;
    obstacle2_yindex_range = (obstacle_2_width + 2*r_ego)/r_map;

    obstacle3_xindex = (x_obstacle3 - local_x_min)/r_map;
    obstacle3_yindex = (y_obstacle3 - local_y_min)/r_map;
    obstacle3_xindex_range = (obstacle_3_length + 2*r_ego)/r_map;
    obstacle3_yindex_range = (obstacle_3_width + 2*r_ego)/r_map;

    map_grid(floor(-obstacle1_yindex_range/2 + obstacle1_yindex):ceil(obstacle1_yindex_range/2 + obstacle1_yindex), floor(-obstacle1_xindex_range/2 + obstacle1_xindex):ceil(obstacle1_xindex_range/2 + obstacle1_xindex)) = 1;
    map_grid(floor(-obstacle2_yindex_range/2 + obstacle2_yindex):ceil(obstacle2_yindex_range/2 + obstacle2_yindex), floor(-obstacle2_xindex_range/2 + obstacle2_xindex):ceil(obstacle2_xindex_range/2 + obstacle2_xindex)) = 1;
    map_grid(floor(-obstacle3_yindex_range/2 + obstacle3_yindex):ceil(obstacle3_yindex_range/2 + obstacle3_yindex), floor(-obstacle3_xindex_range/2 + obstacle3_xindex):ceil(obstacle3_xindex_range/2 + obstacle3_xindex)) = 1;
    
    % Visualize the local occupancy map with the obstacles available
    %figure
    %imagesc(map_grid);
    title('Local Occupancy Map with Obstacles');
    xlabel('X - axis');
    ylabel('Y - axis');
    set(gca,'YDir','normal');
    axis equal;

   %% Step 1: build a local cost map using the local occpuancy map and the r_s
    % Inflation logic
    concentrated_size = ceil(r_s / r_map);
    inflated_size = ceil(r_s / r_map);
    n = 2;

    % Create an empty inflated_cost_map with increased dimensions
    inflated_cost_map = zeros(size(map_grid) + [inflated_size * n ,inflated_size * n]);
    inflated_cost_map(inflated_size + 1:inflated_size + length(map_grid(1,:)), inflated_size + 1:inflated_size+length(map_grid(:,1))) = map_grid;

    % Add larger inflation values to the inflated map based on the concentrated size
    for i = -concentrated_size:concentrated_size
        for j = -concentrated_size:concentrated_size
        inflated_cost_map(inflated_size + 1 +i:inflated_size + i + length(map_grid(1,:)), inflated_size + 1 + j:inflated_size + j + length(map_grid(:,1))) =  inflated_cost_map(inflated_size + 1 + i:inflated_size + i + length(map_grid(1,:)), inflated_size + 1 + j:inflated_size + j+ length(map_grid(:,1)))+ (0.05)*map_grid;
        end
    end

    % Add smaller inflation values to the inflated map based on the inflated size
    for i = -inflated_size:inflated_size
       for j = -inflated_size:inflated_size
        inflated_cost_map(inflated_size+1+i:inflated_size+i+length(map_grid(1,:)), inflated_size+1+j:inflated_size+j+length(map_grid(:,1))) =  inflated_cost_map(inflated_size+1+i:inflated_size+i+length(map_grid(1,:)), inflated_size+1+j:inflated_size+j+length(map_grid(:,1)))+ 0.000001*map_grid;
       end
    end
    
    % Compute the final cost_map_grid by taking the minimum values, excluding the padded regions
    cost_map_grid = min(inflated_cost_map (inflated_size + 1: end-inflated_size, inflated_size + 1: end-inflated_size), 1);
    
    % Visualize the local inflated cost map with obstacles
    figure;
    imagesc(cost_map_grid);
    title('Local inflated cost map with obstacles');
    xlabel('X - axis');
    ylabel('Y - axis');
    set(gca,'YDir','normal');
    axis equal;

    %% Step 2 - Define the trajectory

    t_trajectory = tsim;

    % Circle variables
    radius = 4.0;
    center = [0 , 0];

    x_traj_ref = center(1) + radius * cos(t_trajectory);
    
    y_traj_ref = center(2) + radius * sin(t_trajectory);

    dot_x_traj_ref = radius * -1 * sin(t_trajectory);
    dot_y_traj_ref = radius * cos(t_trajectory);

    ddot_x_traj_ref = radius * cos(t_trajectory);
    ddot_y_traj_ref = radius * -1 * sin(t_trajectory);

    for i = 1:length(t_trajectory)
        theta_traj_ref(i) = atan2(dot_y_traj_ref(i),dot_x_traj_ref(i));
    end

    for i = 1:length(t_trajectory)
        if (sin(theta_traj_ref(i)) == 0)
            u1_traj_ref(i) = dot_x_traj_ref(i)/cos(theta_traj_ref(i));
        else
            u1_traj_ref(i) = dot_y_traj_ref(i)/sin(theta_traj_ref(i));
        end
        if u1_traj_ref(i)~=0
            u2_traj_ref(i) = (ddot_x_traj_ref(i) * sin(theta_traj_ref(i)) - ddot_y_traj_ref(i) * cos(theta_traj_ref(i)))/(-u1_traj_ref(i));
        else
            u2_traj_ref(i) = 1.0/(1.0 + theta_traj_ref(i)^2); 
        end

    end

    curr_x = [x_traj_ref; y_traj_ref; dot_x_traj_ref; dot_y_traj_ref];

    initial_position = curr_x(:,1);

    %% Step 3 - MPC
    % MPC with the double integrator for a simple linear system with
    % constant A and B matrices

    % define the mpc parameters
    number_states = 4;
    number_controls = 2;

    % State vector x = [x;y;xdot;ydot]
    x_vec = [x_traj_ref; y_traj_ref; dot_x_traj_ref; dot_y_traj_ref];

    % Continuous-time dynamics xdot = [xdot;ydot;xddot;yddot]
    xdot = [dot_x_traj_ref; dot_y_traj_ref; ddot_x_traj_ref ;ddot_y_traj_ref];

    % State - transition matrix A = [0 I;0 0]
    A_continuous = [[zeros(2,2) eye(2)]; [zeros(2,2)  zeros(2,2)]];
    

    % Input matrix (assuming u = [ddot_x; ddot_y]), B = [0;I]
    B_continuous = [zeros(2,2); eye(2)];
    new_mat = [[A_continuous B_continuous];[zeros(2,4) zeros(2,2)]];

    % Discretization using matrix exponential
    new_mat_exp = expm(new_mat * dt);

    % Extract the linear matrices of the A and B parts of the MPC
    Ad_val = new_mat_exp(1:4, 1:4);
    Bd_val = new_mat_exp(1:4, 5:6);

    Aineq1 = sdpvar(repmat(2,1,n_horizon-1),repmat(1,1,n_horizon-1));
    bineq1 = sdpvar(repmat(1,1,n_horizon-1),repmat(1,1,n_horizon-1));

  
    % Decision variables
    Cd = [1,0,0;0,1,0];

    % Create a figure outside the loop
    h = figure;
    
    % Define YAMLIPS decision variables: squared real valued matrices of u and x
    u_var = sdpvar(repmat(number_controls, 1, n_horizon - 1), repmat(1, 1, n_horizon - 1));
    x_var = sdpvar(repmat(number_states, 1, n_horizon), repmat(1, 1, n_horizon));
  
    
    % define the reference variable
    r_var =  sdpvar(repmat(number_states,1,n_horizon),repmat(1,1,n_horizon));


    % Objective functions
    objective = 0; 
    constraints = []; 
    Q_cost = diag([50,10,100,100]); 
    R_cost = diag([0.01,0.01]);

    % Riccati equations for optimal control matrices
    % [P, ~, K] = idare(Ad_val, Bd_val, Q_cost, R_cost);
    % K_optimal = K;

    % Optimizer and solver
    for k = 2:n_horizon  
        objective = objective + (x_var{k} - r_var{k})' * Q_cost * (x_var{k} - r_var{k});
    end

    for k = 1:(n_horizon - 1)
        % Riccati-based optimal control law
        %u_var{k} = -K_optimal * (x_var{k} - r_var{k})';

        objective = objective + u_var{k}' * R_cost * u_var{k};
        constraints = [constraints, x_var{k + 1} == Ad_val * x_var{k} + Bd_val * u_var{k}];
        constraints = [constraints, Aineq1{k}' * x_var{k+1}(1:2) <= bineq1{k}];
  
    end


    % Define the parameters going into the controller
    parameters_in = {x_var{1}, r_var{:}, Aineq1{:},bineq1{:}};
    solutions_out = {[x_var{:}],[u_var{:}]};
    controller_straps = optimizer(constraints, objective,sdpsettings('solver','gurobi','gurobi.qcpdual',1, 'verbose',1,'debug',1),parameters_in,solutions_out);
    counter = 1;
    initial_position_bar = [x_traj_ref(1:n_horizon);y_traj_ref(1:n_horizon);dot_x_traj_ref(1:n_horizon); dot_y_traj_ref(1:n_horizon)];
    initial_position_trajectory = zeros(length(tsim), 4); % Assuming 4D position for ancilliary controller

    % Create a figure outside the loop for combined trajectory
    h_combined = figure;
    N_score = 10;
    non_conformity_scores = zeros(1, N_score);
    epsilon_values = zeros(1, N_score);


while (counter < (length(tsim) - 1 - n_horizon))
        input_list = {};
        input_list{1} = initial_position; % initialize the robot position to be the first position in the reference trajectory        sub_counter = 2; 
        sub_counter = 2;    
        for i = 1:n_horizon
            input_list{sub_counter} = [x_traj_ref(counter + i); y_traj_ref(counter + i); dot_x_traj_ref(counter + i); dot_y_traj_ref(counter + i)];
            sub_counter = sub_counter + 1;
        end

        m_term1 = []; m_term2 = [];  b_term_1 = []; b_term_2 = [];
        for iii = 2:n_horizon
            pbar_k = initial_position_bar(1:2,iii-1);                         
            m_term1(:,iii-1) = -(pbar_k - [x_obstacle1;y_obstacle1]);
        end
        for i = 1: n_horizon -1
            input_list{sub_counter} = m_term1(:,i);
            sub_counter = sub_counter+ 1;
        end
        for i = 1: n_horizon
            input_list{sub_counter} = - b_term_1(i);
            sub_counter = sub_counter + 1;
        end
       
        [solutions, diagnostics] = controller_straps{input_list}; 
        x_qp_solution = solutions{1};
        u_qp_solution = solutions{2}; % determine the u_mpc from the optimization
        u_mpc = u_qp_solution;
        u_current_point = u_qp_solution(:,1); % u at the current trajectory
        x_mpc = x_qp_solution; % position obtained when the mpc kicks in
        
        % Define the ancilliary controller lqr that kicks in after the mpc
        % to make sure within the horizon there is some reduction of the
        % oscillations
        Ts = 0.001; % define the 
        [Kd, ~, ~] = lqrd(Ad_val, Bd_val, Q_cost, R_cost,Ts); 
        difference_traj = initial_position - x_mpc;
        u_lqr = Kd * difference_traj;
        u_total = u_current_point + 0*u_lqr; % there seems to be an issue with the lqr where it destabilizes the system instead of making it work.
       
        %% Step 6 - Forward simulation with the combined new controller
        options = odeset('RelTol', 1e-3, 'AbsTol', 1e-6);
        perturbations = 0.01;
        [t, qt_ode] = ode45(@(t, qt) true_dynamics_ugv_ct(t, qt, u_total, perturbations),[tsim(counter),tsim(counter + 1)],initial_position);
        contoller_path_output = qt_ode;
        initial_position = qt_ode(end,:)'; % the last position of the new total controller is the new position

        initial_position_trajectory(counter + 1, :) = initial_position;

        % Compute Euclidean distance between the last step of LQR and first step of MPC
        %non_conformity_combined(counter) = norm(contoller_path_output(end,:) - contoller_path_output(1,:));
   
        %% ADP
        initial_epsilon = 0.1; % Initial value for epsilon
        learning_rate = 0.01; % Learning rate for adaptive epsilon updating

       % Assuming you have conformity scores for each cell, you can populate the cell array
       % if statement sbout the threshold of the cells are computed then we
       % can ignore.
       % print if statements that include the predicted radius, the
       % conformity scores and the 
       % Update non - conformity scores array
       % Compute non-conformity score for the current time ste
       current_non_conformity = norm(contoller_path_output(end,:) - contoller_path_output(1,:),2);
       non_conformity_scores(1:end-1) = non_conformity_scores(2:end);
       non_conformity_scores(end) = current_non_conformity;
          
       % Print non-conformity scores for analysis or tracking
       fprintf('Non-conformity scores: %s\n', mat2str(non_conformity_scores));

       sort_conf_score = sort(non_conformity_scores);
       qcount = ceil((N_score + 1) * (1 - initial_epsilon));
       Z = sort_conf_score(qcount);
       Z_perp = null(Bd_val); % doesnt exist because the matrix is full column rank
       % Update tube radius based on Z
       % r0 = sqrt(norm(Z, 2)^2 / 4); need to calculate after i find
       % the value of alpha
       r_delta_t = sqrt(norm(Z, 2)^2 / 4); 
       %epsilon_values(t_step) = initial_epsilon;
       
       % Apply threshold to get boolean condition
       boolean_condition = current_non_conformity >= Z;
       % Map boolean condition to 0 or 1
       R_binary = double(boolean_condition);
       initial_epsilon = initial_epsilon + learning_rate * (epsilon_desired - R_binary);
       
       % Rewrite the array of the epsilon values
       epsilon_values(1:end-1) = epsilon_values(2:end);
       epsilon_values(end) = initial_epsilon;
         
       fprintf('epsilon values: %s\n', mat2str(epsilon_values));
       fprintf('desired epsilon value: %s\n', mat2str(epsilon_desired));


        %% Step 7 - Visualizations
        figure(h_combined);
        % Plotting the fixed trajectory
        plot(x_traj_ref, y_traj_ref, '--', 'LineWidth', 2, 'DisplayName', 'Reference Trajectory');
        hold on;

        % Plot the trajectroy followed by the controller based on the MPC
        % optimization
        plot(initial_position_trajectory(:, 1), initial_position_trajectory(:, 2), 'go', 'LineWidth', 2, 'DisplayName', 'Controller Trajectory');

        % scatter(initial_position(1), initial_position(2), 100, 'filled', 'MarkerFaceColor', 'r', 'DisplayName', 'Initial Position');
        
        % Plot the obstacles
        scatter(x_obstacle1, y_obstacle1, 'k', 'LineWidth', 2, 'DisplayName', 'Obstacle 1');
        scatter(x_obstacle2, y_obstacle2, 'k', 'LineWidth', 2, 'DisplayName', 'Obstacle 2');
        scatter(x_obstacle3, y_obstacle3, 'k', 'LineWidth', 2, 'DisplayName', 'Obstacle 3');

        title('Combined Controller Trajectory vs Reference Trajectory');
        xlabel('X - axis');
        ylabel('Y - axis');
        legend('Location', 'Best');
        grid on;
        axis equal;

        % Capture frame
        frames_combined(counter) = getframe(h_combined);
        % Introduce a delay to observe the animation
        pause(0.01);
        % Draw the animation
        drawnow;
        hold off;

        counter = counter + 1;
   
end

figure();
% Plot the adaptive epsilon values after the loop
figure;
plot(1:N_score * (length(tsim) - 1 - n_horizon), epsilon_values, 'o-', 'LineWidth', 2);
title('Adaptive Epsilon Evolution');
xlabel('Iteration');
ylabel('Epsilon Value');
grid on;
function cubicTrajectory(q_i, q_f, t_i, t_f, v_i, v_f)
    % cubicTrajectory - Computes and plots a cubic trajectory for a robot joint.
    %
    % Syntax: cubicTrajectory(q_i, q_f, t_i, t_f, v_i, v_f)
    %
    % Inputs:
    %   q_i  - Initial joint position (e.g., in radians or degrees)
    %   q_f  - Final joint position
    %   t_i  - Initial time (seconds)
    %   t_f  - Final time (seconds)
    %   v_i  - Initial joint velocity
    %   v_f  - Final joint velocity
    %
    % Outputs:
    %   Plots of position, velocity, and acceleration profiles.
    %
    % Example:
    %   cubicTrajectory(0, 1, 0, 2, 0, 0.5);

    %% Parameters
    T = t_f - t_i; % Total duration
    Delta_q = q_f - q_i;

    %% Determine the cubic coefficients
    % We assume a cubic polynomial of the form:
    %   q(t) = a0 + a1*(t-t_i) + a2*(t-t_i)^2 + a3*(t-t_i)^3
    % Boundary conditions:
    %   q(t_i)   = q_i   ->  a0 = q_i
    %   qdot(t_i)= v_i   ->  a1 = v_i
    %   q(t_f)   = q_f   ->  q_i + v_i*T + a2*T^2 + a3*T^3 = q_f
    %   qdot(t_f)= v_f   ->  v_i + 2*a2*T + 3*a3*T^2 = v_f

    % Solving these gives:
    a0 = q_i;
    a1 = v_i;
    a3 = (v_f - v_i - 2*Delta_q/T) / T^2;
    a2 = (3*Delta_q/T - 2*v_i - v_f) / T;

    %% Define the symbolic variable and expressions
    syms t real
    tau = t - t_i;  % time shift so that tau=0 corresponds to t_i

    % Position
    q_sym = a0 + a1*tau + a2*tau^2 + a3*tau^3;
    % Velocity (first derivative)
    qd_sym = diff(q_sym, t);
    % Acceleration (second derivative)
    qdd_sym = diff(qd_sym, t);

    %% Convert symbolic expressions to MATLAB functions for evaluation
    q_fun   = matlabFunction(q_sym, 'Vars', t);
    qd_fun  = matlabFunction(qd_sym, 'Vars', t);
    qdd_fun = matlabFunction(qdd_sym, 'Vars', t);

    %% Time vector for plotting
    numPoints = 100;
    time_vec = linspace(t_i, t_f, numPoints);

    %% Evaluate the profiles
    q_vals   = q_fun(time_vec);
    qd_vals  = qd_fun(time_vec);
    qdd_vals = qdd_fun(time_vec);

    %% Plot the results
    figure;

    subplot(3,1,1);
    plot(time_vec, q_vals, 'b', 'LineWidth', 2);
    grid on;
    xlabel('Time (s)');
    ylabel('Position');
    title('Joint Position Profile');
    set(gca, 'FontSize', 12);

    subplot(3,1,2);
    plot(time_vec, qd_vals, 'r', 'LineWidth', 2);
    grid on;
    xlabel('Time (s)');
    ylabel('Velocity');
    title('Joint Velocity Profile');
    set(gca, 'FontSize', 12);

    subplot(3,1,3);
    plot(time_vec, qdd_vals, 'k', 'LineWidth', 2);
    grid on;
    xlabel('Time (s)');
    ylabel('Acceleration');
    title('Joint Acceleration Profile');
    set(gca, 'FontSize', 12);

    set(gcf, 'Position', [100, 100, 600, 800]); % Increase figure height

    %% Display the evaluated expressions (optional)
    disp('Position q(t):');
    disp(vpa(q_sym, 6));
    disp('Velocity qdot(t):');
    disp(vpa(qd_sym, 6));
    disp('Acceleration qddot(t):');
    disp(vpa(qdd_sym, 6));
end

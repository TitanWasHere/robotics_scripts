function sol = eq_solver(qin, qfin, qin_dot, qfin_dot)
    % Define symbolic variables
    % Define symbolic variables
    syms a0 a1 a2 a3 s q_i q_f q_i_dot q_f_dot real
    
    % Define polynomial equations
    q_s = a0 + a1 * s + a2 * s^2 + a3 * s^3;
    q_s_derivative = diff(q_s, s);
    
    % Define boundary conditions
    eq1 = subs(q_s, s, 0) == q_i;
    eq2 = subs(q_s, s, 1) == q_f;
    eq3 = subs(q_s_derivative, s, 0) == q_i_dot;
    eq4 = subs(q_s_derivative, s, 1) == q_f_dot;

    % Solve for coefficients
    solution = solve([eq1, eq2, eq3, eq4], [a0, a1, a2, a3]);
    
    % General solution (symbolic)
    sol = subs(q_s, [a0, a1, a2, a3], [solution.a0, solution.a1, solution.a2, solution.a3]);
    
    % Display results
    disp('Solution for coefficients:');
    disp(['a0 = ', char(solution.a0)]);
    disp(['a1 = ', char(solution.a1)]);
    disp(['a2 = ', char(solution.a2)]);
    disp(['a3 = ', char(solution.a3)]);
    
    fprintf('\nSolution:\n q(s) = %s\n', char(sol));

    % Determine number of joints based on input size
    num_joints = length(qin);
    
    % Initialize solution storage
    sol = sym(zeros(1, num_joints));
    
    % Loop through each joint and solve for the coefficients
    for i = 1:num_joints
        % Define polynomial trajectory
        q_s = a0 + a1 * s + a2 * s^2 + a3 * s^3;
        q_s_derivative = diff(q_s, s);
        
        % Define boundary conditions for the current joint
        eq1 = subs(q_s, s, 0) == qin(i);
        eq2 = subs(q_s, s, 1) == qfin(i);
        eq3 = subs(q_s_derivative, s, 0) == qin_dot(i);
        eq4 = subs(q_s_derivative, s, 1) == qfin_dot(i);

        % Solve for coefficients
        solution = solve([eq1, eq2, eq3, eq4], [a0, a1, a2, a3]);
        
        % Store the joint trajectory
        sol(i) = subs(q_s, [a0, a1, a2, a3], [solution.a0, solution.a1, solution.a2, solution.a3]);
        
         % Display numeric (decimal) coefficients
        disp('Numeric Coefficients:');
        disp(['a0 = ', char(vpa(solution.a0, 6))]);
        disp(['a1 = ', char(vpa(solution.a1, 6))]);
        disp(['a2 = ', char(vpa(solution.a2, 6))]);
        disp(['a3 = ', char(vpa(solution.a3, 6))]);
        fprintf('Numeric q%d(s) = %s\n\n', i, char(vpa(sol(i), 6)));
    end
    
    % If numerical values are given, plot the trajectories
    if nargin == 4
        figure;
        hold on;
        for i = 1:num_joints
            q_func = matlabFunction(sol(i)); % Convert symbolic to function
            fplot(q_func, [0, 1], 'LineWidth', 2);
        end
        xlabel('s');
        ylabel('q(s)');
        title('Joint Trajectories');
        legend(arrayfun(@(j) sprintf('Joint %d', j), 1:num_joints, 'UniformOutput', false));
        grid on;
        hold off;
    end
    sol = vpa(sol, 6);
end

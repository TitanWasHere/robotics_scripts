function [q_out, guesses, cartesian_errors] = newton(q_in, desired_point, f_r, initial_guess, max_iterations, max_cartesian_error, min_joint_increment, max_closeness_singularity)
% [q_out, guesses, cartesian_errors] = newton(q_in, desired_point, f_r, 
%  initial_guess, max_iterations, max_cartesian_error, min_joint_increment, 
%  max_closeness_singularity) takes as inputs:
%   -q_in: The variables of the joints, e.g. [q1;q2; q3; q4] (symbolic)
%   -desired_point: the configuration we wish to reach
%   -f_r: The mapping from joints to points
%   -initial_guess: Initial configuration of joints
%   -max_iterations: The maximum number of iterations we can perform
%   -max_cartesian_error: The level of precision we wish to reach
%   -min_joint_increment: (Optional) The minimum level of increment of accuracy 
%                          between successive iterations. Default = 1e-5.
%   -max_closeness_singularity: (Optional) How close to a singularity we can get.
%                                Default = 1e-4.
% and outputs:
%   -q_out: The best reached configuration
%   -guesses: The history of tested configurations
%   -cartesian_errors: The history of errors

    % Set default values if optional arguments are not provided
    if nargin < 7
        min_joint_increment = 1e-5;
    end
    if nargin < 8
        max_closeness_singularity = 1e-4;
    end

    % Compute the Jacobian
    J = jacobian(f_r, q_in);
    
    % Check system type (redundant or non-redundant)
    n_dof = length(q_in);
    n_config = length(desired_point);
    
    simple_case = n_config == n_dof;
    
    if n_config < n_dof
        disp("Redundant case, let's use the pseudo inverse");
        J_inv = pinv(J);
    else
        disp("Non-redundant case, let's use the inverse");
        J_inv = inv(J);
    end
    
    % Initialize variables
    guesses = zeros(max_iterations + 1, n_dof);
    cartesian_errors = zeros(1, max_iterations + 1);
    guess = initial_guess;
    
    % Iterative Newton-Raphson method
    for i = 1:max_iterations
        guesses(i, :) = guess;
        
        % Compute Cartesian error
        error = norm(desired_point - subs(f_r, q_in, guess));
        cartesian_errors(i) = error;
        
        if error < max_cartesian_error
            fprintf("Finished at iteration %d because the error was lower than the specified amount.\n", i-1);
            break
        end
        
        % Check for singularity
        if simple_case
            if abs(det(subs(J, q_in, guess))) <= max_closeness_singularity
                fprintf("Finished at iteration %d because too close to a singularity.\n", i);
                break
            end
        else
            [~, D, ~] = svd(subs(J, q_in, guess));
            if min(min(D)) >= D(1, 1) - max_closeness_singularity
                fprintf("Finished at iteration %d because too close to a singularity.\n", i);
                break
            end
        end
        
        % Compute the next guess
        new_guess = guess + subs(J_inv, q_in, guess) * (desired_point - subs(f_r, q_in, guess));
        
        % Check for minimal increment
        if norm(new_guess - guess) <= min_joint_increment
            fprintf("Finished at iteration %d because of a too little joints values increment.\n", i);
            break
        end
        
        guess = eval(new_guess);
        fprintf("[q_%d]: \n", i)
        display(guess)
        fprintf("[error]: %d \n", cartesian_errors(i))
    end
    
    % Trim outputs
    guesses = guesses(1:i, :);
    cartesian_errors = cartesian_errors(1:i);
    q_out = guess;
    
    
    
    % If the loop finishes without reaching the specified error tolerance, notify that the algorithm did not converge
    if cartesian_errors(i) >= max_cartesian_error
        fprintf("[ERROR]: Does not converge in %d iterations \n", max_iterations );
    end
    fprintf("[error_norm]: %f\n", cartesian_errors(i))

    fprintf("Point found:\n")
    display(eval(subs(f_r, q_in, q_out)))
end

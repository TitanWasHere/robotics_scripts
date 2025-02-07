function [q_out, guesses, cartesian_errors] = gradient_method(q_in, desired_point, f_r, initial_guess, max_iterations, max_cartesian_error, learning_rate, min_joint_increment, max_closeness_singularity)
% [q_out, guesses, cartesian_errors] = gradient_method(q_in, desired_point, f_r, 
%  initial_guess, max_iterations, max_cartesian_error, learning_rate, 
%  min_joint_increment, max_closeness_singularity) takes as inputs:
%   -q_in: The variables of the joints, e.g. [q1 q2 q3 q4]
%   -desired_point: the configuration we wish to reach
%   -f_r: The mapping from joints to points
%   -initial_guess: Initial configuration of joints
%   -max_iterations: The maximum number of iterations we can perform
%   -max_cartesian_error: The level of precision we wish to reach
%   -learning_rate: The step size to move along the gradient direction
%   -min_joint_increment: (Optional) The minimum level of increment of accuracy 
%                          between successive iterations. Default = 1e-5.
%   -max_closeness_singularity: (Optional) How close to a singularity we can get.
%                                Default = 1e-4.
% and outputs:
%   -q_out: The best reached configuration
%   -guesses: The history of tested configurations
%   -cartesian_errors: The history of errors

    % Set default values if optional arguments are not provided
    if nargin < 8
        min_joint_increment = 1e-5;
    end
    if nargin < 9
        max_closeness_singularity = 1e-4;
    end
    
    % Compute the Jacobian
    J = jacobian(f_r, q_in);
    
    % Initialize variables
    guesses = zeros(max_iterations + 1, length(q_in));
    cartesian_errors = zeros(1, max_iterations + 1);
    guess = initial_guess;
    
    % Iterative gradient descent method
    for i = 1:max_iterations
        guesses(i, :) = guess;
        
        % Compute the Cartesian error
        error = norm(desired_point - subs(f_r, q_in, guess));
        cartesian_errors(i) = error;
        
        if error < max_cartesian_error
            fprintf("Finished at iteration %d because the error was lower than the specified amount.\n", i);
            break
        end
        
        % Check for singularity
        if abs(det(subs(J, q_in, guess))) <= max_closeness_singularity
            fprintf("Finished at iteration %d because too close to a singularity.\n", i);
            break
        end
        
        % Compute the gradient of the error w.r.t. the joint angles
        error_gradient = -subs(J, q_in, guess)' * (desired_point - subs(f_r, q_in, guess));
        
        % Update the guess using the gradient descent rule
        new_guess = guess + learning_rate * error_gradient';
        
        % Check for minimal increment
        if norm(new_guess - guess) <= min_joint_increment
            fprintf("Finished at iteration %d because of a too little joints values increment.\n", i);
            break
        end
        
        guess = new_guess;
    end
    
    % Trim outputs
    guesses = guesses(1:i, :);
    cartesian_errors = cartesian_errors(1:i);
    q_out = guess;
    
    % If the loop finishes without reaching the specified error tolerance, notify that the algorithm did not converge
    if cartesian_errors(i) >= max_cartesian_error
        fprintf("The algorithm did not converge within the specified number of iterations.\n");
    end
end

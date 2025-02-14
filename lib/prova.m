clc; clear; close all;

% Given parameters
r = 0.5;                   % Radius of circular arc [m]
L = 0.8;                   % Arc length [m]
theta_i = atan(0.3/0.4);   % Initial angle [rad]

% Velocity and acceleration constraints
s_max_dot = 0.5;           % Maximum velocity [m/s]
s_max_ddot = 1.5;          % Maximum acceleration [m/s^2]

% Compute time phases
t1 = s_max_dot / s_max_ddot;   % Acceleration time [s]
s1 = 0.5 * s_max_ddot * t1^2;  % Distance covered in acceleration [m]
s2 = L - 2 * s1;               % Coasting distance [m]
t2 = s2 / s_max_dot;           % Coasting time [s]
T = 2*t1 + t2;                 % Total time [s]
display(T)

% Time vector
dt = 0.01;
t = 0:dt:T;
s = zeros(size(t));
dot_s = zeros(size(t));

% Compute s(t) profile
for i = 1:length(t)
    if t(i) < t1
        s(i) = 0.5 * s_max_ddot * t(i)^2;
        dot_s(i) = s_max_ddot * t(i);
    elseif t(i) < t1 + t2
        s(i) = s1 + s_max_dot * (t(i) - t1);
        dot_s(i) = s_max_dot;
    else
        tau = t(i) - (t1 + t2);
        s(i) = s1 + s2 + s_max_dot * tau - 0.5 * s_max_ddot * tau^2;
        dot_s(i) = s_max_dot - s_max_ddot * tau;
    end
end

% RP Robot trajectory
theta_a = theta_i + 2 * s;
dot_theta_a = 2 * dot_s;

% PP Robot trajectory
q1 = r * cos(theta_a) - 0.7;
q2 = r * sin(theta_a) - 0.6;
dot_q1 = -r * sin(theta_a) .* dot_s;
dot_q2 = r * cos(theta_a) .* dot_s;

% Plot results
figure;
subplot(3,1,1);
plot(t, s, 'b', 'LineWidth', 2);
hold on;
yline(L, '--r');
xlabel('Time [s]'); ylabel('s [m]'); title('Path Position s(t)');
grid on;

subplot(3,1,2);
plot(t, dot_s, 'r', 'LineWidth', 2);
hold on;
yline(s_max_dot, '--k');
xlabel('Time [s]'); ylabel('Velocity [m/s]'); title('Path Velocity \dot{s}(t)');
grid on;

subplot(3,1,3);
plot(t, dot_theta_a, 'g', 'LineWidth', 2);
hold on;
yline(1, '--k');
xlabel('Time [s]'); ylabel('\dot{\theta}_a [rad/s]'); title('RP Robot Joint Velocity');
grid on;

figure;
subplot(2,1,1);
plot(t, q1, 'b', t, q2, 'r', 'LineWidth', 2);
xlabel('Time [s]'); ylabel('Position [m]'); title('PP Robot Joint Positions q_1, q_2');
legend('q_1', 'q_2'); grid on;

subplot(2,1,2);
plot(t, dot_q1, 'b', t, dot_q2, 'r', 'LineWidth', 2);
xlabel('Time [s]'); ylabel('Velocity [m/s]'); title('PP Robot Joint Velocities');
legend('\dot{q}_1', '\dot{q}_2'); grid on;

% Parameters
r = 0.4;       % Radius of the helix (m)
h = 0.3;       % Pitch height per turn (m)
T = 10;        % Total time (s)
n_turns = 2;   % Number of turns
s_max = 2 * pi * n_turns; % Maximum value of s

% Time vector
t = linspace(0, T, 1000);

% Time scaling: 5th-degree polynomial for smooth start and stop
a0 = 0; a1 = 0; a2 = 0; % Coefficients for initial conditions
a3 = 10 * s_max / T^3;  % Coefficients for final conditions
a4 = -15 * s_max / T^4;
a5 = 6 * s_max / T^5;

s = a0 + a1*t + a2*t.^2 + a3*t.^3 + a4*t.^4 + a5*t.^5;   % s(t)
ds = a1 + 2*a2*t + 3*a3*t.^2 + 4*a4*t.^3 + 5*a5*t.^4;    % ds/dt
dds = 2*a2 + 6*a3*t + 12*a4*t.^2 + 20*a5*t.^3;           % d²s/dt²

% Helical path p(s)
px = r * cos(s);
py = h * s;
pz = r * sin(s);

% Velocity and acceleration in 3D space
vx = -r * sin(s) .* ds;
vy = h * ds;
vz = r * cos(s) .* ds;

ax = -r * cos(s) .* ds.^2 - r * sin(s) .* dds;
ay = h * dds;
az = -r * sin(s) .* ds.^2 + r * cos(s) .* dds;

% Plot trajectory
figure;
plot3(px, py, pz, 'b', 'LineWidth', 1.5);
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Helical Trajectory');
grid on;

% Plot velocity
figure;
plot(t, [vx; vy; vz]);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('Vx', 'Vy', 'Vz');
title('End-Effector Velocity');
grid on;

% Plot acceleration
figure;
plot(t, [ax; ay; az]);
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
legend('Ax', 'Ay', 'Az');
title('End-Effector Acceleration');
grid on;

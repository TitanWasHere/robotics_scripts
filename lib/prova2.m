%% Time-Optimal Trajectory for a Robot Joint (Bang-Coast-Bang)
% Given parameters:
qi = -2;        % Initial position [rad]
qf = 1;         % Final position [rad]
vi = 0.5;       % Initial speed [rad/s]
vf = 1;         % Final speed [rad/s]
vmax = 2;       % Maximum speed [rad/s]
amax = 3;       % Maximum acceleration [rad/s^2]

% Compute durations of the acceleration and deceleration phases:
ta = (vmax - vi) / amax;        % Time to accelerate from vi to vmax
td = (vmax - vf) / amax;        % Time to decelerate from vmax to vf

% Compute distances covered during acceleration and deceleration:
dq_acc = (vmax^2 - vi^2) / (2 * amax);
dq_dec = (vmax^2 - vf^2) / (2 * amax);

% Total required displacement:
dq_total = qf - qi;

% Compute coast time:
t_coast = (dq_total - (dq_acc + dq_dec)) / vmax;

if t_coast < 0
    error('The given displacement is too short to reach vmax. Use a bang-bang solution.');
end

% Total time:
T = ta + t_coast + td;

% Display the computed times:
fprintf('Acceleration phase: ta = %.4f s\n', ta);
fprintf('Coast phase:        tc = %.4f s\n', t_coast);
fprintf('Deceleration phase: td = %.4f s\n', td);
fprintf('Total time:         T  = %.4f s\n', T);

%% Generate time vectors for each phase
dt = 0.001;  % time step for plotting

t1 = 0:dt:ta;                        % acceleration phase
t2 = (ta+dt):dt:(ta+t_coast);          % coast phase
t3 = (ta+t_coast+dt):dt:T;             % deceleration phase

%% Preallocate arrays for position, velocity, and acceleration
t_total = [t1, t2, t3];
q = zeros(size(t_total));
v = zeros(size(t_total));
a = zeros(size(t_total));

%% Phase 1: Acceleration (a = amax)
q1 = qi + vi*t1 + 0.5*amax*t1.^2;
v1 = vi + amax*t1;
a1 = amax * ones(size(t1));

%% Phase 2: Coast (a = 0, v = vmax)
q_start2 = qi + vi*ta + 0.5*amax*ta^2;  % Position at the end of phase 1
q2 = q_start2 + vmax*(t2 - ta);
v2 = vmax * ones(size(t2));
a2 = zeros(size(t2));

%% Phase 3: Deceleration (a = -amax)
t_dec = t3 - (ta + t_coast);   % time elapsed in deceleration phase
q_start3 = q_start2 + vmax*t_coast;  % Position at start of deceleration phase
q3 = q_start3 + vmax*t_dec - 0.5*amax*t_dec.^2;
v3 = vmax - amax*t_dec;
a3 = -amax * ones(size(t3));

%% Combine the phases
q = [q1, q2, q3];
v = [v1, v2, v3];
a = [a1, a2, a3];

%% Plot the profiles
figure;

subplot(3,1,1)
plot(t_total, q, 'LineWidth', 2)
xlabel('Time [s]')
ylabel('Position [rad]')
title('Position Profile')
grid on

subplot(3,1,2)
plot(t_total, v, 'LineWidth', 2)
xlabel('Time [s]')
ylabel('Velocity [rad/s]')
title('Velocity Profile')
grid on

subplot(3,1,3)
plot(t_total, a, 'LineWidth', 2)
xlabel('Time [s]')
ylabel('Acceleration [rad/s^2]')
title('Acceleration Profile')
grid on
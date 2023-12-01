% Initial state
x0 = [pi/2; -pi/2; 0; 0]; % [q1, q2, dq1, dq2]

% Control inputs (example: zero torque)
u = [0; 0];

% Time span
tspan = [0, 10]; % Start at t=0 and integrate for 10 seconds

% Integrate the dynamics
[t, X] = ode45(@(t, x) manipulator_dynamics(t, x, u), tspan, x0);

% Plot the results (example: joint angles over time)
figure;
plot(t, X(:, 1), 'r', t, X(:, 2), 'b');
xlabel('Time (s)');
ylabel('Joint Angles (rad)');
legend('q1', 'q2');
title('Manipulator Joint Angles');

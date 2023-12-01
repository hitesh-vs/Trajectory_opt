% MAIN - Minimum Time Boundary Value Problem
%
% Solve a minimum-time boundary value problem with simple dynamics (chain
% integrator) and limits on the state and control. Scalar trajectory.
%
% Here we will solve a scalar trajectory, where the position, velocity, 
% and acceleration are states. The jerk (derivative of acceleration) will
% be the only control.
% 

clc; clear;
addpath ../../

lp = 0.1; %length of plate
wp = 4; %width of plate
length = 0.5;
width = 1;
mu = 0.8; %Static friction co-efficient
mass = 1; %Mass of block
g = 10;
% Kinematic Limits:
xLim = [0, 15]; % position
vLim = [-5, 5]; % velocity
aLim = [-mu*g, mu*g]; % acceleration
jLim = 5*[-2, 2]; % jerk 

% Boundary value problem:
xBegin = xLim(1);  % initial state
vBegin = 0;
aBegin = 0;
xFinal = xLim(2);  % final state
vFinal = 0;
aFinal = 0;

% User-defined dynamics and objective functions
problem.func.dynamics = @(t,x,u)( scalarChainIntegrator(x,u) );
problem.func.bndObj = @(t0,x0,tF,xF)( tF - t0 ); % minimum time  -- primary objective
problem.func.pathObj = @(t,x,u)( 0.001*u.^2 ); %minimum jerk  -- regularization

% Problem boundsTime
problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low = 0.1;
problem.bounds.finalTime.upp = 10;

problem.bounds.state.low = [xLim(1); vLim(1); aLim(1)];
problem.bounds.state.upp = [xLim(2); vLim(2); aLim(2)];
problem.bounds.initialState.low = [xBegin; vBegin; aBegin];
problem.bounds.initialState.upp = [xBegin; vBegin; aBegin];
problem.bounds.finalState.low = [xFinal; vFinal; aFinal];
problem.bounds.finalState.upp = [xFinal; vFinal; aFinal];

problem.bounds.control.low = jLim(1);
problem.bounds.control.upp = jLim(2); 

% Guess at the initial trajectory
problem.guess.time = [0,2];
problem.guess.state = [[xBegin; vBegin; aBegin], [xFinal; vFinal; aFinal]];
problem.guess.control = [0, 0];

% Select a solver:
problem.options(1).method = 'trapezoid';
problem.options(1).trapezoid.nGrid = 8;
problem.options(2).method = 'trapezoid';
problem.options(2).trapezoid.nGrid = 16;
problem.options(3).method = 'hermiteSimpson';
problem.options(3).hermiteSimpson.nSegment = 15;

% Solve the problem
soln = optimTraj(problem);
t = soln(end).grid.time;
q = soln(end).grid.state(1,:);
dq = soln(end).grid.state(2,:);
ddq = soln(end).grid.state(3,:);
u = soln(end).grid.control;

% Animation
figure;
for i = 1:size(t,2)
    clf; hold on;
    
    % Plot the block
    rectangle('Position', [q(i), -length/2, width, length], 'FaceColor', 'b');
    hold on;
%     plot([-10, 10], [0, 0], 'k-'); % Adjust the range as needed
    rectangle('Position', [q(i)+(width-wp)/2, -(length/2)-lp, wp, lp], 'FaceColor', 'k');
    hold on;
    % Set axis limits and labels
    axis([xBegin-1, xBegin+10, -5.5, 5.5]);
    xlabel('Position (m)');
    axis off;
    title('Rectangular Block Dynamics');
    
    % Pause to control animation speed
    pause(0.05);
end
figure;
rectangle('Position', [q(i), -length/2, width, length], 'FaceColor', 'b');
    hold on;
%     plot([-10, 10], [0, 0], 'k-'); % Adjust the range as needed
    rectangle('Position', [q(i)+(width-wp)/2, -(length/2)-lp, wp, lp], 'FaceColor', 'k');
    hold on;
    % Set axis limits and labels
    axis([xBegin-1, xBegin+10, -5.5, 5.5]);
    xlabel('Position (m)');
    axis off;

% Plot the solution:
figure(1); 

subplot(4,1,1)
plot(t,q)
ylabel('q')
title('Minimum-time boundary value problem');

subplot(4,1,2)
plot(t,dq)
ylabel('dq')

subplot(4,1,3)
plot(t,ddq)
ylabel('ddq')

subplot(4,1,4)
plot(t,u)
ylabel('dddq')
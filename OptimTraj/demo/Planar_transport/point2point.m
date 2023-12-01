% How to call the solver multiple times? 

clc; 
clear;
addpath ../../

lp = 0.1; %length of plate
wp = 4; %width of plate
length = 0.5;
width = 1;
mu = 0.8; %Static friction co-efficient
mass = 1; %Mass of block
g = 10;
theta = 0.643; %Angle of the path with x axis

% Limits on Zero Moment Point Trajectory
zcom = 0.115;

% Kinematic Limits:
xLim = [0, 10]; % x_position
yLim = [0, 8]; %y_position
vLim = [-4, 4]; % velocity
aLim = [-mu*g, mu*g]; % acceleration
jLim = 5*[-2, 2]; % jerk 

% Boundary value problem:
xBegin = xLim(1);
yBegin = yLim(1); % initial state
vBegin = 0;
aBegin = 0;
xFinal = xLim(2);
yFinal = yLim(2); % final state
vFinal = 0;
aFinal = 0;
%% Problem in x direction

% Solving a 2 dimension planar transportation problem can be simplified
% into solving two independent perpendicular linear motions

% User-defined dynamics and objective functions
problem1.func.dynamics = @(t,x,u)( scalarChainIntegrator(x,u) );
problem1.func.bndObj = @(t0,x0,tF,xF)( tF - t0 ); % minimum time  -- primary objective
problem1.func.pathObj = @(t,x,u)( 0.001*u.^2 ); %minimum jerk  -- regularization

% Problem boundsTime
problem1.bounds.initialTime.low = 0;
problem1.bounds.initialTime.upp = 0;
problem1.bounds.finalTime.low = 0.1;
problem1.bounds.finalTime.upp = 10;

problem1.bounds.state.low = [xLim(1); vLim(1); aLim(1)];
problem1.bounds.state.upp = [xLim(2); vLim(2); aLim(2)];
problem1.bounds.initialState.low = [xBegin; vBegin; aBegin];
problem1.bounds.initialState.upp = [xBegin; vBegin; aBegin];
problem1.bounds.finalState.low = [xFinal; vFinal; aFinal];
problem1.bounds.finalState.upp = [xFinal; vFinal; aFinal];

problem1.bounds.control.low = jLim(1);
problem1.bounds.control.upp = jLim(2); 

% Guess at the initial trajectory
problem1.guess.time = [0,2];
problem1.guess.state = [[xBegin; vBegin; aBegin], [xFinal; vFinal; aFinal]];
problem1.guess.control = [0, 0];

% Select a solver:
problem1.options(1).method = 'trapezoid';
problem1.options(1).trapezoid.nGrid = 8;
problem1.options(2).method = 'trapezoid';
problem1.options(2).trapezoid.nGrid = 16;
problem1.options(3).method = 'hermiteSimpson';
problem1.options(3).hermiteSimpson.nSegment = 15;

% Solve the problem
solnx = optimTraj(problem1);
t = solnx(end).grid.time;
q = solnx(end).grid.state(1,:);
dq = solnx(end).grid.state(2,:);
ddq = solnx(end).grid.state(3,:);
u = solnx(end).grid.control;

qx1 = q*cos(theta);
qy1 = q*sin(theta);
dqx1 = dq*cos(theta);
dqy1 = dq*sin(theta);
ddqx1 = ddq*cos(theta);
ddqy1 = ddq*sin(theta);

qx2 = q;
qy2 = 0;
dqx2 = dq;
dqy2 = 0;
ddqx2 = ddq;
ddqy2 = 0;

qx3 = -q*cos(theta);
qy3 = -q*sin(theta);
dqx3 = -dq*cos(theta);
dqy3 = -dq*sin(theta);
ddqx3 = -ddq*cos(theta);
ddqy3 = -ddq*sin(theta);

qx4 = -q;
qy4 = 0;
dqx4 = -dq;
dqy4 = 0;
ddqx4 = -ddq;
ddqy4 = 0;

% Arrays for plotting and data collection - Parametrize later

accx = [ddqx1 ddqx2 ddqx3 ddqx4];
accy = [ddqy1 zeros(1,31) ddqy3 zeros(1,31)];
acc = sqrt(accx.^2 + accy.^2);
tot_time = [t t+3.8032 t+3.8032*2 t+3.8032*3];
% 3.8032 is the time after executing first motion. So we add that after
% each value in the array to make a continuous time from start to end.

traj_x = [qx1 qx2+8.0030 qx3+8.0030+10 qx4+10];
traj_y = [qy1 zeros(1,31)+5.9960 qy3+5.9960 zeros(1,31)];
traj_z = zeros(size(traj_x)); % Z-coordinate set to zero

%Here 8.003 is the final value of qx1. 5.9960 is final value of qy1. 10 is
%the final value of qx2. Just keep accumulating final values of previous.

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

figure(2);

subplot(4,1,1)
plot(t,qx1)

subplot(4,1,2)
plot(t,qy1)

subplot(4,1,3)
plot(t,dqx1)

subplot(4,1,4)
plot(t,dqy1)

figure(3);
subplot(3,1,1)
plot(tot_time,accx)
subplot(3,1,2)
plot(tot_time,accy)
subplot(3,1,3)
plot(tot_time,acc)




figure(4);
plot3(traj_x, traj_y, traj_z, 'b-', 'LineWidth', 2);
grid("on");
axis([-2, 20, -2, 8]);
hold on;

% Creating XY plane grid
gridSize = 50; % Define the grid size
[X, Y] = meshgrid(-gridSize:0.1:gridSize); % Create a meshgrid
Z = zeros(size(X)); % Z-coordinates for the plane (set to zero)

% Plotting the XY plane grid
surf(X, Y, Z, 'FaceAlpha', 0.1, 'EdgeColor', 'none');
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('2D Trajectory Traced by the End Effector');
grid on;












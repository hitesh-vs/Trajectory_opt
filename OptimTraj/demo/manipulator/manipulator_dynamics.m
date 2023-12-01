function dx = manipulator_dynamics(t, x, u)
    % Parameters
    L1 = 2; % Length of the first link
    L2 = 1.5; % Length of the second link
    m1 = 1; % Mass of the first link
    m2 = 0.5; % Mass of the second link
    g = 9.81; % Gravity acceleration
    
    % Extract state variables
    q1 = x(1); % Joint angle 1
    q2 = x(2); % Joint angle 2
    dq1 = x(3); % Joint angular velocity 1
    dq2 = x(4); % Joint angular velocity 2
    
    %Trig notations
    c1 = cos(q1);
    s1 = sin(q1);
    c2 = cos(q2);
    s2 = sin(q2);
    c12 = cos(q1+q2);
    s12 = sin(q1+q2);
    
    % Control inputs (torques at the joints)
    tau1 = u(1); % Torque at joint 1
    tau2 = u(2); % Torque at joint 2
    
    % Equations of motion
    
    %Mass Matrix
    M11 = ((m1/3 + m2)*L1^2 + (m2*L2^2)/3 + m2*l1*l2*c2);
    M12 = m2*((L2^2)/3 + L1*L2*c2/2);
    M21 = m2*((L2^2)/3 + L1*L2*c2/2);
    M22 = (m2*L2^2)/3;

    %Coriolis Matrix
    C1 = -m2*L1*L2*s2*dq1*dq2
    % State derivatives
    dx = [dq1; dq2; ddq1; ddq2];
end

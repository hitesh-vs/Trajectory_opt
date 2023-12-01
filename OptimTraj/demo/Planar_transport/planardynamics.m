%2d motion dynamics equations
% Input - state vector with pos vel acc in x and y
%       - control input vector of jerk in x and y directions 
% Output - derivative of state vector

function dx = planardynamics(x,u)

if size(x,1) == 1
    dx = u;
else
    dx = [x(2:3, :); u(1,:); x(5:6, :); u(2,:) ];
end

end
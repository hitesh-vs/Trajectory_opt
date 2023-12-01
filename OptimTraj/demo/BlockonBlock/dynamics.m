function dx = dynamics(x,u)
% dx = scalarChainIntegrator(x,u)
%
% Computes the dynamics for a scalar chain integrator:
%     dx(1) = x(2)
%     dx(2) = x(3)
%     ...
%     dx(n) = u
%
k = 0.5;
if size(x,1) == 1
    dx = u;
else
    dx = [x(3:end, :); u; k.*(x(3,:)-x(4,:))];
end

end
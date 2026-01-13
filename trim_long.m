function trim = trim_long(P)
%TRIM_LONG Solve a simple trim: u=w=q=0, find theta and de s.t. xdot = 0.

x0_guess = [0; 0; 0; 0];     % [u w q theta]
de_guess = 0;

z0 = [x0_guess(4); de_guess]; % unknowns: [theta; de]

opts = optimoptions('fsolve','Display','none','FunctionTolerance',1e-10);

z = fsolve(@(zz) trim_residual(zz,P), z0, opts);

trim.u     = 0;
trim.w     = 0;
trim.q     = 0;
trim.theta = z(1);
trim.de    = z(2);

trim.x = [trim.u; trim.w; trim.q; trim.theta];

end

function r = trim_residual(z,P)
theta = z(1);
de    = z(2);

x = [0; 0; 0; theta];

xdot = eom_long_nonlinear(0, x, de, P);

% Enforce steady u-dot, w-dot, q-dot = 0
r = xdot(1:3);
end

function xdot = eom_long_nonlinear(t, x, de, P) %#ok<INUSD>
%EOM_LONG_NONLINEAR Nonlinear longitudinal EoM (simplified).
% x = [u; w; q; theta] where u,w are perturbation velocities about trim U0.
% de = elevator (rad)

u     = x(1);
w     = x(2);
q     = x(3);
theta = x(4);

% Total forward speed approx (trim + perturbation)
U = P.U0 + u;

% Aerodynamic force/moment "increments" using stability derivatives
% Interpreting Xu, Xw, etc as linear coefficients around trim.
X = P.Xu*u + P.Xw*w + P.Xq*q + P.Xde*de;
Z = P.Zu*u + P.Zw*w + P.Zq*q + P.Zde*de;
M = P.Mu*u + P.Mw*w + P.Mq*q + P.Mde*de;

% Nonlinear-ish longitudinal EoM (simplified symmetric flight)
udot     = -q*w + X - P.g*sin(theta + P.theta0);
wdot     =  q*U + Z + P.g*cos(theta + P.theta0);
qdot     =  M;
thetadot =  q;

xdot = [udot; wdot; qdot; thetadot];
end

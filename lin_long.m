function [A,B,C,D,sys] = lin_long(P, trim)
%LIN_LONG Linearize about trim via numerical Jacobian.

x0 = trim.x;
u0 = trim.de;

nx = numel(x0);
nu = 1;

epsx = 1e-6;
epsu = 1e-6;

f0 = eom_long_nonlinear(0, x0, u0, P);

A = zeros(nx,nx);
for i=1:nx
    dx = zeros(nx,1); dx(i)=epsx;
    fp = eom_long_nonlinear(0, x0+dx, u0, P);
    fm = eom_long_nonlinear(0, x0-dx, u0, P);
    A(:,i) = (fp - fm)/(2*epsx);
end

B = zeros(nx,nu);
du = epsu;
fp = eom_long_nonlinear(0, x0, u0+du, P);
fm = eom_long_nonlinear(0, x0, u0-du, P);
B(:,1) = (fp - fm)/(2*epsu);

% Outputs: choose theta and q as typical
C = [0 0 0 1;   % theta
     0 0 1 0];  % q
D = zeros(2,1);

sys = ss(A,B,C,D);

end

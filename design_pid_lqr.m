function ctrl = design_pid_lqr(sys)
%DESIGN_PID_LQR PID and LQR designs for longitudinal model.

% We will control theta (output 1) with input delta_e
Gtheta = sys(1,:);          % SISO from de -> theta (since single input)
Gtheta = ss(Gtheta.A, Gtheta.B, Gtheta.C, Gtheta.D);

% --- PID (tuneable)
% Use pidtune as a baseline, then you can refine.
Cpid = pidtune(Gtheta, "PID");

% Closed-loop for theta tracking (unity feedback)
Tpid = feedback(Cpid*Gtheta, 1);

% --- LQR
[A,B,~,~,~] = ssdata(sys);

% Weighting (tune for your aircraft)
Q = diag([1, 5, 10, 20]);   % penalize w, q, theta more
R = 1;                      % penalize elevator effort

K = lqr(A,B,Q,R);

% Prefilter for reference tracking on theta:
% We want u = -Kx + Nbar*r, with r = theta_cmd.
% Use output matrix for theta: y = Ctheta x
Ctheta = [0 0 0 1];
Nbar = rscale(A,B,Ctheta,0,K); % helper below

ctrl.Cpid = Cpid;
ctrl.Tpid = Tpid;

ctrl.K = K;
ctrl.Nbar = Nbar;
ctrl.Ctheta = Ctheta;

end

function Nbar = rscale(A,B,C,D,K)
%RSCALE Compute reference gain Nbar for state feedback tracking.
% For SISO output y = Cx + Du.
s = size(A,1);
Z = [A - B*K, B; C, D];
N = [zeros(s,1); 1];
X = Z\N;
Nx = X(1:s);
Nu = X(s+1);
Nbar = Nu + K*Nx;
end

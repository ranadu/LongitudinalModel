function plots_analysis(sys, ctrl)
%PLOTS_ANALYSIS Generate step, root locus, Bode, margins.

Gtheta = ss(sys(1,:)); % de -> theta
Gtheta = ss(Gtheta.A,Gtheta.B,Gtheta.C,Gtheta.D);

figure; step(Gtheta);
title("Open-loop: Step response (de -> theta)");

% Root locus of open-loop with PID controller (loop transfer)
Lpid = ctrl.Cpid * Gtheta;
figure; rlocus(Lpid);
title("Root Locus: L(s) = C_pid(s) G_theta(s)");

% Bode + margins
figure; margin(Lpid);
title("Bode + Stability Margins: PID loop transfer");

% Closed-loop step response (theta command)
figure; step(ctrl.Tpid);
title("Closed-loop PID: theta tracking step response");

% LQR closed-loop: xdot = (A - B K) x + B Nbar r
[A,B,C,D] = ssdata(sys);
Acl = A - B*ctrl.K;
Bcl = B*ctrl.Nbar;
Ccl = ctrl.Ctheta;
Dcl = 0;
sysLQR = ss(Acl,Bcl,Ccl,Dcl);

figure; step(sysLQR);
title("Closed-loop LQR: theta tracking step response");

% Compare PID vs LQR
figure; step(ctrl.Tpid, sysLQR);
legend("PID","LQR");
title("PID vs LQR: theta step tracking comparison");

end

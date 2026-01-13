function P = params_long()
%PARAMS_LONG Example longitudinal derivatives and trim point.
% will replace with actual aircraft data.

P.g  = 9.80665;     % m/s^2
P.m  = 7500;        % kg (example)
P.Iy = 45000;       % kg*m^2 (example)

% Trim
P.U0    = 120;      % m/s (trim forward speed)
P.theta0 = 0.0;     % rad (level flight approx)

% Stability derivatives (1/s, m/s^2 per state, etc.)
% State: x = [u; w; q; theta], input: de (rad)
P.Xu = -0.02;     P.Xw =  0.00;   P.Xq =  0.00;    P.Xde =  0.0;
P.Zu = -0.30;     P.Zw = -1.20;   P.Zq = -6.50;    P.Zde = -35.0;
P.Mu =  0.00;     P.Mw = -0.05;   P.Mq = -1.00;    P.Mde = -2.5;

% Note: In the w-dot equation, the classic linearized term includes (Zq + U0)*q
% We'll build A accordingly.
end

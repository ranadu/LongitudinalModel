clear; clc; close all;

P = params_long();
trim = trim_long(P);

[A,B,C,D,sys] = lin_long(P, trim);

disp("Trim:");
disp(trim);

disp("Linearized A:");
disp(A);

ctrl = design_pid_lqr(sys);

plots_analysis(sys, ctrl);

% Optional: auto-build Simulink model
build_simulink_model(sys, ctrl);

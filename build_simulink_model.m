function build_simulink_model(sys, ctrl)
%BUILD_SIMULINK_MODEL Programmatically create a Simulink model.

model = "longitudinal_ctrl_model";
if bdIsLoaded(model); close_system(model,0); end
new_system(model); open_system(model);

[A,B,C,D] = ssdata(sys);

% Blocks
add_block("simulink/Sources/Step", model+"/theta_cmd", "Position",[30 60 60 90]);
add_block("simulink/Math Operations/Sum", model+"/Sum", "Inputs","+-", "Position",[110 60 140 90]);

% PID Controller (continuous)
add_block("simulink/Continuous/PID Controller", model+"/PID", ...
    "Position",[180 45 260 105]);
set_param(model+"/PID","P",num2str(ctrl.Cpid.Kp),"I",num2str(ctrl.Cpid.Ki),"D",num2str(ctrl.Cpid.Kd));

% Plant: State-Space
add_block("simulink/Continuous/State-Space", model+"/Plant", ...
    "Position",[310 45 430 105]);
set_param(model+"/Plant", ...
    "A",mat2str(A), "B",mat2str(B), "C",mat2str(C), "D",mat2str(D));

% Scope
add_block("simulink/Sinks/Scope", model+"/Scope", "Position",[480 50 520 100]);

% Connections
add_line(model,"theta_cmd/1","Sum/1");
add_line(model,"Sum/1","PID/1");
add_line(model,"PID/1","Plant/1");
add_line(model,"Plant/1","Scope/1");
add_line(model,"Plant/1","Sum/2"); % feedback theta (output1)

% Configure Step amplitude and time
set_param(model+"/theta_cmd","Time","1","Before","0","After","0.1"); % 0.1 rad step

save_system(model);
end

%% 6-DOF TILT-ROTOR UNIFIED SIMULATOR & AUTOTUNER (main.m)
clear; clc; close all;

% ADDED : 
% NMPC SOLVER


%% 1. FLIGHT DIRECTOR (MODE SWITCH)
% Change this variable to run different parts of the code.
% Options: 'TUNE_RATES', 'TUNE_TRANSLATION', 'SIMULATE'
RUN_MODE = 'SIMULATE'; 

%% 3. ENVIRONMENT & AIRFRAME
env.grav = 9.81;       
env.rho = 1.225;       

quad.mass = 0.5;      
quad.L = 0.22;         
quad.Ixx = 0.012; quad.Iyy = 0.012; quad.Izz = 0.022;
quad.I = diag([quad.Ixx, quad.Iyy, quad.Izz]);
quad.Irot = 0.0001; 
quad.gamma = [pi/4; 3*pi/4; 5*pi/4; 7*pi/4]; 
quad.lambda = [1;-1;1;-1];
quad.kQ = 0.015;
quad.max_thrust = 2.0 * env.grav;  
quad.min_thrust = 0;                
quad.max_tilt = deg2rad(180);        
quad.min_tilt = deg2rad(-180);     

param.max_angle = deg2rad(75); 
param.max_rate = deg2rad(180);

param.V_cmax = 2;         
param.Climb_max = 3;
param.Sink_max = 1;          
param.wp_rad = 2;          
param.dt = 0.002;            

param.servo_slew_rate = deg2rad(400); 
param.max_accel = 2;

param.noise_gps  = 0.2;          % [m] GPS noise
param.noise_vel  = 0.1;          % [m/s] Optical flow / GPS velocity noise
param.noise_att  = deg2rad(1.0); % [rad] Accelerometer/Gyro fusion error
param.noise_gyro = 0.05;         % [rad/s] Gyroscope vibration/electrical noise

param.tau_pos   = 0.02;   % 100ms delay on position
param.tau_vel   = 0.02;  % 50ms delay on velocity
param.tau_att   = 0.01;  % 20ms delay on attitude
param.tau_rates = 0.01;  % 10ms delay on gyro (Critical for D-term survival)



if strcmp(RUN_MODE, 'SIMULATE')
    % -- NORMAL FLIGHT SIMULATION --
    disp('--- STARTING FULL FLIGHT SIMULATION ---');
    waypoints = [
        0,  0, -5,  0, 0, 0, 2;   
        5, 0, -5, 0, 0, pi/2, 2;
        5, 0, -5,  0, pi/4, 0, 2;        
        % 0,  0, -5,  0, 0, 0, 2;
        % % 0,  0, -5,  0, 0, pi/4, 3;
        % 0, 10, -5,  0, 0, pi/4, 10;  
        % 0,  0, -5,  0, 0, pi/4, 3;  
        % 0,  0, -5,  0, 0, 0, 3;       
        % 0,  0,  0,  0, 0, 0, 3       
    ];  

    IC.States = zeros(16,1);  
    IC.States(3) = 0;
    IC.States_16 = IC.States;
    
    NMPC_Solver = build_nmpc_controller(quad, env, param, IC);

    T_end = 30;

    [Log] = run_flight_loop(waypoints, IC, T_end, quad, env, param, NMPC_Solver);
    plot_telemetry(Log);
    
    disp('Simulation Complete. Rendering...');

    visualize_drone(Log, quad);

else
    error('Invalid RUN_MODE. Choose TUNE_RATES, TUNE_TRANSLATION, or SIMULATE.');
end
% MAIN LOOP FUNCTION
function [Log, crash_flag, ITAE_score] = run_flight_loop(waypoints, IC, T_end, quad, env, param, NMPC_Solver)
    N_steps = round(T_end / param.dt);
    
    Log.Time = zeros(1, N_steps);
    Log.State = zeros(16, N_steps);
    Log.Thrust = zeros(4, N_steps);
    Log.Servo = zeros(4, N_steps);
    Log.Propeller = zeros(4, N_steps);
    Log.Target = zeros(6, N_steps);
    
    Log.State_hat = Log.State;
    Log.Sensors = zeros(12, N_steps);

    Log.Wind_F = zeros(3, N_steps);
    Log.Wind_Tau = zeros(3, N_steps);
    Log.Wind_Est = zeros(3, N_steps);

    State = IC.States;
    Servos_Actual = State(13:16);
    Thrust_Actual = zeros(4,1);
    Prop_ang = zeros(4,1);
    Pos_target_filtered = IC.States(1:3);
    Yaw_target_filtered = IC.States(9);

    Target_16 = zeros(16,1);
    mem_est.Pos   = IC.States(1:3);
    mem_est.Vel   = IC.States(4:6);
    mem_est.Euler = IC.States(7:9);
    mem_est.Rates = IC.States(10:12);

    hover_start_time = NaN;

    % Wind Disturbance Estimator
    F_wind_estimated = zeros(3, 1);     % Estimated wind force [earth frame, N]
    vel_prev = State(4:6);              % For computing acceleration
    alpha_wind = 0.02; 

    current_wp = 1; total_wp = size(waypoints, 1);
    crash_flag = false;

    % Put this BEFORE the for-loop
    U_cmd_current = zeros(8, 1); 
    nmpc_update_rate = 0.05;     
    next_nmpc_time = 0;
    for k = 1:N_steps
        t = (k-1) * param.dt;
       
        
        % --- DEBUG PROGRESS TRACKER ---
        if mod(k, 10) == 0
            fprintf('Simulating... Time: %.2f / %.2f sec\n', t, T_end);
        end

        % Waypoint Manager
        Target = waypoints(current_wp, 1:6)'; 
        Target_16(1:6) = Target;
        Target_raw = Target;
        
        Wait_Time = waypoints(current_wp, 7);

        % --- THE KINEMATIC GHOST (Trajectory Generator) ---
        pos_error = Target_raw(1:3) - Pos_target_filtered;
        dist_to_wp = norm(pos_error);
        
        if dist_to_wp > 0.001
            % Normalize the direction vector
            move_dir = pos_error / dist_to_wp;
            approach_speed = min(param.V_cmax, 1.0 * dist_to_wp); 
            
            step_size = min(approach_speed * param.dt, dist_to_wp);
            Pos_target_filtered = Pos_target_filtered + (move_dir * step_size);
        end
        
        % --- THE KINEMATIC GHOST (Rotation with Brakes) ---
        yaw_error = Target_raw(6) - Yaw_target_filtered;
        if abs(yaw_error) > 0.001
            % NEW: Calculate a proportional approach rate (The '2.0' is the P-gain)
            approach_yaw_rate = min(deg2rad(25), 1.5 * abs(yaw_error)); 
            
            % Apply the safe, decelerating step
            yaw_step = min(approach_yaw_rate * param.dt, abs(yaw_error)) * sign(yaw_error);
            Yaw_target_filtered = Yaw_target_filtered + yaw_step;
        end

        Target(1:3) = Pos_target_filtered;
        Target(4:5) = Target_raw(4:5);
        Target(6) = Yaw_target_filtered;

        % Sensor Reading
        Sensors = sensor_model(State, param);
        [State_hat_12, mem_est] = state_estimator(Sensors, mem_est, param);
        State_hat = [State_hat_12; State(13:16)];

        Pos_est = State_hat(1:3);

        distance = norm(Target_raw(1:3) - Pos_est);

        % State Machine
        if distance < param.wp_rad
            if isnan(hover_start_time)
                hover_start_time = t; 
            elseif (t - hover_start_time) >= Wait_Time
                if current_wp < total_wp
                    current_wp = current_wp + 1;
                    hover_start_time = NaN; 
                end
            end
        end
        
        % Wind Disturbance
        vel_now = State_hat(4:6);
        accel_body_measured = (vel_now - vel_prev) / param.dt;
        vel_prev = vel_now;
        
        % 2. Predicted body-frame acceleration (from the model, no wind)
        phi_e = State_hat(7); theta_e = State_hat(8); psi_e = State_hat(9);
        p_e = State_hat(10); q_e = State_hat(11); r_e = State_hat(12);
        u_e = State_hat(4); v_e = State_hat(5); w_e = State_hat(6);
        
        % Compute model-predicted body forces (using last applied command)
        Servos_now = State(13:16);
        Omega_now = sqrt(max(Thrust_Actual, 0) / 1.5e-5);
        [F_body_model, ~] = force_calc(Thrust_Actual, Servos_now, ...
            U_cmd_current(5:8), State_hat(1:12), Omega_now, quad);
        
        % Coriolis + gravity terms (from EoM)
        g = env.grav;
        u_dot_model = (r_e*v_e - q_e*w_e) - g*sin(theta_e) + F_body_model(1)/quad.mass;
        v_dot_model = (p_e*w_e - r_e*u_e) + g*cos(theta_e)*sin(phi_e) + F_body_model(2)/quad.mass;
        w_dot_model = (q_e*u_e - p_e*v_e) + g*cos(theta_e)*cos(phi_e) + F_body_model(3)/quad.mass;
        
        accel_body_model = [u_dot_model; v_dot_model; w_dot_model];
        
        % 3. Discrepancy = wind acceleration (body frame)
        accel_wind_body = accel_body_measured - accel_body_model;
        
        % 4. Convert to earth-frame force
        R_b2e = [cos(theta_e)*cos(psi_e), sin(phi_e)*sin(theta_e)*cos(psi_e)-cos(phi_e)*sin(psi_e), cos(phi_e)*sin(theta_e)*cos(psi_e)+sin(phi_e)*sin(psi_e);
                 cos(theta_e)*sin(psi_e), sin(phi_e)*sin(theta_e)*sin(psi_e)+cos(phi_e)*cos(psi_e), cos(phi_e)*sin(theta_e)*sin(psi_e)-sin(phi_e)*cos(psi_e);
                -sin(theta_e),            sin(phi_e)*cos(theta_e),                                  cos(phi_e)*cos(theta_e)];
        
        F_wind_measured_earth = quad.mass * (R_b2e * accel_wind_body);
        
        % 5. Low-pass filter
        F_wind_estimated = (1 - alpha_wind) * F_wind_estimated + alpha_wind * F_wind_measured_earth;
        
        % Saturate to reasonable bounds (avoids divergence on transients)
        F_wind_estimated = max(min(F_wind_estimated, 5), -5);

        if t >= next_nmpc_time || k == 1
            State_Current_16 = [State_hat; Servos_Actual];
            
            % CRITICAL FIX: Map the Ghost to the correct NMPC indices
            Target_16(1:3) = Target(1:3); % X, Y, Z Position
            Target_16(4:6) = [0; 0; 0];   % u, v, w Velocity (We want it to stop/hover)
            Target_16(7:9) = Target(4:6); % Phi, Theta, Psi Attitude
            
            phi_ref = Target(4); theta_ref = Target(5);
            servo_ff = -theta_ref; 
            Target_16(13:16) = servo_ff * ones(4,1);

            % Call the compiled CasADi solver
            [U_opt, ~] = NMPC_Solver(State_hat, Target_16, F_wind_estimated);
            U_opt = full(U_opt); 
            
            U_cmd_current = U_opt(:, 1);
            next_nmpc_time = next_nmpc_time + nmpc_update_rate;
        end
        
        Thrust_cmd = U_cmd_current(1:4);
        Servo_rates_cmd = U_cmd_current(5:8);
        
        % Actuators
        Servo_rates = max(min(Servo_rates_cmd, param.servo_slew_rate), -param.servo_slew_rate);
        Servos_Actual = Servos_Actual + Servo_rates * param.dt;
        Servos_Actual = max(min(Servos_Actual, quad.max_tilt), quad.min_tilt);
        
        Thrust_Actual = Thrust_cmd; 
        Omega = sqrt(max(Thrust_Actual, 0) / 1.5e-5);

        servos_now = State(13:16);
        at_max = (servos_now >= quad.max_tilt) & (Servo_rates > 0);
        at_min = (servos_now <= quad.min_tilt) & (Servo_rates < 0);
        Servo_rates(at_max | at_min) = 0;
        
        % Physics
        [F_body, Tau_body] = force_calc(Thrust_Actual, State(13:16), Servo_rates, State(1:12), Omega, quad);

        % Wind Model
        [F_wind_earth, Tau_wind_body] = wind_model(t, param);
        % F_wind_earth = [0;0;0];
        % Tau_wind_body = [0;0;0];

        phi_real = State(7); theta_real = State(8); psi_real = State(9);
        R_b_to_ned_real = [cos(theta_real)*cos(psi_real), sin(phi_real)*sin(theta_real)*cos(psi_real)-cos(phi_real)*sin(psi_real), cos(phi_real)*sin(theta_real)*cos(psi_real)+sin(phi_real)*sin(psi_real);
                           cos(theta_real)*sin(psi_real), sin(phi_real)*sin(theta_real)*sin(psi_real)+cos(phi_real)*cos(psi_real), cos(phi_real)*sin(theta_real)*sin(psi_real)-sin(phi_real)*cos(psi_real);
                          -sin(theta_real),               sin(phi_real)*cos(theta_real),                                           cos(phi_real)*cos(theta_real)];

        F_wind_body = R_b_to_ned_real' * F_wind_earth;

        F_body = F_body + F_wind_body;
        Tau_body = Tau_body + Tau_wind_body;

        State = RK4(@EoM, State, F_body, Tau_body, quad, env, param.dt, Servo_rates);
        State(13:16) = max(min(State(13:16), quad.max_tilt), quad.min_tilt);
      
        % Log
        Prop_ang = Prop_ang + (quad.lambda(:) .* Omega) * param.dt;
        Log.Time(k) = t; Log.State(:, k) = State; Log.Thrust(:, k) = Thrust_Actual;
        Log.Servo(:, k) = State(13:16); Log.Propeller(:, k) = Prop_ang;
        Log.Wind_F(:, k) = F_wind_earth;
        Log.Wind_Tau(:, k) = Tau_wind_body;
        Log.Wind_Est(:,k) = F_wind_estimated;

        Log.Target(:, k) = Target; 

        Log.Sensors(:, k) = [Sensors.Pos; Sensors.Vel; Sensors.Euler; Sensors.Rates];
        Log.State_hat(:, k) = State_hat;
        
        if State(3) > 1 || abs(State(7)) > pi/2 || abs(State(8)) > pi/2
            crash_flag = true;
            disp('CRASH DETECTED. Stopping integration.');
            Log.Time = Log.Time(1:k); Log.State = Log.State(:, 1:k); Log.Thrust = Log.Thrust(:, 1:k);
            Log.Servo = Log.Servo(:, 1:k); Log.Propeller = Log.Propeller(:, 1:k); Log.Target = Log.Target(:, 1:k); 
            Log.Sensors = Log.Sensors(:, 1:k); Log.State_hat = Log.State_hat(:, 1:k); 
            
            if isfield(Log, 'Wind_F')
                Log.Wind_F = Log.Wind_F(:, 1:k); 
                Log.Wind_Tau = Log.Wind_Tau(:, 1:k); 
            end
            break;
        end
    end
end

%% NMPC SOLVER
function NMPC_Controller = build_nmpc_controller(quad, env, param, IC)
    import casadi.*

    % States 
    x = SX.sym('x'); y = SX.sym('y'); z = SX.sym('z');
    u = SX.sym('u'); v = SX.sym('v'); w = SX.sym('w');
    phi = SX.sym('phi'); theta = SX.sym('theta'); psi = SX.sym('psi');
    p = SX.sym('p'); q = SX.sym('q'); r = SX.sym('r');
    S1 = SX.sym('S1'); S2 = SX.sym('S2'); S3 = SX.sym('S3'); S4 = SX.sym('S4');
    
    states = [x; y; z; u; v; w; phi; theta; psi; p; q; r; S1; S2; S3; S4];
    
    % Inputs
    T1 = SX.sym('T1'); T2 = SX.sym('T2'); T3 = SX.sym('T3'); T4 = SX.sym('T4');
    S1_dot = SX.sym('S1_dot'); S2_dot = SX.sym('S2_dot'); 
    S3_dot = SX.sym('S3_dot'); S4_dot = SX.sym('S4_dot');
    
    controls = [T1; T2; T3; T4; S1_dot; S2_dot; S3_dot; S4_dot];
    
    % Wind
    wind_sym = SX.sym('wind_sym', 3, 1);

    % Physics
    Thrusts = [T1; T2; T3; T4];
    Servos  = [S1; S2; S3; S4];
    Servo_rates = [S1_dot; S2_dot; S3_dot; S4_dot];
    
    s_gamma = sin(quad.gamma);
    c_gamma = cos(quad.gamma);
    s_a = sin(Servos);
    c_a = cos(Servos);
    
    H = Thrusts .* s_a;
    V_lift = Thrusts .* c_a;
    
    % Body Forces
    Fx = sum(-H .* s_gamma);
    Fy = sum(H .* c_gamma);
    Fz = sum(-V_lift);

    % Rotation E2B
    R_e2b_11 = cos(theta)*cos(psi);
    R_e2b_12 = cos(theta)*sin(psi);
    R_e2b_13 = -sin(theta);
    R_e2b_21 = sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi);
    R_e2b_22 = sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi);
    R_e2b_23 = sin(phi)*cos(theta);
    R_e2b_31 = cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
    R_e2b_32 = cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
    R_e2b_33 = cos(phi)*cos(theta);

    % Wind in body frame
    Fx_wind_body = R_e2b_11*wind_sym(1) + R_e2b_12*wind_sym(2) + R_e2b_13*wind_sym(3);
    Fy_wind_body = R_e2b_21*wind_sym(1) + R_e2b_22*wind_sym(2) + R_e2b_23*wind_sym(3);
    Fz_wind_body = R_e2b_31*wind_sym(1) + R_e2b_32*wind_sym(2) + R_e2b_33*wind_sym(3);

    % Add to existing body forces
    Fx = Fx + Fx_wind_body;
    Fy = Fy + Fy_wind_body;
    Fz = Fz + Fz_wind_body;
    
    % Body Torque
    % propeller drag
    Drag_mag = quad.lambda .* quad.kQ .* Thrusts;
    
    Tx_drag = sum(Drag_mag .* (-s_a .* s_gamma));
    Ty_drag = sum(Drag_mag .* (s_a .* c_gamma));
    Tz_drag = sum(Drag_mag .* c_a);
    
    % gyroscopic
    Omega_sym = sqrt((Thrusts+1e-6) / 1.5e-5); 
    
    Tx_gyro = 0; Ty_gyro = 0; Tz_gyro = 0;
    for i = 1:4
        H_mag = quad.Irot * quad.lambda(i) * Omega_sym(i);
        
        Tx_gyro = Tx_gyro + H_mag * (Servo_rates(i)*c_a(i)*s_gamma(i) + q*c_a(i) + r*s_a(i)*c_gamma(i));
        Ty_gyro = Ty_gyro + H_mag * (-Servo_rates(i)*c_a(i)*c_gamma(i) - p*c_a(i) + r*s_a(i)*s_gamma(i));
        Tz_gyro = Tz_gyro + H_mag * (-Servo_rates(i)*s_a(i) - p*s_a(i)*c_gamma(i) - q*s_a(i)*s_gamma(i));
    end
    
    % total body torques
    Tx = sum(-quad.L .* V_lift .* s_gamma) + Tx_drag + Tx_gyro;
    Ty = sum(quad.L .* V_lift .* c_gamma)  + Ty_drag + Ty_gyro;
    Tz = sum(quad.L .* H)                  + Tz_drag + Tz_gyro;
    
    % EQUATIONS OF MOTION
    % Translation Dynamics
    u_dot = (r*v - q*w) - env.grav*sin(theta)          + (Fx / quad.mass);
    v_dot = (p*w - r*u) + env.grav*cos(theta)*sin(phi) + (Fy / quad.mass);
    w_dot = (q*u - p*v) + env.grav*cos(theta)*cos(phi) + (Fz / quad.mass);
    
    % Rotational Dynamics
    p_dot = (Tx - (quad.Izz - quad.Iyy)*q*r) / quad.Ixx;
    q_dot = (Ty - (quad.Ixx - quad.Izz)*p*r) / quad.Iyy;
    r_dot = (Tz - (quad.Iyy - quad.Ixx)*p*q) / quad.Izz;
    
    % Kinematics (Earth Frame)
    x_dot = u*cos(theta)*cos(psi) + v*(sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi)) + w*(cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi));
    y_dot = u*cos(theta)*sin(psi) + v*(sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi)) + w*(cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi)); 
    z_dot = -u*sin(theta)         + v*sin(phi)*cos(theta)                                + w*cos(phi)*cos(theta);
    
    % Euler Angle Rates
    phi_dot   = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
    theta_dot = q*cos(phi)            - r*sin(phi);
    psi_dot   = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta);
    
    % Servo Kinematics (The rates ARE the inputs)
    S1_state_dot = S1_dot;
    S2_state_dot = S2_dot;
    S3_state_dot = S3_dot;
    S4_state_dot = S4_dot;
    
    rhs = [x_dot; y_dot; z_dot; u_dot; v_dot; w_dot; phi_dot; theta_dot; psi_dot; p_dot; q_dot; r_dot; S1_state_dot; S2_state_dot; S3_state_dot; S4_state_dot];
    
    f_physics = Function('f_physics', {states, controls, wind_sym}, {rhs});

    % Optimizer
    opti = casadi.Opti();

    
    
    N = 10; 
    dt_nmpc = 0.08; 
    
    X = opti.variable(16, N+1); 
    U = opti.variable(8, N);    
    
    X0_param = opti.parameter(16, 1);
    X_ref_param = opti.parameter(16, 1);
    Wind_param = opti.parameter(3,1);
    
    % Cost Matrices
    Q = diag([70, 70, 70, ... % x, y, z 
              5.0, 5.0, 10.0, ... % u, v, w (The Speed Limit)
              30,  30,  100,  ... % phi, theta, psi 
              2,   2,   2,   ... % p, q, r 
              0.5,   0.5,   0.5,   0.5]);% Servos
              
    % --- THE DESTINATION (Terminal Cost) ---
    Q_terminal = diag([100, 100, 200, ... % x, y, z 
                       30, 30, 50, ... % u, v, w (Translational brakes)
                       150,  150,  300, ... % phi, theta, psi 
                       20,  20,  20,  ... % p, q, r (CRITICAL: Rotational Brakes)
                       1,   1,   1,   1]);% Servos
    
    R = diag([100, 100, 100, 100, ... % T1..T4 (Deviation from hover)
              10.0, 10.0, 10.0, 10.0]);   % S1_dot..S4_dot 

    % NEW: Penalize violent changes in control commands (Thrust chatter & Servo snapping)
    R_delta = diag([10, 10, 10, 10, ... % Strongly punish thrust spikes
                    50, 50, 50, 50]);   % Brutally punish servo teleportation

    T_hover = (quad.mass * env.grav) / 4; 
    U_hover = [T_hover; T_hover; T_hover; T_hover; 0; 0; 0; 0];

    opti.set_initial(X, repmat(IC.States_16, 1, N+1));
    opti.set_initial(U, repmat(U_hover, 1, N));

    % --- 4. THE OBJECTIVE FUNCTION ---
    J = 0;
    for k = 1:N
        % State Error Cost
        state_error = X(:, k) - X_ref_param;
        J = J + state_error' * Q * state_error;
        
        % Input Deviation Cost 
        input_error = U(:, k) - U_hover; 
        J = J + input_error' * R * input_error;
    end
    
    % NEW: The Smoothing Penalty (Calculates difference between step k and step k+1)
    for k = 1:N-1
        delta_U = U(:, k+1) - U(:, k);
        J = J + delta_U' * R_delta * delta_U;
    end
    
    % Terminal Cost
    state_error_final = X(:, N+1) - X_ref_param;
    J = J + state_error_final' * Q_terminal * state_error_final;

    opti.minimize(J);
    
    % Constraints
    opti.subject_to(X(:, 1) == X0_param);
    for k = 1:N
        k1 = f_physics(X(:, k), U(:, k), Wind_param);
        k2 = f_physics(X(:, k) + dt_nmpc/2 * k1, U(:, k), Wind_param);
        k3 = f_physics(X(:, k) + dt_nmpc/2 * k2, U(:, k), Wind_param);
        k4 = f_physics(X(:, k) + dt_nmpc * k3, U(:, k), Wind_param);
        x_next_sim = X(:, k) + (dt_nmpc/6) * (k1 + 2*k2 + 2*k3 + k4);
        opti.subject_to(X(:, k+1) == x_next_sim);
    end
    
    opti.subject_to(0.5 <= U(1:4, :) <= quad.max_thrust); 
    opti.subject_to(-param.servo_slew_rate <= U(5:8, :) <= param.servo_slew_rate);
    opti.subject_to(quad.min_tilt <= X(13:16, :) <= quad.max_tilt);
    opti.subject_to(X(3, :) <= 1); 

    
    opti.subject_to(-param.max_angle <= X(7:8, :) <= param.max_angle);
    opti.subject_to(-param.max_rate <= X(10:12, :) <= param.max_rate);
        
    % Solver Settings
    p_opts = struct('expand',  true, ...
                    'print_time', 0);

    s_opts = struct('max_iter', 50, ...
                    'print_level', 0, ...
                    'sb', 'yes', ...
                    'tol', 1e-3, ... 
                    'hessian_approximation', 'limited-memory');

    opti.solver('ipopt', p_opts, s_opts);
    
    NMPC_Controller = opti.to_function('NMPC_Controller', {X0_param, X_ref_param, Wind_param}, {U, X});
end


%% PHYSICS FORCE CALCULATOR
function [F_body, Tau_body] = force_calc(Thrusts, Servos, Servo_rates, State, Omega, quad)
    p = State(10); q = State(11); r = State(12);
    s = sin(quad.gamma(:)); c = cos(quad.gamma(:));
    s_a = sin(Servos(:)); c_a = cos(Servos(:));
    
    H = Thrusts(:) .* s_a; T = Thrusts(:) .* c_a;
    Fx = sum(-H .* s); Fy = sum(H .* c); Fz = sum(-T);
    
    Tx_gyro = 0; Ty_gyro = 0; Tz_gyro = 0;
    for i = 1:4
        H_mag = quad.Irot * quad.lambda(i) * Omega(i);
        Tx_gyro = Tx_gyro + H_mag * (Servo_rates(i)*c_a(i)*s(i) + q*c_a(i) + r*s_a(i)*c(i));
        Ty_gyro = Ty_gyro + H_mag * (-Servo_rates(i)*c_a(i)*c(i) - p*c_a(i) + r*s_a(i)*s(i));
        Tz_gyro = Tz_gyro + H_mag * (-Servo_rates(i)*s_a(i) - p*s_a(i)*c(i) - q*s_a(i)*s(i));
    end
    
    Tx = sum(-quad.L .* T .* s) + Tx_gyro;
    Ty = sum(quad.L .* T .* c) + Ty_gyro;
    Tz = sum(quad.lambda(:) .* quad.kQ .* T + quad.L .* H) + Tz_gyro; 
    
    F_body = [Fx; Fy; Fz]; Tau_body = [Tx; Ty; Tz];
end

%% EQUATIONS OF MOTION & INTEGRATOR
function State_next = RK4(EoM_func, State, F_body, Tau_body, quad, env, dt, Servo_rates)
    k1 = EoM_func(State, F_body, Tau_body, quad, env, Servo_rates);
    k2 = EoM_func(State + 0.5*dt*k1, F_body, Tau_body, quad, env, Servo_rates);
    k3 = EoM_func(State + 0.5*dt*k2, F_body, Tau_body, quad, env, Servo_rates);
    k4 = EoM_func(State + dt*k3, F_body, Tau_body, quad, env, Servo_rates);
    State_next = State + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
end

function State_dot = EoM(State, F_body, Tau_body, quad, env, Servo_rates)
    u = State(4); v = State(5); w = State(6);
    phi = State(7); theta = State(8); psi = State(9);
    p = State(10); q = State(11); r = State(12);
    
    Fx = F_body(1); Fy = F_body(2); Fz = F_body(3);
    Tx = Tau_body(1); Ty = Tau_body(2); Tz = Tau_body(3);
    m = quad.mass; g = env.grav;
    Ixx = quad.Ixx; Iyy = quad.Iyy; Izz = quad.Izz;
    
    u_dot = (r*v - q*w) - g*sin(theta)           + (Fx / m);
    v_dot = (p*w - r*u) + g*cos(theta)*sin(phi)  + (Fy / m);
    w_dot = (q*u - p*v) + g*cos(theta)*cos(phi)  + (Fz / m);
    
    p_dot = (Tx - (Izz - Iyy)*q*r) / Ixx;
    q_dot = (Ty - (Ixx - Izz)*p*r) / Iyy;
    r_dot = (Tz - (Iyy - Ixx)*p*q) / Izz;
    
    x_dot = u*cos(theta)*cos(psi) + v*(sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi)) + w*(cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi));
    y_dot = u*cos(theta)*sin(psi) + v*(sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi)) + w*(cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi)); 
    z_dot = -u*sin(theta)         + v*sin(phi)*cos(theta)                                + w*cos(phi)*cos(theta);
    
    phi_dot   = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
    theta_dot = q*cos(phi)            - r*sin(phi);
    psi_dot   = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta);
    
    State_dot_12 = [x_dot; y_dot; z_dot; u_dot; v_dot; w_dot; phi_dot; theta_dot; psi_dot; p_dot; q_dot; r_dot];
    State_dot = [State_dot_12; Servo_rates];
end


%% STATE ESTIMATOR
function [State_hat, mem_est] = state_estimator(Sensors, mem_est, param)
    % Calculate Alpha weights based on the time step and time constants
    a_pos   = param.dt / (param.tau_pos + param.dt);
    a_vel   = param.dt / (param.tau_vel + param.dt);
    a_att   = param.dt / (param.tau_att + param.dt);
    a_rates = param.dt / (param.tau_rates + param.dt);
    
    % First-Order Low-Pass Filter (PT1)
    State_hat = zeros(12,1);
    State_hat(1:3)   = (1 - a_pos)   * mem_est.Pos   + a_pos   * Sensors.Pos;
    State_hat(4:6)   = (1 - a_vel)   * mem_est.Vel   + a_vel   * Sensors.Vel;
    State_hat(7:9)   = (1 - a_att)   * mem_est.Euler + a_att   * Sensors.Euler;
    State_hat(10:12) = (1 - a_rates) * mem_est.Rates + a_rates * Sensors.Rates;
    
    % Save current estimate for the next loop's integration
    mem_est.Pos   = State_hat(1:3);
    mem_est.Vel   = State_hat(4:6);
    mem_est.Euler = State_hat(7:9);
    mem_est.Rates = State_hat(10:12);
end

%% SENSOR NOISE
function Sensors = sensor_model(State, param)
    % randn() generates standard Gaussian white noise
    Sensors.Pos   = State(1:3)   + param.noise_gps  * randn(3,1);
    Sensors.Vel   = State(4:6)   + param.noise_vel  * randn(3,1);
    Sensors.Euler = State(7:9)   + param.noise_att  * randn(3,1);
    Sensors.Rates = State(10:12) + param.noise_gyro * randn(3,1);
end

%% WIND DISTURBANCE MODEL
function [F_wind_earth, Tau_wind_body] = wind_model(t, param)
    persistent next_change_t current_F current_Tau
    
    % Reset the wind generator at the start of every simulation
    if isempty(next_change_t) || t < param.dt * 0.5
        next_change_t = 0; % Phase 4 starts at 15 seconds
        current_F = [0; 0; 0];
        current_Tau = [0; 0; 0];
    end

    F_wind_earth = [0; 0; 0];
    Tau_wind_body = [0; 0; 0];

    % if t >= 0 && t < 10
    %     % Phase 1: Strong wind pushing North (+X direction)
    %     % 1.5 Newtons is roughly 30% of the drone's total weight!
    %     F_wind_earth = [1; 0; 0]; 
    % 
    % elseif t >= 10 && t < 20
    %     % Phase 2: Wind shifts abruptly, pushing East (+Y direction)
    %     F_wind_earth = [0; 1.0; 0];
        % Tau_wind_body = [0.05; 0; 0.01];
        
    % elseif t >= 10 && t < 15
    %     % Phase 3: "Couple Wind" - A turbulent gust hitting the propellers,
    %     % causing unwanted Roll and Yaw twisting torques.
    %     Tau_wind_body = [0.05; 0; 0.1];
    if t >= 0
        if t >= next_change_t
            % 1. Generate a new random wind vector (up to ~1.5N force)
            current_F = 0.5 * randn(3,1); 
            
            % Add slight twisting torque to the gust
            current_Tau = 0.01 * randn(3,1); 
            
            % 2. Calculate a random duration between 1.0 and 5.0 seconds
            random_duration = 1.0 + (4.0 * rand());
            
            % 3. Set the timer for the next gust
            next_change_t = t + random_duration;          
        end
        
        % Apply the sustained gust to the current frame
        F_wind_earth = current_F;
        Tau_wind_body = current_Tau;
    end
end

%% TELEMETRY PLOTTING ENGINE
function plot_telemetry(Log)
    t = Log.Time;
    
    % -- Extract Data & Convert to Degrees --
    pos_act = Log.State(1:3, :);
    att_act = rad2deg(Log.State(7:9, :));
    
    pos_tgt = Log.Target(1:3, :);
    att_tgt = rad2deg(Log.Target(4:6, :));
    
    err_pos = pos_tgt - pos_act;
    err_att = rad2deg(wrapToPi(Log.Target(4:6, :) - Log.State(7:9, :)));
    
    thrust = Log.Thrust;
    servo = rad2deg(Log.Servo);

    % -- FIGURE 1: POSITION TRACKING --
    figure('Name', 'Position Tracking', 'Color', 'w', 'Position', [50, 100, 600, 800]);
    labels = {'X (North) [m]', 'Y (East) [m]', 'Z (Down) [m]'};
    for i = 1:3
        subplot(3, 1, i); hold on; grid on;
        plot(t, pos_tgt(i,:), 'r--', 'LineWidth', 1.5);
        plot(t, pos_act(i,:), 'b-', 'LineWidth', 1.5);
        ylabel(labels{i});
        if i == 1; title('POSITION: Target vs Actual'); legend('Target', 'Actual', 'Location', 'best'); end
    end
    xlabel('Time [s]');
    
    % -- FIGURE 2: ATTITUDE TRACKING --
    figure('Name', 'Attitude Tracking', 'Color', 'w', 'Position', [150, 100, 600, 800]);
    labels = {'Roll \phi [deg]', 'Pitch \theta [deg]', 'Yaw \psi [deg]'};
    for i = 1:3
        subplot(3, 1, i); hold on; grid on;
        plot(t, att_tgt(i,:), 'r--', 'LineWidth', 1.5);
        plot(t, att_act(i,:), 'b-', 'LineWidth', 1.5);
        ylabel(labels{i});
        if i == 1; title('ATTITUDE: Target vs Actual'); legend('Target', 'Actual', 'Location', 'best'); end
    end
    xlabel('Time [s]');
    
    % % -- FIGURE 3: POSITION ERROR --
    % figure('Name', 'Position Error', 'Color', 'w', 'Position', [250, 100, 600, 800]);
    % labels = {'X Error [m]', 'Y Error [m]', 'Z Error [m]'};
    % for i = 1:3
    %     subplot(3, 1, i); hold on; grid on;
    %     plot(t, err_pos(i,:), 'k-', 'LineWidth', 1.5);
    %     yline(0, 'r--', 'LineWidth', 1);
    %     ylabel(labels{i});
    %     if i == 1; title('POSITION ERROR (Target - Actual)'); end
    % end
    % xlabel('Time [s]');
    % 
    % % -- FIGURE 4: ATTITUDE ERROR --
    % figure('Name', 'Attitude Error', 'Color', 'w', 'Position', [350, 100, 600, 800]);
    % labels = {'Roll Error [deg]', 'Pitch Error [deg]', 'Yaw Error [deg]'};
    % for i = 1:3
    %     subplot(3, 1, i); hold on; grid on;
    %     plot(t, err_att(i,:), 'k-', 'LineWidth', 1.5);
    %     yline(0, 'r--', 'LineWidth', 1);
    %     ylabel(labels{i});
    %     if i == 1; title('ATTITUDE ERROR (Target - Actual)'); end
    % end
    % xlabel('Time [s]');
    
    % -- FIGURE 5: ACTUATOR EFFORT --
    figure('Name', 'Actuator Effort', 'Color', 'w', 'Position', [450, 100, 600, 600]);
    
    subplot(2, 1, 1); hold on; grid on;
    plot(t, thrust(1,:), 'LineWidth', 1.2);
    plot(t, thrust(2,:), 'LineWidth', 1.2);
    plot(t, thrust(3,:), 'LineWidth', 1.2);
    plot(t, thrust(4,:), 'LineWidth', 1.2);
    ylabel('Thrust [N]');
    title('MOTOR THRUST');
    legend('M1', 'M2', 'M3', 'M4', 'Location', 'best');
    
    subplot(2, 1, 2); hold on; grid on;
    plot(t, servo(1,:), 'LineWidth', 1.2);
    plot(t, servo(2,:), 'LineWidth', 1.2);
    plot(t, servo(3,:), 'LineWidth', 1.2);
    plot(t, servo(4,:), 'LineWidth', 1.2);
    ylabel('Servo Angle [deg]');
    title('TILT SERVO ANGLES');
    legend('S1', 'S2', 'S3', 'S4', 'Location', 'best');
    xlabel('Time [s]');

    % -- FIGURE 6: ESTIMATOR PERFORMANCE (Truth vs Noise vs Filter) --
    if isfield(Log, 'State_hat') && isfield(Log, 'Sensors')
        figure('Name', 'State Estimator Diagnostics', 'Color', 'w', 'Position', [550, 100, 700, 800]);
        
        % Extract Data
        z_true = Log.State(3, :);
        z_sens = Log.Sensors(3, :);
        z_hat  = Log.State_hat(3, :);
        
        pitch_true = rad2deg(Log.State(8, :));
        pitch_sens = rad2deg(Log.Sensors(8, :));
        pitch_hat  = rad2deg(Log.State_hat(8, :));
        
        q_true = rad2deg(Log.State(11, :));
        q_sens = rad2deg(Log.Sensors(11, :));
        q_hat  = rad2deg(Log.State_hat(11, :));
        
        % Plot 1: Z Position
        subplot(3, 1, 1); hold on; grid on;
        plot(t, z_sens, 'Color', [0.8 0.8 0.8], 'LineWidth', 1); % Gray Noise
        plot(t, z_true, 'k-', 'LineWidth', 1.5);                 % Black Truth
        plot(t, z_hat, 'b-', 'LineWidth', 1.5);                  % Blue Estimate
        ylabel('Z Position [m]');
        title('FILTER DIAGNOSTICS: Z Altitude');
        legend('Raw Sensor (Noisy)', 'Ground Truth', 'Estimated State', 'Location', 'best');
        
        % Plot 2: Pitch Angle
        subplot(3, 1, 2); hold on; grid on;
        plot(t, pitch_sens, 'Color', [0.8 0.8 0.8], 'LineWidth', 1);
        plot(t, pitch_true, 'k-', 'LineWidth', 1.5);
        plot(t, pitch_hat, 'b-', 'LineWidth', 1.5);
        ylabel('Pitch \theta [deg]');
        title('FILTER DIAGNOSTICS: Pitch Angle');
        
        % Plot 3: Pitch Rate (Critical for D-Gain Survival)
        subplot(3, 1, 3); hold on; grid on;
        plot(t, q_sens, 'Color', [0.8 0.8 0.8], 'LineWidth', 1);
        plot(t, q_true, 'k-', 'LineWidth', 1.5);
        plot(t, q_hat, 'b-', 'LineWidth', 1.5);
        ylabel('Pitch Rate q [deg/s]');
        title('FILTER DIAGNOSTICS: Pitch Rate (Gyro)');
        xlabel('Time [s]');
    end

    if isfield(Log, 'Wind_F')
        figure('Name', 'Wind Disturbance', 'Color', 'w', 'Position', [650, 100, 600, 600]);
        
        subplot(2, 1, 1); hold on; grid on;
        plot(t, Log.Wind_F(1,:), 'r', 'LineWidth', 1.5);
        plot(t, Log.Wind_F(2,:), 'g', 'LineWidth', 1.5);
        plot(t, Log.Wind_F(3,:), 'b', 'LineWidth', 1.5);
        title('WIND FORCE (Earth Frame)');
        ylabel('Force [N]');
        legend('F_x (North)', 'F_y (East)', 'F_z (Down)', 'Location', 'best');
        
        subplot(2, 1, 2); hold on; grid on;
        plot(t, Log.Wind_Tau(1,:), 'r', 'LineWidth', 1.5);
        plot(t, Log.Wind_Tau(2,:), 'g', 'LineWidth', 1.5);
        plot(t, Log.Wind_Tau(3,:), 'b', 'LineWidth', 1.5);
        title('WIND TORQUE (Body Frame Turbulence)');
        ylabel('Torque [Nm]');
        xlabel('Time [s]');
        legend('\tau_x (Roll)', '\tau_y (Pitch)', '\tau_z (Yaw)', 'Location', 'best');
    end
    % -- FIGURE 8: POSITION vs WIND BLEND (Disturbance Rejection) --
    if isfield(Log, 'Wind_F')
        figure('Name', 'Position & Wind Blend', 'Color', 'w', 'Position', [750, 100, 700, 800]);
        
        pos_labels = {'X (North) [m]', 'Y (East) [m]', 'Z (Down) [m]'};
        wind_labels = {'F_x Wind [N]', 'F_y Wind [N]', 'F_z Wind [N]'};
        
        for i = 1:3
            subplot(3, 1, i); hold on; grid on;
            
            % --- LEFT AXIS: POSITION ---
            yyaxis left
            % Reset color order for left axis so we don't get default MATLAB colors
            ax = gca; ax.YColor = 'k'; 
            
            p1 = plot(t, pos_tgt(i,:), 'r--', 'LineWidth', 1.5);
            p2 = plot(t, pos_act(i,:), 'b-', 'LineWidth', 1.5);
            ylabel(pos_labels{i}, 'Color', 'k', 'FontWeight', 'bold');
            
            % --- RIGHT AXIS: WIND FORCE ---
            yyaxis right
            ax.YColor = 'm'; % Make right axis magenta to match the 3D vector
            
            % Plot the wind as a filled area/staircase to easily see the "block" gusts
            p3 = plot(t, Log.Wind_F(i,:), 'm-', 'LineWidth', 1.5);
            
            ylabel(wind_labels{i}, 'Color', 'm', 'FontWeight', 'bold');
            
            % Formatting
            if i == 1
                title('DISTURBANCE REJECTION: Position vs Wind Force');
                legend([p1, p2, p3], {'Target Position', 'Actual Position', 'Wind Force'}, 'Location', 'best');
            end
        end
        xlabel('Time [s]', 'FontWeight', 'bold');
    end
end

%% VISUALIZER
function visualize_drone(Log, quad)
   fig = figure('Name', 'Tilt-Rotor', 'Color', 'w', ...
             'Position', [100, 100, 1280, 720], ...
             'Resize', 'off', ...
             'MenuBar', 'none', ...        % Removes menu bar (changes height)
             'ToolBar', 'none');           % Removes toolbar (changes height)
    view(3); grid on; axis equal; hold on;

    set(gca, 'ZDir', 'reverse', 'YDir', 'reverse'); 
    
    h_title = title('Time: 0.00 s | Simulating...', 'FontSize', 14, 'FontWeight', 'bold');
    h_wind_hud = annotation('textbox', [0.80, 0.4, 0.18, 0.5], ...
        'String', 'WIND: OFF', 'EdgeColor', 'k', 'LineWidth', 1.5, ...
        'BackgroundColor', [0.95 0.95 0.95], 'FontSize', 11, 'FontName', 'Courier');

    arm_lines = zeros(4,3);
    for i = 1:4; arm_lines(i,:) = [quad.L*cos(quad.gamma(i)), quad.L*sin(quad.gamma(i)), 0]; end
    h_arms = gobjects(1,4); h_thrust = gobjects(1,4); h_prop = gobjects(1,4); 
    for i = 1:4
        h_arms(i) = plot3([0 0], [0 0], [0 0], 'k', 'LineWidth', 3);   
        h_thrust(i) = plot3([0 0], [0 0], [0 0], 'r', 'LineWidth', 2); 
        h_prop(i) = plot3([0 0], [0 0], [0 0], 'g', 'LineWidth', 2);   
    end
    h_traj = plot3(Log.State(1,1), Log.State(2,1), Log.State(3,1), 'b:', 'LineWidth', 1);
    
    h_wind_line = plot3([0 0], [0 0], [0 0], 'c-', 'LineWidth', 2);
    h_wind_head = plot3(0, 0, 0, 'c^', 'MarkerSize', 6, 'MarkerFaceColor', 'c');

    video_flag = false; % Set to false if you want to run without recording
    if video_flag
        vid_obj = VideoWriter('tilt_rotor_nmpc.mp4', 'MPEG-4');
        vid_obj.FrameRate = 25; % 500Hz / 20 (loop downsample) = 25 FPS
        vid_obj.Quality = 100;
        open(vid_obj);
        disp('Recording video...');
    end

    drawnow;
    target_size = [];
    for k = 1:20:length(Log.Time)
        pos = Log.State(1:3, k); phi = Log.State(7, k); theta = Log.State(8, k); psi = Log.State(9, k);

        set(h_title, 'String', sprintf('Time: %05.2f s | Altitude: %0.1f m', Log.Time(k), -pos(3)));

        xlim([pos(1)-2, pos(1)+2]); ylim([pos(2)-2, pos(2)+2]); zlim([pos(3)-2, pos(3)+2]);
        set(h_traj, 'XData', Log.State(1, 1:k), 'YData', Log.State(2, 1:k), 'ZData', Log.State(3, 1:k));
        R = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
             cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
            -sin(theta),          sin(phi)*cos(theta),                            cos(phi)*cos(theta)];
        for i = 1:4
            motor_pos = pos + R * arm_lines(i,:)';
            set(h_arms(i), 'XData', [pos(1), motor_pos(1)], 'YData', [pos(2), motor_pos(2)], 'ZData', [pos(3), motor_pos(3)]);
            s_a = sin(Log.Servo(i, k)); c_a = cos(Log.Servo(i, k));
            v_thrust = R * ([-s_a * sin(quad.gamma(i)); s_a * cos(quad.gamma(i)); -c_a] * 0.3);
            set(h_thrust(i), 'XData', [motor_pos(1), motor_pos(1)+v_thrust(1)], 'YData', [motor_pos(2), motor_pos(2)+v_thrust(2)], 'ZData', [motor_pos(3), motor_pos(3)+v_thrust(3)]);
            tp = Log.Propeller(i, k); ub = [cos(quad.gamma(i)); sin(quad.gamma(i)); 0]; vb = [c_a*sin(quad.gamma(i)); -c_a*cos(quad.gamma(i)); -s_a];
            p1 = motor_pos + R * (0.127*(cos(tp)*ub + sin(tp)*vb)); p2 = motor_pos - R * (0.127*(cos(tp)*ub + sin(tp)*vb));
            set(h_prop(i), 'XData', [p1(1), p2(1)], 'YData', [p1(2), p2(2)], 'ZData', [p1(3), p2(3)]);
        end

        if isfield(Log, 'Wind_F')
            wf = Log.Wind_F(:, k);
            wtau = Log.Wind_Tau(:, k);

            % 1. Update the Text Box
            hud_text = sprintf('WIND DISTURBANCE\n-----------------\nFx(N) : %5.2f N\nFy(E) : %5.2f N\nFz(D) : %5.2f N\n\nTx(R) : %5.2f Nm\nTy(P) : %5.2f Nm\nTz(Y) : %5.2f Nm', ...
                wf(1), wf(2), wf(3), wtau(1), wtau(2), wtau(3));
            set(h_wind_hud, 'String', hud_text);

            % 2. Update the Magenta Vector
            if norm(wf) > 0.05
                wind_end = pos + (wf * 1.5); % Visually scale the line length
                set(h_wind_line, 'XData', [pos(1), wind_end(1)], 'YData', [pos(2), wind_end(2)], 'ZData', [pos(3), wind_end(3)]);
                set(h_wind_head, 'XData', wind_end(1), 'YData', wind_end(2), 'ZData', wind_end(3));
            else
                set(h_wind_line, 'XData', [pos(1), pos(1)], 'YData', [pos(2), pos(2)], 'ZData', [pos(3), pos(3)]);
                set(h_wind_head, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3));
            end
        end
        drawnow;
        
        % --- CAPTURE FRAME ---
        if video_flag
            frame = getframe(fig);   % Use the figure handle directly
            
            if isempty(target_size)
                target_size = size(frame.cdata);
            elseif ~isequal(size(frame.cdata), target_size)
                frame.cdata = imresize(frame.cdata, target_size(1:2));
            end
            
            writeVideo(vid_obj, frame);
        end
    end 
    % --- CLOSE VIDEO ---
    if video_flag
        close(vid_obj);
        disp(['Video saved as ', vid_obj.Filename]);
    end
end 



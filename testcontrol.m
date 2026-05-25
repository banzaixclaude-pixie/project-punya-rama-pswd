%% 6-DOF TILT-ROTOR UNIFIED SIMULATOR AND AUTOTUNER
clear; clc; close all;

%% FLIGHT DIRECTOR
RUN_MODE = 'SIMULATE';

%% GAINS
% Position loop
gains.Kp_pos = [1.0; 1.0; 1.5];
gains.Kp_vel = [2.5; 2.5; 4.5];
gains.Ki_vel = [1.5; 1.5; 2.5];
gains.Kd_vel = [0.1; 0.1; 0.2];

% Attitude loop
gains.Kp_ang  = [4.0; 4.0; 2.0];
gains.Kp_rate = [28.0; 28.0; 9.0];
gains.Ki_rate = [5.0; 5.0; 2.0];
gains.Kd_rate = [0.2; 0.2; 0.1];

%% ENVIRONMENT AND AIRFRAME
env.grav = 9.81;
env.rho = 1.225;

quad.mass = 0.5;
quad.L = 0.22;
quad.Ixx = 0.012; quad.Iyy = 0.012; quad.Izz = 0.022;
quad.I = diag([quad.Ixx, quad.Iyy, quad.Izz]);
quad.Irot = 0.0001;
quad.gamma = [pi/4, 3*pi/4, 5*pi/4, 7*pi/4];
quad.lambda = [1;-1;1;-1];
quad.kQ = 0.015;
quad.max_thrust = 2.0 * env.grav;
quad.min_thrust = 0;
quad.max_tilt = deg2rad(75);
quad.min_tilt = deg2rad(-75);

%% FLIGHT PARAMETERS
param.V_cmax = 2.5;
param.Climb_max = 1.5;
param.Sink_max = 1;
param.wp_rad = 2;
param.dt = 0.002;
param.servo_slew_rate = deg2rad(200);
param.max_accel = 3;

% Sensor noise
param.noise_gps  = 0.2;
param.noise_vel  = 0.1;
param.noise_att  = deg2rad(1.0);
param.noise_gyro = 0.05;

% Estimator time constants
param.tau_pos   = 0.1;
param.tau_vel   = 0.05;
param.tau_att   = 0.02;
param.tau_rates = 0.01;

%% MIXER PRECOMPUTATION
s = sin(quad.gamma); c = cos(quad.gamma);
B = [ 0, 0, 0, 0, -s(1), -s(2), -s(3), -s(4);
      0, 0, 0, 0,  c(1),  c(2),  c(3),  c(4);
     -1,-1,-1,-1,   0,     0,     0,     0;
    -quad.L*s(1), -quad.L*s(2), -quad.L*s(3), -quad.L*s(4), 0, 0, 0, 0;
     quad.L*c(1),  quad.L*c(2),  quad.L*c(3),  quad.L*c(4), 0, 0, 0, 0;
     quad.kQ, -quad.kQ, quad.kQ, -quad.kQ, quad.L, quad.L, quad.L, quad.L];
W = diag([1, 1, 1, 1, 1000, 1000, 1000, 1000]);
quad.B_pinv = inv(W) * B' * inv(B * inv(W) * B');

%% EXECUTION
if strcmp(RUN_MODE, 'TUNE_RATES')
    disp('--- STARTING RATE AUTOTUNER ---');
    init_guess = [gains.Kp_rate(1), gains.Kp_rate(2), gains.Kp_rate(3), ...
                  gains.Ki_rate(1), gains.Ki_rate(2), gains.Ki_rate(3), ...
                  gains.Kd_rate(1), gains.Kd_rate(2), gains.Kd_rate(3)];

    options = optimset('Display', 'iter', 'TolX', 1e-4, 'TolFun', 1e-4, 'MaxIter', 500);
    best = fminsearch(@(x) evaluate_cost(x, 'RATES', quad, env, param, gains), init_guess, options);

    fprintf('\nUPDATE YOUR GAINS:\n');
    fprintf('gains.Kp_rate = [%.4f; %.4f; %.4f];\n', best(1), best(2), best(3));
    fprintf('gains.Ki_rate = [%.4f; %.4f; %.4f];\n', best(4), best(5), best(6));
    fprintf('gains.Kd_rate = [%.4f; %.4f; %.4f];\n', best(7), best(8), best(9));

elseif strcmp(RUN_MODE, 'TUNE_TRANSLATION')
    disp('--- STARTING TRANSLATION AUTOTUNER ---');
    init_guess = [gains.Kp_pos(1), gains.Kp_pos(3), gains.Kp_vel(1), gains.Kp_vel(3), gains.Kd_vel(1), gains.Kd_vel(3)];
    options = optimset('Display', 'iter', 'TolX', 1e-3, 'TolFun', 1e-3, 'MaxIter', 150);
    best = fminsearch(@(x) evaluate_cost(x, 'TRANS', quad, env, param, gains), init_guess, options);

    fprintf('\nUPDATE YOUR GAINS:\n');
    fprintf('gains.Kp_pos = [%.4f; %.4f; %.4f];\n', best(1), best(1), best(2));
    fprintf('gains.Kp_vel = [%.4f; %.4f; %.4f];\n', best(3), best(3), best(4));
    fprintf('gains.Kd_vel = [%.4f; %.4f; %.4f];\n', best(5), best(5), best(6));

elseif strcmp(RUN_MODE, 'SIMULATE')
    disp('--- STARTING FLIGHT SIMULATION ---');
    waypoints = [
        0, 0, -5, 0, 0, 0, 2;
        5, 0, -5, 0, 0, pi/2, 2;
        5, 0, -5, 0, pi/4, 0, 2;
    ];

    IC.States = zeros(12,1);
    T_end = 30;

    [Log] = run_flight_loop(waypoints, IC, T_end, quad, env, param, gains);
    plot_telemetry(Log);

    disp('Simulation Complete. Rendering...');
    visualize_drone(Log, quad);

else
    error('Invalid RUN_MODE. Choose TUNE_RATES, TUNE_TRANSLATION, or SIMULATE.');
end

%% AUTOTUNE COST EVALUATOR
function cost = evaluate_cost(test_gains, mode, quad, env, param, base_gains)
    if any(test_gains < 0); cost = 1e6; return; end

    IC.States = zeros(12,1);

    if strcmp(mode, 'RATES')
        base_gains.Kp_rate = [test_gains(1); test_gains(2); test_gains(3)];
        base_gains.Ki_rate = [test_gains(4); test_gains(5); test_gains(6)];
        base_gains.Kd_rate = [test_gains(7); test_gains(8); test_gains(9)];

        IC.States(3) = -10;
        test_waypoints = [0, 0, -10, 0, deg2rad(15), 0, 0];

        % Constant disturbance torque to exercise the I-term
        param.tuning_torque = [0.1; 0.1; 0.05];
        param.tuning_wind = [0; 0; 0];
        T_end = 2.5;

    elseif strcmp(mode, 'TRANS')
        base_gains.Kp_pos = [test_gains(1); test_gains(1); test_gains(2)];
        base_gains.Kp_vel = [test_gains(3); test_gains(3); test_gains(4)];
        base_gains.Kd_vel = [test_gains(5); test_gains(5); test_gains(6)];

        IC.States(3) = -10;
        test_waypoints = [5, 5, -15, 0, 0, 0, 0];
        T_end = 4.0;
    end

    clear wind_model;
    [~, crash_flag, ITAE_score] = run_flight_loop(test_waypoints, IC, T_end, quad, env, param, base_gains);

    if crash_flag
        cost = 1e6;
    else
        cost = ITAE_score;
    end
end

%% FLIGHT LOOP
function [Log, crash_flag, ITAE_score] = run_flight_loop(waypoints, IC, T_end, quad, env, param, gains)
    N_steps = round(T_end / param.dt);

    % Preallocate logs
    Log.Time = zeros(1, N_steps);
    Log.State = zeros(12, N_steps);
    Log.Thrust = zeros(4, N_steps);
    Log.Servo = zeros(4, N_steps);
    Log.Propeller = zeros(4, N_steps);
    Log.Target = zeros(6, N_steps);
    Log.State_hat = zeros(12, N_steps);
    Log.Sensors = zeros(12, N_steps);
    Log.Wind_F = zeros(3, N_steps);
    Log.Wind_Tau = zeros(3, N_steps);

    % Initial state
    State = IC.States;
    mem.vel_i = [0;0;0];
    mem.rate_i = [0;0;0];
    mem.rate_prev = [0;0;0];
    mem.euler_target_prev = IC.States(7:9);
    mem.rate_target_prev = zeros(3,1);
    mem.vel_target_prev = [0;0;0];

    phi0 = IC.States(7); theta0 = IC.States(8); psi0 = IC.States(9);
    R0 = [  cos(theta0)*cos(psi0), sin(phi0)*sin(theta0)*cos(psi0)-cos(phi0)*sin(psi0), cos(phi0)*sin(theta0)*cos(psi0)+sin(phi0)*sin(psi0);
            cos(theta0)*sin(psi0), sin(phi0)*sin(theta0)*sin(psi0)+cos(phi0)*cos(psi0), cos(phi0)*sin(theta0)*sin(psi0)-sin(phi0)*cos(psi0);
            -sin(theta0),           sin(phi0)*cos(theta0),                               cos(phi0)*cos(theta0)];
    mem.vel_prev = R0 * IC.States(4:6);

    mem_est.Pos   = IC.States(1:3);
    mem_est.Vel   = IC.States(4:6);
    mem_est.Euler = IC.States(7:9);
    mem_est.Rates = IC.States(10:12);

    % Waypoint state machine
    current_wp = 1;
    total_wp = size(waypoints, 1);
    hover_start_time = NaN;

    % Actuator states
    Servos_Actual = zeros(4,1);
    Thrust_Actual = ones(4,1) * (quad.mass * env.grav / 4);
    Prop_ang = zeros(4,1);
    Pos_target_filtered = IC.States(1:3);
    Servo_rates = zeros(4,1);
    Omega = zeros(4,1);

    crash_flag = false;
    ITAE_score = 0;

    for k = 1:N_steps
        t = (k-1) * param.dt;
        Target = waypoints(current_wp, 1:6)';
        Target_raw = Target;
        Wait_Time = waypoints(current_wp, 7);

        % Smooth position target
        Pos_target_filtered = Pos_target_filtered + (Target(1:3) - Pos_target_filtered) * (param.dt / param.tau_pos);
        Target(1:3) = Pos_target_filtered;
        Log.Target(:, k) = Target;

        % Estimation
        Sensors = sensor_model(State, param);
        [State_hat, mem_est] = state_estimator(Sensors, mem_est, param);
        Log.Sensors(:, k) = [Sensors.Pos; Sensors.Vel; Sensors.Euler; Sensors.Rates];
        Log.State_hat(:, k) = State_hat;

        Pos_est = State_hat(1:3);
        Ang_est = State_hat(7:9);
        Rates_est = State_hat(10:12);
        distance = norm(Target_raw(1:3) - Pos_est);

        % Waypoint state machine
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

        % Control
        [F_effort, Tau_effort, mem] = controller(State_hat, Target, quad, env, param, mem, gains, Servos_Actual, Servo_rates, Omega);
        [Thrusts, Servos] = motor_mixer(F_effort, Tau_effort, quad);

        % Actuator dynamics
        servo_err_clamped = max(min(Servos - Servos_Actual, param.servo_slew_rate*param.dt), -param.servo_slew_rate*param.dt);
        Servos_Actual = Servos_Actual + servo_err_clamped;
        Servo_rates = servo_err_clamped / param.dt;

        Omega = sqrt(max(Thrust_Actual, 0) / 1.5e-5);
        thrust_err_clamped = max(min(Thrusts - Thrust_Actual, 50*param.dt), -50*param.dt);
        Thrust_Actual = Thrust_Actual + thrust_err_clamped;

        % Physics
        [F_body, Tau_body] = force_calc(Thrust_Actual, Servos_Actual, Servo_rates, State, Omega, quad);

        F_wind_earth = [0;0;0];
        Tau_wind_body = [0;0;0];
        if isfield(param, 'tuning_torque')
            Tau_wind_body = param.tuning_torque;
            F_wind_earth = param.tuning_wind;
        end

        % Transform wind to body frame
        phi_real = State(7); theta_real = State(8); psi_real = State(9);
        R_b_to_ned_real = [cos(theta_real)*cos(psi_real), sin(phi_real)*sin(theta_real)*cos(psi_real)-cos(phi_real)*sin(psi_real), cos(phi_real)*sin(theta_real)*cos(psi_real)+sin(phi_real)*sin(psi_real);
                           cos(theta_real)*sin(psi_real), sin(phi_real)*sin(theta_real)*sin(psi_real)+cos(phi_real)*cos(psi_real), cos(phi_real)*sin(theta_real)*sin(psi_real)-sin(phi_real)*cos(psi_real);
                          -sin(theta_real),               sin(phi_real)*cos(theta_real),                                           cos(phi_real)*cos(theta_real)];

        F_body = F_body + R_b_to_ned_real' * F_wind_earth;
        Tau_body = Tau_body + Tau_wind_body;

        % Integrate
        State = RK4(@EoM, State, F_body, Tau_body, quad, env, param.dt);

        % Log
        Prop_ang = Prop_ang + (quad.lambda(:) .* Omega) * param.dt;
        Log.Time(k) = t;
        Log.State(:, k) = State;
        Log.Thrust(:, k) = Thrust_Actual;
        Log.Servo(:, k) = Servos_Actual;
        Log.Propeller(:, k) = Prop_ang;
        Log.Wind_F(:, k) = F_wind_earth;
        Log.Wind_Tau(:, k) = Tau_wind_body;

        % ITAE cost for autotuner
        pos_err = norm(Target(1:3) - Pos_est);
        ang_err = Target(4:6) - Ang_est; ang_err(3) = wrapToPi(ang_err(3));
        rate_err = norm(Rates_est);

        t_weighted = max(t, param.dt);
        ITAE_score = ITAE_score + (pos_err + norm(ang_err)*10 + rate_err) * t_weighted * param.dt;

        % Crash detection
        if State(3) > 1 || abs(State(7)) > pi/2 || abs(State(8)) > pi/2
            crash_flag = true;
            disp('CRASH DETECTED. Stopping integration.');
            Log.Time = Log.Time(1:k); Log.State = Log.State(:, 1:k);
            Log.Thrust = Log.Thrust(:, 1:k); Log.Servo = Log.Servo(:, 1:k);
            Log.Propeller = Log.Propeller(:, 1:k); Log.Target = Log.Target(:, 1:k);
            Log.Sensors = Log.Sensors(:, 1:k); Log.State_hat = Log.State_hat(:, 1:k);
            Log.Wind_F = Log.Wind_F(:, 1:k); Log.Wind_Tau = Log.Wind_Tau(:, 1:k);
            break;
        end
    end
end

%% CONTROLLER
function [F_body, Tau_body, mem] = controller(State, Target, quad, env, param, mem, gains, Servos, Servo_rates, Omega)
    Pos_actual = State(1:3); Vel_b = State(4:6); Euler = State(7:9); Rates = State(10:12);
    phi = Euler(1); theta = Euler(2); psi = Euler(3);

    R_b_to_ned = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
                  cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
                 -sin(theta),          sin(phi)*cos(theta),                            cos(phi)*cos(theta)];

    % Gain scheduling with forward speed
    forward_speed = norm(State(4:5));
    speed_factor = min(forward_speed / param.V_cmax, 1.0);
    active_Kp_rate = gains.Kp_rate .* (1 - 0.25 * speed_factor);

    % Degrade speed limits at high tilt angles
    current_tilt = max(abs(phi), abs(theta));
    tilt_margin = max((quad.max_tilt - current_tilt) / quad.max_tilt, 0);
    speed_degrade = max(min(tilt_margin * 2.0, 1.0), 0.1);
    active_V_cmax = param.V_cmax * speed_degrade;
    active_max_accel = param.max_accel * speed_degrade;

    % POSITION LOOP - Square root velocity shaping
    Pos_err = Target(1:3) - Pos_actual;
    Vel_target_xy = shape_velocity(Pos_err(1:2), gains.Kp_pos(1), active_V_cmax, active_max_accel);

    if Pos_err(3) > 0
        Vel_z_limit = param.Sink_max;
    else
        Vel_z_limit = param.Climb_max;
    end
    Vel_target_z = shape_velocity(Pos_err(3), gains.Kp_pos(3), Vel_z_limit, active_max_accel);
    Vel_target = [Vel_target_xy(1); Vel_target_xy(2); Vel_target_z];

    % Rate-limit velocity target changes
    dv = Vel_target - mem.vel_target_prev;
    dv_mag = norm(dv);
    max_dv = active_max_accel * param.dt;
    if dv_mag > max_dv
        Vel_target = mem.vel_target_prev + (dv * (max_dv / dv_mag));
    end

    % Acceleration feedforward
    accel_ff = (Vel_target - mem.vel_target_prev) / param.dt;
    mem.vel_target_prev = Vel_target;

    % VELOCITY LOOP - PID
    Vel_Earth = R_b_to_ned * Vel_b;
    Vel_err = Vel_target - Vel_Earth;

    mem.vel_i = mem.vel_i + (Vel_err * param.dt);
    vel_d = -(Vel_Earth - mem.vel_prev) / param.dt;
    mem.vel_prev = Vel_Earth;

    Accel_unconstrained = (gains.Kp_vel .* Vel_err) + (gains.Ki_vel .* mem.vel_i) + (gains.Kd_vel .* vel_d) + accel_ff;
    F_Earth_unconstrained = quad.mass .* (Accel_unconstrained + [0; 0; -env.grav]);

    % THRUST LIMITING - Z priority, then XY gets leftovers
    total_max_thrust = quad.max_thrust * 4;
    F_Earth_constrained = F_Earth_unconstrained;

    if F_Earth_constrained(3) > 0
        F_Earth_constrained(3) = 0;
    end
    if abs(F_Earth_constrained(3)) > total_max_thrust
        F_Earth_constrained(3) = -total_max_thrust;
    end

    max_xy_thrust = sqrt(max(total_max_thrust^2 - F_Earth_constrained(3)^2, 0));
    xy_mag = norm(F_Earth_constrained(1:2));
    if xy_mag > max_xy_thrust
        F_Earth_constrained(1:2) = F_Earth_constrained(1:2) * (max_xy_thrust / xy_mag);
    end

    % Anti-windup back-calculation
    F_deficit = F_Earth_unconstrained - F_Earth_constrained;
    Accel_deficit = F_deficit / quad.mass;
    mem.vel_i = mem.vel_i - (gains.Ki_vel .* Accel_deficit * param.dt);
    F_body = R_b_to_ned' * F_Earth_constrained;

    % ATTITUDE LOOP
    Ang_err = Target(4:6) - Euler;
    Ang_err(3) = wrapToPi(Ang_err(3));
    T_inv = [1,  0,        -sin(theta);
             0,  cos(phi),  sin(phi)*cos(theta);
             0, -sin(phi),  cos(phi)*cos(theta)];

    % Attitude target feedforward
    Euler_rate_ff = (Target(4:6) - mem.euler_target_prev) / param.dt;
    mem.euler_target_prev = Target(4:6);
    Euler_rate_ff = max(min(Euler_rate_ff, deg2rad(180)), -deg2rad(180));

    Euler_rate_des = (gains.Kp_ang .* Ang_err) + Euler_rate_ff;
    Euler_rate_des = max(min(Euler_rate_des, deg2rad(90)), -deg2rad(90));
    Rate_target = T_inv * Euler_rate_des;

    % Rate target feedforward
    alpha_ff = (Rate_target - mem.rate_target_prev) / param.dt;
    mem.rate_target_prev = Rate_target;
    alpha_ff = max(min(alpha_ff, 100), -100);

    % RATE LOOP - PID
    Rate_err = Rate_target - Rates;
    mem.rate_i = max(min(mem.rate_i + (Rate_err * param.dt), 2.0), -2.0);
    rate_d = -(Rates - mem.rate_prev) / param.dt;
    mem.rate_prev = Rates;

    Alpha_des = alpha_ff + (active_Kp_rate .* Rate_err) + (gains.Ki_rate .* mem.rate_i) + (gains.Kd_rate .* rate_d);

    % Cross-coupling compensation
    p = Rates(1); q = Rates(2); r = Rates(3);
    s_a = sin(Servos(:)); c_a = cos(Servos(:));
    s = sin(quad.gamma(:)); c = cos(quad.gamma(:));

    Ixx = quad.Ixx; Iyy = quad.Iyy; Izz = quad.Izz;
    tau_cross = [ (Izz - Iyy)*q*r;
                  (Ixx - Izz)*p*r;
                  (Iyy - Ixx)*p*q ];

    Tau_pid = diag([Ixx, Iyy, Izz]) * Alpha_des + tau_cross;

    % Gyroscopic precession cancellation
    Tx_gyro = 0; Ty_gyro = 0; Tz_gyro = 0;
    for i = 1:4
        H_mag = quad.Irot * quad.lambda(i) * Omega(i);
        Tx_gyro = Tx_gyro + H_mag * (Servo_rates(i)*c_a(i)*s(i) + q*c_a(i) + r*s_a(i)*c(i));
        Ty_gyro = Ty_gyro + H_mag * (-Servo_rates(i)*c_a(i)*c(i) - p*c_a(i) + r*s_a(i)*s(i));
        Tz_gyro = Tz_gyro + H_mag * (-Servo_rates(i)*s_a(i) - p*s_a(i)*c(i) - q*s_a(i)*s(i));
    end

    Tau_body = Tau_pid - [Tx_gyro; Ty_gyro; Tz_gyro];
end

%% MOTOR MIXER
function [Thrusts, Servos] = motor_mixer(F_effort, Tau_effort, quad)
    U_virt  = quad.B_pinv * [F_effort; Tau_effort];
    T_cmd   = U_virt(1:4);
    H_cmd   = U_virt(5:8);
    Thrusts = zeros(4,1);
    Servos  = zeros(4,1);

    for i = 1:4
        % Minimum thrust floor for attitude authority
        if T_cmd(i) < 0.1; T_cmd(i) = 0.1; end

        % Saturate: preserve vertical, clip horizontal
        F_total = sqrt(T_cmd(i)^2 + H_cmd(i)^2);
        if F_total > quad.max_thrust
            if T_cmd(i) > quad.max_thrust
                T_cmd(i) = quad.max_thrust;
                H_cmd(i) = 0;
            else
                available_H = sqrt(quad.max_thrust^2 - T_cmd(i)^2);
                H_cmd(i) = sign(H_cmd(i)) * min(abs(H_cmd(i)), available_H);
            end
        end

        Thrusts(i) = sqrt(T_cmd(i)^2 + H_cmd(i)^2);
        if Thrusts(i) > 0.1
            Servos(i) = max(min(atan2(H_cmd(i), T_cmd(i)), quad.max_tilt), quad.min_tilt);
        else
            Servos(i) = 0;
        end
    end
end

%% FORCE CALCULATOR
function [F_body, Tau_body] = force_calc(Thrusts, Servos, Servo_rates, State, Omega, quad)
    p = State(10); q = State(11); r = State(12);
    s = sin(quad.gamma(:)); c = cos(quad.gamma(:));
    s_a = sin(Servos(:)); c_a = cos(Servos(:));

    H = Thrusts(:) .* s_a;
    T = Thrusts(:) .* c_a;
    Fx = sum(-H .* s);
    Fy = sum(H .* c);
    Fz = sum(-T);

    % Gyroscopic torques from tilted spinning rotors
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

    F_body = [Fx; Fy; Fz];
    Tau_body = [Tx; Ty; Tz];
end

%% EQUATIONS OF MOTION
function State_next = RK4(EoM_func, State, F_body, Tau_body, quad, env, dt)
    k1 = EoM_func(State, F_body, Tau_body, quad, env);
    k2 = EoM_func(State + 0.5*dt*k1, F_body, Tau_body, quad, env);
    k3 = EoM_func(State + 0.5*dt*k2, F_body, Tau_body, quad, env);
    k4 = EoM_func(State + dt*k3, F_body, Tau_body, quad, env);
    State_next = State + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
end

function State_dot = EoM(State, F_body, Tau_body, quad, env)
    u = State(4); v = State(5); w = State(6);
    phi = State(7); theta = State(8); psi = State(9);
    p = State(10); q = State(11); r = State(12);

    Fx = F_body(1); Fy = F_body(2); Fz = F_body(3);
    Tx = Tau_body(1); Ty = Tau_body(2); Tz = Tau_body(3);
    m = quad.mass; g = env.grav;
    Ixx = quad.Ixx; Iyy = quad.Iyy; Izz = quad.Izz;

    % Body-frame accelerations
    u_dot = (r*v - q*w) - g*sin(theta)           + (Fx / m);
    v_dot = (p*w - r*u) + g*cos(theta)*sin(phi)  + (Fy / m);
    w_dot = (q*u - p*v) + g*cos(theta)*cos(phi)  + (Fz / m);

    % Angular accelerations
    p_dot = (Tx - (Izz - Iyy)*q*r) / Ixx;
    q_dot = (Ty - (Ixx - Izz)*p*r) / Iyy;
    r_dot = (Tz - (Iyy - Ixx)*p*q) / Izz;

    % NED position kinematics
    x_dot = u*cos(theta)*cos(psi) + v*(sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi)) + w*(cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi));
    y_dot = u*cos(theta)*sin(psi) + v*(sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi)) + w*(cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi));
    z_dot = -u*sin(theta)         + v*sin(phi)*cos(theta)                                + w*cos(phi)*cos(theta);

    % Euler angle kinematics
    phi_dot   = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
    theta_dot = q*cos(phi)            - r*sin(phi);
    psi_dot   = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta);

    State_dot = [x_dot; y_dot; z_dot; u_dot; v_dot; w_dot; phi_dot; theta_dot; psi_dot; p_dot; q_dot; r_dot];
end

%% SQUARE ROOT VELOCITY SHAPER
function V_target = shape_velocity(Pos_err, Kp, max_vel, max_accel)
    err_mag = norm(Pos_err);
    if err_mag == 1e-6
        V_target = zeros(size(Pos_err));
        return;
    end

    % Boundary between linear P and kinematic braking profile
    linear_dist = max_accel / (Kp^2);

    if err_mag > linear_dist
        V_mag = sqrt(2.0 * max_accel * (err_mag - (linear_dist / 2.0)));
    else
        V_mag = Kp * err_mag;
    end

    V_mag = min(V_mag, max_vel);
    V_target = (Pos_err / err_mag) * V_mag;
end

%% SENSOR MODEL
function Sensors = sensor_model(State, param)
    Sensors.Pos   = State(1:3)   + param.noise_gps  * randn(3,1);
    Sensors.Vel   = State(4:6)   + param.noise_vel  * randn(3,1);
    Sensors.Euler = State(7:9)   + param.noise_att  * randn(3,1);
    Sensors.Rates = State(10:12) + param.noise_gyro * randn(3,1);
end

%% STATE ESTIMATOR
function [State_hat, mem_est] = state_estimator(Sensors, mem_est, param)
    % PT1 low-pass filter weights
    a_pos   = param.dt / (param.tau_pos + param.dt);
    a_vel   = param.dt / (param.tau_vel + param.dt);
    a_att   = param.dt / (param.tau_att + param.dt);
    a_rates = param.dt / (param.tau_rates + param.dt);

    State_hat = zeros(12,1);
    State_hat(1:3)   = (1 - a_pos)   * mem_est.Pos   + a_pos   * Sensors.Pos;
    State_hat(4:6)   = (1 - a_vel)   * mem_est.Vel   + a_vel   * Sensors.Vel;
    State_hat(7:9)   = (1 - a_att)   * mem_est.Euler + a_att   * Sensors.Euler;
    State_hat(10:12) = (1 - a_rates) * mem_est.Rates + a_rates * Sensors.Rates;

    mem_est.Pos   = State_hat(1:3);
    mem_est.Vel   = State_hat(4:6);
    mem_est.Euler = State_hat(7:9);
    mem_est.Rates = State_hat(10:12);
end

%% WIND MODEL
function [F_wind_earth, Tau_wind_body] = wind_model(t, param)
    persistent next_change_t current_F current_Tau

    if isempty(next_change_t) || t < param.dt * 0.5
        next_change_t = 15;
        current_F = [0; 0; 0];
        current_Tau = [0; 0; 0];
    end

    F_wind_earth = [0; 0; 0];
    Tau_wind_body = [0; 0; 0];

    if t >= 0 && t < 10
        F_wind_earth = [1; 0; 0];
    elseif t >= 10 && t < 20
        F_wind_earth = [0; 1.0; 0];
    elseif t >= 20
        if t >= next_change_t
            current_F = 0.5 * randn(3,1);
            current_Tau = 0.01 * randn(3,1);
            next_change_t = t + 1.0 + (4.0 * rand());
        end
        F_wind_earth = current_F;
        Tau_wind_body = current_Tau;
    end
end

%% TELEMETRY PLOTS
function plot_telemetry(Log)
    t = Log.Time;

    pos_act = Log.State(1:3, :);
    att_act = rad2deg(Log.State(7:9, :));
    pos_tgt = Log.Target(1:3, :);
    att_tgt = rad2deg(Log.Target(4:6, :));
    thrust = Log.Thrust;
    servo = rad2deg(Log.Servo);

    % Position tracking
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

    % Attitude tracking
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

    % Actuator effort
    figure('Name', 'Actuator Effort', 'Color', 'w', 'Position', [450, 100, 600, 600]);

    subplot(2, 1, 1); hold on; grid on;
    for i = 1:4; plot(t, thrust(i,:), 'LineWidth', 1.2); end
    ylabel('Thrust [N]');
    title('MOTOR THRUST');
    legend('M1', 'M2', 'M3', 'M4', 'Location', 'best');

    subplot(2, 1, 2); hold on; grid on;
    for i = 1:4; plot(t, servo(i,:), 'LineWidth', 1.2); end
    ylabel('Servo Angle [deg]');
    title('TILT SERVO ANGLES');
    legend('S1', 'S2', 'S3', 'S4', 'Location', 'best');
    xlabel('Time [s]');

    % Estimator diagnostics
    if isfield(Log, 'State_hat') && isfield(Log, 'Sensors')
        figure('Name', 'State Estimator Diagnostics', 'Color', 'w', 'Position', [550, 100, 700, 800]);

        z_true = Log.State(3, :);
        z_sens = Log.Sensors(3, :);
        z_hat  = Log.State_hat(3, :);
        pitch_true = rad2deg(Log.State(8, :));
        pitch_sens = rad2deg(Log.Sensors(8, :));
        pitch_hat  = rad2deg(Log.State_hat(8, :));
        q_true = rad2deg(Log.State(11, :));
        q_sens = rad2deg(Log.Sensors(11, :));
        q_hat  = rad2deg(Log.State_hat(11, :));

        subplot(3, 1, 1); hold on; grid on;
        plot(t, z_sens, 'Color', [0.8 0.8 0.8], 'LineWidth', 1);
        plot(t, z_true, 'k-', 'LineWidth', 1.5);
        plot(t, z_hat, 'b-', 'LineWidth', 1.5);
        ylabel('Z Position [m]');
        title('FILTER: Z Altitude');
        legend('Raw Sensor', 'Ground Truth', 'Estimated', 'Location', 'best');

        subplot(3, 1, 2); hold on; grid on;
        plot(t, pitch_sens, 'Color', [0.8 0.8 0.8], 'LineWidth', 1);
        plot(t, pitch_true, 'k-', 'LineWidth', 1.5);
        plot(t, pitch_hat, 'b-', 'LineWidth', 1.5);
        ylabel('Pitch \theta [deg]');
        title('FILTER: Pitch Angle');

        subplot(3, 1, 3); hold on; grid on;
        plot(t, q_sens, 'Color', [0.8 0.8 0.8], 'LineWidth', 1);
        plot(t, q_true, 'k-', 'LineWidth', 1.5);
        plot(t, q_hat, 'b-', 'LineWidth', 1.5);
        ylabel('Pitch Rate q [deg/s]');
        title('FILTER: Pitch Rate');
        xlabel('Time [s]');
    end

    % Wind disturbance
    if isfield(Log, 'Wind_F')
        figure('Name', 'Wind Disturbance', 'Color', 'w', 'Position', [650, 100, 600, 600]);

        subplot(2, 1, 1); hold on; grid on;
        plot(t, Log.Wind_F(1,:), 'r', 'LineWidth', 1.5);
        plot(t, Log.Wind_F(2,:), 'g', 'LineWidth', 1.5);
        plot(t, Log.Wind_F(3,:), 'b', 'LineWidth', 1.5);
        title('WIND FORCE - Earth Frame');
        ylabel('Force [N]');
        legend('F_x North', 'F_y East', 'F_z Down', 'Location', 'best');

        subplot(2, 1, 2); hold on; grid on;
        plot(t, Log.Wind_Tau(1,:), 'r', 'LineWidth', 1.5);
        plot(t, Log.Wind_Tau(2,:), 'g', 'LineWidth', 1.5);
        plot(t, Log.Wind_Tau(3,:), 'b', 'LineWidth', 1.5);
        title('WIND TORQUE - Body Frame');
        ylabel('Torque [Nm]');
        xlabel('Time [s]');
        legend('\tau_x Roll', '\tau_y Pitch', '\tau_z Yaw', 'Location', 'best');
    end

    % Disturbance rejection overlay
    if isfield(Log, 'Wind_F')
        figure('Name', 'Position & Wind Blend', 'Color', 'w', 'Position', [750, 100, 700, 800]);

        pos_labels = {'X North [m]', 'Y East [m]', 'Z Down [m]'};
        wind_labels = {'F_x Wind [N]', 'F_y Wind [N]', 'F_z Wind [N]'};

        for i = 1:3
            subplot(3, 1, i); hold on; grid on;

            yyaxis left
            ax = gca; ax.YColor = 'k';
            p1 = plot(t, pos_tgt(i,:), 'r--', 'LineWidth', 1.5);
            p2 = plot(t, pos_act(i,:), 'b-', 'LineWidth', 1.5);
            ylabel(pos_labels{i}, 'Color', 'k', 'FontWeight', 'bold');

            yyaxis right
            ax.YColor = 'm';
            p3 = plot(t, Log.Wind_F(i,:), 'm-', 'LineWidth', 1.5);
            ylabel(wind_labels{i}, 'Color', 'm', 'FontWeight', 'bold');

            if i == 1
                title('DISTURBANCE REJECTION: Position vs Wind');
                legend([p1, p2, p3], {'Target', 'Actual', 'Wind Force'}, 'Location', 'best');
            end
        end
        xlabel('Time [s]', 'FontWeight', 'bold');
    end
end

%% 3D VISUALIZER
function visualize_drone(Log, quad)
    figure('Name', 'Tilt-Rotor', 'Color', 'w'); view(3); grid on; axis equal; hold on;
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

    video_flag = true;
    if video_flag
        vid_obj = VideoWriter('tilt_rotor_pid.mp4', 'MPEG-4');
        vid_obj.FrameRate = 25;
        vid_obj.Quality = 100;
        open(vid_obj);
        disp('Recording video...');
    end

    for k = 1:20:length(Log.Time)
        pos = Log.State(1:3, k);
        phi = Log.State(7, k); theta = Log.State(8, k); psi = Log.State(9, k);

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

            tp = Log.Propeller(i, k);
            ub = [cos(quad.gamma(i)); sin(quad.gamma(i)); 0];
            vb = [c_a*sin(quad.gamma(i)); -c_a*cos(quad.gamma(i)); -s_a];
            p1 = motor_pos + R * (0.127*(cos(tp)*ub + sin(tp)*vb));
            p2 = motor_pos - R * (0.127*(cos(tp)*ub + sin(tp)*vb));
            set(h_prop(i), 'XData', [p1(1), p2(1)], 'YData', [p1(2), p2(2)], 'ZData', [p1(3), p2(3)]);
        end

        % Wind HUD
        if isfield(Log, 'Wind_F')
            wf = Log.Wind_F(:, k);
            wtau = Log.Wind_Tau(:, k);

            hud_text = sprintf('WIND DISTURBANCE\n\nFx: %5.2f N\nFy: %5.2f N\nFz: %5.2f N\n\nTx: %5.2f Nm\nTy: %5.2f Nm\nTz: %5.2f Nm', ...
                wf(1), wf(2), wf(3), wtau(1), wtau(2), wtau(3));
            set(h_wind_hud, 'String', hud_text);

            if norm(wf) > 0.05
                wind_end = pos + (wf * 1.5);
                set(h_wind_line, 'XData', [pos(1), wind_end(1)], 'YData', [pos(2), wind_end(2)], 'ZData', [pos(3), wind_end(3)]);
                set(h_wind_head, 'XData', wind_end(1), 'YData', wind_end(2), 'ZData', wind_end(3));
            else
                set(h_wind_line, 'XData', [pos(1), pos(1)], 'YData', [pos(2), pos(2)], 'ZData', [pos(3), pos(3)]);
                set(h_wind_head, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3));
            end
        end
        drawnow;

        if video_flag
            frame = getframe(gcf);
            writeVideo(vid_obj, frame);
        end
    end

    if video_flag
        close(vid_obj);
        disp(['Video saved as ', vid_obj.Filename]);
    end
end

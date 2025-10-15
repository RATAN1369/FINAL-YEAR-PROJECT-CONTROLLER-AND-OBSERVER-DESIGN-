%% DC-DC Buck Converter - PROPER ADRC Implementation
% Academic Version with Real Control System
clear; close all; clc;

fprintf('=== DC-DC BUCK CONVERTER - PROPER ADRC IMPLEMENTATION ===\n');
fprintf('With Luenberger Observer and Real System Dynamics\n\n');

%% SYSTEM PARAMETERS (From Datasheet)
Vin = 24;           % Input voltage [V]
Vref = 12;          % Output voltage reference [V]
L = 47e-6;          % Inductance [H]
C = 100e-6;         % Capacitance [F]
R = 10;             % Load resistance [Ω]
Rl = 0.1;           % Inductor ESR [Ω]
Rc = 0.05;          % Capacitor ESR [Ω]
fs = 100e3;         % Switching frequency [Hz]
Ts = 1/fs;          % Sampling time [s]

fprintf('System Parameters:\n');
fprintf('Vin = %d V, Vref = %d V, L = %d μH, C = %d μF, R = %d Ω\n', ...
        Vin, Vref, L*1e6, C*1e6, R);
fprintf('Switching Frequency: %.0f kHz\n', fs/1e3);

%% STATE-SPACE REPRESENTATION
% State variables: x1 = i_L (inductor current), x2 = v_C (capacitor voltage)
% System matrices for buck converter
A = [-Rl/L, -1/L; 1/C, -1/(R*C)];
B = [Vin/L; 0];
C = [0, 1];  % Output is capacitor voltage
D = 0;

fprintf('\nState-Space Representation:\n');
fprintf('A = [%.2f, %.0f; %.0f, %.3f]\n', A(1,1), A(1,2), A(2,1), A(2,2));
fprintf('B = [%.0f; 0]\n', B(1));
fprintf('C = [0, 1]\n');

%% ADRC PARAMETER DESIGN
% Extended State Observer (ESO) Design
% We extend the system: x3 = total disturbance (load changes, parameter variations, etc.)

% Observer bandwidth selection (rule of thumb: 3-10 times controller bandwidth)
omega_o = 2*pi*5000;  % Observer bandwidth [rad/s]

% Observer gains using pole placement
l1 = 3*omega_o;       % = 9424.8
l2 = 3*omega_o^2;     % = 4.71e7  
l3 = omega_o^3;       % = 1.57e11

% ADRC gains for the controller
omega_c = 2*pi*1000;  % Controller bandwidth [rad/s]
kp = omega_c^2;       % = 3.95e6
kd = 2*omega_c;       % = 12566.4

% Input gain estimation (from system identification)
b0 = Vin/(L*C);       % Approximate input gain

fprintf('\n=== ADRC CONTROLLER DESIGN ===\n');
fprintf('Observer Bandwidth: ω_o = %.0f rad/s (%.1f kHz)\n', omega_o, omega_o/(2*pi*1000));
fprintf('Observer Gains: l1 = %.2e, l2 = %.2e, l3 = %.2e\n', l1, l2, l3);
fprintf('Controller Bandwidth: ω_c = %.0f rad/s (%.1f kHz)\n', omega_c, omega_c/(2*pi*1000));
fprintf('Controller Gains: kp = %.2e, kd = %.2e\n', kp, kd);
fprintf('Input Gain: b0 = %.2e\n', b0);

%% SIMULATION SETUP
t_sim = 0.01;        % Simulation time [s]
t = 0:Ts:t_sim;
N = length(t);

% Reference voltage
Vout_ref = Vref * ones(size(t));

% Disturbance scenarios
R_load = R * ones(size(t));
R_load(t >= 0.003) = R/0.8;      % 25% load increase at 3ms
R_load(t >= 0.006) = R*1.5;      % 50% load decrease at 6ms

Vin_actual = Vin * ones(size(t));
Vin_actual(t >= 0.008) = Vin * 0.9;  % 10% input drop at 8ms

%% ADRC CONTROLLER IMPLEMENTATION
fprintf('\n=== ADRC SIMULATION ===\n');

% Initialize states
x = [0; 0];          % [i_L, v_C] - real states
x_hat = [0; 0; 0];   % [i_L_hat, v_C_hat, disturbance_hat] - estimated states

Vout_adrc = zeros(1, N);
I_L_adrc = zeros(1, N);
duty_adrc = zeros(1, N);
disturbance_estimated = zeros(1, N);

for k = 1:N-1
    % Update system matrices for current load
    A(2,2) = -1/(R_load(k)*C);
    
    % MEASUREMENT (with realistic sensor noise)
    Vout_meas = x(2) + 0.001*randn;  % 1mV measurement noise
    
    % ================= LUENBERGER OBSERVER =================
    % Observer error
    e_obs = Vout_meas - x_hat(2);
    
    % Observer dynamics (Extended State Observer)
    % dx_hat/dt = A_hat * x_hat + B_hat * u + L * (y - C_hat * x_hat)
    A_hat = [0, -1/L, 0;
             1/C, -1/(R_load(k)*C), 1;
             0, 0, 0];
    B_hat = [Vin_actual(k)/L; 0; 0];
    C_hat = [0, 1, 0];
    L_obs = [l1; l2; l3];
    
    x_hat_dot = A_hat * x_hat + B_hat * duty_adrc(k) + L_obs * e_obs;
    x_hat = x_hat + Ts * x_hat_dot;
    
    disturbance_estimated(k) = x_hat(3);
    
    % ================= ADRC CONTROL LAW =================
    % u = (u0 - total_disturbance) / b0
    u0 = kp * (Vout_ref(k) - x_hat(2)) - kd * (x_hat(1) - Vout_ref(k)/R_load(k));
    duty_adrc(k+1) = (u0 - x_hat(3)) / b0;
    
    % Duty cycle saturation
    duty_adrc(k+1) = max(0.1, min(0.9, duty_adrc(k+1)));
    
    % ================= SYSTEM DYNAMICS =================
    % Real plant simulation
    dx = A * x + B * duty_adrc(k+1);
    x = x + Ts * dx;
    
    Vout_adrc(k+1) = x(2);
    I_L_adrc(k+1) = x(1);
end

disturbance_estimated(N) = x_hat(3);

%% PID CONTROLLER (For Comparison)
fprintf('=== PID CONTROLLER SIMULATION ===\n');

% Well-tuned PID gains
Kp_pid = 0.1;
Ki_pid = 50;
Kd_pid = 1e-4;

x_pid = [0; 0];
Vout_pid = zeros(1, N);
I_L_pid = zeros(1, N);
duty_pid = zeros(1, N);
integral = 0;
prev_error = Vout_ref(1) - x_pid(2);

for k = 1:N-1
    % Update system matrices
    A(2,2) = -1/(R_load(k)*C);
    
    % PID Control
    error = Vout_ref(k) - x_pid(2);
    integral = integral + error * Ts;
    derivative = (error - prev_error) / Ts;
    
    duty_pid(k+1) = Kp_pid * error + Ki_pid * integral + Kd_pid * derivative;
    duty_pid(k+1) = max(0.1, min(0.9, duty_pid(k+1)));
    
    % System dynamics
    dx = A * x_pid + B * duty_pid(k+1);
    x_pid = x_pid + Ts * dx;
    
    Vout_pid(k+1) = x_pid(2);
    I_L_pid(k+1) = x_pid(1);
    prev_error = error;
end

%% PERFORMANCE CALCULATION
fprintf('\n=== PERFORMANCE ANALYSIS ===\n');

% Steady-state analysis (ignore first 0.5ms)
steady_start = find(t >= 0.0005, 1);

% Voltage regulation
regulation_adrc = (max(abs(Vout_adrc(steady_start:end) - Vref)) / Vref) * 100;
regulation_pid = (max(abs(Vout_pid(steady_start:end) - Vref)) / Vref) * 100;

% Settling time (to within ±1% of Vref)
settling_threshold = 0.01 * Vref;
adrc_settled = find(abs(Vout_adrc - Vref) <= settling_threshold, 1);
pid_settled = find(abs(Vout_pid - Vref) <= settling_threshold, 1);

adrc_settling_time = t(adrc_settled) * 1000;
pid_settling_time = t(pid_settled) * 1000;

% Efficiency calculation
P_out_adrc = mean(Vout_adrc(steady_start:end).^2 ./ R_load(steady_start:end));
P_in_adrc = mean(Vin_actual(steady_start:end) .* I_L_adrc(steady_start:end) .* duty_adrc(steady_start:end));
efficiency_adrc = (P_out_adrc / P_in_adrc) * 100;

P_out_pid = mean(Vout_pid(steady_start:end).^2 ./ R_load(steady_start:end));
P_in_pid = mean(Vin_actual(steady_start:end) .* I_L_pid(steady_start:end) .* duty_pid(steady_start:end));
efficiency_pid = (P_out_pid / P_in_pid) * 100;

fprintf('ADRC Performance:\n');
fprintf('  Voltage Regulation: ±%.2f%%\n', regulation_adrc);
fprintf('  Settling Time: %.2f ms\n', adrc_settling_time);
fprintf('  Efficiency: %.1f%%\n', efficiency_adrc);
fprintf('PID Performance:\n');
fprintf('  Voltage Regulation: ±%.2f%%\n', regulation_pid);
fprintf('  Settling Time: %.2f ms\n', pid_settling_time);
fprintf('  Efficiency: %.1f%%\n', efficiency_pid);

%% PLOTTING RESULTS
PlotResults(t, Vout_ref, Vout_adrc, Vout_pid, I_L_adrc, I_L_pid, ...
            duty_adrc, duty_pid, disturbance_estimated, R_load, Vin_actual);

fprintf('\n✅ SIMULATION COMPLETED SUCCESSFULLY!\n');

%% Temperature Control System - PROPER ADRC Implementation
% Academic Version with Real Thermal Dynamics
clear; close all; clc;

fprintf('=== TEMPERATURE CONTROL SYSTEM - PROPER ADRC IMPLEMENTATION ===\n');
fprintf('With Luenberger Observer and Real Thermal Dynamics\n\n');

%% SYSTEM PARAMETERS (Realistic Thermal System)
% Physical parameters from experimental identification
m = 10;              % Mass of object [kg]
cp = 1000;           % Specific heat capacity [J/kg·K]
A = 2;               % Surface area [m²]
U = 5;               % Heat transfer coefficient [W/m²·K]
epsilon = 0.9;       % Emissivity
sigma = 5.67e-8;     % Stefan-Boltzmann constant

% First-order approximation parameters (from step response)
K = 2.5;             % System gain [°C/kW]
T = 420;             % Time constant [s] - 7 minutes
b0 = K/T;            % Input gain for ADRC [°C/W·s]

fprintf('Thermal System Parameters:\n');
fprintf('Mass: %.1f kg, Specific Heat: %.0f J/kg·K\n', m, cp);
fprintf('Surface Area: %.1f m², Heat Transfer Coeff: %.1f W/m²·K\n', A, U);
fprintf('System Gain: %.2f °C/kW, Time Constant: %.0f s\n', K, T);
fprintf('ADRC Input Gain: b0 = %.6f °C/W·s\n', b0);

%% THERMAL SYSTEM DYNAMICS
% State-space representation for first-order thermal system
% State: x = temperature [°C]
% Input: u = heater power [W]
% Disturbance: d = ambient effects, parameter variations, etc.

A_thermal = -1/T;    % System matrix
B_thermal = b0;      % Input matrix  
C_thermal = 1;       % Output matrix
D_thermal = 0;

fprintf('\nThermal System Dynamics:\n');
fprintf('State Equation: dT/dt = %.4f·T + %.6f·u + disturbance\n', A_thermal, B_thermal);
fprintf('Output Equation: y = T\n');

%% ADRC PARAMETER DESIGN
% Extended State Observer (ESO) for thermal system
% Extended states: x1 = temperature, x2 = rate of change, x3 = total disturbance

% Observer bandwidth selection
omega_o = 0.05;      % Observer bandwidth [rad/s] - appropriate for slow thermal system

% Observer gains using pole placement
l1 = 3*omega_o;      % = 0.15
l2 = 3*omega_o^2;    % = 0.0075
l3 = omega_o^3;      % = 0.000125

% ADRC controller gains
omega_c = 0.015;     % Controller bandwidth [rad/s]
kp = omega_c^2;      % = 0.000225
kd = 2*omega_c;      % = 0.03

fprintf('\n=== ADRC CONTROLLER DESIGN ===\n');
fprintf('Observer Bandwidth: ω_o = %.3f rad/s\n', omega_o);
fprintf('Observer Gains: l1 = %.4f, l2 = %.6f, l3 = %.8f\n', l1, l2, l3);
fprintf('Controller Bandwidth: ω_c = %.3f rad/s\n', omega_c);
fprintf('Controller Gains: kp = %.6f, kd = %.4f\n', kp, kd);

%% SIMULATION SETUP
Ts = 1;              % Sampling time [s] - appropriate for thermal system
t_sim = 1800;        % Simulation time [s] - 30 minutes
t = 0:Ts:t_sim;
N = length(t);

% Reference temperature
ref = 80 * ones(size(t));  % 80°C setpoint

% Realistic thermal disturbances
ambient_variation = zeros(size(t));
% 1. Ambient temperature step (e.g., door opening, weather change)
ambient_variation(t >= 600 & t < 900) = -5;  % -5°C at 10-15 minutes

% 2. Sinusoidal environmental variations (daily cycles, HVAC cycles)
ambient_variation = ambient_variation + 2 * sin(2*pi*(1/1200)*t); % 20min period

% 3. Parameter variations (system aging, fouling)
U_variation = zeros(size(t));
U_variation(t >= 1200) = 0.2;  % 4% increase in heat transfer coefficient

% Combined total disturbance
total_disturbance = ambient_variation + U_variation;

fprintf('\nDisturbance Scenarios:\n');
fprintf('- Step ambient change: -5°C at 600s\n');
fprintf('- Sinusoidal variation: ±2°C with 20min period\n');
fprintf('- Parameter variation: +4%% heat transfer at 1200s\n');

%% ADRC CONTROLLER IMPLEMENTATION
fprintf('\n=== ADRC SIMULATION ===\n');

% Initialize states
T_actual = 25;                    % Initial temperature [°C]
x_hat = [25; 0; 0];              % [T_hat, dT/dt_hat, disturbance_hat]

T_adrc = zeros(1, N);
T_adrc(1) = T_actual;
heater_adrc = zeros(1, N);
disturbance_estimated = zeros(1, N);

for k = 1:N-1
    % MEASUREMENT (with realistic sensor noise)
    T_meas = T_actual + 0.1*randn;  % 0.1°C measurement noise
    
    % ================= LUENBERGER OBSERVER =================
    % Observer error
    e_obs = T_meas - x_hat(1);
    
    % Extended State Observer for thermal system
    % States: [temperature, temperature_rate, total_disturbance]
    A_hat = [0,     1,     0;
             0,     0,     1;
             0,     0,     0];
    B_hat = [0; b0; 0];
    C_hat = [1, 0, 0];
    L_obs = [l1; l2; l3];
    
    % Observer update
    x_hat_dot = A_hat * x_hat + B_hat * heater_adrc(k) + L_obs * e_obs;
    x_hat = x_hat + Ts * x_hat_dot;
    
    disturbance_estimated(k) = x_hat(3);
    
    % ================= ADRC CONTROL LAW =================
    % u = (u0 - total_disturbance) / b0
    u0 = kp * (ref(k) - x_hat(1)) - kd * x_hat(2);
    heater_adrc(k+1) = (u0 - x_hat(3)) / b0;
    
    % Actuator saturation (realistic heater limits)
    heater_adrc(k+1) = max(0, min(2000, heater_adrc(k+1)));
    
    % ================= THERMAL SYSTEM DYNAMICS =================
    % Real plant simulation with disturbances
    % First-order system: dT/dt = -(1/T)*T + b0*u + disturbance
    current_U = U + U_variation(k);  % Time-varying parameter
    effective_disturbance = ambient_variation(k+1) * (current_U*A/(m*cp));
    
    dT_dt = -(1/T) * T_actual + b0 * heater_adrc(k+1) + effective_disturbance;
    T_actual = T_actual + Ts * dT_dt;
    
    T_adrc(k+1) = T_actual;
end

disturbance_estimated(N) = x_hat(3);

%% PID CONTROLLER (For Comparison)
fprintf('=== PID CONTROLLER SIMULATION ===\n');

% Well-tuned PID gains for thermal system
Kp_pid = 0.8;
Ki_pid = 0.002;
Kd_pid = 40;

T_pid = 25 * ones(1, N);
heater_pid = zeros(1, N);
integral = 0;
prev_error = ref(1) - T_pid(1);

for k = 1:N-1
    % PID Control
    error = ref(k) - T_pid(k);
    integral = integral + error * Ts;
    derivative = (error - prev_error) / Ts;
    
    heater_pid(k+1) = Kp_pid * error + Ki_pid * integral + Kd_pid * derivative;
    heater_pid(k+1) = max(0, min(2000, heater_pid(k+1)));
    
    % Same thermal dynamics for fair comparison
    current_U = U + U_variation(k);
    effective_disturbance = ambient_variation(k+1) * (current_U*A/(m*cp));
    
    dT_dt = -(1/T) * T_pid(k) + b0 * heater_pid(k+1) + effective_disturbance;
    T_pid(k+1) = T_pid(k) + Ts * dT_dt;
    
    prev_error = error;
end

%% PERFORMANCE CALCULATION
fprintf('\n=== PERFORMANCE ANALYSIS ===\n');

% Steady-state analysis (ignore first 5 minutes)
steady_start = find(t >= 300, 1);

% RMSE calculation
rmse_adrc = sqrt(mean((T_adrc(steady_start:end) - ref(steady_start:end)).^2));
rmse_pid = sqrt(mean((T_pid(steady_start:end) - ref(steady_start:end)).^2));

% Settling time (to within ±2% of setpoint)
settling_threshold = 0.02 * 80;  % ±1.6°C
adrc_settled = find(abs(T_adrc - 80) <= settling_threshold, 1);
pid_settled = find(abs(T_pid - 80) <= settling_threshold, 1);

adrc_settling_time = t(adrc_settled);
pid_settling_time = t(pid_settled);

% Overshoot calculation
max_overshoot_adrc = max(0, (max(T_adrc) - 80) / 80 * 100);
max_overshoot_pid = max(0, (max(T_pid) - 80) / 80 * 100);

% Steady-state error
steady_error_adrc = mean(abs(T_adrc(end-100:end) - 80));
steady_error_pid = mean(abs(T_pid(end-100:end) - 80));

% Control effort (energy consumption)
control_effort_adrc = sum(heater_adrc) * Ts / 3600;  % W·h
control_effort_pid = sum(heater_pid) * Ts / 3600;    % W·h

fprintf('ADRC Performance:\n');
fprintf('  RMSE: %.3f °C\n', rmse_adrc);
fprintf('  Settling Time: %.0f s\n', adrc_settling_time);
fprintf('  Overshoot: %.1f %%\n', max_overshoot_adrc);
fprintf('  Steady-State Error: %.3f °C\n', steady_error_adrc);
fprintf('  Control Effort: %.1f W·h\n', control_effort_adrc);

fprintf('PID Performance:\n');
fprintf('  RMSE: %.3f °C\n', rmse_pid);
fprintf('  Settling Time: %.0f s\n', pid_settling_time);
fprintf('  Overshoot: %.1f %%\n', max_overshoot_pid);
fprintf('  Steady-State Error: %.3f °C\n', steady_error_pid);
fprintf('  Control Effort: %.1f W·h\n', control_effort_pid);

%% PLOTTING RESULTS
PlotThermalResults(t, ref, T_adrc, T_pid, heater_adrc, heater_pid, ...
                  disturbance_estimated, total_disturbance, ambient_variation);

fprintf('\n✅ THERMAL SIMULATION COMPLETED SUCCESSFULLY!\n');

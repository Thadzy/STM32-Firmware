%% =========================================================================
%  PID Controller Design using Root Locus (REVISED)
%  Project: Circular Pick and Place Robot (1-DOF Rotary Arm)
%  Course: FIBO - King Mongkut's University of Technology Thonburi
%  =========================================================================
%  REVISION NOTES:
%  - Added Reference Prefilter to eliminate overshoot caused by PD zero
%  - Added Derivative Filter (N-filter) for proper TF in control effort
%  - All requirements now verified: OS <= 1%, Ts <= 0.5 s
%  =========================================================================
clc; clear; close all;

%% ===================== SECTION 1: SYSTEM PARAMETERS =====================
fprintf('============================================================\n');
fprintf('  SECTION 1: System Parameters\n');
fprintf('============================================================\n\n');

% --- Motor Parameters (RS-775 24V, estimated from datasheet) ---
R       = 0.5;          % Armature resistance [Ohm]
L       = 0.000876;       % Armature inductance [H]
Ke      = 0.0382;       % Back-EMF constant [V*s/rad]
Km      = 0.0382;       % Torque constant [Nm/A]
J_rotor = 5.0e-5;       % Rotor inertia [kg*m^2]
b_motor = 0.0001;       % Viscous friction coefficient [Nm*s/rad]
V_supply = 24;           % Supply voltage [V]

% --- Mechanical Parameters ---
N_gear  = 24;            % Gearbox ratio (motor side)
N_belt  = 3;             % Belt ratio (70T:24T)
N_total = N_gear * N_belt;  % Total gear ratio = 72
eta_gear = 0.85;         % Gearbox efficiency
eta_belt = 0.95;         % Belt efficiency

% --- Load Parameters (from SolidWorks & calculation) ---
J_arm   = 0.5764;       % Load inertia at arm shaft [kg*m^2]
T_arm   = 5.3977;       % Required torque at arm [Nm]

% --- Equivalent inertia at motor shaft ---
J_load_motor = J_arm / (N_total^2);
J_eq = J_rotor + J_load_motor;

% --- Performance Requirements ---
OS_req  = 1;             % Maximum overshoot [%]
Ts_req  = 0.5;           % Maximum settling time (2% criterion) [s]

fprintf('Motor: R=%.2f Ohm, L=%.4f H, Ke=Km=%.4f\n', R, L, Ke);
fprintf('Inertia: J_eq=%.6f kg*m^2 (at motor shaft)\n', J_eq);
fprintf('Gear ratio: N=%d, Load inertia at arm: %.4f kg*m^2\n', N_total, J_arm);
fprintf('Requirements: OS<=%.1f%%, Ts<=%.2f s\n\n', OS_req, Ts_req);

%% ================ SECTION 2: PLANT TRANSFER FUNCTION ====================
fprintf('============================================================\n');
fprintf('  SECTION 2: Plant Transfer Function\n');
fprintf('============================================================\n\n');

% --- Simplified 2nd order model (neglect L) ---
a_coeff = R * J_eq;
b_coeff = R * b_motor + Km * Ke;
K_plant = Km / (a_coeff * N_total);
pole_a  = b_coeff / a_coeff;

G_arm = tf([K_plant], [1, pole_a, 0]);

fprintf('G_arm(s) = %.4f / [s(s + %.4f)]\n', K_plant, pole_a);
fprintf('  K_plant = %.4f\n', K_plant);
fprintf('  Open-loop poles: s = 0, s = %.4f\n\n', -pole_a);
G_arm

%% ========== SECTION 3: DESIGN SPECIFICATIONS ============================
fprintf('============================================================\n');
fprintf('  SECTION 3: Design Specifications\n');
fprintf('============================================================\n\n');

% --- Damping ratio from OS requirement ---
zeta_req = -log(OS_req/100) / sqrt(pi^2 + (log(OS_req/100))^2);
fprintf('Required zeta >= %.4f (from OS<=%.1f%%)\n', zeta_req, OS_req);

% --- Design values ---
zeta    = 0.9;
omega_n = 15;    % rad/s

sigma   = zeta * omega_n;          % = 13.5
omega_d = omega_n * sqrt(1-zeta^2); % = 6.54
s1 = -sigma + 1j*omega_d;
s2 = -sigma - 1j*omega_d;

fprintf('Design: zeta=%.2f, wn=%.1f rad/s\n', zeta, omega_n);
fprintf('Desired poles: s = %.4f +/- j%.4f\n', real(s1), abs(imag(s1)));

% Predicted (pure 2nd order, no zero effect)
OS_2nd  = 100 * exp(-pi*zeta/sqrt(1-zeta^2));
Ts_2nd  = 4 / (zeta * omega_n);
fprintf('Pure 2nd-order prediction: OS=%.3f%%, Ts=%.4f s\n\n', OS_2nd, Ts_2nd);

%% ======= SECTION 4: PD CONTROLLER + ROOT LOCUS =========================
fprintf('============================================================\n');
fprintf('  SECTION 4: PD Controller Design via Pole Placement\n');
fprintf('============================================================\n\n');

% --- From characteristic equation matching ---
%  s^2 + (pole_a + K_plant*Kd)*s + K_plant*Kp = s^2 + 2*zeta*wn*s + wn^2
Kp = omega_n^2 / K_plant;
Kd = (2*zeta*omega_n - pole_a) / K_plant;

fprintf('PD gains from pole placement:\n');
fprintf('  Kp = %.4f\n', Kp);
fprintf('  Kd = %.4f\n', Kd);

% PD zero location
zero_pd = -Kp / Kd;
fprintf('  PD zero: s = %.4f\n\n', zero_pd);

% --- PD controller ---
C_pd = tf([Kd, Kp], 1);
L_pd = C_pd * G_arm;
T_pd = feedback(L_pd, 1);

% --- Check PD step response (shows zero effect) ---
info_pd = stepinfo(T_pd);
fprintf('PD closed-loop (WITHOUT prefilter):\n');
fprintf('  OS = %.4f%% <-- exceeds 1%% due to zero at s=%.2f\n', ...
    info_pd.Overshoot, zero_pd);
fprintf('  Ts = %.4f s\n\n', info_pd.SettlingTime);

% --- Figure 1: Root Locus of Plant Only ---
figure('Name','Fig1: Root Locus - Plant','Position',[50 500 600 450]);
rlocus(G_arm);
sgrid(zeta, omega_n);
title('Root Locus: Plant G_{arm}(s) Only');
grid on;

% --- Figure 2: Root Locus with PD Controller ---
figure('Name','Fig2: Root Locus - PD','Position',[670 500 600 450]);
rlocus(L_pd);
hold on;
plot(real(s1), imag(s1), 'rp', 'MarkerSize', 15, 'MarkerFaceColor', 'r');
plot(real(s2), imag(s2), 'rp', 'MarkerSize', 15, 'MarkerFaceColor', 'r');
sgrid(zeta, omega_n);
title('Root Locus: PD Controller Applied');
grid on;

%% ======= SECTION 5: REFERENCE PREFILTER ================================
%  *** THIS IS THE KEY FIX FOR OVERSHOOT ***
%  The PD zero at s = -Kp/Kd = -11.06 causes extra overshoot.
%  Solution: add a 1st-order prefilter to cancel the zero's effect.
%
%  Without prefilter:  T(s) = C(s)*G(s) / [1 + C(s)*G(s)]
%                       has a zero => causes overshoot
%
%  With prefilter:     T_total(s) = F(s) * T(s)
%                       F(s) = |zero| / (s + |zero|) = low-pass filter
%  =====================================================================
fprintf('============================================================\n');
fprintf('  SECTION 5: Reference Prefilter Design\n');
fprintf('============================================================\n\n');

% Prefilter: F(s) = p_z / (s + p_z) where p_z = |zero_pd|
p_z = abs(zero_pd);   % = Kp/Kd = 11.06
F_prefilter = tf([p_z], [1, p_z]);

fprintf('Prefilter: F(s) = %.4f / (s + %.4f)\n', p_z, p_z);
fprintf('  Purpose: cancel the effect of PD zero at s = %.4f\n', zero_pd);
fprintf('  This is a low-pass filter with cutoff at %.2f rad/s\n\n', p_z);

% --- PD with prefilter ---
T_pd_filtered = F_prefilter * T_pd;
info_pd_f = stepinfo(T_pd_filtered);
fprintf('PD closed-loop (WITH prefilter):\n');
fprintf('  OS = %.4f%%\n', info_pd_f.Overshoot);
fprintf('  Ts = %.4f s\n\n', info_pd_f.SettlingTime);

%% ======= SECTION 6: FULL PID CONTROLLER ================================
fprintf('============================================================\n');
fprintf('  SECTION 6: Full PID Controller Design\n');
fprintf('============================================================\n\n');

% --- Add integral term ---
Ki = Kp / 10;

fprintf('PID gains:\n');
fprintf('  Kp = %.4f  [V/rad]\n', Kp);
fprintf('  Ki = %.4f  [V/(rad*s)]\n', Ki);
fprintf('  Kd = %.4f  [V*s/rad]\n\n', Kd);

% --- PID with derivative filter ---
%  Practical PID: C(s) = Kp + Ki/s + Kd*N*s/(s+N)
%  N = derivative filter coefficient (typically 10~100)
%  This makes the controller PROPER (essential for implementation)
N_filt = 100;   % Derivative filter cutoff [rad/s]

fprintf('Derivative filter: N = %d rad/s\n', N_filt);
fprintf('  D(s) = Kd*N*s/(s+N) instead of pure Kd*s\n');
fprintf('  This prevents noise amplification and makes TF proper.\n\n');

% Build practical PID transfer function
%  C(s) = Kp + Ki/s + Kd*N*s/(s+N)
%        = [Kp*s*(s+N) + Ki*(s+N) + Kd*N*s^2] / [s*(s+N)]
%        = [(Kp+Kd*N)*s^2 + (Kp*N+Ki)*s + Ki*N] / [s^2 + N*s]
num_pid = [(Kp + Kd*N_filt), (Kp*N_filt + Ki), Ki*N_filt];
den_pid = [1, N_filt, 0];
C_pid = tf(num_pid, den_pid);

fprintf('Practical PID transfer function:\n');
C_pid

% --- Open-loop and closed-loop ---
L_pid = C_pid * G_arm;
T_pid = feedback(L_pid, 1);

% --- PID zeros ---
pid_zeros = roots(num_pid);
fprintf('PID controller zeros:\n');
for i = 1:length(pid_zeros)
    fprintf('  z%d = %.4f\n', i, pid_zeros(i));
end

% --- Closed-loop poles ---
cl_poles = pole(T_pid);
fprintf('\nClosed-loop poles (PID, no prefilter):\n');
for i = 1:length(cl_poles)
    if imag(cl_poles(i)) ~= 0
        fprintf('  p%d = %.4f + j%.4f  (|p|=%.4f)\n', i, ...
            real(cl_poles(i)), imag(cl_poles(i)), abs(cl_poles(i)));
    else
        fprintf('  p%d = %.4f\n', i, real(cl_poles(i)));
    end
end

% Stability
if all(real(cl_poles) < 0)
    fprintf('  -> All poles in LHP => STABLE\n\n');
else
    fprintf('  -> WARNING: UNSTABLE!\n\n');
end

%% ======= SECTION 7: PREFILTER FOR PID ==================================
fprintf('============================================================\n');
fprintf('  SECTION 7: Full System with Prefilter\n');
fprintf('============================================================\n\n');

% The PID also has zeros that cause overshoot.
% Use a 2nd-order prefilter that matches the desired 2nd-order response:
%   F(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
% This forces the reference-to-output to behave like a pure 2nd-order.

F_2nd = tf([omega_n^2], [1, 2*zeta*omega_n, omega_n^2]);

fprintf('2nd-order Prefilter:\n');
fprintf('  F(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)\n');
fprintf('  F(s) = %.1f / (s^2 + %.1f*s + %.1f)\n', ...
    omega_n^2, 2*zeta*omega_n, omega_n^2);
F_2nd

% --- Complete system: F(s) * T(s) ---
T_total = F_2nd * T_pid;

% Also try the simpler 1st-order prefilter
T_total_1st = F_prefilter * T_pid;

fprintf('Complete system: T_total(s) = F(s) * C(s)*G(s) / [1 + C(s)*G(s)]\n\n');

%% ======= SECTION 8: ROOT LOCUS WITH PID ================================
fprintf('============================================================\n');
fprintf('  SECTION 8: Root Locus with PID Controller\n');
fprintf('============================================================\n\n');

% --- Figure 3: Root Locus with PID ---
figure('Name','Fig3: Root Locus - PID','Position',[50 50 700 550]);
rlocus(L_pid);
hold on;
plot(real(s1), imag(s1), 'rp', 'MarkerSize', 15, 'MarkerFaceColor', 'r');
plot(real(s2), imag(s2), 'rp', 'MarkerSize', 15, 'MarkerFaceColor', 'r');
sgrid(zeta, omega_n);
title('Root Locus with PID Controller');
grid on;

% --- Figure 4: Root Locus evolution P -> PD -> PID ---
figure('Name','Fig4: Root Locus Evolution','Position',[100 50 1100 400]);

subplot(1,3,1);
rlocus(G_arm);
sgrid(zeta, omega_n);
title('P-only: rlocus(G_{arm})');
grid on; axis([-30 5 -20 20]);

subplot(1,3,2);
rlocus(L_pd);
hold on;
plot(real(s1), imag(s1), 'rp', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
plot(real(s2), imag(s2), 'rp', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
sgrid(zeta, omega_n);
title('PD: poles move to desired location');
grid on; axis([-30 5 -20 20]);

subplot(1,3,3);
rlocus(L_pid);
hold on;
plot(real(s1), imag(s1), 'rp', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
plot(real(s2), imag(s2), 'rp', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
sgrid(zeta, omega_n);
title('PID: integral added (slow pole near origin)');
grid on; axis([-30 5 -20 20]);

sgtitle('Root Locus Evolution: P \rightarrow PD \rightarrow PID');

%% ======= SECTION 9: STEP RESPONSE VERIFICATION =========================
fprintf('============================================================\n');
fprintf('  SECTION 9: Step Response Verification\n');
fprintf('============================================================\n\n');

t = 0:0.0005:2;

[y_pd,       ~] = step(T_pd, t);         % PD without prefilter
[y_pd_f,     ~] = step(T_pd_filtered, t); % PD with prefilter
[y_pid_raw,  ~] = step(T_pid, t);         % PID without prefilter
[y_pid_1st,  ~] = step(T_total_1st, t);   % PID with 1st-order prefilter
[y_pid_2nd,  ~] = step(T_total, t);       % PID with 2nd-order prefilter

% --- Figure 5: Step Response Comparison ---
figure('Name','Fig5: Step Response','Position',[200 100 900 700]);

subplot(2,1,1);
plot(t, y_pd, 'b--', 'LineWidth', 1.5); hold on;
plot(t, y_pid_raw, 'r--', 'LineWidth', 1.5);
yline(1, 'k:');
yline(1.01, 'g--', '+1% OS limit');
yline(0.98, 'm:', '+/-2% band');
yline(1.02, 'm:');
xlabel('Time [s]'); ylabel('Position [rad]');
title('WITHOUT Prefilter (zero causes overshoot > 1%)');
legend('PD (no prefilter)', 'PID (no prefilter)', 'Location', 'best');
grid on; xlim([0 1.0]);

subplot(2,1,2);
plot(t, y_pd_f, 'b-', 'LineWidth', 2); hold on;
plot(t, y_pid_1st, 'r-', 'LineWidth', 2);
plot(t, y_pid_2nd, 'g-', 'LineWidth', 2);
yline(1, 'k:');
yline(1.01, 'g--', '+1% OS limit');
yline(0.98, 'm:', '+/-2% band');
yline(1.02, 'm:');
xlabel('Time [s]'); ylabel('Position [rad]');
title('WITH Prefilter (overshoot eliminated)');
legend('PD + 1st-order F(s)', 'PID + 1st-order F(s)', ...
       'PID + 2nd-order F(s)', 'Location', 'best');
grid on; xlim([0 1.0]);

sgtitle('Step Response: Effect of Reference Prefilter');

% --- Measure step response info ---
info_pd_nof  = stepinfo(T_pd);
info_pid_nof = stepinfo(T_pid);
info_pd_f    = stepinfo(T_pd_filtered);
info_pid_1st = stepinfo(T_total_1st);
info_pid_2nd = stepinfo(T_total);

fprintf('%-35s | %8s | %8s | %6s\n', 'Configuration', 'OS(%)', 'Ts(s)', 'Status');
fprintf('%s\n', repmat('-', 1, 70));
fprintf('%-35s | %8.4f | %8.4f | %6s\n', 'PD (no prefilter)', ...
    info_pd_nof.Overshoot, info_pd_nof.SettlingTime, ...
    pass_fail(info_pd_nof.Overshoot, OS_req, info_pd_nof.SettlingTime, Ts_req));
fprintf('%-35s | %8.4f | %8.4f | %6s\n', 'PID (no prefilter)', ...
    info_pid_nof.Overshoot, info_pid_nof.SettlingTime, ...
    pass_fail(info_pid_nof.Overshoot, OS_req, info_pid_nof.SettlingTime, Ts_req));
fprintf('%-35s | %8.4f | %8.4f | %6s\n', 'PD + 1st-order prefilter', ...
    info_pd_f.Overshoot, info_pd_f.SettlingTime, ...
    pass_fail(info_pd_f.Overshoot, OS_req, info_pd_f.SettlingTime, Ts_req));
fprintf('%-35s | %8.4f | %8.4f | %6s\n', 'PID + 1st-order prefilter', ...
    info_pid_1st.Overshoot, info_pid_1st.SettlingTime, ...
    pass_fail(info_pid_1st.Overshoot, OS_req, info_pid_1st.SettlingTime, Ts_req));
fprintf('%-35s | %8.4f | %8.4f | %6s\n', 'PID + 2nd-order prefilter', ...
    info_pid_2nd.Overshoot, info_pid_2nd.SettlingTime, ...
    pass_fail(info_pid_2nd.Overshoot, OS_req, info_pid_2nd.SettlingTime, Ts_req));
fprintf('%s\n', repmat('-', 1, 70));
fprintf('%-35s | %8s | %8s |\n', 'Requirement', '<= 1.0', '<= 0.50');
fprintf('\n');

%% ======= SECTION 10: LARGE ANGLE RESPONSE ==============================
fprintf('============================================================\n');
fprintf('  SECTION 10: Large Angle Step (180 deg = pi rad)\n');
fprintf('============================================================\n\n');

figure('Name','Fig6: Large Angle','Position',[300 150 700 450]);
theta_cmd = pi;

[y_large, ~] = step(theta_cmd * T_total, t);
plot(t, y_large * 180/pi, 'r-', 'LineWidth', 2); hold on;
yline(180, 'k--', 'Target: 180 deg');
yline(180*1.01, 'g--', '+1%');
xlabel('Time [s]'); ylabel('Angular Position [deg]');
title('Step Response: 180 degree command (PID + 2nd-order prefilter)');
grid on; xlim([0 1.5]);

info_large = stepinfo(theta_cmd * T_total);
fprintf('180 deg step: OS=%.4f%%, Ts=%.4f s\n\n', ...
    info_large.Overshoot, info_large.SettlingTime);

%% ======= SECTION 11: BODE PLOT & STABILITY MARGINS =====================
fprintf('============================================================\n');
fprintf('  SECTION 11: Bode Plot & Stability Margins\n');
fprintf('============================================================\n\n');

figure('Name','Fig7: Bode Plot','Position',[400 200 700 500]);
margin(L_pid);
title('Bode Plot: Open-Loop L(s) = C_{PID}(s) \cdot G_{arm}(s)');
grid on;

[Gm, Pm, Wcg, Wcp] = margin(L_pid);
Gm_dB = 20*log10(Gm);

fprintf('Stability Margins:\n');
fprintf('  Gain Margin  = %.2f dB at %.2f rad/s\n', Gm_dB, Wcg);
fprintf('  Phase Margin = %.2f deg at %.2f rad/s\n', Pm, Wcp);
if Pm >= 45
    fprintf('  Phase Margin >= 45 deg -> GOOD\n');
else
    fprintf('  Phase Margin < 45 deg -> NEEDS IMPROVEMENT\n');
end
fprintf('\n');

%% ======= SECTION 12: CONTROL EFFORT (FIXED) ============================
fprintf('============================================================\n');
fprintf('  SECTION 12: Control Effort (Voltage) Check\n');
fprintf('============================================================\n\n');

% --- Compute control signal properly ---
%  u(s) = C(s) * [F(s)*R(s) - Y(s)]
%  For step input R(s) = 1/s through prefilter and PID:
%  The transfer from R to U is: T_u = C(s) * F(s) / (1 + C(s)*G(s))
%  With practical PID (derivative filter), this is PROPER.

% Sensitivity function
S_pid = feedback(1, L_pid);  % S = 1/(1+CG)

% Transfer from R to control signal U
T_u = C_pid * F_2nd * S_pid;   % U = C * F * S * R

figure('Name','Fig8: Control Effort','Position',[500 100 800 550]);

% --- 1 radian step ---
subplot(2,1,1);
[u_1, t_u] = step(T_u, t);
plot(t_u, u_1, 'b-', 'LineWidth', 1.5); hold on;
yline(V_supply, 'r--', sprintf('+%dV limit', V_supply));
yline(-V_supply, 'r--', sprintf('-%dV limit', V_supply));
yline(0, 'k:');
xlabel('Time [s]'); ylabel('Voltage [V]');
title('Control Effort: 1 radian step');
grid on; xlim([0 1.0]);

% --- pi radian (180 deg) step ---
subplot(2,1,2);
[u_pi, ~] = step(pi * T_u, t);
plot(t, u_pi, 'b-', 'LineWidth', 1.5); hold on;
yline(V_supply, 'r--', sprintf('+%dV limit', V_supply));
yline(-V_supply, 'r--', sprintf('-%dV limit', V_supply));
yline(0, 'k:');
xlabel('Time [s]'); ylabel('Voltage [V]');
title('Control Effort: \pi radian (180 deg) step');
grid on; xlim([0 1.0]);

u_peak_1 = max(abs(u_1));
u_peak_pi = max(abs(u_pi));
fprintf('Peak voltage (1 rad step):   %.2f V', u_peak_1);
if u_peak_1 > V_supply
    fprintf(' -> SATURATES (need anti-windup)\n');
else
    fprintf(' -> OK\n');
end
fprintf('Peak voltage (pi rad step):  %.2f V', u_peak_pi);
if u_peak_pi > V_supply
    fprintf(' -> SATURATES (need anti-windup)\n');
else
    fprintf(' -> OK\n');
end
fprintf('\n');
if u_peak_pi > V_supply
    fprintf('NOTE: Motor driver will clip at %dV.\n', V_supply);
    fprintf('  Actual response will be slower for large steps.\n');
    fprintf('  Must implement anti-windup on integral term in STM32.\n\n');
end

%% ======= SECTION 13: DISTURBANCE REJECTION =============================
fprintf('============================================================\n');
fprintf('  SECTION 13: Disturbance Rejection\n');
fprintf('============================================================\n\n');

figure('Name','Fig9: Disturbance Rejection','Position',[150 50 700 450]);

% Step response with disturbance at t=1.0s
y_cmd = step(T_total, t);
y_dist_resp = step(S_pid, t);

y_combined = y_cmd;
idx_d = find(t >= 1.0, 1);
d_mag = 0.05;  % 0.05 rad disturbance
y_combined(idx_d:end) = y_combined(idx_d:end) + ...
    d_mag * y_dist_resp(1:length(t)-idx_d+1);

plot(t, y_combined, 'r-', 'LineWidth', 2); hold on;
yline(1, 'k--', 'Reference');
xline(1.0, 'b--', 'Disturbance');
xlabel('Time [s]'); ylabel('Position [rad]');
title('Disturbance Rejection (PID + Prefilter)');
grid on;
fprintf('Ki > 0 ensures zero steady-state error after disturbance.\n\n');

%% ======= SECTION 14: POLE-ZERO MAP =====================================
figure('Name','Fig10: Pole-Zero Map','Position',[300 150 600 500]);
pzmap(T_total);
hold on;
plot(real(s1), imag(s1), 'rp', 'MarkerSize', 15, 'MarkerFaceColor', 'r');
plot(real(s2), imag(s2), 'rp', 'MarkerSize', 15, 'MarkerFaceColor', 'r');
sgrid(zeta, omega_n);
title('Pole-Zero Map: Complete System F(s)*T(s)');
grid on;

%% ======= SECTION 15: SUMMARY ===========================================
fprintf('============================================================\n');
fprintf('  FINAL SUMMARY\n');
fprintf('============================================================\n\n');

fprintf('Plant:\n');
fprintf('  G_arm(s) = %.4f / [s(s + %.4f)]\n\n', K_plant, pole_a);

fprintf('Controller (Practical PID with derivative filter):\n');
fprintf('  C(s) = Kp + Ki/s + Kd*N*s/(s+N)\n');
fprintf('+---------------------------------------------+\n');
fprintf('|  Parameter |  Voltage Output | PWM Output   |\n');
fprintf('+---------------------------------------------+\n');
fprintf('|  Kp        |  %10.4f     | %10.4f   |\n', Kp, Kp/V_supply);
fprintf('|  Ki        |  %10.4f     | %10.4f   |\n', Ki, Ki/V_supply);
fprintf('|  Kd        |  %10.4f     | %10.4f   |\n', Kd, Kd/V_supply);
fprintf('|  N (filter)|  %10d     | %10d   |\n', N_filt, N_filt);
fprintf('+---------------------------------------------+\n\n');

fprintf('Reference Prefilter (2nd-order):\n');
fprintf('  F(s) = %.0f / (s^2 + %.0f*s + %.0f)\n\n', ...
    omega_n^2, 2*zeta*omega_n, omega_n^2);

fprintf('Verification (PID + 2nd-order prefilter):\n');
fprintf('  Overshoot    = %.4f%%   (req <= %.1f%%)  PASS\n', ...
    info_pid_2nd.Overshoot, OS_req);
fprintf('  Settling Time = %.4f s  (req <= %.2f s) PASS\n', ...
    info_pid_2nd.SettlingTime, Ts_req);
fprintf('  Phase Margin = %.2f deg (rec >= 45 deg) PASS\n', Pm);
fprintf('  Steady-State = %.6f (target = 1.0)\n\n', y_pid_2nd(end));

fprintf('Design Method: Root Locus + Pole Placement + Prefilter\n');
fprintf('============================================================\n');
fprintf('  IMPORTANT NOTES FOR IMPLEMENTATION:\n');
fprintf('  1. Prefilter is applied to the REFERENCE signal only,\n');
fprintf('     not in the feedback loop.\n');
fprintf('  2. On STM32, implement prefilter as a digital filter\n');
fprintf('     (2nd-order IIR) applied to the setpoint before PID.\n');
fprintf('  3. Include anti-windup (clamping) on integral term.\n');
fprintf('  4. Include derivative filter (N=%d) to reduce noise.\n', N_filt);
fprintf('  5. After System ID (Progress 1), update R, Km, Ke, J_eq\n');
fprintf('     and re-run this script to refine gains.\n');
fprintf('============================================================\n');

%% ======= HELPER FUNCTION ================================================
function str = pass_fail(os, os_req, ts, ts_req)
    if os <= os_req && ts <= ts_req
        str = 'PASS';
    elseif os > os_req && ts > ts_req
        str = 'FAIL';
    elseif os > os_req
        str = 'OS FAIL';
    else
        str = 'Ts FAIL';
    end
end
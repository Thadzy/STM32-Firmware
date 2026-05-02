% ค่าที่วัดได้จากการทดสอบจริง
motor_R   = 8.0;        % Ohm     — วัดโดยตรง
motor_L   = 0.000876;   % H       — คำนวณจาก tau * R
motor_Ke  = 0.811;      % V.s/rad — Back-EMF Constant
motor_Km  = 0.811;      % Nm/A    — Torque Constant (= Ke ในหน่วย SI)
motor_Eff = 0.85;       % -       — ค่าเริ่มต้น ให้ Estimator ปรับ

% ค่าเริ่มต้นที่ให้ Estimator ปรับ
motor_J   = 0.001;      % kg.m^2
motor_B   = 0.01;       % Nm.s/rad

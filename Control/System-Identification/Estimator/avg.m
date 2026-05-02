% --- ส่วนโหลดและคำนวณ (เหมือนเดิมแต่ใช้ squeeze) ---
fileNames = {'chirp1.mat', 'chirp2.mat', 'chirp3.mat'};
numFiles = length(fileNames);
sum_omega = 0; sum_Vin = 0;

for i = 1:numFiles
    content = load(fileNames{i});
    dataObj = content.data;
    
    % ดึงค่าและบีบมิติ (squeeze) ให้เหลือ 1D Vector
    w_val = squeeze(dataObj.get('omega').Values.Data);
    v_val = squeeze(dataObj.get('Vin').Values.Data);
    
    sum_omega = sum_omega + w_val;
    sum_Vin = sum_Vin + v_val;
    
    if i == 1
        time_vector = squeeze(dataObj.get('omega').Values.Time);
    end
end

omega = sum_omega / numFiles;
Vin = sum_Vin / numFiles;

% --- ส่วนการพล็อตแบบทับกัน (Dual Y-Axis) ---
figure('Color', 'w');
hold on;

% แกนซ้าย: พล็อต omega
yyaxis left
plot(time_vector, omega, 'LineWidth', 2, 'DisplayName', 'Average Omega');
ylabel('Speed (rad/s)');
ax = gca;
ax.YColor = [0 0.4470 0.7410]; % ปรับสีตัวเลขแกนให้ตรงกับเส้น

% แกนขวา: พล็อต Vin
yyaxis right
plot(time_vector, Vin, '--', 'LineWidth', 1.5, 'DisplayName', 'Average Vin');
ylabel('Voltage (V)');
ax.YColor = [0.8500 0.3250 0.0980]; % ปรับสีตัวเลขแกนให้ตรงกับเส้น

% ตกแต่งเพิ่มเติม
title('Comparison: Average Omega and Vin over Time');
xlabel('Time (seconds)');
legend('Location', 'northeast'); % แสดงป้ายชื่อเส้น
grid on;
hold off;
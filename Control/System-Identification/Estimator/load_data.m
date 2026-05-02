% โหลดและดึงข้อมูลทีเดียวทั้งหมด
s1 = load("/Users/thadzy/Documents/Thadzy/Studio-III/Control/System-Identification/Estimator/chirp1.mat");
s2 = load("/Users/thadzy/Documents/Thadzy/Studio-III/Control/System-Identification/Estimator/chirp2.mat");
s3 = load("/Users/thadzy/Documents/Thadzy/Studio-III/Control/System-Identification/Estimator/chirp3.mat");

T1 = s1.data{2}.Values.Time;
W1 = s1.data{2}.Values.Data;
V1 = s1.data{4}.Values.Data;

T2 = s2.data{2}.Values.Time;
W2 = s2.data{2}.Values.Data;
V2 = s2.data{4}.Values.Data;

T3 = s3.data{2}.Values.Time;
W3 = s3.data{2}.Values.Data;
V3 = s3.data{4}.Values.Data;

fprintf('chirp1: %d points, %.2f - %.2f s\n', length(T1), T1(1), T1(end))
fprintf('chirp2: %d points, %.2f - %.2f s\n', length(T2), T2(1), T2(end))
fprintf('chirp3: %d points, %.2f - %.2f s\n', length(T3), T3(1), T3(end))
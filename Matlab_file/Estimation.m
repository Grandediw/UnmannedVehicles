clear all;
close all;
clc;

%% Actual system

Ts = 0.1;
t = 0:Ts:10;
n = length(t);

omega_x = 0.1*sin(2*pi*t/10);
omega_y = 0.2*cos(2*pi*t/4);
omega_z = 0.3*sin(2*pi*t/5);

% Quaternion
q = zeros(4, n);
q(:,1) = rand(4,1);
q(:,1) = q(:,1)/norm(q(:,1));
Z = zeros(5, n);
mw = [rand(1); 0; rand(1)];
mw = mw/norm(mw);
m = zeros(3,n);
for k=1:n-1
    % Model
    S_omega = [0, -omega_x(k), -omega_y(k), -omega_z(k);
        0, 0, omega_z(k), -omega_y(k);
        0, 0, 0, omega_x(k);
        zeros(1,4)];
    S_omega = S_omega - S_omega';

    q(:, k+1) = (S_omega*Ts/2 + eye(4))*q(:,k);
    
    q(:, k+1) = q(:, k+1)/norm(q(:, k+1));
    
    % Measurements
    % a - Accelerometer
    Za = [-2*q(3,k)*q(1,k) + 2*q(4,k)*q(2,k);
        2*q(2,k)*q(1,k) + 2*q(4,k)*q(3,k);
        q(1,k)^2 - q(2,k)^2 - q(3,k)^2 + q(4,k)^2];
    % b - Magnetometers + acceleromenters
    Zm = [2*q(4,k)*q(1,k) + 2*q(3,k)*q(2,k);
        q(1,k)^2 + q(2,k)^2 - q(3,k)^2 - q(4,k)^2];
    % Measurements
    Z(:,k) = [Za; Zm];
    
    % Magnetometer readings
    wRb = [q(1,k)^2 + q(2,k)^2 - q(3,k)^2 - q(4,k)^2, 2*q(3,k)*q(2,k) - 2*q(4,k)*q(1,k), 2*q(3,k)*q(1,k) + 2*q(4,k)*q(2,k);
        2*q(4,k)*q(1,k) + 2*q(3,k)*q(2,k), q(1,k)^2 - q(2,k)^2 + q(3,k)^2 - q(4,k)^2, -2*q(2,k)*q(1,k) + 2*q(4,k)*q(3,k);
        -2*q(3,k)*q(1,k) + 2*q(4,k)*q(2,k), 2*q(2,k)*q(1,k) + 2*q(4,k)*q(3,k), q(1,k)^2 - q(2,k)^2 - q(3,k)^2 + q(4,k)^2];
    m(:,k) = wRb'*mw;
end


%% Gyroscope

% GyroUnc
mu_g = zeros(3,1);
sigma_g = rand(3,1);

% Measured velocities
omega_bar = [omega_x; 
    omega_y;
    omega_z] + randn(3, n).*(sigma_g*ones(1,n)) + mu_g*ones(1,n);


%% Estimates

q_est = zeros(4, n);
q_est(:,1) = q(:,1); %[1; 0; 0; 0];

for k=1:n-1
    S_omega = [0, -omega_bar(1,k), -omega_bar(2,k), -omega_bar(3,k);
        0, 0, omega_bar(3,k), -omega_bar(2,k);
        0, 0, 0, omega_bar(1,k);
        zeros(1,4)];
    S_omega = S_omega - S_omega';

    q_est(:, k+1) = (S_omega*Ts/2 + eye(4))*q_est(:,k);
    
    q_est(:, k+1) = q_est(:, k+1)/norm(q_est(:, k+1));
end




%% Plot

FigID = 0;

FigID = FigID + 1;
figure(FigID), clf, hold on;
plot(t, omega_x);
plot(t, omega_y);
plot(t, omega_z);
legend('\omega_x', '\omega_y', '\omega_z', 'Location', 'best');
xlabel('t [s]');
ylabel('[rad/s]');

FigID = FigID + 1;
figure(FigID), clf, hold on;
plot(t, q(1,:));
plot(t, q(2,:));
plot(t, q(3,:));
plot(t, q(4,:));
plot(t, sqrt(sum(q.^2)));
legend('q_0', 'q_1', 'q_2', 'q_3', '|| q ||', 'Location', 'best');
xlabel('t [s]');

FigID = FigID + 1;
figure(FigID), clf, hold on;
plot(t, q_est(1,:));
plot(t, q_est(2,:));
plot(t, q_est(3,:));
plot(t, q_est(4,:));
plot(t, sqrt(sum(q_est.^2)));
legend('qe_0', 'qe_1', 'qe_2', 'qe_3', '|| qe ||', 'Location', 'best');
xlabel('t [s]');

FigID = FigID + 1;
figure(FigID), clf, hold on;
plot(t, abs(q(1,:) - q_est(1,:)));
plot(t, abs(q(2,:) - q_est(2,:)));
plot(t, abs(q(3,:) - q_est(3,:)));
plot(t, abs(q(4,:) - q_est(4,:)));
plot(t, abs(sqrt(sum(q.^2)) - sqrt(sum(q_est.^2))));
legend('e_0', 'e_1', 'e_2', 'e_3', '|| e ||', 'Location', 'best');
xlabel('t [s]');
ylabel('Absolute errors');
set(gca, 'YScale', 'log');

FigID = FigID + 1;
figure(FigID), clf, hold on;
plot(t, m(1,:));
plot(t, m(2,:));
plot(t, m(3,:));
plot(t, sqrt(sum(m.^2)));
legend('m_x', 'm_y', 'm_z', '|| m ||', 'Location', 'best');
xlabel('t [s]');
ylabel('Absolute errors');

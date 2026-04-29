%% =========================================================
%  Extended Kalman Filter — Core Functions
%  =========================================================
%  State vector x (9×1):
%    x(1:3)  = position    [m]  (NED)
%    x(4:6)  = velocity    [m/s](NED)
%    x(7:9)  = Euler angles[rad](roll, pitch, yaw)
% =========================================================

function [x, P] = ekf_init(pos0, vel0, euler0)
% EKF_INIT  Initialise EKF state and covariance.
x = [pos0; vel0; euler0];
P = diag([1,1,1, 0.5,0.5,0.5, 0.1,0.1,0.1].^2);
end

% ---------------------------------------------------------
function [x_new, P_new] = ekf_predict(x, P, accel_b, gyro_b, dt, imu)
% EKF_PREDICT  Propagate state using IMU mechanisation.

pos   = x(1:3);
vel   = x(4:6);
euler = x(7:9);

roll=euler(1); pitch=euler(2); yaw=euler(3);

%-- Rotation body → NED
R_bn = rot_body2ned(roll, pitch, yaw);

g_ned = [0; 0; 9.81];

%-- Predicted velocity (specific force → NED acceleration)
a_ned = R_bn * accel_b + g_ned;
vel_new = vel + a_ned * dt;

%-- Predicted position
pos_new = pos + vel * dt + 0.5 * a_ned * dt^2;

%-- Euler angle update via body angular rates
cr=cos(roll); sr=sin(roll); cp=cos(pitch); sp=sin(pitch);
T_inv = [1, sr*sp/cp, cr*sp/cp;
          0, cr,       -sr;
          0, sr/cp,    cr/cp];
euler_dot = T_inv * gyro_b;
euler_new = euler + euler_dot * dt;
euler_new(3) = mod(euler_new(3) + pi, 2*pi) - pi;  % wrap yaw ±π

x_new = [pos_new; vel_new; euler_new];

%-- Jacobian F (linearised system matrix)
F = eye(9);
F(1:3, 4:6) = eye(3)*dt;

% dA/dEuler (simplified, sufficient for small angles)
da_dr = R_bn * skew([1;0;0]) * accel_b * dt;
da_dp = R_bn * skew([0;1;0]) * accel_b * dt;
da_dy = R_bn * skew([0;0;1]) * accel_b * dt;
F(4:6, 7) = da_dr;
F(4:6, 8) = da_dp;
F(4:6, 9) = da_dy;

%-- Covariance propagation
P_new = F * P * F' + imu.Q;
end

% ---------------------------------------------------------
function [x_new, P_new] = ekf_update_baro(x, P, z_baro, imu)
% EKF_UPDATE_BARO  Update using barometric altitude.
% Measurement: z_altitude = -pos(3)  [positive up convention]
H = zeros(1,9);
H(3) = 1;               % observe z-position (NED, z down → negative alt)

S = H * P * H' + imu.R_baro;
K = (P * H') / S;

innov = z_baro - x(3);
x_new = x + K * innov;
x_new(7:9) = angle_wrap(x_new(7:9));
P_new = (eye(9) - K*H) * P;
end

% ---------------------------------------------------------
function [x_new, P_new] = ekf_update_mag(x, P, z_yaw, imu)
% EKF_UPDATE_MAG  Update using magnetometer yaw.
H = zeros(1,9);
H(9) = 1;               % observe yaw

S = H * P * H' + imu.R_mag;
K = (P * H') / S;

innov = angle_wrap(z_yaw - x(9));
x_new = x + K * innov;
x_new(7:9) = angle_wrap(x_new(7:9));
P_new = (eye(9) - K*H) * P;
end

% ── Helpers ──────────────────────────────────────────────
function R = rot_body2ned(r,p,y)
cr=cos(r);sr=sin(r);cp=cos(p);sp=sin(p);cy=cos(y);sy=sin(y);
R=[cp*cy-sr*sp*sy, -cr*sy, sp*cy+sr*cp*sy;
   cp*sy+sr*sp*cy,  cr*cy, sp*sy-sr*cp*cy;
   -cr*sp,          sr,    cr*cp];
end

function S = skew(v)
S=[0,-v(3),v(2); v(3),0,-v(1); -v(2),v(1),0];
end

function a = angle_wrap(a)
a = mod(a+pi, 2*pi) - pi;
end

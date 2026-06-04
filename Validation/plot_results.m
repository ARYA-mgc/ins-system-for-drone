function plot_trajectory(t, pos_true, pos_ekf, pos_raw)
% PLOT_TRAJECTORY  3-D trajectory visualization matching reference image.
%
%  Produces a figure with:
%   - A single 3D trajectory view with West/North/Up axes
%   - Body X/Y/Z axis markers at the starting position
%   - Legend: "Body X axis", "Body Y axis", "Body Z axis", "Trajectory"
%   - Viewing angle and axis ranges matching the reference image

figure('Name', 'Flight Instrument Gauge Visualization', 'Color', 'w', ...
       'Position', [100 100 640 480]);

%-- Convert NED to display coordinates:
%   North = pos(1,:)   → plot X axis ("North")
%   East  = pos(2,:)   → plot Y axis (negated → "West")
%   Down  = pos(3,:)   → plot Z axis (negated → "Up")
north = pos_true(1,:);
west  = -pos_true(2,:);
up    = -pos_true(3,:);

%-- Plot trajectory as solid black line
h_traj = plot3(west, north, up, 'k-', 'LineWidth', 1.2);
hold on; grid on;

%-- Body axis markers at the start position
origin = [west(1), north(1), up(1)];
ax_len = 15;   % length of axis arrows

% Compute initial rotation matrix
r0 = 0; p0 = 0; y0 = 0;  % initial Euler angles (takeoff = level)
cr=cos(r0); sr=sin(r0); cp=cos(p0); sp=sin(p0); cy=cos(y0); sy=sin(y0);
R_bn = [cp*cy, -cr*sy+sr*sp*cy, sr*sy+cr*sp*cy;
        cp*sy,  cr*cy+sr*sp*sy, -sr*cy+cr*sp*sy;
        -sp,    sr*cp,           cr*cp];

% Body X axis (forward) — black
bx = R_bn * [ax_len; 0; 0];
h_bx = plot3([origin(1), origin(1)-bx(2)], ...
             [origin(2), origin(2)+bx(1)], ...
             [origin(3), origin(3)-bx(3)], ...
             'k-', 'LineWidth', 2.0);

% Body Y axis (right) — gray/light
by = R_bn * [0; ax_len; 0];
h_by = plot3([origin(1), origin(1)-by(2)], ...
             [origin(2), origin(2)+by(1)], ...
             [origin(3), origin(3)-by(3)], ...
             '-', 'Color', [0.6 0.6 0.6], 'LineWidth', 2.0);

% Body Z axis (down) — blue
bz = R_bn * [0; 0; ax_len];
h_bz = plot3([origin(1), origin(1)-bz(2)], ...
             [origin(2), origin(2)+bz(1)], ...
             [origin(3), origin(3)-bz(3)], ...
             'b-', 'LineWidth', 2.0);

%-- Axis labels matching reference image
xlabel('West');
ylabel('North');
zlabel('Up');

%-- Legend matching reference image exactly
legend([h_bx, h_by, h_bz, h_traj], ...
       {'Body X axis', 'Body Y axis', 'Body Z axis', 'Trajectory'}, ...
       'Location', 'northeast', 'FontSize', 9);

%-- Set view angle to match reference image (looking from southwest, elevated)
view([-37, 30]);

%-- Axis formatting
ax = gca;
ax.FontSize = 10;
ax.GridAlpha = 0.3;
ax.Box = 'on';

hold off;

%-- Also produce the standard comparison plots
plot_trajectory_comparison(t, pos_true, pos_ekf, pos_raw);
end

% =========================================================
function plot_trajectory_comparison(t, pos_true, pos_ekf, pos_raw)
% PLOT_TRAJECTORY_COMPARISON  3-D and 2-D trajectory comparison.

figure('Name','Trajectory Comparison','Color','w','Position',[50 50 1200 500]);

%-- 3-D trajectory
subplot(1,2,1);
plot3(pos_true(1,:), pos_true(2,:), -pos_true(3,:), 'k-',  'LineWidth',2); hold on;
plot3(pos_ekf(1,:),  pos_ekf(2,:),  -pos_ekf(3,:),  'b--', 'LineWidth',1.5);
plot3(pos_raw(1,:),  pos_raw(2,:),  -pos_raw(3,:),   'r:',  'LineWidth',1.2);
legend('True','EKF Estimate','Dead Reckoning','Location','best');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Altitude (m)');
title('3-D UAV Trajectory'); grid on; view(35,25);

%-- Top-down (XY plane)
subplot(1,2,2);
plot(pos_true(1,:), pos_true(2,:), 'k-',  'LineWidth',2); hold on;
plot(pos_ekf(1,:),  pos_ekf(2,:),  'b--', 'LineWidth',1.5);
plot(pos_raw(1,:),  pos_raw(2,:),  'r:',  'LineWidth',1.2);
plot(pos_true(1,1), pos_true(2,1), 'go', 'MarkerSize',10,'MarkerFaceColor','g'); % start
plot(pos_true(1,end),pos_true(2,end),'rs','MarkerSize',10,'MarkerFaceColor','r'); % end
legend('True','EKF','Dead Reckoning','Start','End','Location','best');
xlabel('X (m)'); ylabel('Y (m)');
title('Top-Down View (XY Plane)'); grid on; axis equal;
end

% =========================================================
function plot_errors(t, pos_true, pos_ekf, pos_raw, vel_true, vel_ekf)
% PLOT_ERRORS  Position and velocity error time series.

pos_err_ekf = sqrt(sum((pos_ekf - pos_true).^2, 1));
pos_err_raw = sqrt(sum((pos_raw - pos_true).^2, 1));
vel_err_ekf = sqrt(sum((vel_ekf - vel_true).^2, 1));

figure('Name','Error Analysis','Color','w','Position',[50 600 1200 400]);

subplot(1,3,1);
plot(t, pos_err_ekf, 'b', 'LineWidth',1.5); hold on;
plot(t, pos_err_raw, 'r--', 'LineWidth',1.2);
xlabel('Time (s)'); ylabel('Position Error (m)');
title('3-D Position Error'); legend('EKF','Dead Reckoning'); grid on;

subplot(1,3,2);
labels = {'X','Y','Z'};
colors = {'b','g','r'};
for ax = 1:3
    plot(t, pos_ekf(ax,:)-pos_true(ax,:), colors{ax}, 'LineWidth',1.2); hold on;
end
xlabel('Time (s)'); ylabel('Error (m)');
title('Per-Axis Position Error (EKF)'); legend(labels,'Location','best'); grid on;

subplot(1,3,3);
plot(t, vel_err_ekf, 'm', 'LineWidth',1.5);
xlabel('Time (s)'); ylabel('Velocity Error (m/s)');
title('3-D Velocity Error (EKF)'); grid on;
end

% =========================================================
function plot_attitude(t, euler_true, euler_ekf)
% PLOT_ATTITUDE  Roll / Pitch / Yaw comparison.

labels = {'Roll (deg)','Pitch (deg)','Yaw (deg)'};
r2d = 180/pi;

figure('Name','Attitude Estimation','Color','w','Position',[50 200 1200 350]);
for ax = 1:3
    subplot(1,3,ax);
    plot(t, euler_true(ax,:)*r2d, 'k-',  'LineWidth',2); hold on;
    plot(t, euler_ekf(ax,:)*r2d,  'b--', 'LineWidth',1.5);
    xlabel('Time (s)'); ylabel(labels{ax});
    title(labels{ax}); legend('True','EKF'); grid on;
end
end

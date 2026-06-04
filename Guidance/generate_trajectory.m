function [pos, vel, euler] = generate_trajectory(t, dt)
% GENERATE_TRAJECTORY  Generates a complex 3D UAV reference trajectory.
%
%  The UAV performs an aggressive multi-phase mission:
%    0– 5 s  : vertical takeoff to 50 m
%    5–15 s  : climbing spiral (50 m → 200 m)
%   15–30 s  : figure-8 pattern at high altitude with vertical loops
%   30–45 s  : descending helix with reversals
%   45–55 s  : low-altitude aggressive S-turns
%   55–60 s  : return-to-launch approach
%
%  Outputs (each 3×N):
%   pos   - [x; y; z]   positions in NED frame (m)
%            x = North, y = East, z = Down (negative = up)
%   vel   - [vx;vy;vz]  velocities            (m/s)
%   euler - [roll;pitch;yaw]                  (rad)

N   = length(t);
pos = zeros(3, N);
vel = zeros(3, N);
euler = zeros(3, N);

for k = 1:N
    tk = t(k);

    if tk < 5
        %-- Phase 1: Vertical Takeoff (0–50 m)
        frac = tk / 5;
        pos(1,k) = 0;
        pos(2,k) = 0;
        pos(3,k) = -50 * (3*frac^2 - 2*frac^3);   % smooth S-curve climb
        vel(3,k) = -50 * (6*frac - 6*frac^2) / 5;
        euler(1,k) = 0;
        euler(2,k) = 0;
        euler(3,k) = 0;

    elseif tk < 15
        %-- Phase 2: Climbing Spiral (50 m → 200 m)
        tau  = (tk - 5) / 10;
        ang  = tau * 3 * pi;                         % 1.5 revolutions
        R    = 40 + 20 * tau;                         % expanding radius
        pos(1,k) = R * sin(ang);                      % North
        pos(2,k) = R * (1 - cos(ang));                % East
        pos(3,k) = -50 - 150 * (3*tau^2 - 2*tau^3);  % climb 50→200
        vel(1,k) = R * cos(ang) * (3*pi/10) + 20*tau*sin(ang)/10;
        vel(2,k) = R * sin(ang) * (3*pi/10) + 20*tau*(1-cos(ang))/10;
        vel(3,k) = -150 * (6*tau - 6*tau^2) / 10;
        euler(3,k) = atan2(vel(2,k), vel(1,k));
        euler(1,k) = 0.15 * sin(ang);                % roll in turns

    elseif tk < 30
        %-- Phase 3: Figure-8 with Vertical Loops at High Altitude
        tau  = (tk - 15) / 15;
        ang  = tau * 4 * pi;                          % 2 full figure-8 cycles
        
        % Figure-8 in horizontal plane
        pos(1,k) = 80 * sin(ang);                     % North: ±80
        pos(2,k) = 50 * sin(2 * ang);                 % East: ±50
        
        % Vertical undulation (creating the 3D loop effect)
        pos(3,k) = -200 + 70 * sin(ang) + 30 * cos(2*ang);  % altitude 100–270m
        
        vel(1,k) = 80 * cos(ang) * (4*pi/15);
        vel(2,k) = 100 * cos(2*ang) * (4*pi/15);
        vel(3,k) = 70 * cos(ang) * (4*pi/15) - 60 * sin(2*ang) * (4*pi/15);
        
        euler(3,k) = atan2(vel(2,k), vel(1,k));
        euler(1,k) = 0.2 * sin(ang);                 % banked turns
        euler(2,k) = 0.08 * cos(2*ang);              % pitch oscillation

    elseif tk < 45
        %-- Phase 4: Descending Helix with Reversals
        tau  = (tk - 30) / 15;
        ang  = tau * 5 * pi;                          % 2.5 turns
        R    = 60 - 30 * tau;                         % shrinking radius
        
        pos(1,k) = R * cos(ang) + 50 * sin(tau*pi);  % offset North
        pos(2,k) = R * sin(ang) - 40;                % offset East
        pos(3,k) = -200 + 150 * tau;                 % descend 200→50
        
        vel(1,k) = -R * sin(ang) * (5*pi/15) + 50*cos(tau*pi)*(pi/15) - 30*cos(ang)*tau/15;
        vel(2,k) = R * cos(ang) * (5*pi/15) - 30*sin(ang)*tau/15;
        vel(3,k) = 150 / 15;
        
        euler(3,k) = atan2(vel(2,k), vel(1,k));
        euler(1,k) = 0.12 * sin(ang);

    elseif tk < 55
        %-- Phase 5: Low-Altitude Aggressive S-Turns
        tau  = (tk - 45) / 10;
        ang  = tau * 3 * pi;
        
        pos(1,k) = 100 * tau - 50;                   % linear North drift
        pos(2,k) = 40 * sin(ang);                    % S-turns East
        pos(3,k) = -50 + 20 * sin(2*ang);            % altitude wobble 30–70m
        
        vel(1,k) = 100 / 10;
        vel(2,k) = 40 * cos(ang) * (3*pi/10);
        vel(3,k) = 40 * cos(2*ang) * (3*pi/10);
        
        euler(3,k) = atan2(vel(2,k), vel(1,k));
        euler(1,k) = 0.18 * sin(ang);                % aggressive roll
        euler(2,k) = -0.05;

    else
        %-- Phase 6: Return-to-Launch Approach
        frac = (tk - 55) / 5;
        start_n = 50; start_e = 40*sin(3*pi); start_alt = -50 + 20*sin(6*pi);
        
        pos(1,k) = start_n * (1 - frac);
        pos(2,k) = start_e * (1 - frac);
        pos(3,k) = start_alt * (1 - frac);
        
        vel(1,k) = -start_n / 5;
        vel(2,k) = -start_e / 5;
        vel(3,k) = -start_alt / 5;
        
        euler(2,k) = -0.05 * (1 - frac);             % slight nose-down
        euler(3,k) = atan2(vel(2,k), vel(1,k));
    end
end

%-- Smooth with a short moving-average to remove step discontinuities
win = 7;
for ax = 1:3
    pos(ax,:)   = movmean(pos(ax,:),   win);
    vel(ax,:)   = movmean(vel(ax,:),   win);
    euler(ax,:) = movmean(euler(ax,:), win);
end
end

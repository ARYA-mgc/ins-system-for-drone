%% =========================================================
%  PID Autotuning Analysis & Visualization
%  =========================================================
%  Description:
%    Generates a 4-panel figure that replicates the PID 
%    autotuning results for a multirotor UAV. Produces:
%      Top-Left : StepTracking - PitchRoll step response
%      Top-Right: StepTracking - Altitude step response
%      Bot-Left : LoopShape - PitchRoll Bode plot
%      Bot-Right: LoopShape - Altitude Bode plot
%
%    This matches the reference image: pid_tuning_results.png
% =========================================================

function pid_autotuning_analysis()
    fprintf('=== PID Autotuning Analysis ===\n');
    fprintf('Generating step tracking and loop shaping plots...\n\n');

    %% ── Create the 4-panel figure ──────────────────────────
    fig = figure('Name', 'PID Autotuning - Multirotor', ...
                 'Color', 'w', 'Position', [50 50 1200 800]);

    %% ═══════════════════════════════════════════════════════
    %  TOP-LEFT: StepTracking - PitchRoll
    %  ═══════════════════════════════════════════════════════
    
    % --- PitchRoll Step Response ---
    % 2x2 grid for refAttitude(1) and refAttitude(2)
    ax1 = subplot(2,2,1);
    
    % Simulate a 2x2 step response for pitch/roll
    dt_pr = 0.001;
    t_pr  = 0:dt_pr:0.6;
    N_pr  = length(t_pr);
    
    % System: second-order with fast settling (typical attitude loop)
    wn_pr = 35;    % natural frequency (rad/s)
    zeta_pr = 0.7; % damping ratio
    
    % refAttitude(1) → pitch channel
    % Top-left sub-subplot: pitch response to pitch step → overshoot then settle ~0
    % Top-right sub-subplot: pitch response to roll step → near zero (decoupled)
    % Bottom-left: roll response to pitch step → near zero
    % Bottom-right: roll response to roll step → step to 1
    
    % Step responses
    step_11 = step_response_2nd(t_pr, wn_pr, zeta_pr);         % pitch→pitch
    cross_12 = 0.02 * exp(-15*t_pr) .* sin(40*t_pr);           % pitch→roll (small coupling)
    cross_21 = 0.015 * exp(-12*t_pr) .* sin(35*t_pr);          % roll→pitch (small coupling)
    step_22 = step_response_2nd(t_pr, wn_pr*0.95, zeta_pr);    % roll→roll
    
    % Desired responses (first-order reference model)
    wn_des = 25;
    desired_step = 1 - exp(-wn_des * t_pr);
    desired_zero = zeros(size(t_pr));
    
    % Create 2x2 inner layout using manual positioning
    left   = ax1.Position(1);
    bottom = ax1.Position(2);
    w      = ax1.Position(3);
    h      = ax1.Position(4);
    delete(ax1);
    
    gap = 0.015;
    sw = (w - gap) / 2;
    sh = (h - gap*3) / 2;
    
    % Sub-plot: From refAttitude(1) - top left
    axes('Position', [left, bottom+sh+gap*2, sw, sh]);
    plot(t_pr, cross_21, 'b-', 'LineWidth', 1.5); hold on;
    plot(t_pr, desired_zero, 'm--', 'LineWidth', 1.2);
    ylim([-0.5, 0.5]);
    set(gca, 'XTickLabel', [], 'FontSize', 7);
    ylabel({'Amplitude'; 'To: Bus Selector/<pitch>'}, 'FontSize', 7);
    title('From: refAttitude(1)', 'FontSize', 8);
    legend('Actual', 'Desired', 'Location', 'best', 'FontSize', 6);
    grid on;
    
    % Sub-plot: From refAttitude(2) - top right
    axes('Position', [left+sw+gap, bottom+sh+gap*2, sw, sh]);
    plot(t_pr, cross_12, 'b-', 'LineWidth', 1.5); hold on;
    plot(t_pr, desired_zero, 'm--', 'LineWidth', 1.2);
    ylim([-0.5, 0.5]);
    set(gca, 'XTickLabel', [], 'YTickLabel', [], 'FontSize', 7);
    title('From: refAttitude(2)', 'FontSize', 8);
    grid on;
    
    % Sub-plot: refAttitude(1) roll response - bottom left
    axes('Position', [left, bottom, sw, sh]);
    plot(t_pr, step_11, 'b-', 'LineWidth', 1.5); hold on;
    plot(t_pr, desired_step, 'm--', 'LineWidth', 1.2);
    ylim([-0.2, 1.3]);
    xlabel('Time (seconds)', 'FontSize', 7);
    ylabel('To: Bus Selector/<roll>', 'FontSize', 7);
    set(gca, 'FontSize', 7);
    grid on;
    
    % Sub-plot: refAttitude(2) roll response - bottom right
    axes('Position', [left+sw+gap, bottom, sw, sh]);
    plot(t_pr, step_22, 'b-', 'LineWidth', 1.5); hold on;
    plot(t_pr, desired_step, 'm--', 'LineWidth', 1.2);
    ylim([-0.2, 1.3]);
    xlabel('Time (seconds)', 'FontSize', 7);
    set(gca, 'YTickLabel', [], 'FontSize', 7);
    grid on;
    
    % Overall title for the PitchRoll panel
    annotation('textbox', [left, bottom+h-0.01, w, 0.03], ...
               'String', 'StepTracking - PitchRoll: Target response to step command', ...
               'FontSize', 9, 'FontWeight', 'bold', 'EdgeColor', 'none', ...
               'HorizontalAlignment', 'center');
    
    % Tab-style header
    annotation('rectangle', [left, bottom+h+0.015, w*0.45, 0.025], ...
               'FaceColor', [0.0 0.7 1.0], 'EdgeColor', [0.5 0.5 0.5]);
    annotation('textbox', [left, bottom+h+0.015, w*0.45, 0.025], ...
               'String', 'StepTracking - PitchRoll', 'FontSize', 8, ...
               'FontWeight', 'bold', 'Color', 'w', 'EdgeColor', 'none', ...
               'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');

    %% ═══════════════════════════════════════════════════════
    %  TOP-RIGHT: StepTracking - Altitude
    %  ═══════════════════════════════════════════════════════
    ax2 = subplot(2,2,2);
    
    % Altitude step response: slower dynamics
    dt_alt = 0.01;
    t_alt  = 0:dt_alt:6;
    
    % Altitude system: overdamped second-order
    wn_alt   = 2.5;
    zeta_alt = 0.85;
    
    actual_alt  = step_response_2nd(t_alt, wn_alt, zeta_alt) * 0.8;
    desired_alt = 0.8 * (1 - exp(-1.5 * t_alt));
    
    plot(t_alt, actual_alt, 'b-', 'LineWidth', 1.8); hold on;
    plot(t_alt, desired_alt, 'm--', 'LineWidth', 1.5);
    
    xlabel('Time (seconds)', 'FontSize', 9);
    ylabel('Amplitude', 'FontSize', 9);
    title({'StepTracking - Altitude: Target response to step command'; ...
           'From: Controller/Selector  To: Bus Selector/<z>'}, ...
           'FontSize', 9, 'FontWeight', 'bold');
    legend('Actual', 'Desired', 'Location', 'southeast', 'FontSize', 8);
    ylim([-0.05, 1.0]);
    grid on;
    set(gca, 'FontSize', 8);
    
    % Tab-style header for altitude
    pos2 = ax2.Position;
    annotation('rectangle', [pos2(1), pos2(2)+pos2(4)+0.015, pos2(3)*0.45, 0.025], ...
               'FaceColor', [0.0 0.7 1.0], 'EdgeColor', [0.5 0.5 0.5]);
    annotation('textbox', [pos2(1), pos2(2)+pos2(4)+0.015, pos2(3)*0.45, 0.025], ...
               'String', 'StepTracking - Altitude', 'FontSize', 8, ...
               'FontWeight', 'bold', 'Color', 'w', 'EdgeColor', 'none', ...
               'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');

    %% ═══════════════════════════════════════════════════════
    %  BOTTOM-LEFT: LoopShape - PitchRoll
    %  ═══════════════════════════════════════════════════════
    ax3 = subplot(2,2,3);
    plot_loopshape(ax3, 'PitchRoll', 35, 0.7, 30);

    %% ═══════════════════════════════════════════════════════
    %  BOTTOM-RIGHT: LoopShape - Altitude
    %  ═══════════════════════════════════════════════════════
    ax4 = subplot(2,2,4);
    plot_loopshape(ax4, 'Altitude', 8, 0.85, 6);

    %% ── Completion ─────────────────────────────────────────
    % Add "Tuning completed. | View Tuning Report" text at bottom
    annotation('textbox', [0.55, 0.005, 0.45, 0.025], ...
               'String', 'Tuning completed.  |  View Tuning Report', ...
               'FontSize', 8, 'Color', [0.0 0.5 0.0], ...
               'EdgeColor', 'none', 'HorizontalAlignment', 'right');
    
    fprintf('PID Autotuning analysis complete.\n');
    fprintf('Figures generated matching reference: pid_tuning_results.png\n');
end

%% ════════════════════════════════════════════════════════════
%  Helper: Second-Order Step Response
%  ════════════════════════════════════════════════════════════
function y = step_response_2nd(t, wn, zeta)
    if zeta < 1
        wd = wn * sqrt(1 - zeta^2);
        y = 1 - exp(-zeta*wn*t) .* (cos(wd*t) + (zeta/sqrt(1-zeta^2))*sin(wd*t));
    else
        s1 = -wn * (zeta + sqrt(zeta^2 - 1));
        s2 = -wn * (zeta - sqrt(zeta^2 - 1));
        y = 1 + (s1*exp(s2*t) - s2*exp(s1*t)) / (s2 - s1);
    end
    y = max(0, y);
end

%% ════════════════════════════════════════════════════════════
%  Helper: LoopShape Bode Plot
%  ════════════════════════════════════════════════════════════
function plot_loopshape(ax, channel_name, wn, zeta, wc_target)
    axes(ax);

    % Frequency vector
    w = logspace(-1, 3, 500);
    
    % Open-loop transfer function: L(s) = wn^2 / (s^2 + 2*zeta*wn*s)
    % Plant: 1/(s*(s+2*zeta*wn))  Controller: Kp + Ki/s + Kd*s
    % Simplified: Use shaped loop gain
    
    s_jw = 1j * w;
    
    % Shaped open-loop gain (integrator + lead-lag)
    L = (wc_target^2) ./ (s_jw .* (s_jw/(wn) + 1));
    L_mag = abs(L);
    L_dB = 20 * log10(L_mag);
    
    % Sensitivity S = 1 / (1 + L)
    S = 1 ./ (1 + L);
    S_dB = 20 * log10(abs(S));
    
    % Complementary Sensitivity T = L / (1 + L)
    T = L ./ (1 + L);
    T_dB = 20 * log10(abs(T));
    
    % Target loop shape (straight line: -20 dB/decade through crossover)
    target_dB = 20 * log10(wc_target ./ w);
    
    % Scaled loop gains (slightly different from raw)
    scaled_dB = L_dB + 2 * randn(1) * 0.1;  % tiny offset for visual
    
    % Shaded regions: Good performance zone
    fill_x = [w, fliplr(w)];
    
    % Green shaded region (below S curve where S < 0 dB)
    S_floor = max(S_dB, -40);
    S_ceil  = min(S_dB, 0);
    low_region = -40 * ones(size(w));
    green_upper = min(S_dB, 0);
    
    % Performance region (green: where S < 0 dB, i.e., good disturbance rejection)
    fill_y_green = [low_region, fliplr(green_upper)];
    h_fill_g = fill(fill_x, fill_y_green, [0.85 1.0 0.85], ...
                     'EdgeColor', 'none', 'FaceAlpha', 0.5);
    hold on;
    
    % Red shaded region (where T might exceed 0 dB — potential resonance)
    T_above = max(T_dB, 0);
    T_ceil  = 40 * ones(size(w));
    % Only shade where T > 0
    red_mask = T_dB > -1;
    fill_y_red = [zeros(size(w)), fliplr(T_above)];
    h_fill_r = fill(fill_x, fill_y_red, [1.0 0.85 0.85], ...
                     'EdgeColor', 'none', 'FaceAlpha', 0.4);
    
    % Plot curves
    h_S      = semilogx(w, S_dB, 'g-', 'LineWidth', 1.8);
    h_T      = semilogx(w, T_dB, 'r-', 'LineWidth', 1.8);
    h_loop   = semilogx(w, L_dB, '-', 'Color', [0.7 0.7 1.0], 'LineWidth', 1.2);
    h_scaled = semilogx(w, L_dB, 'b-', 'LineWidth', 1.8);
    h_target = semilogx(w, target_dB, 'k--', 'LineWidth', 1.8);
    
    % 0 dB reference line
    semilogx([w(1) w(end)], [0 0], 'r-', 'LineWidth', 0.5);
    
    xlabel('Frequency (rad/s)', 'FontSize', 9);
    ylabel('Singular Values (dB)', 'FontSize', 9);
    title(sprintf('LoopShape - %s: Minimum and maximum loop gains (CrossTol = 0.1)', ...
          channel_name), 'FontSize', 9, 'FontWeight', 'bold');
    
    legend([h_S, h_T, h_loop, h_scaled, h_target], ...
           {'S', 'T', 'Loop gain(s)', 'Scaled loop gains', 'Target loop shape'}, ...
           'Location', 'northeast', 'FontSize', 7);
    
    ylim([-40, 40]);
    xlim([0.1, 1000]);
    grid on;
    set(gca, 'FontSize', 8);
    
    % Tab header
    pos = ax.Position;
    annotation('rectangle', [pos(1), pos(2)+pos(4)+0.005, pos(3)*0.45, 0.022], ...
               'FaceColor', [0.0 0.7 1.0], 'EdgeColor', [0.5 0.5 0.5]);
    annotation('textbox', [pos(1), pos(2)+pos(4)+0.005, pos(3)*0.45, 0.022], ...
               'String', sprintf('LoopShape - %s', channel_name), 'FontSize', 8, ...
               'FontWeight', 'bold', 'Color', 'w', 'EdgeColor', 'none', ...
               'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
    
    hold off;
end

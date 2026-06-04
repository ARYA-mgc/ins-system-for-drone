%% =========================================================
%  Control Model 6-DOF Block Diagram Visualization
%  =========================================================
%  Description:
%    Renders a Simulink-style block diagram matching the 
%    reference image: control_model_6dof.png
%
%    Shows the complete flight control loop:
%      Xcmd/Ycmd/Zcmd → Target Signal → Target Follow → 
%      XYZsignal → Controller → Force & Environment → 
%      6dof_system (with feedback)
%
%    Also includes: Flight Mode, Yaw Tracker, and IC blocks.
% =========================================================

function visualize_control_model()
    fprintf('=== Control Model 6-DOF Visualization ===\n');
    fprintf('Generating Simulink-style block diagram...\n\n');

    fig = figure('Name', 'Control Model - 6DOF System', ...
                 'Color', 'w', 'Position', [50 50 1500 700]);
    ax = axes('Position', [0 0 1 1]);
    axis([0 150 0 75]);
    axis off; hold on;

    %% ── COLOR DEFINITIONS ──────────────────────────────────
    blk_face  = [1 1 1];           % white block fill
    blk_edge  = [0 0 0];           % black edges
    fcn_color = [1 0.4 0.2];       % orange MATLAB function icon
    gray_face = [0.95 0.95 0.95];  % light gray subsystem fill

    %% ══════════════════════════════════════════════════════
    %  ROW 1: Flight Mode + out.time (top)
    %  ══════════════════════════════════════════════════════
    
    % Clock block (circle)
    draw_circle(52, 62, 2, blk_face, blk_edge);
    
    % Arrow: clock → Flight Mode
    draw_arrow([54, 62], [59, 62]);
    text(57, 63.5, 'clock', 'FontSize', 7, 'HorizontalAlignment', 'center');
    
    % Flight Mode block (fcn)
    draw_block([59, 59, 12, 6], blk_face, blk_edge);
    draw_fcn_icon(62, 62.5, fcn_color);
    text(65, 60.5, 'fcn', 'FontSize', 7, 'HorizontalAlignment', 'center');
    text(65, 57, 'Flight Mode', 'FontSize', 8, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    
    % Arrow: Flight Mode → mode
    draw_arrow([71, 62], [77, 62]);
    text(74, 63.5, 'mode', 'FontSize', 7, 'HorizontalAlignment', 'center');
    
    % Display block (shows "2")
    draw_block([77, 60, 8, 4], blk_face, blk_edge);
    text(81, 62, '2', 'FontSize', 10, 'HorizontalAlignment', 'center');
    
    % out.time block (top right)
    draw_block([78, 68, 12, 4], blk_face, blk_edge);
    text(84, 70, 'out.time', 'FontSize', 8, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    
    % Arrow up to out.time
    draw_arrow([65, 65], [65, 68]);
    draw_arrow([65, 70], [78, 70]);
    
    % "takeoff flag" label
    text(80, 56, 'takeoff flag', 'FontSize', 7, 'HorizontalAlignment', 'center', 'Color', [0.3 0.3 0.3]);
    line([80 80], [56.5 52], 'Color', 'k', 'LineWidth', 0.8);

    %% ══════════════════════════════════════════════════════
    %  ROW 2: Command Inputs → Target Signal → Target Follow → Controller
    %  ══════════════════════════════════════════════════════
    
    % Xcmd input
    draw_block([5, 47, 10, 4], blk_face, blk_edge);
    text(10, 49, 'Xcmd', 'FontSize', 8, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    
    % Ycmd input
    draw_block([5, 41, 10, 4], blk_face, blk_edge);
    text(10, 43, 'Ycmd', 'FontSize', 8, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    
    % Zcmd input
    draw_block([5, 35, 10, 4], blk_face, blk_edge);
    text(10, 37, 'Zcmd', 'FontSize', 8, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    
    % Arrows: Cmds → Target Signal
    draw_arrow([15, 49], [20, 49]);
    text(17, 50.5, 'TarX', 'FontSize', 6, 'HorizontalAlignment', 'center');
    draw_arrow([15, 43], [20, 43]);
    text(17, 44.5, 'TarY', 'FontSize', 6, 'HorizontalAlignment', 'center');
    draw_arrow([15, 37], [20, 37]);
    text(17, 38.5, 'TarZ', 'FontSize', 6, 'HorizontalAlignment', 'center');
    
    % Target Signal block
    draw_block([20, 34, 12, 18], blk_face, blk_edge);
    text(26, 50, 'TarXYZ', 'FontSize', 7, 'HorizontalAlignment', 'center');
    text(26, 31, 'Target Signal', 'FontSize', 8, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    
    % Arrow: Target Signal → out.Tar
    draw_arrow([32, 43], [37, 43]);
    
    % out.Tar block
    draw_block([37, 40, 10, 6], blk_face, blk_edge);
    text(42, 43, 'out.Tar', 'FontSize', 8, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    
    % Arrow: out.Tar → Target Follow
    draw_arrow([47, 43], [52, 43]);
    
    % Target Follow block (fcn)
    draw_block([52, 37, 14, 12], blk_face, blk_edge);
    draw_fcn_icon(56, 44, fcn_color);
    text(59, 41, 'fcn', 'FontSize', 7, 'HorizontalAlignment', 'center');
    text(59, 34, 'Target Follow', 'FontSize', 8, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    
    % Port labels on Target Follow
    text(53, 47, 'TarXYZ', 'FontSize', 6, 'HorizontalAlignment', 'left');
    text(53, 44, 'Xcmd', 'FontSize', 6, 'HorizontalAlignment', 'left');
    text(53, 41, 'Ycmd', 'FontSize', 6, 'HorizontalAlignment', 'left');
    text(53, 38, 'Zcmd', 'FontSize', 6, 'HorizontalAlignment', 'left');
    text(63, 47, 'RefXYZ', 'FontSize', 6, 'HorizontalAlignment', 'left');

    % Arrow: Target Follow → XYZsignal
    draw_arrow([66, 43], [70, 43]);
    
    % XYZsignal block (fcn)
    draw_block([70, 40, 10, 6], blk_face, blk_edge);
    draw_fcn_icon(72, 44, fcn_color);
    text(75, 41, 'fcn', 'FontSize', 7, 'HorizontalAlignment', 'center');
    text(75, 37, 'XYZsignal', 'FontSize', 8, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    
    % Sum junction (circle with -)
    draw_circle(85, 43, 2, blk_face, blk_edge);
    text(85, 43, '(-)', 'FontSize', 8, 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    draw_arrow([80, 43], [83, 43]);
    text(81, 44.5, 'refXYZ', 'FontSize', 6, 'HorizontalAlignment', 'center');

    %% ══════════════════════════════════════════════════════
    %  Controller Block (large)
    %  ══════════════════════════════════════════════════════
    draw_block([89, 33, 18, 22], gray_face, blk_edge);
    text(98, 32, 'Controller', 'FontSize', 9, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    
    % Controller input labels
    text(90, 53, 'takeoff flag', 'FontSize', 6, 'HorizontalAlignment', 'left');
    text(90, 49, 'XYZ error', 'FontSize', 6, 'HorizontalAlignment', 'left');
    text(90, 45, 'VXYZ', 'FontSize', 6, 'HorizontalAlignment', 'left');
    text(90, 41, 'ItoB', 'FontSize', 6, 'HorizontalAlignment', 'left');
    text(90, 37, 'yaw error', 'FontSize', 6, 'HorizontalAlignment', 'left');
    text(90, 34.5, 'EulerAngles', 'FontSize', 6, 'HorizontalAlignment', 'left');
    
    % Controller output labels
    text(104, 49, 'Thrust', 'FontSize', 6, 'HorizontalAlignment', 'left');
    text(104, 37, 'M (body)', 'FontSize', 6, 'HorizontalAlignment', 'left');
    
    % Arrows into controller
    draw_arrow([87, 43], [89, 43]);  % from sum

    %% ══════════════════════════════════════════════════════
    %  Force and Environment Block
    %  ══════════════════════════════════════════════════════
    draw_block([110, 42, 16, 12], gray_face, blk_edge);
    text(118, 40, 'Force and Environment', 'FontSize', 7, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    
    text(111, 52, 'Thrust', 'FontSize', 6, 'HorizontalAlignment', 'left');
    text(111, 48, 'ItoB Matrix', 'FontSize', 6, 'HorizontalAlignment', 'left');
    text(111, 44, 'takeoff flag', 'FontSize', 6, 'HorizontalAlignment', 'left');
    text(123, 52, 'F (body)', 'FontSize', 6, 'HorizontalAlignment', 'left');
    text(123, 44, 'M (body)', 'FontSize', 6, 'HorizontalAlignment', 'left');
    
    % Arrows: Controller → Force and Environment
    draw_arrow([107, 49], [110, 49]);
    draw_arrow([107, 37], [108, 37]);
    line([108, 108], [37, 44], 'Color', 'k', 'LineWidth', 0.8);
    draw_arrow([108, 44], [110, 44]);

    %% ══════════════════════════════════════════════════════
    %  F (Body Frame) / M (Body Frame) → 6dof_system
    %  ══════════════════════════════════════════════════════
    
    % F (Body Frame) label
    draw_arrow([126, 52], [131, 52]);
    text(128, 53.5, 'F (Body Frame)', 'FontSize', 6, 'HorizontalAlignment', 'center');
    
    % M (Body Frame) label
    draw_arrow([126, 44], [131, 44]);
    text(128, 45.5, 'M (Body Frame)', 'FontSize', 6, 'HorizontalAlignment', 'center');

    %% ══════════════════════════════════════════════════════
    %  6dof_system Block (right side)
    %  ══════════════════════════════════════════════════════
    draw_block([131, 25, 16, 35], gray_face, blk_edge);
    text(139, 22, '6dof\_system', 'FontSize', 9, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    
    % 6dof_system output labels
    text(145, 56, 'XYZ', 'FontSize', 7, 'HorizontalAlignment', 'left');
    text(145, 50, 'EulerAngles', 'FontSize', 7, 'HorizontalAlignment', 'left');
    text(145, 44, 'ItoB Matrix', 'FontSize', 7, 'HorizontalAlignment', 'left');
    text(145, 38, 'VXYZ', 'FontSize', 7, 'HorizontalAlignment', 'left');
    text(145, 32, 'Euler\_rates', 'FontSize', 7, 'HorizontalAlignment', 'left');
    
    % 6dof_system input labels
    text(132, 49, 'XYZ\_0', 'FontSize', 6, 'HorizontalAlignment', 'left');
    text(132, 43, 'Euler\_0', 'FontSize', 6, 'HorizontalAlignment', 'left');
    text(132, 37, 'body\_rate\_0', 'FontSize', 6, 'HorizontalAlignment', 'left');

    %% ══════════════════════════════════════════════════════
    %  Initial Condition Blocks (left of 6dof_system)
    %  ══════════════════════════════════════════════════════
    
    % XYZ_0
    draw_block([115, 29, 12, 4], blk_face, blk_edge);
    text(121, 31, 'XYZ\_0', 'FontSize', 8, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    draw_arrow([127, 31], [131, 31]);
    text(128, 32.5, 'XYZ\_0', 'FontSize', 6, 'HorizontalAlignment', 'center');
    
    % Euler_0
    draw_block([115, 23, 12, 4], blk_face, blk_edge);
    text(121, 25, 'Euler\_0', 'FontSize', 8, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    draw_arrow([127, 25], [131, 25]);
    text(128, 26.5, 'Euler\_0', 'FontSize', 6, 'HorizontalAlignment', 'center');
    
    % body_rate_0
    draw_block([115, 17, 12, 4], blk_face, blk_edge);
    text(121, 19, 'body\_rate\_0', 'FontSize', 8, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    draw_arrow([127, 19], [131, 19]);
    text(128, 20.5, 'body\_rate\_0', 'FontSize', 6, 'HorizontalAlignment', 'center');

    %% ══════════════════════════════════════════════════════
    %  Yaw Tracker Block
    %  ══════════════════════════════════════════════════════
    draw_block([52, 20, 12, 10], blk_face, blk_edge);
    text(58, 28, 'TR', 'FontSize', 6, 'HorizontalAlignment', 'center');
    text(58, 25, 'pos', 'FontSize', 6, 'HorizontalAlignment', 'center');
    text(58, 22, 'psi', 'FontSize', 6, 'HorizontalAlignment', 'center');
    draw_fcn_icon(55, 25, fcn_color);
    text(60, 22, 'fcn', 'FontSize', 7, 'HorizontalAlignment', 'center');
    text(58, 17, 'Yaw Tracker', 'FontSize', 8, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    
    % Output: delta_psi
    draw_arrow([64, 25], [70, 25]);
    text(67, 26.5, 'delta\_psi', 'FontSize', 6, 'HorizontalAlignment', 'center');
    % Connect to Controller yaw error
    line([70, 88], [25, 25], 'Color', 'k', 'LineWidth', 0.8);
    line([88, 88], [25, 37], 'Color', 'k', 'LineWidth', 0.8);
    draw_arrow([88, 37], [89, 37]);

    %% ══════════════════════════════════════════════════════
    %  Feedback Loop (bottom - Bus Creator)
    %  ══════════════════════════════════════════════════════
    
    % Bus Creator at bottom center (thick vertical bar)
    rectangle('Position', [75, 8, 1.5, 8], 'FaceColor', 'k', 'EdgeColor', 'k');
    
    % Feedback lines from 6dof_system outputs back to controller inputs
    % Main feedback line along bottom
    line([147, 147], [38, 5], 'Color', 'k', 'LineWidth', 0.8);
    line([147, 76], [5, 5], 'Color', 'k', 'LineWidth', 0.8);
    line([76, 76], [5, 8], 'Color', 'k', 'LineWidth', 0.8);
    
    % From bus creator back up to controller
    line([76, 76], [16, 34], 'Color', 'k', 'LineWidth', 0.8);
    
    % EulerAngles feedback to controller
    line([76, 89], [34.5, 34.5], 'Color', 'k', 'LineWidth', 0.8);

    hold off;
    
    fprintf('Control model visualization complete.\n');
    fprintf('Figure generated matching reference: control_model_6dof.png\n');
end

%% ════════════════════════════════════════════════════════════
%  Drawing Helpers
%  ════════════════════════════════════════════════════════════

function draw_block(rect, face_color, edge_color)
% DRAW_BLOCK  Draw a rectangular block. rect = [x, y, w, h]
    rectangle('Position', rect, 'FaceColor', face_color, ...
              'EdgeColor', edge_color, 'LineWidth', 1.0);
end

function draw_circle(cx, cy, r, face_color, edge_color)
% DRAW_CIRCLE  Draw a circle at (cx,cy) with radius r.
    theta = linspace(0, 2*pi, 50);
    fill(cx + r*cos(theta), cy + r*sin(theta), face_color, ...
         'EdgeColor', edge_color, 'LineWidth', 1.0);
end

function draw_arrow(from, to)
% DRAW_ARROW  Draw an arrow from [x1,y1] to [x2,y2].
    dx = to(1) - from(1);
    dy = to(2) - from(2);
    annotation('arrow', ...
        [from(1)/150, to(1)/150], ...
        [from(2)/75,  to(2)/75], ...
        'HeadLength', 6, 'HeadWidth', 5, 'Color', 'k', 'LineWidth', 0.8);
end

function draw_fcn_icon(x, y, color)
% DRAW_FCN_ICON  Draw a small MATLAB function icon (triangle).
    fill([x-1.5, x+1.5, x], [y-1, y-1, y+1.5], color, ...
         'EdgeColor', color, 'LineWidth', 0.5);
end

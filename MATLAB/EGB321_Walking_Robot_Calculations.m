%% EGB321 Assignment 1 Linkage Simulation
%
% All angles are taken ccw from the positive x-axis
% Real components are in the x-direction, Imaginary are in the y-direction
%
% a, b, c, d, p: Length of the links
% theta: Angle between link and positive x-axis
% delta: Angle between link b and link p
% omega: Angular velocity of link
% alpha: Angular acceleration of link

clear; close all; clc;

% Front Linkage
a_f = 18;                       % mm, length of front link a
b_f = 90;                       % mm, length of front link b
c_f = 44;                       % mm, length of front link c
d_f_end = -80 + 40j;            % mm, x-y position link d end
d_f = abs(d_f_end);             % mm, length of front link d
p_f = 74;                       % mm, length of front link p
delta_f = 36;                   % degrees, angle from link b and link p
gamma_f = angle(d_f_end);       % rad, angle ground link is rotated around global x-axis so that theta1 = 0
gamma_f_deg = rad2deg(gamma_f);

theta2_f = 60;                  % degrees, angle between local x-axis and link a
omega2_f = 12.56;               % rad/s, angular velocity of link a
alpha2_f = 0;                   % rad/s^2, angular acceleration of link a

% Rear Linkage
a_r = 15;                       % mm, length of rear link a
b_r = 47;                       % mm, length of rear link b
c_r = 28;                       % mm, length of rear link c
d_r_end = 35 + 40j;             % mm, x-y position link d end
d_r = abs(d_r_end);             % mm, length of rear link d
p_r = 54;                       % mm, length of rear link p
delta_r = -76;                  % degrees, angle from link b and link p
gamma_r = angle(d_r_end);       % rad, angle ground link is rotated around global x-axis so that theta1 = 0
gamma_r_deg = rad2deg(gamma_r);

lambda = 150;                   % degrees, angle between front and rear link a in global coordinates
epsilon = lambda + (gamma_f_deg - gamma_r_deg); % degrees, the angle between the front and rear input links,
% when the front and rear linakge sets are rotated so that theta1=0.
theta2_r = theta2_f + epsilon;  % degrees, angle between local x-axis and link a
omega2_r = omega2_f;            % rad/s, angular velocity of link a
alpha2_r = alpha2_f;            % rad/s^2, angular acceleration of link a

%% Analysis at specified theta2_f
% Run analysis for front linkage at one angle
fprintf("----------------------------------------------------------------\n")
fprintf("Front Linkage\n")
fprintf("\n")
pos_vel_acc_analysis(a_f, b_f, c_f, d_f, p_f, theta2_f, omega2_f, alpha2_f, delta_f, true, true);

% Run analysis for rear linkage at one angle
fprintf("----------------------------------------------------------------\n")
fprintf("Rear Linkage\n")
fprintf("\n")
pos_vel_acc_analysis(a_r, b_r, c_r, d_r, p_r, theta2_r, omega2_r, alpha2_r, delta_r, false, true);

%% Linkage Simulation
% Define range and increment
start_angle = 0;
finish_angle = 360;
angle_increment = 1;
theta2_vec = start_angle:angle_increment:finish_angle;
beta2_vec  = theta2_vec + epsilon; % The rear input link is offset from the front by lambda

% Pre-allocate vectors for front linkage (suffix _f)
n = length(theta2_vec);
t3_f = zeros(1, n); t4_f = zeros(1, n);
o3_f = zeros(1, n); o4_f = zeros(1, n);
a3_f = zeros(1, n); a4_f = zeros(1, n);
mu_vec_f   = zeros(1,n);

R_A_vec_f  = complex(zeros(1,n));
R_B_vec_f  = complex(zeros(1,n));
R_PA_vec_f = complex(zeros(1,n));
R_P_vec_f  = complex(zeros(1,n));

V_A_vec_f  = complex(zeros(1,n));
V_B_vec_f  = complex(zeros(1,n));
V_PA_vec_f = complex(zeros(1,n));
V_P_vec_f  = complex(zeros(1,n));

A_A_vec_f  = complex(zeros(1,n));
A_B_vec_f  = complex(zeros(1,n));
A_PA_vec_f = complex(zeros(1,n));
A_P_vec_f  = complex(zeros(1,n));

% Pre-allocate vectors for rear linkage (suffix _r)
t3_r = zeros(1, n); t4_r = zeros(1, n);
o3_r = zeros(1, n); o4_r = zeros(1, n);
a3_r = zeros(1, n); a4_r = zeros(1, n);
mu_vec_r   = zeros(1,n);

R_A_vec_r  = complex(zeros(1,n));
R_B_vec_r  = complex(zeros(1,n));
R_PA_vec_r = complex(zeros(1,n));
R_P_vec_r  = complex(zeros(1,n));

V_A_vec_r  = complex(zeros(1,n));
V_B_vec_r  = complex(zeros(1,n));
V_PA_vec_r = complex(zeros(1,n));
V_P_vec_r  = complex(zeros(1,n));

A_A_vec_r  = complex(zeros(1,n));
A_B_vec_r  = complex(zeros(1,n));
A_PA_vec_r = complex(zeros(1,n));
A_P_vec_r  = complex(zeros(1,n));

% Loop through each angle and run the function for both linkages
for i = 1:n
    % Front linkage
    [t3_f(i), t4_f(i), o3_f(i), o4_f(i), a3_f(i), a4_f(i), mu_vec_f(i), ...
     R_A_vec_f(i), R_B_vec_f(i), R_PA_vec_f(i), R_P_vec_f(i), ...
     V_A_vec_f(i), V_B_vec_f(i), V_PA_vec_f(i), V_P_vec_f(i), ...
     A_A_vec_f(i), A_B_vec_f(i), A_PA_vec_f(i), A_P_vec_f(i)] = ...
        pos_vel_acc_analysis(a_f, b_f, c_f, d_f, p_f, theta2_vec(i), omega2_f, alpha2_f, delta_f, true, false);

    % Rear linkage
    [t3_r(i), t4_r(i), o3_r(i), o4_r(i), a3_r(i), a4_r(i), mu_vec_r(i), ...
     R_A_vec_r(i), R_B_vec_r(i), R_PA_vec_r(i), R_P_vec_r(i), ...
     V_A_vec_r(i), V_B_vec_r(i), V_PA_vec_r(i), V_P_vec_r(i), ...
     A_A_vec_r(i), A_B_vec_r(i), A_PA_vec_r(i), A_P_vec_r(i)] = ...
        pos_vel_acc_analysis(a_r, b_r, c_r, d_r, p_r, beta2_vec(i), omega2_r, alpha2_r, delta_r, false, false);
end

%% Plots for the simulation data
% Plot Settings
axis_label_fontsize = 16;
axis_fontname = 'Times New Roman';
subtitle_fontsize = 16;
xlim_range = [min(theta2_vec), max(theta2_vec)]; % limit for x-axis

% Link angles and velocities
leg_link_angular_vel(theta2_vec, o3_f, o3_r, epsilon, axis_label_fontsize, axis_fontname, subtitle_fontsize, xlim_range)

additional_figures(theta2_vec, t3_f, t4_f, t3_r, t4_r, o3_f, o4_f, o3_r, o4_r, a3_f, a4_f, a3_r, a4_r, mu_vec_f, mu_vec_r, axis_label_fontsize, axis_fontname, subtitle_fontsize, xlim_range)

theta2_index = theta2_f;
% Front Linkage Schematic in Local Coordinates
plot_linkage_schematic_local(R_A_vec_f, R_B_vec_f, R_P_vec_f, d_f, theta2_vec, theta2_index, 'Linkage Schematic (Front)', axis_label_fontsize, axis_fontname, subtitle_fontsize);
% Front Linkage Schematic in Local Coordinates
plot_linkage_schematic_local(R_A_vec_r, R_B_vec_r, R_P_vec_r, d_r, theta2_vec, theta2_index, 'Linkage Schematic (Rear)', axis_label_fontsize, axis_fontname, subtitle_fontsize);

% Linkage Schematic in Global Coordinates
plot_linkage_schematic_global(R_A_vec_f, R_B_vec_f, R_P_vec_f, d_f, R_A_vec_r, R_B_vec_r, R_P_vec_r, d_r, theta2_vec, theta2_index, gamma_f, gamma_r, 'Rotated Linkages (Front & Rear)', axis_label_fontsize, axis_fontname, subtitle_fontsize);

%% Kinematic Analysis
function [theta3, theta4, omega3, omega4, alpha3, alpha4, mu, ...
          R_A, R_B, R_PA, R_P, ...
          V_A, V_B, V_PA, V_P, ...
          A_A, A_B, A_PA, A_P] = pos_vel_acc_analysis(a, b, c, d, p, ...
          theta2, omega2, alpha2, delta, open_config, display_results)
    % Kinematic analysis of open 4-bar linkage
    % Analysis is done in the local coordinate system of each linkage,
    % where theta1 = 0. Gamma is the angle between the local and global
    % x-axis, so to get global position and vectors they are
    % rotated by gamma, or translated by gamma around the input axis.

    % Positional Analysis
    K1 = d/a;
    K2 = d/c;
    K3 = (a^2-b^2+c^2+d^2) / (2*a*c);
    
    A_pos = cosd(theta2) - K1 - K2*cosd(theta2) + K3;
    B_pos = -2*sind(theta2);
    C_pos = K1 - (K2 + 1)*cosd(theta2) + K3;
    
    if open_config
        theta4 = 2*atand((-B_pos - sqrt(B_pos^2 - 4*A_pos*C_pos)) / (2*A_pos));
    else
        theta4 = 2*atand((-B_pos + sqrt(B_pos^2 - 4*A_pos*C_pos)) / (2*A_pos));
    end

    theta3 = asind((-a*sind(theta2) + c*sind(theta4)) / b);

    % Ensure theta3 and theta4 are positive angles
    theta3 = mod(theta3, 360);
    if theta3 < 0
        theta3 = theta3 + 360;
    end

    theta4 = mod(theta4, 360);
    if theta4 < 0
        theta4 = theta4 + 360;
    end
    
    R_A  = a * (cosd(theta2) + 1i*sind(theta2));
    R_BC = c * (cosd(theta4) + 1i*sind(theta4));
    R_B = R_BC + d;
    R_PA = p * (cosd(theta3+delta) + 1i*sind(theta3+delta));
    R_P  = R_A + R_PA;
    
    % Internal Angle
    if open_config
        phi2 = theta3 - theta2 + 180;
        phi3 = theta4 - theta3;
        phi4 = 180 - theta4;
    else
        phi2 = theta2 - theta3 + 180;
        phi3 = theta3 - theta4;
        phi4 = theta4 - 180;
    end

    % Transmission Angle
    if phi3 > 90
        mu = 180 - phi3;
    else
        mu = phi3;
    end
    
    % Velocity Analysis
    omega3 = (a*omega2*sind(theta2 - theta4)) / (b*sind(theta4 - theta3));
    omega4 = (a*omega2*sind(theta2 - theta3)) / (c*sind(theta4 - theta3));

    V_A  = a*omega2*(-sind(theta2) + 1i*cosd(theta2));
    V_B  = c*omega4*(-sind(theta4) + 1i*cosd(theta4));
    V_PA = p*omega3*(-sind(theta3 + delta) + 1i*cosd(theta3 + delta));
    V_P  = V_PA + V_A;
    
    % Acceleration Analysis
    A_acc = c*sind(theta4);
    B_acc = b*sind(theta3);
    C_acc = a*alpha2*sind(theta2) + a*omega2^2*cosd(theta2) ...
          + b*omega3^2*cosd(theta3) - c*omega4^2*cosd(theta4);
    
    D_acc = c*cosd(theta4);
    E_acc = b*cosd(theta3);
    F_acc = a*alpha2*cosd(theta2) - a*omega2^2*sind(theta2) ...
          - b*omega3^2*sind(theta3) + c*omega4^2*sind(theta4);
    
    alpha3 = (C_acc*D_acc - A_acc*F_acc) / (A_acc*E_acc - B_acc*D_acc);
    alpha4 = (C_acc*E_acc - B_acc*F_acc) / (A_acc*E_acc - B_acc*D_acc);

    A_A  = -a * omega2^2 * (cosd(theta2) + 1i*sind(theta2)) ...
          + a * alpha2 * (-sind(theta2) + 1i*cosd(theta2));
    A_B  = -c * omega4^2 * (cosd(theta4) + 1i*sind(theta4)) ...
          + c * alpha4 * (-sind(theta4) + 1i*cosd(theta4));
    A_PA = -p * omega3^2 * (cosd(theta3 + delta) + 1i*sind(theta3 + delta)) ...
          + p * alpha3 * (-sind(theta3 + delta) + 1i*cosd(theta3 + delta));
    A_P  = A_PA + A_A;
    
    if display_results
        
        % Link Angles
        fprintf('Link angles (local frame):\n');
        fprintf('  theta2 = %.4f deg\n', theta2);
        fprintf('  theta3 = %.4f deg\n', theta3);
        fprintf('  theta4 = %.4f deg\n', theta4);
        fprintf('\n');

        % Positions (complex vectors)
        fprintf('Positions (local frame):\n');
        fprintf('  R_A  = %.4f %+.4fi mm\n', real(R_A), imag(R_A));
        fprintf('  R_B  = %.4f %+.4fi mm\n', real(R_B), imag(R_B));
        fprintf('  R_PA = %.4f %+.4fi mm\n', real(R_PA), imag(R_PA));
        fprintf('  R_P  = %.4f %+.4fi mm\n', real(R_P), imag(R_P));
        
        % Polar form for R_P
        r_RP = abs(R_P);
        ang_RP = atan2d(imag(R_P), real(R_P)); % degrees
        if ang_RP < 0
            ang_RP = ang_RP + 360;
        end
        fprintf('  R_P (polar) = %.4f mm @ %.4f deg (local frame)\n', r_RP, ang_RP);
        fprintf('\n');

        % Internal angles
        fprintf('Internal angles:\n');
        fprintf('  phi2 = %.4f deg\n', phi2);
        fprintf('  phi3 = %.4f deg\n', phi3);
        fprintf('  phi4 = %.4f deg\n', phi4);
        fprintf('\n');
        
        % Transmission angle
        fprintf('Transmission angle\n');
        fprintf('  mu = %.4f deg\n', mu);
        fprintf('\n');

        % Angular velocities
        fprintf('Angular velocities:\n');
        fprintf('  omega2 = %.4f rad/s\n', omega2);
        fprintf('  omega3 = %.4f rad/s\n', omega3);
        fprintf('  omega4 = %.4f rad/s\n', omega4);
        fprintf('\n');

        % Velocities (complex)
        fprintf('Velocities (local frame):\n');
        fprintf('  V_A  = %.4f %+.4fi mm/s\n', real(V_A), imag(V_A));
        fprintf('  V_B  = %.4f %+.4fi mm/s\n', real(V_B), imag(V_B));
        fprintf('  V_PA = %.4f %+.4fi mm/s\n', real(V_PA), imag(V_PA));
        fprintf('  V_P  = %.4f %+.4fi mm/s\n', real(V_P), imag(V_P));
        
        % Polar form for V_P
        mag_VP = abs(V_P);
        ang_VP = atan2d(imag(V_P), real(V_P)); % degrees
        if ang_VP < 0
            ang_VP = ang_VP + 360;
        end
        fprintf('  V_P (polar) = %.4f mm/s @ %.4f deg (local frame)\n', mag_VP, ang_VP);
        fprintf('\n');

        % Angular accelerations
        fprintf('Angular accelerations:\n');
        fprintf('  alpha2 = %.4f rad/s^2\n', alpha2);
        fprintf('  alpha3 = %.4f rad/s^2\n', alpha3);
        fprintf('  alpha4 = %.4f rad/s^2\n', alpha4);
        fprintf('\n');

        % Accelerations (complex)
        fprintf('Accelerations (local frame):\n');
        fprintf('  A_A  = %.4f %+.4fi mm/s^2\n', real(A_A), imag(A_A));
        fprintf('  A_B  = %.4f %+.4fi mm/s^2\n', real(A_B), imag(A_B));
        fprintf('  A_PA = %.4f %+.4fi mm/s^2\n', real(A_PA), imag(A_PA));
        fprintf('  A_P  = %.4f %+.4fi mm/s^2\n', real(A_P), imag(A_P));
        
        % Polar form for A_P
        mag_AP = abs(A_P);
        ang_AP = atan2d(imag(A_P), real(A_P)); % degrees
        if ang_AP < 0
            ang_AP = ang_AP + 360;
        end
        fprintf('  A_P (polar) = %.4f mm/s^2 @ %.4f deg (local frame)\n', mag_AP, ang_AP);
        fprintf('----------------------------------------------------------------\n');
        fprintf('\n');
        fprintf('\n');
    end
end

%% Leg Link Angular Velocities
function [] = leg_link_angular_vel(theta2_vec, o3_f, o3_r, epsilon, axis_label_fontsize, axis_fontname, subtitle_fontsize, xlim_range)
    % Angular Velocities figure (front top, rear bottom)
    figure('Name','Leg Link Angular Velocities','NumberTitle','off');
    
    % Top subplot: front omega3 vs theta2
    subplot(2,1,1);
    plot(theta2_vec, o3_f, 'b-','LineWidth',1.2);
    xlabel('\theta_2 (deg)','FontSize',axis_label_fontsize,'FontName',axis_fontname);
    ylabel('\omega_3 (rad/s)','FontSize',axis_label_fontsize,'FontName',axis_fontname);
    xlim(xlim_range); grid on;
    xticks(xlim_range(1):15:xlim_range(2));
    title('Front','FontName',axis_fontname,'FontSize',subtitle_fontsize);
    
    % Bottom subplot: rear omega3 vs beta2 (theta2 + lambda)
    subplot(2,1,2);
    beta2_vec = theta2_vec + epsilon;
    plot(beta2_vec, o3_r, 'r-','LineWidth',1.2);
    xlabel('\beta_2 (deg)','FontSize',axis_label_fontsize,'FontName',axis_fontname);
    ylabel('\omega_3 (rad/s)','FontSize',axis_label_fontsize,'FontName',axis_fontname);
    xlim([min(beta2_vec), max(beta2_vec)]); grid on;
    tick_inc = 15; % x-tick increment
    xticks(ceil(min(beta2_vec)/tick_inc)*tick_inc:tick_inc:floor(max(beta2_vec)/tick_inc)*tick_inc);
    title('Rear','FontName',axis_fontname,'FontSize',subtitle_fontsize);
end

%% Link angles (theta3 & theta4)
function [] = additional_figures(theta2_vec, t3_f, t4_f, t3_r, t4_r, o3_f, o4_f, o3_r, o4_r, a3_f, a4_f, a3_r, a4_r, mu_vec_f, mu_vec_r, ...
    axis_label_fontsize, axis_fontname, subtitle_fontsize, xlim_range)

    % x-tick increment (degrees)
    tick_inc = 30;

    % Define subplot position/size variables so each subplot has the same width and height
    % Layout parameters (normalized units): left, bottom, width, height
    left_col = 0.08;
    mid_col  = 0.525;
    right_col = 0.08; % unused but kept for clarity
    bottom_row1 = 0.55;
    bottom_row2 = 0.08;
    sb_width = 0.36;   % same width for all subplots
    sb_height = 0.36;  % same height for all subplots

    figure('Name','Link Angles (\theta_3 and \theta_4)','NumberTitle','off');
    % Front theta3 & theta4
    subplot('Position',[left_col, bottom_row1, sb_width, sb_height]);
    plot(theta2_vec, t3_f, 'b-','LineWidth',1.2);
    ylabel('\theta_3 (deg)','FontSize',axis_label_fontsize,'FontName',axis_fontname);
    xlim(xlim_range); grid on;
    xticks(xlim_range(1):tick_inc:xlim_range(2));
    title('Front','FontName',axis_fontname,'FontSize',subtitle_fontsize);
    
    subplot('Position',[left_col, bottom_row2, sb_width, sb_height]);
    plot(theta2_vec, t4_f, 'r-','LineWidth',1.2);
    ylabel('\theta_4 (deg)','FontSize',axis_label_fontsize,'FontName',axis_fontname);
    xlabel('\theta_2 (deg)','FontSize',axis_label_fontsize,'FontName',axis_fontname);
    xlim(xlim_range); grid on;
    xticks(xlim_range(1):tick_inc:xlim_range(2));
    
    % Rear theta3 & theta4
    subplot('Position',[mid_col, bottom_row1, sb_width, sb_height]);
    plot(theta2_vec, t3_r, 'b-','LineWidth',1.2);
    ylabel('\theta_3 (deg)','FontSize',axis_label_fontsize,'FontName',axis_fontname);
    title('Rear','FontName',axis_fontname,'FontSize',subtitle_fontsize);
    xlim(xlim_range); grid on;
    xticks(xlim_range(1):tick_inc:xlim_range(2));
    
    subplot('Position',[mid_col, bottom_row2, sb_width, sb_height]);
    plot(theta2_vec, t4_r, 'r-','LineWidth',1.2);
    ylabel('\theta_4 (deg)','FontSize',axis_label_fontsize,'FontName',axis_fontname);
    xlabel('\theta_2 (deg)','FontSize',axis_label_fontsize,'FontName',axis_fontname);
    xlim(xlim_range); grid on;
    xticks(xlim_range(1):tick_inc:xlim_range(2));
    
    
    % Angular velocities (alpha3 & alpha4)
    % Reuse same subplot sizing variables for consistent appearance
    figure('Name','Angular Velocities (\omega_3 and \omega_4)','NumberTitle','off');
    % Front omega3 & omega4
    subplot('Position',[left_col, bottom_row1, sb_width, sb_height]);
    plot(theta2_vec, o3_f, 'b-','LineWidth',1.2);
    ylabel('\omega_3 (rad/s)','FontSize',axis_label_fontsize,'FontName',axis_fontname);
    xlim(xlim_range); grid on;
    xticks(xlim_range(1):tick_inc:xlim_range(2));
    title('Front','FontName',axis_fontname,'FontSize',subtitle_fontsize);
    
    subplot('Position',[left_col, bottom_row2, sb_width, sb_height]);
    plot(theta2_vec, o4_f, 'r-','LineWidth',1.2);
    ylabel('\omega_4 (rad/s)','FontSize',axis_label_fontsize,'FontName',axis_fontname);
    xlabel('\theta_2 (deg)','FontSize',axis_label_fontsize,'FontName',axis_fontname);
    xlim(xlim_range); grid on;
    xticks(xlim_range(1):tick_inc:xlim_range(2));
    
    % Rear omega3 & omega4
    subplot('Position',[mid_col, bottom_row1, sb_width, sb_height]);
    plot(theta2_vec, o3_r, 'b-','LineWidth',1.2);
    ylabel('\omega_3 (rad/s)','FontSize',axis_label_fontsize,'FontName',axis_fontname);
    title('Rear','FontName',axis_fontname,'FontSize',subtitle_fontsize);
    xlim(xlim_range); grid on;
    xticks(xlim_range(1):tick_inc:xlim_range(2));
    
    subplot('Position',[mid_col, bottom_row2, sb_width, sb_height]);
    plot(theta2_vec, o4_r, 'r-','LineWidth',1.2);
    ylabel('\omega_4 (rad/s)','FontSize',axis_label_fontsize,'FontName',axis_fontname);
    xlabel('\theta_2 (deg)','FontSize',axis_label_fontsize,'FontName',axis_fontname);
    xlim(xlim_range); grid on;
    xticks(xlim_range(1):tick_inc:xlim_range(2));
    
    
    % Angular accelerations (alpha3 & alpha4)
    figure('Name','Angular Accelerations (\alpha_3 and \alpha_4)','NumberTitle','off');
    % Front alpha3 & alpha4
    subplot('Position',[left_col, bottom_row1, sb_width, sb_height]);
    plot(theta2_vec, a3_f, 'b-','LineWidth',1.2);
    ylabel('\alpha_3 (rad/s^2)','FontSize',axis_label_fontsize,'FontName',axis_fontname);
    xlim(xlim_range); grid on;
    xticks(xlim_range(1):tick_inc:xlim_range(2));
    title('Front','FontName',axis_fontname,'FontSize',subtitle_fontsize);
    
    subplot('Position',[left_col, bottom_row2, sb_width, sb_height]);
    plot(theta2_vec, a4_f, 'r-','LineWidth',1.2);
    ylabel('\alpha_4 (rad/s^2)','FontSize',axis_label_fontsize,'FontName',axis_fontname);
    xlabel('\theta_2 (deg)','FontSize',axis_label_fontsize,'FontName',axis_fontname);
    xlim(xlim_range); grid on;
    xticks(xlim_range(1):tick_inc:xlim_range(2));
    
    % Rear alpha3 & alpha4
    subplot('Position',[mid_col, bottom_row1, sb_width, sb_height]);
    plot(theta2_vec, a3_r, 'b-','LineWidth',1.2);
    ylabel('\alpha_3 (rad/s^2)','FontSize',axis_label_fontsize,'FontName',axis_fontname);
    title('Rear','FontName',axis_fontname,'FontSize',subtitle_fontsize);
    xlim(xlim_range); grid on;
    xticks(xlim_range(1):tick_inc:xlim_range(2));
    
    subplot('Position',[mid_col, bottom_row2, sb_width, sb_height]);
    plot(theta2_vec, a4_r, 'r-','LineWidth',1.2);
    ylabel('\alpha_4 (rad/s^2)','FontSize',axis_label_fontsize,'FontName',axis_fontname);
    xlabel('\theta_2 (deg)','FontSize',axis_label_fontsize,'FontName',axis_fontname);
    xlim(xlim_range); grid on;
    xticks(xlim_range(1):tick_inc:xlim_range(2));
        
    % Mu (front and rear)
    figure('Name','Mu (front and rear)','NumberTitle','off');
    subplot('Position',[left_col, bottom_row1, sb_width, sb_height]);
    plot(theta2_vec, mu_vec_f, 'b-','LineWidth',1.2);
    ylabel('\mu (deg)','FontSize',axis_label_fontsize,'FontName',axis_fontname);
    xlabel('\theta_2 (deg)','FontSize',axis_label_fontsize,'FontName',axis_fontname);
    title('Front','FontName',axis_fontname,'FontSize',subtitle_fontsize);
    xlim(xlim_range); grid on;
    xticks(xlim_range(1):tick_inc:xlim_range(2));
    
    subplot('Position',[mid_col, bottom_row1, sb_width, sb_height]);
    plot(theta2_vec, mu_vec_r, 'r-','LineWidth',1.2);
    ylabel('\mu (deg)','FontSize',axis_label_fontsize,'FontName',axis_fontname);
    xlabel('\theta_2 (deg)','FontSize',axis_label_fontsize,'FontName',axis_fontname);
    title('Rear','FontName',axis_fontname,'FontSize',subtitle_fontsize);
    xlim(xlim_range); grid on;
    xticks(xlim_range(1):tick_inc:xlim_range(2));
end

%% Linkage Schematic in Local Coordinates
% Function to plot linkage schematic in the local coordinate system
function plot_linkage_schematic_local(R_A_vec, R_B_vec, R_P_vec, d, theta2_vec, theta2_index, fig_name, axis_label_fontsize, axis_fontname, subtitle_fontsize)
    figure('Name',fig_name,'NumberTitle','off');
    hold on; axis equal; grid on;
    xlabel('X (mm)','FontSize',axis_label_fontsize,'FontName',axis_fontname);
    ylabel('Y (mm)','FontSize',axis_label_fontsize,'FontName',axis_fontname);
    title('Linkage Schematic','FontSize',subtitle_fontsize,'FontName',axis_fontname);

    % Clamp index to valid range
    theta2_index = max(1,min(theta2_index,numel(theta2_vec)));

    % Origin and fixed point C
    O   = [0,0];
    R_C = [d, 0];

    % Extract points for linkage at chosen index, handle complex parts
    R_A = [real(R_A_vec(theta2_index)), imag(R_A_vec(theta2_index))];
    R_B = [real(R_B_vec(theta2_index)), imag(R_B_vec(theta2_index))];
    R_P = [real(R_P_vec(theta2_index)), imag(R_P_vec(theta2_index))];

    % Plot lines
    plot([O(1),   R_C(1)], [O(2),   R_C(2)], 'k--', 'LineWidth', 1.2);   % O -> R_C
    plot([O(1),   R_A(1)], [O(2),   R_A(2)], 'r-',  'LineWidth', 1.5);   % O -> R_A
    plot([R_A(1), R_B(1)], [R_A(2), R_B(2)], 'b-',  'LineWidth', 1.5);   % R_A -> R_B
    plot([R_C(1), R_B(1)], [R_C(2), R_B(2)], 'g-',  'LineWidth', 1.5);   % R_C -> R_B
    plot([R_A(1), R_P(1)], [R_A(2), R_P(2)], 'm-',  'LineWidth', 1.5);   % R_A -> R_P
    plot([R_B(1), R_P(1)], [R_B(2), R_P(2)], 'm-',  'LineWidth', 1.2);   % R_B -> R_P

    % Plot points
    plot(O(1),   O(2),  'ko', 'MarkerFaceColor', 'k');
    plot(R_A(1), R_A(2),'ko', 'MarkerFaceColor', 'k');
    plot(R_B(1), R_B(2),'ko', 'MarkerFaceColor', 'k');
    plot(R_P(1), R_P(2),'ko', 'MarkerFaceColor', 'k');
    plot(R_C(1), R_C(2),'ko', 'MarkerFaceColor', 'k');

    
    % Plot trajectories (paths) of A, B, and P over all theta2_vec in black
    % Extract full trajectories (real, imag) and ensure real arrays
    A_traj = [real(R_A_vec(:)), imag(R_A_vec(:))];
    B_traj = [real(R_B_vec(:)), imag(R_B_vec(:))];
    P_traj = [real(R_P_vec(:)), imag(R_P_vec(:))];

    % Plot as black line segments for each trajectory
    nPts = numel(theta2_vec);
    for k = 1:(nPts-1)
        % A
        plot([A_traj(k,1), A_traj(k+1,1)], [A_traj(k,2), A_traj(k+1,2)], 'k-', 'LineWidth', 1.2);
        % B
        plot([B_traj(k,1), B_traj(k+1,1)], [B_traj(k,2), B_traj(k+1,2)], 'k-', 'LineWidth', 1.2);
        % P
        plot([P_traj(k,1), P_traj(k+1,1)], [P_traj(k,2), P_traj(k+1,2)], 'k-', 'LineWidth', 1.2);
    end

    % Add padding to the x and y limits, accounting for full trajectories
    % Combine current linkage points and full trajectory extents
    allX = [O(1), R_A(1), R_B(1), R_P(1), R_C(1), A_traj(:,1)', B_traj(:,1)', P_traj(:,1)'];
    allY = [O(2), R_A(2), R_B(2), R_P(2), R_C(2), A_traj(:,2)', B_traj(:,2)', P_traj(:,2)'];
    x_max = max(allX); x_min = min(allX);
    y_max = max(allY); y_min = min(allY);

    % Relative padding fraction; ensure nonzero span with eps
    padding = 0.08;
    x_span = x_max - x_min + eps;
    y_span = y_max - y_min + eps;
    x_pad = padding * x_span;
    y_pad = padding * y_span;

    xlim([x_min - x_pad, x_max + x_pad]);
    ylim([y_min - y_pad, y_max + y_pad]);

    hold off;
end

%% Linkage Schematic in Global Coordinates
% Function to plot both front and rear linkages within the global coordinate system
function plot_linkage_schematic_global(R_A_vec_f, R_B_vec_f, R_P_vec_f, d_f, ...
                               R_A_vec_r, R_B_vec_r, R_P_vec_r, d_r, ...
                               theta2_vec, theta2_index, gamma_f, gamma_r, ...
                               fig_name, axis_label_fontsize, axis_fontname, subtitle_fontsize)
    % Clamp index to valid range
    theta2_index = max(1,min(theta2_index,numel(theta2_vec)));

    % Rotation matrices
    Rrot_f = [cos(gamma_f), -sin(gamma_f); sin(gamma_f), cos(gamma_f)];
    Rrot_r = [cos(gamma_r), -sin(gamma_r); sin(gamma_r), cos(gamma_r)];

    % Helper to extract and rotate complex point
    rotPoint = @(Rrot, z) (Rrot * [real(z); imag(z)])';

    % Origin and fixed points (before rotation)
    O   = [0,0];
    R_C_f = [d_f, 0];
    R_C_r = [d_r, 0];

    % Extract and rotate front points
    RA_f = rotPoint(Rrot_f, R_A_vec_f(theta2_index));
    RB_f = rotPoint(Rrot_f, R_B_vec_f(theta2_index));
    RP_f = rotPoint(Rrot_f, R_P_vec_f(theta2_index));
    RC_f = (Rrot_f * R_C_f')';

    % Extract and rotate rear points
    RA_r = rotPoint(Rrot_r, R_A_vec_r(theta2_index));
    RB_r = rotPoint(Rrot_r, R_B_vec_r(theta2_index));
    RP_r = rotPoint(Rrot_r, R_P_vec_r(theta2_index));
    RC_r = (Rrot_r * R_C_r')';

    O_f = (Rrot_f * O')';
    O_r = (Rrot_r * O')';

    % Debug, get angle between RA_f and RA_r
    % Compute vectors from global origin to rotated front and rear A points
    v_f = RA_f - O_f;
    v_r = RA_r - O_r;
    % Compute signed angle from v_r to v_f
    ang_between = rad2deg(atan2(det([v_r; v_f]), dot(v_r, v_f)));
    %fprintf('Debug: Angle between RA_f & RA_r: %.1f degrees\n', ang_between);

    % Create figure
    figure('Name',fig_name,'NumberTitle','off');
    hold on; axis equal; grid on;
    xlabel('X (mm)','FontSize',axis_label_fontsize,'FontName',axis_fontname);
    ylabel('Y (mm)','FontSize',axis_label_fontsize,'FontName',axis_fontname);
    title('Front (blue) and Rear (red) Linkages (rotated)','FontSize',subtitle_fontsize,'FontName',axis_fontname);

    % Plot front linkage (blue/magenta)
    plot([O_f(1), RC_f(1)], [O_f(2), RC_f(2)], 'k--', 'LineWidth', 1.0);
    plot([O_f(1), RA_f(1)], [O_f(2), RA_f(2)], 'b-', 'LineWidth', 1.5);
    plot([RA_f(1), RB_f(1)], [RA_f(2), RB_f(2)], 'b-', 'LineWidth', 1.5);
    plot([RC_f(1), RB_f(1)], [RC_f(2), RB_f(2)], 'b-', 'LineWidth', 1.5);
    plot([RA_f(1), RP_f(1)], [RA_f(2), RP_f(2)], 'm-', 'LineWidth', 1.5);
    plot([RB_f(1), RP_f(1)], [RB_f(2), RP_f(2)], 'm-', 'LineWidth', 1.2);

    % Plot rear linkage (red/magenta)
    plot([O_r(1), RC_r(1)], [O_r(2), RC_r(2)], 'k--', 'LineWidth', 1.0);
    plot([O_r(1), RA_r(1)], [O_r(2), RA_r(2)], 'r-', 'LineWidth', 1.5);
    plot([RA_r(1), RB_r(1)], [RA_r(2), RB_r(2)], 'r-', 'LineWidth', 1.5);
    plot([RC_r(1), RB_r(1)], [RC_r(2), RB_r(2)], 'r-', 'LineWidth', 1.5);
    plot([RA_r(1), RP_r(1)], [RA_r(2), RP_r(2)], 'm-', 'LineWidth', 1.5);
    plot([RB_r(1), RP_r(1)], [RB_r(2), RP_r(2)], 'm-', 'LineWidth', 1.2);

    % Plot points (front)
    plot(O_f(1),   O_f(2),  'ko', 'MarkerFaceColor', 'k');
    plot(RA_f(1), RA_f(2),'bo', 'MarkerFaceColor', 'b');
    plot(RB_f(1), RB_f(2),'bo', 'MarkerFaceColor', 'b');
    plot(RP_f(1), RP_f(2),'bo', 'MarkerFaceColor', 'b');
    plot(RC_f(1), RC_f(2),'bo', 'MarkerFaceColor', 'b');

    % Plot points (rear)
    plot(O_r(1),   O_r(2),  'ko', 'MarkerFaceColor', 'k');
    plot(RA_r(1), RA_r(2),'ro', 'MarkerFaceColor', 'r');
    plot(RB_r(1), RB_r(2),'ro', 'MarkerFaceColor', 'r');
    plot(RP_r(1), RP_r(2),'ro', 'MarkerFaceColor', 'r');
    plot(RC_r(1), RC_r(2),'ro', 'MarkerFaceColor', 'r');

    % Include full trajectories for front and rear linkages when computing axis limits
    % Extract trajectories (real, imag) for front and rotate by Rrot_f
    A_traj_f = ([real(R_A_vec_f(:)), imag(R_A_vec_f(:))] * Rrot_f') ;
    B_traj_f = ([real(R_B_vec_f(:)), imag(R_B_vec_f(:))] * Rrot_f') ;
    P_traj_f = ([real(R_P_vec_f(:)), imag(R_P_vec_f(:))] * Rrot_f') ;

    % Extract trajectories (real, imag) for rear and rotate by Rrot_r
    A_traj_r = ([real(R_A_vec_r(:)), imag(R_A_vec_r(:))] * Rrot_r') ;
    B_traj_r = ([real(R_B_vec_r(:)), imag(R_B_vec_r(:))] * Rrot_r') ;
    P_traj_r = ([real(R_P_vec_r(:)), imag(R_P_vec_r(:))] * Rrot_r') ;

    % Plot trajectories in black for both front and rear
    nPts = numel(theta2_vec);

    for k = 1:(nPts-1)
        % front trajectories (rotated) - black
        plot([A_traj_f(k,1), A_traj_f(k+1,1)], [A_traj_f(k,2), A_traj_f(k+1,2)], '-', 'Color', [0 0 0], 'LineWidth', 1.2);
        plot([B_traj_f(k,1), B_traj_f(k+1,1)], [B_traj_f(k,2), B_traj_f(k+1,2)], '-', 'Color', [0 0 0], 'LineWidth', 1.2);
        plot([P_traj_f(k,1), P_traj_f(k+1,1)], [P_traj_f(k,2), P_traj_f(k+1,2)], '-', 'Color', [0 0 0], 'LineWidth', 1.2);
        % rear trajectories (rotated) - black
        plot([A_traj_r(k,1), A_traj_r(k+1,1)], [A_traj_r(k,2), A_traj_r(k+1,2)], '-', 'Color', [0 0 0], 'LineWidth', 1.2);
        plot([B_traj_r(k,1), B_traj_r(k+1,1)], [B_traj_r(k,2), B_traj_r(k+1,2)], '-', 'Color', [0 0 0], 'LineWidth', 1.2);
        plot([P_traj_r(k,1), P_traj_r(k+1,1)], [P_traj_r(k,2), P_traj_r(k+1,2)], '-', 'Color', [0 0 0], 'LineWidth', 1.2);
    end

    % Determine axis limits covering both instantaneous linkages and full trajectories
    allX = [O_f(1), RA_f(1), RB_f(1), RP_f(1), RC_f(1), ...
            O_r(1), RA_r(1), RB_r(1), RP_r(1), RC_r(1), ...
            A_traj_f(:,1)', B_traj_f(:,1)', P_traj_f(:,1)', ...
            A_traj_r(:,1)', B_traj_r(:,1)', P_traj_r(:,1)'];
    allY = [O_f(2), RA_f(2), RB_f(2), RP_f(2), RC_f(2), ...
            O_r(2), RA_r(2), RB_r(2), RP_r(2), RC_r(2), ...
            A_traj_f(:,2)', B_traj_f(:,2)', P_traj_f(:,2)', ...
            A_traj_r(:,2)', B_traj_r(:,2)', P_traj_r(:,2)'];

    x_max = max(allX); x_min = min(allX);
    y_max = max(allY); y_min = min(allY);

    % Use relative padding but ensure minimum absolute padding to avoid clipping small spans
    padding_frac = 0.08;      % relative padding fraction
    min_pad_abs = 5;         % minimum padding in same units as coordinates (e.g., mm)

    x_span = x_max - x_min + eps;
    y_span = y_max - y_min + eps;
    x_pad = max(padding_frac * x_span, min_pad_abs);
    y_pad = max(padding_frac * y_span, min_pad_abs);

    xlim([x_min - x_pad, x_max + x_pad]);
    ylim([y_min - y_pad, y_max + y_pad]);

    hold off;
end


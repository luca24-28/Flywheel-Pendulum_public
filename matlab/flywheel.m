clear;clf;close all;
x0 = [deg2rad(5); 5; 3; 1]
%system parameters
m_f = .1055;
m_t = .1;
k_teff = .05;
k_eff = .061;
V = 6;
R = 16.7;
N = (35/11)^2;
L = .0013; %still need accurate value
l_c = .052;
l_b = .2037835;
J_p = .000315;
J_gt = .00253389
J_f = 0.0001334;
k_g = m_t*l_c*9.81;
J_0 = .000968;

%state space matrices
A = [0 1 0 0; (k_g/J_0) 0 0 (-k_teff/J_0); -(k_g/J_0) 0 0 k_teff*((1/J_f)+(1/J_0)); 0 0 (-(k_eff)/L) (-R/L)];
B = [0; 0; 0; V/L];
C = [1 0 0 0;
    0 0 1 0;
    0 0 0 1];
D = [0; 0; 0];

%K optimization
Q = [500 0 0 0;0 50 0 0; 0 0 0.0000001 0;0 0 0 1];
Re = [10];
K = lqr(A, B, Q, Re);

sys = ss((A - B*K), B, C, D);
sysC = ss(A, B, C, D);
figure(1);
t = 0:0.005:5;
[theta, t, x] = initial(sys, x0, t);

plot(t, theta);

t = 0:0.0001:2;
u = 0.2*ones(size(t));
figure(2);
[y,t,x] = lsim(sys,u,t);

plot(t, y);

%discrete system
Ts = 0.002;

sysD = c2d((sysC), Ts, 'zoh');

Ad = sysD.A;
Bd = sysD.B;
Cd = sysD.C;

Kd = dlqr(Ad, Bd, Q, Re);

Bd
Kd

Q
R
%diagnostic tests for LQR gain
x_test1 = [deg2rad(5); 0; 0; 0];
x_test2 = [deg2rad(10); 1; 0; 0];
x_test3 = [deg2rad(15); 2; 0; 0];
u_test1 = -Kd*x_test1;
u_test2 = -Kd*x_test2;
u_test3 = -Kd*x_test3;
duty_test1 = 100*u_test1
duty_test2 = 100*u_test2
duty_test3 = 100*u_test3

Q1_vals = [1 2 5 10 20 50 100 200];

for n = 1:length(Q1_vals)

    Q = diag([Q1_vals(n), 2, 0.001, 1]);

    Ktest = dlqr(Ad, Bd, Q, Re);

    fprintf('Q11 = %6.1f | K = ', Q1_vals(n));
    fprintf('%9.4f ', Ktest);

    ratio = abs(Ktest(3)*25 / ...
                (Ktest(1)*deg2rad(10)));

    fprintf('| rho/theta ratio @10deg,25rad/s = %.3f\n', ...
            ratio);
end

tests = [ ...
    deg2rad(0.5)  0;
    deg2rad(1.0)  0;
    deg2rad(2.0)  0;
    deg2rad(2.0)  0.2;
    deg2rad(5.0)  0.5];

for n = 1:size(tests,1)

    x = [tests(n,1);
         tests(n,2);
         0;
         0];

    duty = 100*(-Kd*x);

    fprintf('theta = %5.2f deg, thdot = %5.2f -> duty = %7.2f %%\n',...
        rad2deg(x(1)),x(2),duty);
end



%discrete kalman filter
Gd = [0 0;
      1 0;
      0 1;
      0 0];
q_theta = 0.062729;
q_phi = 10; 

Qd = diag([q_theta, q_phi]);

sigma_theta = 0.002;
sigma_ro    = 2.0;
sigma_i     = 0.01;

Rd = diag([sigma_theta^2, ...
           sigma_ro^2, ...
           sigma_i^2]);

[Lk,P,E] = dlqe(Ad,Gd,C,Qd,Rd);


%discrete sim
Nsim = length(t);

x = zeros(4,Nsim);
xhat = zeros(4, Nsim);
u = zeros(1,Nsim);

x(:,1) = x0;
xhat(:,1) = 0;
for k = 1:Nsim-1;

    %estimate
    %xhat(:, k+1) = Ad*x(:,k) + Bd*u(k);

    % LQR control
    u(k) = -Kd*(x(:,k));

    % PWM saturation
    u(k) = max(min(u(k),1),-1);

    % discrete plant
    x(:,k+1) = Ad*x(:,k) + Bd*u(k);
    % x(4,k+1) = max(min(x(4,k+1),3),-3)

end
%check properties of Kalman gains
eig((eye(4) - Lk*C)*Ad)

max(abs(eig((eye(4) - Lk*C)*Ad)))

rank(obsv(Ad,C))
cond(obsv(Ad,C))

Ae = (eye(4)-Lk*C)*Ad;
[V,D] = eig(Ae);

diag(D)
V

%% =========================================================
%CHATGPT GENERATED!!
% FAST DISCRETE KALMAN OBSERVER SWEEP
%
% State order:
%   x = [theta; theta_dot; rho_dot; current]
%
% Measurement order:
%   y = [theta; rho_dot; current]
%
% Assumes these already exist:
%   Ad, C, Ts
%
% Primary goal:
%   Minimize dominant observer time constant
%
% Hard constraints:
%   |M21| <= max_M21
%   |M33| <= max_M33
%   ||M||_F <= max_gain_norm
%   max(abs(M(:))) <= max_abs_entry
%
% If no tuning satisfies every constraint, the script
% reports the best stable compromise.
%% =========================================================


%% ---------------------------------------------------------
% Process noise input matrix
% ----------------------------------------------------------

Gd = [0 0;
      1 0;
      0 1;
      0 0];


%% ---------------------------------------------------------
% Fixed noise parameters
% ----------------------------------------------------------

% Flywheel velocity measurement standard deviation
sigma_ro = 1.75;          % rad/s

% From previous sigma_i / q_phi sweep
q_phi   = 0.023500071;
sigma_i = 0.50118723;     % A


%% ---------------------------------------------------------
% Sweep theta-side noise parameters
% ----------------------------------------------------------

% Process noise applied to theta_dot
q_theta_vals = logspace(-6, -1, 180);

% Pendulum angle measurement standard deviation
% 0.001 rad to about 0.050 rad
sigma_theta_vals = logspace(-3, -1.3, 180);


%% ---------------------------------------------------------
% Observer constraints
% ----------------------------------------------------------

% Allowed dominant-pole magnitude
rho_min_allowed = 0.80;
rho_max_allowed = 0.98;

% Kalman gain restrictions
max_M21 = 5.0;          % angle error -> theta_dot correction
max_M33 = 1.0;          % current error -> rho_dot correction

max_gain_norm = 8.0;
max_abs_entry = 6.0;


%% ---------------------------------------------------------
% Valid-filter cost settings
%
% Time constant is the main objective.
% These are only mild gain penalties.
% ----------------------------------------------------------

w_valid_M21  = 0.002;
w_valid_M33  = 0.002;
w_valid_gain = 0.001;


%% ---------------------------------------------------------
% Best-compromise settings
% ----------------------------------------------------------

% Preferred time-constant scale
tau_ref = 0.030;      % 30 ms

% Penalties for violating constraints
w_comp_tau      = 1.0;
w_comp_M21      = 8.0;
w_comp_M33      = 8.0;
w_comp_gain     = 3.0;
w_comp_entry    = 3.0;
w_comp_rho_high = 3.0;
w_comp_rho_low  = 1.0;

% Mild penalties even before limits are exceeded
w_comp_M21_base  = 0.02;
w_comp_M33_base  = 0.02;
w_comp_gain_base = 0.01;


%% ---------------------------------------------------------
% Allocate result arrays
% ----------------------------------------------------------

nq = length(q_theta_vals);
ns = length(sigma_theta_vals);

rho_map   = NaN(nq, ns);
tau_map   = NaN(nq, ns);
M21_map   = NaN(nq, ns);
M33_map   = NaN(nq, ns);
gain_map  = NaN(nq, ns);
maxM_map  = NaN(nq, ns);

valid_map = false(nq, ns);

% Cost for filters that meet every hard constraint
cost_map = Inf(nq, ns);


%% =========================================================
% MAIN SWEEP
%% =========================================================

for i = 1:nq

    q_theta = q_theta_vals(i);

    for j = 1:ns

        sigma_theta = sigma_theta_vals(j);

        % Process covariance
        Qd = diag([q_theta, q_phi]);

        % Measurement covariance
        Rd = diag([sigma_theta^2, ...
                   sigma_ro^2, ...
                   sigma_i^2]);

        % --------------------------------------------------
        % Calculate discrete Kalman gain
        % --------------------------------------------------

        try
            [M, P, E] = dlqe(Ad, Gd, C, Qd, Rd);
        catch
            continue;
        end

        % --------------------------------------------------
        % Observer error dynamics
        %
        % xpred = Ad*xhat + Bd*u
        %
        % xhat = xpred + M*(y - C*xpred)
        %
        % Therefore:
        %
        % Ae = (I - M*C)*Ad
        % --------------------------------------------------

        Ae = (eye(size(Ad,1)) - M*C) * Ad;

        poles = eig(Ae);

        rho = max(abs(poles));

        % Ignore invalid or unstable filters
        if ~isfinite(rho) || rho <= 0 || rho >= 1
            continue;
        end

        % Dominant discrete-time time constant
        tau = -Ts / log(rho);

        % --------------------------------------------------
        % Gain metrics
        % --------------------------------------------------

        M21 = M(2,1);
        M33 = M(3,3);

        gain_norm = norm(M, 'fro');
        max_entry = max(abs(M(:)));

        % --------------------------------------------------
        % Store results
        % --------------------------------------------------

        rho_map(i,j)  = rho;
        tau_map(i,j)  = tau;

        M21_map(i,j)  = M21;
        M33_map(i,j)  = M33;

        gain_map(i,j) = gain_norm;
        maxM_map(i,j) = max_entry;


        %% -------------------------------------------------
        % HARD CONSTRAINTS
        % --------------------------------------------------

        if rho < rho_min_allowed
            continue;
        end

        if rho > rho_max_allowed
            continue;
        end

        if abs(M21) > max_M21
            continue;
        end

        if abs(M33) > max_M33
            continue;
        end

        if gain_norm > max_gain_norm
            continue;
        end

        if max_entry > max_abs_entry
            continue;
        end


        %% -------------------------------------------------
        % This is a fully valid filter
        % --------------------------------------------------

        valid_map(i,j) = true;


        %% -------------------------------------------------
        % Valid-filter cost
        %
        % Tau is the primary objective.
        % Gain penalties act only as tie-breakers.
        % --------------------------------------------------

        M21_ratio = abs(M21) / max_M21;
        M33_ratio = abs(M33) / max_M33;
        gain_ratio = gain_norm / max_gain_norm;

        gain_penalty = ...
            w_valid_M21  * M21_ratio^2 + ...
            w_valid_M33  * M33_ratio^2 + ...
            w_valid_gain * gain_ratio^2;

        cost_map(i,j) = tau + gain_penalty;

    end
end


%% =========================================================
% FIND FASTEST VALID FILTER
%% =========================================================

[min_cost, linear_idx] = min(cost_map(:));

if isfinite(min_cost)

    % ------------------------------------------------------
    % A fully valid filter exists
    % ------------------------------------------------------

    [i_best, j_best] = ind2sub(size(cost_map), linear_idx);

    best_q_theta = q_theta_vals(i_best);
    best_sigma_theta = sigma_theta_vals(j_best);

    compromise_used = false;

else

    %% =====================================================
    % NO FULLY VALID FILTER
    % Find best stable compromise
    %% =====================================================

    fprintf('\n');
    fprintf('========================================\n');
    fprintf('NO POINT MET ALL HARD CONSTRAINTS\n');
    fprintf('Searching for best compromise...\n');
    fprintf('========================================\n\n');

    compromise_cost = Inf(nq, ns);

    for i = 1:nq

        for j = 1:ns

            rho = rho_map(i,j);

            % Skip points that were invalid or unstable
            if isnan(rho) || rho <= 0 || rho >= 1
                continue;
            end

            tau = tau_map(i,j);

            M21 = abs(M21_map(i,j));
            M33 = abs(M33_map(i,j));

            gain_norm = gain_map(i,j);
            max_entry = maxM_map(i,j);


            % ------------------------------------------------
            % Normalize quantities relative to desired limits
            % ------------------------------------------------

            tau_ratio = tau / tau_ref;

            M21_ratio = M21 / max_M21;
            M33_ratio = M33 / max_M33;

            gain_ratio = gain_norm / max_gain_norm;
            entry_ratio = max_entry / max_abs_entry;


            % ------------------------------------------------
            % Constraint violations
            %
            % Zero while within limit.
            % ------------------------------------------------

            M21_violation = max(0, M21_ratio - 1);
            M33_violation = max(0, M33_ratio - 1);

            gain_violation = max(0, gain_ratio - 1);
            entry_violation = max(0, entry_ratio - 1);


            % Pole too close to instability
            if rho > rho_max_allowed
                rho_high_violation = ...
                    (rho - rho_max_allowed) / ...
                    (1 - rho_max_allowed);
            else
                rho_high_violation = 0;
            end


            % Pole faster than lower bound
            if rho < rho_min_allowed
                rho_low_violation = ...
                    (rho_min_allowed - rho) / ...
                    rho_min_allowed;
            else
                rho_low_violation = 0;
            end


            % ------------------------------------------------
            % Compromise cost
            %
            % Fast response is important, but large violations
            % of M21/M33 are strongly discouraged.
            % ------------------------------------------------

            compromise_cost(i,j) = ...
                w_comp_tau * tau_ratio^2 + ...
                w_comp_M21 * M21_violation^2 + ...
                w_comp_M33 * M33_violation^2 + ...
                w_comp_gain * gain_violation^2 + ...
                w_comp_entry * entry_violation^2 + ...
                w_comp_rho_high * rho_high_violation^2 + ...
                w_comp_rho_low * rho_low_violation^2 + ...
                w_comp_M21_base * M21_ratio^2 + ...
                w_comp_M33_base * M33_ratio^2 + ...
                w_comp_gain_base * gain_ratio^2;

        end
    end


    %% -----------------------------------------------------
    % Select best stable compromise
    % ------------------------------------------------------

    [best_compromise_cost, linear_idx] = ...
        min(compromise_cost(:));

    if ~isfinite(best_compromise_cost)
        error('No stable observer was found anywhere in the sweep.');
    end

    [i_best, j_best] = ...
        ind2sub(size(compromise_cost), linear_idx);

    best_q_theta = q_theta_vals(i_best);
    best_sigma_theta = sigma_theta_vals(j_best);

    compromise_used = true;

end


%% =========================================================
% RECOMPUTE SELECTED FILTER
%% =========================================================

Qd_best = diag([best_q_theta, q_phi]);

Rd_best = diag([best_sigma_theta^2, ...
                sigma_ro^2, ...
                sigma_i^2]);

[M_best, P_best, E_best] = ...
    dlqe(Ad, Gd, C, Qd_best, Rd_best);

Ae_best = ...
    (eye(size(Ad,1)) - M_best*C) * Ad;

poles_best = eig(Ae_best);

rho_best = max(abs(poles_best));

tau_best = -Ts / log(rho_best);

gain_norm_best = norm(M_best, 'fro');

max_entry_best = max(abs(M_best(:)));


%% =========================================================
% PRINT RESULTS
%% =========================================================

fprintf('\n');
fprintf('========================================\n');

if compromise_used
    fprintf('BEST COMPROMISE OBSERVER\n');
else
    fprintf('FASTEST VALID OBSERVER\n');
end

fprintf('========================================\n');


fprintf('\nNoise tuning:\n');

fprintf('q_theta     = %.10g\n', best_q_theta);
fprintf('sigma_theta = %.10g rad\n', best_sigma_theta);

fprintf('q_phi       = %.10g\n', q_phi);
fprintf('sigma_ro    = %.10g rad/s\n', sigma_ro);
fprintf('sigma_i     = %.10g A\n', sigma_i);


fprintf('\nObserver performance:\n');

fprintf('rho max     = %.8f\n', rho_best);
fprintf('tau         = %.6f s\n', tau_best);
fprintf('tau         = %.2f ms\n', tau_best*1000);


fprintf('\nGain metrics:\n');

fprintf('M21         = %.8f', M_best(2,1));

if abs(M_best(2,1)) > max_M21
    fprintf('   <-- exceeds limit');
end

fprintf('\n');


fprintf('M33         = %.8f', M_best(3,3));

if abs(M_best(3,3)) > max_M33
    fprintf('   <-- exceeds limit');
end

fprintf('\n');


fprintf('||M||_F     = %.8f', gain_norm_best);

if gain_norm_best > max_gain_norm
    fprintf('   <-- exceeds limit');
end

fprintf('\n');


fprintf('max |Mij|   = %.8f', max_entry_best);

if max_entry_best > max_abs_entry
    fprintf('   <-- exceeds limit');
end

fprintf('\n');


fprintf('\nCurrent-estimator gain M:\n');
disp(M_best);

fprintf('Observer poles:\n');
disp(poles_best);

fprintf('Valid points = %d / %d\n', ...
    nnz(valid_map), numel(valid_map));


%% =========================================================
% PLOT 1: TIME CONSTANT
%% =========================================================

figure;

imagesc( ...
    log10(sigma_theta_vals), ...
    log10(q_theta_vals), ...
    tau_map*1000);

set(gca, 'YDir', 'normal');

xlabel('log_{10}(\sigma_\theta)');
ylabel('log_{10}(q_\theta)');
title('Dominant Observer Time Constant [ms]');

colorbar;

hold on;

plot( ...
    log10(best_sigma_theta), ...
    log10(best_q_theta), ...
    'rx', ...
    'MarkerSize', 12, ...
    'LineWidth', 2);

hold off;


%% =========================================================
% PLOT 2: |M21|
%% =========================================================

figure;

imagesc( ...
    log10(sigma_theta_vals), ...
    log10(q_theta_vals), ...
    abs(M21_map));

set(gca, 'YDir', 'normal');

xlabel('log_{10}(\sigma_\theta)');
ylabel('log_{10}(q_\theta)');
title('|M_{21}| : angle innovation -> theta dot');

colorbar;

hold on;

contour( ...
    log10(sigma_theta_vals), ...
    log10(q_theta_vals), ...
    abs(M21_map), ...
    [max_M21 max_M21], ...
    'k', ...
    'LineWidth', 1.5);

plot( ...
    log10(best_sigma_theta), ...
    log10(best_q_theta), ...
    'rx', ...
    'MarkerSize', 12, ...
    'LineWidth', 2);

hold off;


%% =========================================================
% PLOT 3: |M33|
%% =========================================================

figure;

imagesc( ...
    log10(sigma_theta_vals), ...
    log10(q_theta_vals), ...
    abs(M33_map));

set(gca, 'YDir', 'normal');

xlabel('log_{10}(\sigma_\theta)');
ylabel('log_{10}(q_\theta)');
title('|M_{33}| : current innovation -> rho dot');

colorbar;

hold on;

contour( ...
    log10(sigma_theta_vals), ...
    log10(q_theta_vals), ...
    abs(M33_map), ...
    [max_M33 max_M33], ...
    'k', ...
    'LineWidth', 1.5);

plot( ...
    log10(best_sigma_theta), ...
    log10(best_q_theta), ...
    'rx', ...
    'MarkerSize', 12, ...
    'LineWidth', 2);

hold off;


%% =========================================================
% PLOT 4: GAIN NORM
%% =========================================================

figure;

imagesc( ...
    log10(sigma_theta_vals), ...
    log10(q_theta_vals), ...
    gain_map);

set(gca, 'YDir', 'normal');

xlabel('log_{10}(\sigma_\theta)');
ylabel('log_{10}(q_\theta)');
title('Kalman Gain Frobenius Norm ||M||_F');

colorbar;

hold on;

contour( ...
    log10(sigma_theta_vals), ...
    log10(q_theta_vals), ...
    gain_map, ...
    [max_gain_norm max_gain_norm], ...
    'k', ...
    'LineWidth', 1.5);

plot( ...
    log10(best_sigma_theta), ...
    log10(best_q_theta), ...
    'rx', ...
    'MarkerSize', 12, ...
    'LineWidth', 2);

hold off;


%% =========================================================
% PLOT 5: DOMINANT POLE
%% =========================================================

figure;

imagesc( ...
    log10(sigma_theta_vals), ...
    log10(q_theta_vals), ...
    rho_map);

set(gca, 'YDir', 'normal');

xlabel('log_{10}(\sigma_\theta)');
ylabel('log_{10}(q_\theta)');
title('Dominant Observer Pole Magnitude');

colorbar;

hold on;

contour( ...
    log10(sigma_theta_vals), ...
    log10(q_theta_vals), ...
    rho_map, ...
    [rho_max_allowed rho_max_allowed], ...
    'k', ...
    'LineWidth', 1.5);

plot( ...
    log10(best_sigma_theta), ...
    log10(best_q_theta), ...
    'rx', ...
    'MarkerSize', 12, ...
    'LineWidth', 2);

hold off;


%% =========================================================
% PLOT 6: VALID REGION
%% =========================================================

figure;

imagesc( ...
    log10(sigma_theta_vals), ...
    log10(q_theta_vals), ...
    double(valid_map));

set(gca, 'YDir', 'normal');

xlabel('log_{10}(\sigma_\theta)');
ylabel('log_{10}(q_\theta)');
title('Fully Valid Observer Region');

colorbar;

hold on;

plot( ...
    log10(best_sigma_theta), ...
    log10(best_q_theta), ...
    'rx', ...
    'MarkerSize', 12, ...
    'LineWidth', 2);

hold off;
%% ==========================================================================
%  STEP 2+3 — INTEGRITY + ROBUSTNESS SWEEP (final)
%  PINN trust (sigma_pinn) x VO {on/off/corrupted} + baselines, at BOTH GNSS grades.
%  Prints two tables (RTK and commercial), each with Euclidean and per-axis peaks
%  as mean / std / best / worst.
%
%  RUN AFTER: INITIAL SETUP + Perform Visual Odometry (or the .mat load).
%
%  >>> IMPORTANT: in KalmanFilter.m, comment out BOTH hard-coded
%      `gnss_covariance = [...]` lines and keep only the guarded
%      `if ~exist('gnss_covariance','var'); gnss_covariance = [0.01 0.01 0.01]; end`
%      Otherwise the hard-coded line overrides the grade set below and the two
%      tables come out identical.
%
%  COST: 2 grades x 18 rows x N_SEEDS runs. To go faster: lower N_SEEDS,
%        or comment out one row of `grades`.
%  ==========================================================================
close all;

assert(exist('ground_truth','var')==1 && exist('tspan_kalman_filter','var')==1 && ...
       exist('keyframe_storage','var')==1, ...
   'Run INITIAL SETUP and Perform Visual Odometry in FigureCreation.m first.');

% ---- knobs ----
N_SEEDS         = 20;     % 10 is enough; 20 for a tighter baseline std
GNSS_FAIL_START = 510;    % outage window in cycles (t ~ 4.25 .. 33 s)
GNSS_FAIL_END   = 4000;
ENABLE_INPUT_KF = 1;      % process-variance preset (matches your paper-repro block)

grades = { 'RTK',        [0.01 0.01 0.01]
           'commercial', [1.5  1.5  1.5 ] };   % comment a row out to run one grade

% name, use_pinn, vo_dropout, sigma_pinn, corrupt_sigma
sweep = {
%   name                          use_pinn  vo_dropout  sigma_pinn  corrupt_sigma
    'PINN+VO    sigma=1',          true,     false,      1,          0
    'PINN+VO    sigma=2',          true,     false,      2,          0
    'PINN+VO    sigma=5',          true,     false,      5,          0
    'PINN+VO    sigma=8',          true,     false,      8,          0
    'PINN-noVO  sigma=1',          true,     true,       1,          0
    'PINN-noVO  sigma=2',          true,     true,       2,          0
    'PINN-noVO  sigma=5',          true,     true,       5,          0
    'PINN-noVO  sigma=8',          true,     true,       8,          0
    'baseline VO on',              false,    false,      2,          0
    'baseline VO off',             false,    true,       2,          0
    'PINN  corruptVO  s=0.5',      true,     false,      2,          0.5
    'PINN  corruptVO  s=1',        true,     false,      2,          1
    'PINN  corruptVO  s=2',        true,     false,      2,          2
    'PINN  corruptVO  s=4',        true,     false,      2,          4
    'base  corruptVO  s=0.5',      false,    false,      2,          0.5
    'base  corruptVO  s=1',        false,    false,      2,          1
    'base  corruptVO  s=2',        false,    false,      2,          2
    'base  corruptVO  s=4',        false,    false,      2,          4
};

for gi = 1:size(grades,1)
    GNSS_COV = grades{gi,2};
    fprintf('\n=================  %s GNSS  (cov = %.2f m^2)  =================\n', grades{gi,1}, GNSS_COV(1));
    fprintf('Metrics:  Euclid = max sqrt(dx^2+dy^2)   |   PerAxis = max(|dx|,|dy|)\n\n');
    fprintf('%-28s | %26s | %26s\n', '', 'EUCLIDEAN (m)', 'PER-AXIS (m)');
    fprintf('%-28s | %6s %5s %6s %6s | %6s %5s %6s %6s\n', 'config','mean','std','best','wrst','mean','std','best','wrst');
    fprintf('%s\n', repmat('-', 1, 90));

    for ci = 1:size(sweep,1)
        peaks    = zeros(N_SEEDS,1);   % Euclidean
        peaks_ax = zeros(N_SEEDS,1);   % per-axis
        for s = 1:N_SEEDS
            rng(s);                    % same seed across configs -> fair comparison

            % --- reset EVERY toggle each run (KalmanFilter.m modifies some) ---
            enable_observer = 0;   enable_camera = 1;   enable_gnss = 1;
            camera_correct_orientation = 1;   camera_correct_velocity = 1;
            enable_input_kf = ENABLE_INPUT_KF;   offset = 1;
            gnss_fail_start = GNSS_FAIL_START;   gnss_fail_end = GNSS_FAIL_END;
            vo_drop_start   = GNSS_FAIL_START;   vo_drop_end   = GNSS_FAIL_END;
            use_pinn      = sweep{ci,2};
            vo_dropout    = sweep{ci,3};
            sigma_pinn    = sweep{ci,4};
            corrupt_sigma = sweep{ci,5};
            vo_corrupt    = (corrupt_sigma > 0);   % corruption on only when sigma>0
            gnss_covariance = GNSS_COV;            % <-- RESTORED: apply this grade

            run('KalmanFilter.m');

            gti = interp1(tspan_ground_truth, ground_truth, tspan_kalman_filter);
            ex  = abs(pose_storage(1,:)' - gti(:,1));
            ey  = abs(pose_storage(2,:)' - gti(:,2));
            win = gnss_fail_start:gnss_fail_end;
            peaks(s)    = max(hypot(ex(win), ey(win)));      % Euclidean
            peaks_ax(s) = max(max(ex(win)), max(ey(win)));   % per-axis (larger axis)
        end
        fprintf('%-28s | %6.2f %5.2f %6.2f %6.2f | %6.2f %5.2f %6.2f %6.2f\n', sweep{ci,1}, ...
            mean(peaks),    std(peaks),    min(peaks),    max(peaks), ...
            mean(peaks_ax), std(peaks_ax), min(peaks_ax), max(peaks_ax));
    end
end

fprintf('\nNotes:\n');
fprintf(' - If RTK and commercial tables are identical, a hard-coded gnss_covariance in KalmanFilter.m is overriding the grade.\n');
fprintf(' - baseline (VO on) is seed-variable (handover noise) -> report its MEAN, not a single run.\n');
fprintf(' - tight trust (sigma 1-2): PINN+VO ~ PINN-noVO  -> PINN carries position (the "flat" result).\n');
fprintf(' - loose trust (sigma 5-8): PINN+VO < PINN-noVO   -> fusion is live (VO helps when allowed).\n');
fprintf(' - baseline VO off ~63 m -> clean failure-mode number (observer blinded during outage).\n');
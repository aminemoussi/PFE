%% ==========================================================================
%  STEP 3 (capstone) — OUTAGE-DURATION SWEEP
%  How does peak drift grow as the GPS outage lengthens?  Current drive, RTK.
%
%  Outage START is fixed at cycle 510; the END is swept outward. The PINN path
%  is anchored at 510 and already covers the whole window, so a shorter outage
%  simply uses a PREFIX of it -> valid, no new data or retraining needed.
%
%  RUN AFTER: INITIAL SETUP + Perform Visual Odometry.
%  PREREQ:    KalmanFilter.m must use the GUARDED gnss_covariance (no hard-coded
%             line active) -- same fix as the integrity sweep.
%  COST: numel(END_CYCLES) x 4 configs x N_SEEDS runs (~15-25 min at the defaults).
%  ==========================================================================
close all;

assert(exist('ground_truth','var')==1 && exist('tspan_kalman_filter','var')==1 && ...
       exist('keyframe_storage','var')==1, ...
   'Run INITIAL SETUP and Perform Visual Odometry in FigureCreation.m first.');

% ---- knobs ----
N_SEEDS         = 8;                                  % baseline rows need a few; PINN is seed-stable
GNSS_FAIL_START = 510;                                % outage start (fixed)
END_CYCLES      = [1000 1500 2000 2500 3000 3500 4000];% outage end -> durations ~4..29 s
%GNSS_COV        = [0.01 0.01 0.01];                   % RTK
GNSS_COV = [1.5 1.5 1.5];      % commercial   (was [0.01 0.01 0.01] RTK)
ENABLE_INPUT_KF = 1;
timestep        = 0.00825;

% configs: name, use_pinn, vo_dropout
cfgs = {
  'PINN+VO',     true,  false
  'base VOon',   false, false
  'PINN noVO',   true,  true
  'base VOoff',  false, true
};

durs = (END_CYCLES - GNSS_FAIL_START) * timestep;     % seconds
PEAK = zeros(numel(END_CYCLES), size(cfgs,1));        % mean peak Euclidean over outage
PSTD = zeros(numel(END_CYCLES), size(cfgs,1));
TERM = zeros(numel(END_CYCLES), size(cfgs,1));        % mean drift AT recovery (terminal)

for ei = 1:numel(END_CYCLES)
    gfe = END_CYCLES(ei);
    for ci = 1:size(cfgs,1)
        pk = zeros(N_SEEDS,1);  tm = zeros(N_SEEDS,1);
        for s = 1:N_SEEDS
            rng(s);
            enable_observer = 0;  enable_camera = 1;  enable_gnss = 1;
            camera_correct_orientation = 1;  camera_correct_velocity = 1;
            enable_input_kf = ENABLE_INPUT_KF;  offset = 1;
            gnss_fail_start = GNSS_FAIL_START;  gnss_fail_end = gfe;
            vo_drop_start   = GNSS_FAIL_START;  vo_drop_end   = gfe;   % VO drop tracks the outage
            vo_corrupt = false;  corrupt_sigma = 0;
            use_pinn   = cfgs{ci,2};
            vo_dropout = cfgs{ci,3};
            sigma_pinn = 2;
            gnss_covariance = GNSS_COV;

            run('KalmanFilter.m');

            gti = interp1(tspan_ground_truth, ground_truth, tspan_kalman_filter);
            d = hypot(pose_storage(1,:)' - gti(:,1), pose_storage(2,:)' - gti(:,2));
            pk(s) = max(d(gnss_fail_start:gnss_fail_end));   % worst error during the outage
            tm(s) = d(gfe);                                  % error just before GPS returns
        end
        PEAK(ei,ci) = mean(pk);  PSTD(ei,ci) = std(pk);  TERM(ei,ci) = mean(tm);
    end
end

% ---- compact table: PEAK Euclidean (mean), pasteable ----
fprintf('\nPEAK Euclidean drift (mean over %d seeds), RTK\n', N_SEEDS);
fprintf('%8s', 'dur[s]');
for ci = 1:size(cfgs,1), fprintf(' | %10s', cfgs{ci,1}); end
fprintf('\n%s\n', repmat('-', 1, 8 + size(cfgs,1)*13));
for ei = 1:numel(END_CYCLES)
    fprintf('%8.1f', durs(ei));
    for ci = 1:size(cfgs,1), fprintf(' | %10.2f', PEAK(ei,ci)); end
    fprintf('\n');
end

% ---- second table: TERMINAL drift at recovery (mean) ----
fprintf('\nTERMINAL drift at GPS-return (mean over %d seeds), RTK\n', N_SEEDS);
fprintf('%8s', 'dur[s]');
for ci = 1:size(cfgs,1), fprintf(' | %10s', cfgs{ci,1}); end
fprintf('\n%s\n', repmat('-', 1, 8 + size(cfgs,1)*13));
for ei = 1:numel(END_CYCLES)
    fprintf('%8.1f', durs(ei));
    for ci = 1:size(cfgs,1), fprintf(' | %10.2f', TERM(ei,ci)); end
    fprintf('\n');
end

% ---- quick on-screen plot (no file save) ----
figure('Position',[100 100 780 520]);
co = [0 .45 .74; .4 .4 .4; .85 .33 .10; .8 .1 .1];
for ci = 1:size(cfgs,1)
    plot(durs, PEAK(:,ci), '-o', 'LineWidth', 2, 'Color', co(ci,:), 'MarkerFaceColor', co(ci,:));
    hold on;
end
xlabel('GPS outage duration [s]'); ylabel('peak drift over outage [m]');
title('Drift vs outage duration (current drive, RTK)');
legend(cfgs(:,1), 'Location', 'northwest'); grid on;

fprintf('\nExpected: the two VO-fed rows (PINN+VO, base VOon) and PINN-noVO stay low and flat;\n');
fprintf('base VOoff climbs steeply with duration -> the PINN''s value is slowing drift growth when VO is gone.\n');
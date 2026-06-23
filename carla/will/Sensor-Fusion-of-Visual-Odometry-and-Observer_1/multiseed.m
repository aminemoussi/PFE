seeds = 1:20;
nS = numel(seeds);
gt = interp1(tspan_ground_truth, ground_truth, tspan_kalman_filter);
o  = gnss_fail_start:gnss_fail_end;

for d = [0 1]                          % d=0 normal VO, d=1 VO dropout
    vo_dropout = logical(d);
    pb = zeros(nS,1); pp = zeros(nS,1);
    for k = 1:nS
        % reset flags EVERY run (KalmanFilter zeroes camera_correct_orientation in the outage)
        enable_observer=0; enable_camera=1; camera_correct_orientation=1; camera_correct_velocity=1; enable_gnss=1;
        rng(seeds(k)); use_pinn=false; KalmanFilter;
        pb(k) = max(hypot(pose_storage(1,o)'-gt(o,1), pose_storage(2,o)'-gt(o,2)));

        enable_observer=0; enable_camera=1; camera_correct_orientation=1; camera_correct_velocity=1; enable_gnss=1;
        rng(seeds(k)); use_pinn=true;  KalmanFilter;
        pp(k) = max(hypot(pose_storage(1,o)'-gt(o,1), pose_storage(2,o)'-gt(o,2)));
    end
    if d==0, lbl='normal VO'; else, lbl='VO DROPOUT'; end
    fprintf('\n=== %s (%d seeds) ===\n', lbl, nS);
    fprintf('baseline:  mean %.2f  std %.2f  WORST %.2f\n', mean(pb), std(pb), max(pb));
    fprintf('PINN    :  mean %.2f  std %.2f  WORST %.2f\n', mean(pp), std(pp), max(pp));
    if d==0, base_normal=pb; pinn_normal=pp; else, base_drop=pb; pinn_drop=pp; end
end
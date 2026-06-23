sigmas = [2 3 5 8];
seeds = 1:20; nS = numel(seeds);
gt = interp1(tspan_ground_truth, ground_truth, tspan_kalman_filter);
o  = gnss_fail_start:gnss_fail_end;
vo_dropout = false;
for s = sigmas
    pp = zeros(nS,1);
    for k = 1:nS
        enable_observer=0; enable_camera=1; camera_correct_orientation=1; camera_correct_velocity=1; enable_gnss=1;
        sigma_pinn = s;
        rng(seeds(k)); use_pinn=true; KalmanFilter;
        pp(k) = max(hypot(pose_storage(1,o)'-gt(o,1), pose_storage(2,o)'-gt(o,2)));
    end
    fprintf('sigma %4.1f :  mean %.2f  std %.2f  WORST %.2f\n', s, mean(pp), std(pp), max(pp));
end
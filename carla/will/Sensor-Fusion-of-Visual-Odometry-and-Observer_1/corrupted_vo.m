seeds = 1:20; nS = numel(seeds);
gt = interp1(tspan_ground_truth, ground_truth, tspan_kalman_filter);
o  = gnss_fail_start:gnss_fail_end;
vo_dropout=false; vo_corrupt=true; corrupt_sigma=1.0; sigma_pinn=2.0;
vo_drop_start=find(tspan_kalman_filter>=15,1); vo_drop_end=find(tspan_kalman_filter>=25,1);
pb=zeros(nS,1); pp=zeros(nS,1);
for k=1:nS
    enable_observer=0; enable_camera=1; camera_correct_orientation=1; camera_correct_velocity=1; enable_gnss=1;
    rng(seeds(k)); use_pinn=false; KalmanFilter;
    pb(k)=max(hypot(pose_storage(1,o)'-gt(o,1), pose_storage(2,o)'-gt(o,2)));
    rng(seeds(k)); use_pinn=true;  KalmanFilter;
    pp(k)=max(hypot(pose_storage(1,o)'-gt(o,1), pose_storage(2,o)'-gt(o,2)));
end
fprintf('\n=== corrupted VO (sigma %.1f, %d seeds) ===\n', corrupt_sigma, nS);
fprintf('baseline:  mean %.2f  std %.2f  WORST %.2f\n', mean(pb), std(pb), max(pb));
fprintf('PINN    :  mean %.2f  std %.2f  WORST %.2f\n', mean(pp), std(pp), max(pp));
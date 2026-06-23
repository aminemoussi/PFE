seeds=1:5; nS=numel(seeds);
gt = interp1(tspan_ground_truth, ground_truth, tspan_kalman_filter);
o  = gnss_fail_start:gnss_fail_end;
Ls=[5 10 15 20 25];                 % blackout length, seconds
vo_dropout=true; vo_corrupt=false; sigma_pinn=2.0;
fprintf('\nblackout(s)  baseline   PINN\n');
for Lsec=Ls
    vo_drop_start = gnss_fail_start;
    vo_drop_end   = gnss_fail_start + round(Lsec/timestep);
    pb=zeros(nS,1); pp=zeros(nS,1);
    for k=1:nS
        enable_observer=0; enable_camera=1; camera_correct_orientation=1; camera_correct_velocity=1; enable_gnss=1;
        rng(seeds(k)); use_pinn=false; KalmanFilter;
        pb(k)=max(hypot(pose_storage(1,o)'-gt(o,1), pose_storage(2,o)'-gt(o,2)));
        rng(seeds(k)); use_pinn=true;  KalmanFilter;
        pp(k)=max(hypot(pose_storage(1,o)'-gt(o,1), pose_storage(2,o)'-gt(o,2)));
    end
    fprintf('%6d      %6.2f   %6.2f\n', Lsec, mean(pb), mean(pp));
end
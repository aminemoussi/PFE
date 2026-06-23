if(enable_camera)
    %run('VisualOdometry.m');
    cam_data = poses(keyframe_storage);
    scaled_translation_data = vertcat(cam_data.AbsolutePose.Translation) * scale;
    visual_translation_data = zeros(height(scaled_translation_data), 3);
    %figure;
    %plot3(scaled_translation_data(:,1),scaled_translation_data(:,2),scaled_translation_data(:,3))
    
    for frame_index = 2:height(visual_translation_data)
         visual_translation_data(frame_index,:) = scaled_translation_data(frame_index,:) - scaled_translation_data(frame_index-1,:);
    end
    
    visual_rotation_data = zeros(3, 3, height(keyframe_storage.Views));
    for frame_index = 1:height(keyframe_storage.Views)
        visual_rotation_data(:,:,frame_index) = keyframe_storage.Views.AbsolutePose(frame_index,1).R;
    end
else
    cam_data = poses(keyframe_storage);
    scaled_translation_data = vertcat(cam_data.AbsolutePose.Translation) * scale;
    visual_translation_data = zeros(height(scaled_translation_data), 3);
    visual_rotation_data = zeros(3, 3, height(keyframe_storage.Views));
end
%figure;
%plot3(visual_translation_data(:,1),visual_translation_data(:,2),visual_translation_data(:,3))


% Setup ground truth 
ground_truth_file = fopen(strcat(FILEPATH, "ground_truth.txt"), "r");
format_spec_ground_truth = "X, %f, Y, %f, Z, %f\n";
ground_truth_size = [3 Inf];
ground_truth = fscanf(ground_truth_file, format_spec_ground_truth, ground_truth_size)';
fclose(ground_truth_file);
if(offset)
    ground_truth = ground_truth - ground_truth(1,:);
end

gt_truth = interp1(tspan_ground_truth, ground_truth, tspan_kalman_filter);  % true path, for the wiring test

% Setup GNSS Sensor
gnss_file = fopen(strcat(FILEPATH, "noisy_gnss.txt"), "r");
format_spec_gnss = "X, %f, Y, %f, Z, %f\n";
gnss_size = [3 Inf];
gnss_data = fscanf(gnss_file, format_spec_gnss, gnss_size)';
fclose(gnss_file);
if(offset)
    gnss_data = gnss_data - gnss_data(1,:);
end

% Add noise and bias to GNSS Sensor
gnss_bias = [0, 0, 0];
% RTK GNSS 0.01m accuracy
%gnss_covariance = [0.01, 0.01, 0.01];
% Normal GNSS 1.5m accuracy
%gnss_covariance = [1.5, 1.5, 1.5];

if ~exist('gnss_covariance','var')
    gnss_covariance = [0.01, 0.01, 0.01];   % default RTK
    %gnss_covariance = [1.5, 1.5, 1.5];  %commertial
end

gnss_data = gnss_data + gnss_bias + (sqrt(gnss_covariance) .* randn(size(gnss_data)));

% Define Initial Conditions
pose0 = [5;5;v(1);atan2(5,5)];
covariance0 = [1,0,0,0;...
               0,1,0,0;...
               0,0,1,0;...
               0,0,0,1];
X_hat_k = [pose0, covariance0];
x_hat_observer_k = m0;
pose_storage_gnss = pose0;
covariance_storage_gnss = covariance0;
pose_storage = pose0;
covariance_storage = covariance0;

% Process variance
initial_process_variance = 1e1;
process_variance = 1e1;

if(enable_input_kf)
    initial_process_variance = 1e-2;
    process_variance = 1e-3;
end


F = [1,0,0,0;...
     0,1,0,0;...
     0,0,1,0;...
     0,0,0,1];

% If both observer and camera outputs are being used, the X and Y
% predictions will stack, meaning their estimates are too high. Dividing by
% two means an average, more accurate prediction is made. 
if(enable_observer & enable_camera)
    B_kalman = [0.5,0,  0,0;...
         0,  0.5,0,0;...
         0,  0,  1,0;...
         0,  0,  0,1];
else
    B_kalman = [1,0,0,0;...
         0,1,0,0;...
         0,0,1,0;...
         0,0,0,1];
end

% Additive multivariate covariance. Summates at each iteration. 
Q_initial_observer = initial_process_variance * [enable_observer,0,0,0;...
                                 0,enable_camera,0,0;...
                                 0,0,camera_correct_velocity,0;...
                                 0,0,0,camera_correct_orientation];

Q_initial_camera = initial_process_variance * [enable_camera,0,0,0;...
                               0,enable_camera,0,0;...
                               0,0,0,0;...
                               0,0,0,0];

Q_observer = process_variance * [enable_observer,0,0,0;...
                                 0,enable_camera,0,0;...
                                 0,0,camera_correct_velocity,0;...
                                 0,0,0,camera_correct_orientation];

Q_camera = process_variance * [enable_camera,0,0,0;...
                               0,enable_camera,0,0;...
                               0,0,0,0;...
                               0,0,0,0];

Q_process = process_variance *  [1,0,0,0;...
                                 0,1,0,0;...
                                 0,0,1,0;...
                                 0,0,0,1];

H = [1,0,0,0;...
     0,1,0,0;...
     0,0,1,0;...
     0,0,0,1];

R = [gnss_covariance(1),    0,                  0,0;...
     0,                     gnss_covariance(2), 0,0;...
     0,                     0,                  0,0;...
     0,                     0,                  0,0];

% Initial Camera index
camera_frame_index = 1;
tic;

tspan_gnss(length(tspan_gnss)) = 35;
gnss_interped_data = interp1(tspan_gnss, gnss_data, tspan_observer, "nearest");

% Initial kalman run for GNSS smoothing, for observer.
for cycle = 1:length(tspan_kalman_filter)-1
   
    dx_hat_observer = [0;0;0;0];
    u_camera = [0;0;0;timestep * 4];

    X_hat_k1 = X_hat_k;
    
    % Predict then Update!
    X_invertedhat_k = PredictFilter(X_hat_k1, dx_hat_observer, u_camera, F, B_kalman, Q_initial_observer, Q_initial_camera, Q_process, 0);
    X_hat_k = UpdateFilter(X_invertedhat_k, gnss_interped_data(cycle,:), H, R);
   
    % Store outputs in storage!
    pose_storage_gnss = cat(2,pose_storage_gnss,X_hat_k(:,1));
    covariance_storage_gnss = cat(2,covariance_storage_gnss,X_hat_k(:,2:5));
end

% Then collect the computed data.
run("HighGainObserver.m");

x_hat_observer2 = interp1(tspan_observer, x_hat_observer2, tspan_kalman_filter);

% Reset initials for second run
X_hat_k = [pose0, covariance0];
x_hat_observer_k = m0;
v_k = v(1);
camera_frame_index = 1;
last_known_rotation = visual_rotation_data(:,:,1);


% ===== VO-dropout experiment setup =====
%use_pinn = true;            % <-- run once as false (baseline), once as true (PINN)

if ~exist('use_pinn','var');   use_pinn   = false; end
if ~exist('vo_dropout','var'); vo_dropout = false; end
if ~exist('vo_corrupt','var');    vo_corrupt    = false; end
if ~exist('corrupt_sigma','var'); corrupt_sigma = 1.0;   end
if ~exist('vo_drop_start','var'); vo_drop_start = find(tspan_kalman_filter >= 15, 1); end
if ~exist('vo_drop_end','var');   vo_drop_end   = find(tspan_kalman_filter >= 25, 1); end

P = readmatrix('pinn_predicted_xy.csv');     % Will's-drive PINN path (tf, x, y)
pinn_x_rel = interp1(P(:,1), P(:,2), tspan_kalman_filter, 'linear', 'extrap');
pinn_y_rel = interp1(P(:,1), P(:,3), tspan_kalman_filter, 'linear', 'extrap');

%sigma_pinn = 2.0;
%R_pinn = diag([sigma_pinn^2, sigma_pinn^2, 0, 0]);

if ~exist('sigma_pinn','var'); sigma_pinn = 2.0; end
R_pinn = diag([sigma_pinn^2, sigma_pinn^2, 0, 0]);

pinn_anchor = [0 0];

%vo_drop_start = find(tspan_kalman_filter >= 15, 1);   % camera fails at t = 15 s
%vo_drop_end   = find(tspan_kalman_filter >= 25, 1);   % camera recovers at t = 25 s

pose_storage = pose0;        % reset so each re-run is clean
covariance_storage = covariance0;
% =======================================



for cycle = 1:length(tspan_kalman_filter)-1
    % Get difference between sensor outputs.
    x_hat_observer_k1 = x_hat_observer_k;
    x_hat_observer_k =  x_hat_observer2(cycle,:)';

    v_k1 = v_k;
    v_k = v(cycle);

    if (enable_observer)
        dx_hat_observer = [x_hat_observer_k(1) - x_hat_observer_k1(1);...
            x_hat_observer_k(4) - x_hat_observer_k1(4);...
            v_k - v_k1;...
            atan2(x_hat_observer_k(5),x_hat_observer_k(2)) - atan2(x_hat_observer_k1(5),x_hat_observer_k1(2))];
    elseif (camera_correct_velocity && camera_correct_orientation)
        dx_hat_observer =  [0;...
                            0;...
                            v_k - v_k1;...
                            atan2(x_hat_observer_k(5),x_hat_observer_k(2)) - atan2(x_hat_observer_k1(5),x_hat_observer_k1(2))];
    elseif (camera_correct_velocity)
        dx_hat_observer =  [0;...
                            0;...
                            v_k - v_k1;...
                            0];
    elseif (camera_correct_orientation)
        dx_hat_observer =  [0;...
                            0;...
                            0;...
                            atan2(x_hat_observer_k(5),x_hat_observer_k(2)) - atan2(x_hat_observer_k1(5),x_hat_observer_k1(2))];
    else
        dx_hat_observer = [0;0;0;0];
    end

    % if (enable_observer)
    %     dx_hat_observer = [x_hat_observer_k(1) - x_hat_observer_k1(1);...
    %         x_hat_observer_k(4) - x_hat_observer_k1(4);...
    %         sqrt((x_hat_observer_k(5)^2 + x_hat_observer_k(2)^2)) - sqrt((x_hat_observer_k1(5)^2 + x_hat_observer_k1(2)^2));...
    %         atan2(x_hat_observer_k(5),x_hat_observer_k(2)) - atan2(x_hat_observer_k1(5),x_hat_observer_k1(2))];
    % elseif (camera_correct_velocity && camera_correct_orientation)
    %     dx_hat_observer =  [0;...
    %                         0;...
    %                         sqrt((x_hat_observer_k(5)^2 + x_hat_observer_k(2)^2)) - sqrt((x_hat_observer_k1(5)^2 + x_hat_observer_k1(2)^2));...
    %                         atan2(x_hat_observer_k(5),x_hat_observer_k(2)) - atan2(x_hat_observer_k1(5),x_hat_observer_k1(2))];
    % elseif (camera_correct_velocity)
    %     dx_hat_observer =  [0;...
    %                         0;...
    %                         sqrt((x_hat_observer_k(5)^2 + x_hat_observer_k(2)^2)) - sqrt((x_hat_observer_k1(5)^2 + x_hat_observer_k1(2)^2));...
    %                         0];
    % elseif (camera_correct_orientation)
    %     dx_hat_observer =  [0;...
    %                         0;...
    %                         0;...
    %                         atan2(x_hat_observer_k(5),x_hat_observer_k(2)) - atan2(x_hat_observer_k1(5),x_hat_observer_k1(2))];
    % else
    %     dx_hat_observer = [0;0;0;0];
    % end

    X_hat_k1 = X_hat_k;
    
    % Get Camera data index, and check for update
    if(enable_camera && camera_frame_index < height(added_frames_index) && added_frames_index(camera_frame_index) * 4 == cycle)
         
        % Orient the camera transform correctly.
        if(camera_correct_orientation && camera_frame_index > 1)
            visual_translation_data(camera_frame_index,:) = visual_translation_data(camera_frame_index,:) * visual_rotation_data(:,:,camera_frame_index) * roty(-90) * rotx(-90);
            last_known_rotation = visual_rotation_data(:,:,camera_frame_index);
        else
            visual_translation_data(camera_frame_index,:) = visual_translation_data(camera_frame_index,:) * last_known_rotation * roty(-90) * rotx(-90);
        end
        
        % Get rid of horrible transients! 
        if norm(visual_translation_data(camera_frame_index,:)) > 200
             visual_translation_data(camera_frame_index,:) = [0,0,0];
        end
        
        % Make sure to add the data about the frametime.
        if(camera_frame_index == 1)
            u_camera = [visual_translation_data(camera_frame_index,:)'; timestep * 4];
        else
            u_camera = [visual_translation_data(camera_frame_index,:)'; timestep * 4 * (added_frames_index(camera_frame_index)-added_frames_index(camera_frame_index-1))];
        end
        camera_frame_index = camera_frame_index + 1;
    else
        u_camera = [0;0;0;timestep*4];
    end

    % --- bad-vision window: clean dropout (HGO carries) OR corrupted VO ---
    if cycle >= vo_drop_start && cycle <= vo_drop_end
        if vo_dropout
            u_camera = [0;0;0;timestep*4];
            dx_hat_observer = [x_hat_observer_k(1)-x_hat_observer_k1(1); ...
                x_hat_observer_k(4)-x_hat_observer_k1(4); ...
                v_k - v_k1; ...
                atan2(x_hat_observer_k(5),x_hat_observer_k(2)) - atan2(x_hat_observer_k1(5),x_hat_observer_k1(2))];
        elseif vo_corrupt && any(u_camera(1:3))
            u_camera(1:3) = u_camera(1:3) + corrupt_sigma * randn(3,1);   % rogue features
        end
    end
    
    % Predict and...
    X_invertedhat_k = PredictFilter(X_hat_k1, dx_hat_observer, u_camera, F, B_kalman, Q_observer, Q_camera, Q_process, camera_correct_velocity);

    % ...If GNSS is enabled, use the data to update our prediction
    if(cycle < gnss_fail_start || cycle > gnss_fail_end)
        if(enable_gnss)
            X_hat_k = UpdateFilter(X_invertedhat_k, gnss_interped_data(cycle,:), H, R);
        else
            X_hat_k = X_invertedhat_k;
        end
    else   % GNSS outage
        if cycle == gnss_fail_start
            pinn_anchor = X_hat_k(1:2,1)';
        end
        if use_pinn
            pinn_meas = [pinn_anchor(1) + pinn_x_rel(cycle), ...
                pinn_anchor(2) + pinn_y_rel(cycle)];
            X_hat_k = UpdateFilter(X_invertedhat_k, pinn_meas, H, R_pinn);
        else
            X_hat_k = X_invertedhat_k;   % Will's coast — no PINN
        end
        camera_correct_velocity_was_on = 0;
        camera_correct_orientation = 0;
    end
    %else
    %    if cycle == gnss_fail_start
    %        pinn_anchor = X_hat_k(1:2,1)';   % filter's last good position, at handover
    %    end
    %    pinn_meas = [pinn_anchor(1) + pinn_x_rel(cycle), ...
    %                 pinn_anchor(2) + pinn_y_rel(cycle)];
    %    X_hat_k = UpdateFilter(X_invertedhat_k, pinn_meas, H, R_pinn);
    %    camera_correct_velocity_was_on = 0;
    %    camera_correct_orientation = 0;
    %end
    
    % Once again, store the output.
    pose_storage = cat(2,pose_storage,X_hat_k(:,1));
    covariance_storage = cat(2,covariance_storage,X_hat_k(:,2:5));
end

timing = toc;

% Predict function.
function X_invertedhat_k = PredictFilter(X_hat_k1, u_observer, u_camera, F, B_kalman, Q_observer, Q_camera, Q_process, camera_correct_velocity)
    
    ut1 = X_hat_k1(:,1);
    Pt1 = X_hat_k1(:,2:5);

    C_sin =    [0,-1,0,0;...
                1,0,0,0;...
                0,0,0,0;...
                0,0,0,0];

    C_cos =    [1,0,0,0;...
                0,1,0,0;...
                0,0,0,0;...
                0,0,0,0];
    
    % Calculate orientation to rotation matrix for camera correction.
    C = C_sin * sin(ut1(4)) + C_cos * cos(ut1(4));
    %C = C_cos;

    % Calculate camera velocity correction
    if(camera_correct_velocity)
        speed = ut1(3);
        frametime = u_camera(4);
        predicted_distance = norm(C*u_camera,2);
        
        if(predicted_distance ~= 0)
            direction = C * u_camera / predicted_distance;
        else
            direction = C * u_camera / 1000;
        end
        rot_scaled_cam = direction * speed * frametime;
    else
        rot_scaled_cam = C*u_camera;
    end
    
    % Finally predict using our possibly corrected data.
    predicted_state_k = F*ut1 + B_kalman*u_observer + B_kalman*rot_scaled_cam;
    predicted_covariance_k = F*Pt1*F' + B_kalman*Q_process*B_kalman' + B_kalman*Q_observer*B_kalman' + C*Q_camera*C';

    X_invertedhat_k = [predicted_state_k, predicted_covariance_k];
end

function X_hat_k = UpdateFilter(X_invertedhat_k, gnss_data, H, R)

    Pt = X_invertedhat_k(:,2:5);
    ut = X_invertedhat_k(:,1);

    % Update the prediction using GNSS sensor
    gnss_data = [gnss_data(:,1); gnss_data(:,2); ut(3); ut(4)];
    kalman_gain = (Pt*H') * inv(H*Pt*H' + R);
    updated_state_k = ut + (kalman_gain * (gnss_data - H*ut));
    updated_covariance_k = (eye(4,4) - kalman_gain*H)*Pt;
    X_hat_k = [updated_state_k, updated_covariance_k];
end
% Code adapted from Mathworks Example: https://uk.mathworks.com/help/vision/ug/monocular-visual-simultaneous-localization-and-mapping.html

addpath('D:\Documents\MATLAB\Examples\R2023b\vision\MonocularVisualSimultaneousLocalizationAndMappingExample\')
addpath('D:\Documents\MATLAB\Examples\R2023b\vision\MonocularVisualInertialSLAMExample\')

% Get images
imds          = imageDatastore(FILEPATH);

% Inspect the first image
current_frame_index = 1;
current_frame = readimage(imds, current_frame_index);
figure;
himage = imshow(current_frame);

% Set random seed for reproducibility
rng(1);

fov = 110;
image_size = [1080 1920];
optical_centre = [960 540]; 
focal_length = 1920 / (2 * tand(110 / 2));
focal_length_xy = [focal_length focal_length];
intrinsics = cameraIntrinsics(focal_length, optical_centre, image_size);


% Detect and extract ORB features
orb_scale_factor = 1.2;
orb_levels   = 8;
orb_points   = 2000;
[initial_features, initial_points] = helperDetectAndExtractFeatures(current_frame, orb_scale_factor, orb_levels, orb_points, intrinsics); 

current_frame_index = current_frame_index + 1;
first_frame       = current_frame; % Preserve the first frame 

isMapInitialized  = false;

% Map initialization loop
while ~isMapInitialized && current_frame_index < numel(imds.Files)
    current_frame = readimage(imds, current_frame_index);

    [current_features, current_points] = helperDetectAndExtractFeatures(current_frame, orb_scale_factor, orb_levels, orb_points, intrinsics); 

    current_frame_index = current_frame_index + 1;

    % Find putative feature matches
    index_pairs = matchFeatures(initial_features, current_features, Unique=true, ...
        MaxRatio=0.9, MatchThreshold=40);

    previous_matched_points  = initial_points(index_pairs(:,1),:);
    current_matched_points = current_points(index_pairs(:,2),:);

    % If not enough matches are found, check the next frame
    minimum_matches = 100;
    if size(index_pairs, 1) < minimum_matches
        continue
    end

    previous_matched_points  = initial_points(index_pairs(:,1),:);
    current_matched_points = current_points(index_pairs(:,2),:);

    % Compute homography and evaluate reconstruction
    [transform_homography, homography_score, homography_inlier_index] = helperComputeHomography(previous_matched_points, current_matched_points);

    % Compute fundamental matrix and evaluate reconstruction
    [transform_fundamental, fundamental_score, fundamental_inlier_index] = helperComputeFundamentalMatrix(previous_matched_points, current_matched_points);

    % Select the model based on a heuristic
    ratio = homography_score/(homography_score + fundamental_score);
    ratioThreshold = 0.45;
    if ratio > ratioThreshold
        inlier_transform_index = homography_inlier_index;
        tform          = transform_homography;
    else
        inlier_transform_index = fundamental_inlier_index;
        tform          = transform_fundamental;
    end

    % Computes the camera location up to scale. Use half of the 
    % points to reduce computation
    inlierinitial_points  = previous_matched_points(inlier_transform_index);
    inliercurrent_points = current_matched_points(inlier_transform_index);
    [relative_pose, valid_fraction] = estrelpose(tform, intrinsics, ...
        inlierinitial_points(1:2:end), inliercurrent_points(1:2:end));
    
    % Orient the initial camera pose correctly. In reality this would be
    % provided by another sensor.
    % Along the y-axis, according to plotCamera
    initial_pose = rigidtform3d(rotx(90),[0,0,0]);

    % If not enough inliers are found, move to the next frame
    if valid_fraction < 0.9 || numel(relative_pose)>=2
        continue
    end

    % Triangulate two views to obtain 3-D map points
    minimum_parallax = 3; % In degrees
    [valid, world_points, inlier_triangulation_index] = helperTriangulateTwoFrames(...
        rigidtform3d, relative_pose, inlierinitial_points, inliercurrent_points, intrinsics, minimum_parallax);

    if ~valid
        continue
    end

    % Get the original index of features in the two key frames
    index_pairs = index_pairs(inlier_transform_index(inlier_triangulation_index),:);

    isMapInitialized = true;

    disp(['Map initialized with frame 1 and frame ', num2str(current_frame_index-1)])
end % End of map initialization loop

if isMapInitialized
    close(himage.Parent.Parent); % Close the previous figure
    % Show matched featuresaddView
    figure;
    hfeature = showMatchedFeatures(first_frame, current_frame, initial_points(index_pairs(:,1)), ...
        current_points(index_pairs(:, 2)), "Montage");
else
    error('Unable to initialize the map.')
end

% Create an empty imageviewset object to store key frames
keyframe_storage = imageviewset;

% Create an empty worldpointset object to store 3-D map points
map_point_set   = worldpointset;

% Add the first key frame. Place the camera associated with the first 
% key frame at the origin, oriented according to initial_pose
initial_view_id     = 1;
keyframe_storage = addView(keyframe_storage, initial_view_id, initial_pose, Points=initial_points,...
    Features=initial_features.Features);

% Add the second key frame
current_view_id    = 2;
keyframe_storage = addView(keyframe_storage, current_view_id, relative_pose, Points=current_points,...
    Features=current_features.Features);

% Add connection between the first and the second key frame
keyframe_storage = addConnection(keyframe_storage, initial_view_id, current_view_id, relative_pose, Matches=index_pairs);

% Add 3-D map points
[map_point_set, new_point_index] = addWorldPoints(map_point_set, world_points);

% Add observations of the map points
initial_locations  = initial_points.Location;
current_locations = current_points.Location;
initial_scales     = initial_points.Scale;
current_scales    = current_points.Scale;

% Add image points corresponding to the map points in the first key frame
map_point_set   = addCorrespondences(map_point_set, initial_view_id, new_point_index, index_pairs(:,1));

% Add image points corresponding to the map points in the second key frame
map_point_set   = addCorrespondences(map_point_set, current_view_id, new_point_index, index_pairs(:,2));

% Run full bundle adjustment on the first two key frames
tracks       = findTracks(keyframe_storage);
camera_poses  = poses(keyframe_storage);

[refined_points, refined_absolute_poses] = bundleAdjustment(world_points, tracks, ...
    camera_poses, intrinsics, FixedViewIDs=1, ...
    PointsUndistorted=true, AbsoluteTolerance=1e-7,...
    RelativeTolerance=1e-15, MaxIteration=20, ...
    Solver="preconditioned-conjugate-gradient");

% Scale the map and the camera pose using the median depth of map points
median_depth   = median(vecnorm(refined_points.'));
refined_points = refined_points / median_depth;

refined_absolute_poses.AbsolutePose(current_view_id).Translation = ...
    refined_absolute_poses.AbsolutePose(current_view_id).Translation / median_depth;
relative_pose.Translation = relative_pose.Translation/median_depth;

% Update key frames with the refined poses
keyframe_storage = updateView(keyframe_storage, refined_absolute_poses);
keyframe_storage = updateConnection(keyframe_storage, initial_view_id, current_view_id, relative_pose);

% Update map points with the refined positions
map_point_set = updateWorldPoints(map_point_set, new_point_index, refined_points);

% Update view direction and depth 
map_point_set = updateLimitsAndDirection(map_point_set, new_point_index, keyframe_storage.Views);

% Update representative view
map_point_set = updateRepresentativeView(map_point_set, new_point_index, keyframe_storage.Views);

% Visualize matched features in the current frame
close(hfeature.Parent.Parent);
feature_plot   = helperVisualizeMatchedFeatures(current_frame, current_points(index_pairs(:,2)));

% Visualize initial map points and camera trajectory
map_plot       = helperVisualizeMotionAndStructure(keyframe_storage, map_point_set);

% Show legend
showLegend(map_plot);

% ViewId of the current key frame
current_keyframe_id   = current_view_id;

% ViewId of the last key frame
last_keyframe_id   = current_view_id;

% Index of the last key frame in the input image sequence
last_keyframe_index  = current_frame_index - 1; 

% Indices of all the key frames in the input image sequence
added_frames_index   = [1; last_keyframe_index];

% Update legend
showLegend(map_plot);

% Main loop
isLastFrameKeyFrame = true;
while current_frame_index < numel(imds.Files)  
    current_frame = readimage(imds, current_frame_index);

    [current_features, current_points] = helperDetectAndExtractFeatures(current_frame, orb_scale_factor, orb_levels, orb_points, intrinsics);

    % Track the last key frame
    % mapPointsIdx:   Indices of the map points observed in the current frame
    % featureIdx:     Indices of the corresponding feature points in the 
    %                 current frame

    [current_pose, mapPointsIdx, featureIdx] = helperTrackLastKeyFrame(map_point_set, ...
        keyframe_storage.Views, current_features, current_points, last_keyframe_id, intrinsics, orb_scale_factor);

    if ~isa(current_pose,'rigidtform3d')
        current_pose = rigidtform3d;
    end
    
    % Track the local map and check if the current frame is a key frame.
    % A frame is a key frame if both of the following conditions are satisfied:
    %
    % 1. At least 20 frames have passed since the last key frame or the
    %    current frame tracks fewer than 100 map points.
    % 2. The map points tracked by the current frame are fewer than 90% of
    %    points tracked by the reference key frame.
    %
    % Tracking performance is sensitive to the value of orb_pointsKeyFrame.  
    % If tracking is lost, try a larger value.
    %
    % localKeyFrameIds:   ViewId of the connected key frames of the current frame
    numSkipFrames     = 20;
    orb_pointsKeyFrame = 80;
    [localKeyFrameIds, current_pose, mapPointsIdx, featureIdx, isKeyFrame] = ...
        helperTrackLocalMap(map_point_set, keyframe_storage, mapPointsIdx, ...
        featureIdx, current_pose, current_features, current_points, intrinsics, orb_scale_factor, orb_levels, ...
        isLastFrameKeyFrame, last_keyframe_index, current_frame_index, numSkipFrames, orb_pointsKeyFrame);

    % Visualize matched features
    updatePlot(feature_plot, current_frame, current_points(featureIdx));

    if ~isKeyFrame
        current_frame_index        = current_frame_index + 1;
        isLastFrameKeyFrame = false;
        continue
    else
        isLastFrameKeyFrame = true;
    end

    % Update current key frame ID
    current_keyframe_id  = current_keyframe_id + 1;

    % Add the new key frame 
    [map_point_set, keyframe_storage] = helperAddNewKeyFrame(map_point_set, keyframe_storage, ...
        current_pose, current_features, current_points, mapPointsIdx, featureIdx, localKeyFrameIds);

    % Remove outlier map points that are observed in fewer than 3 key frames
    outlierIdx    = setdiff(new_point_index, mapPointsIdx);
    if ~isempty(outlierIdx)
        map_point_set   = removeWorldPoints(map_point_set, outlierIdx);
    end

    % Create new map points by triangulation
    minNumMatches = 10;
    minimum_parallax   = 3;
    [map_point_set, keyframe_storage, new_point_index] = helperCreateNewMapPoints(map_point_set, keyframe_storage, ...
        current_keyframe_id, intrinsics, orb_scale_factor, minNumMatches, minimum_parallax);

    % Local bundle adjustment
    [refinedViews, dist] = connectedViews(keyframe_storage, current_keyframe_id, MaxDistance=2);
    if ~isempty(refinedViews)
        refinedKeyFrameIds = refinedViews.ViewId;
        fixedViewIds = refinedKeyFrameIds(dist==2);
        fixedViewIds = fixedViewIds(1:min(10, numel(fixedViewIds)));

        % Refine local key frames and map points
        [map_point_set, keyframe_storage, mapPointIdx] = bundleAdjustment(...
            map_point_set, keyframe_storage, [refinedKeyFrameIds; current_keyframe_id], intrinsics, ...
            FixedViewIDs=fixedViewIds, PointsUndistorted=true, AbsoluteTolerance=1e-7,...
            RelativeTolerance=1e-16, Solver="preconditioned-conjugate-gradient", ...
            MaxIteration=10);
    end

    % Update view direction and depth
    map_point_set = updateLimitsAndDirection(map_point_set, mapPointIdx, keyframe_storage.Views);

    % Update representative view
    map_point_set = updateRepresentativeView(map_point_set, mapPointIdx, keyframe_storage.Views);

    % Visualize 3D world points and camera trajectory
    updatePlot(map_plot, keyframe_storage, map_point_set);

    % Update IDs and indices
    last_keyframe_id  = current_keyframe_id;
    last_keyframe_index = current_frame_index;
    added_frames_index  = [added_frames_index; current_frame_index]; 
    current_frame_index    = current_frame_index + 1;
end % End of main loop
toc
unoptimised_poses = poses(keyframe_storage);

% Setup ground truth 
ground_truth_file = fopen(strcat(FILEPATH, "ground_truth.txt"), "r");
format_spec = "X, %f, Y, %f, Z, %f\n";
ground_truth_size = [3 Inf];
ground_truth = fscanf(ground_truth_file,format_spec, ground_truth_size)';
fclose(ground_truth_file);

ground_truth = ground_truth - ground_truth(1,:);

figure;
tiledlayout(2,3)
nexttile([1 3])
actual_trajectory  = vertcat(ground_truth);
plot3(actual_trajectory(:,1), actual_trajectory(:,2), actual_trajectory(:,3), ...
    'g','LineWidth',2, 'DisplayName', 'Actual trajectory');

hold on;
title('Visual Odometry Optimised Data')
xlabel('X [m]')
ylabel('Y [m]')
zlabel('Z [m]')
xlim([-250 250])
ylim([-250 250])
zlim([-250 250])

unoptimized_trajectory = vertcat(unoptimised_poses.AbsolutePose.Translation);

scale = median(vecnorm(actual_trajectory, 2, 2))/ median(vecnorm(unoptimized_trajectory, 2, 2))
scaled_trajectory = unoptimized_trajectory * scale;

% Path rotations in xyz, in degrees.
rx = 0;
ry = -60;
rz = 215;
rotation = rotx(rz) * roty(ry);

optimised_trajectory1 = scaled_trajectory * rotz(rz);
optimised_trajectory2 = optimised_trajectory1 * roty(ry);
% Path translation, in metres, after rotation.
tx = 0;
ty = 10;
tz = -17;

optimised_trajectory2(:,1) = optimised_trajectory2(:,1) + tx;
optimised_trajectory2(:,2) = optimised_trajectory2(:,2) + ty;
optimised_trajectory2(:,3) = optimised_trajectory2(:,3) + tz;

plot3(scaled_trajectory(:,1), scaled_trajectory(:,2), scaled_trajectory(:,3), ...
               'r','LineWidth',2, 'DisplayName', 'Scaled trajectory');
plot3(optimised_trajectory1(:,1), optimised_trajectory1(:,2), optimised_trajectory1(:,3), ...
               'b','LineWidth',2, 'DisplayName', 'Optimised trajectory');
plot3(optimised_trajectory2(:,1), optimised_trajectory2(:,2), optimised_trajectory2(:,3), ...
               'r','LineWidth',2, 'DisplayName', 'Optimised trajectory');

legend("location","northeast");
grid on;


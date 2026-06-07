%% READ BEFORE USING CODE %%

% There are system options  to choose from. Run only the appropriate
% sections (Separated by double percentage).

% "Initial Setup" 
%   MUST be run first, regardless of systems being run.

% "Perform Visual Odometry"
%   This finds the estimated path from the visual odometry. This takes the
%   longest time, so a workspace containing the required outputs is
%   provided as "SkipVisualOdometryWorkspace.mat".

% "Post VO Setup"
%   MUST be performed after either performing Visual Odometry, or importing
%   the visual odometry from the workspace.

%   The rest of the systems can then be run independantly, and in any order
%   as needed.

% "High-Gain Observer Only"
%   This validates the high-gain observer, by comparing the estimated to
%   the actual orientation.

% "GNSS Only"
%   This plots the RTK GNSS data.

% "Camera Only"
%   This plots the orientation and velocity (scale) corrected camera
%   estimate of the Visual Odometry.

% "Camera and Observer"
%   This plots the observer-corrected Visual Odometry. This also contains
%   the absolute position from the GNSS, which has been "estimated" through
%   (input to) the Observer.

% "Camera, Observer and GNSS"
%   This is the full system, as described in the paper.

% "Camera, Observer and GNSS, with GNSS Failure"
%   This is the full system, where there is a GNSS outage, as the vehicle
%   approaches the roundabout.

% "All Systems"
%   This shows the comparison of all the different systems, all plotted on
%   the same axis to show their differences.


%% INITIAL SETUP

% Use these options for your own routes. Offset determines if the initial
% position is at the origin or not.
Route = 4;
offset = 1;

% You may need to add your own MOSEK path, dependant on installation.
addpath F:\MOSEK\10.1\toolbox\r2017a;

% This determines the image and sensor source for your selected route.
% The System NEEDS:
%   FILEPATH: (eg: "F:/OpticalCameraOutput/.../")
%   ---> XXXXXX.png (Camera outputs, with 6 character frame number as the filename)
%   ---> ground_truth.txt (Formatted as requirements.txt)
%   ---> noisy_gnss.txt (Formatted as requirements.txt)
%   ---> vehicle_sensors.txt (Formatted as requirements.txt)
switch Route
    case (1)
        FILEPATH = "D:/Documents/OneDrive - University of Southampton/Part III Project-Automotive/Carla Programs/OpticalCameraOutput/30Hz/";
        tspan_observer = [0:0.00825:29.99];
        tspan_sensors = [0:0.00825:29.99];
        tspan_kalman_filter = tspan_observer;
    case (2)
        FILEPATH = "D:/Documents/OneDrive - University of Southampton/Part III Project-Automotive/Carla Programs/OpticalCameraOutput/30Hz2/";
        tspan_observer = [0:0.00825:29.99];
        tspan_sensors = [0:0.00825:29.99]; 
        tspan_kalman_filter = tspan_observer;
    case (3)
        FILEPATH = "D:/Documents/OneDrive - University of Southampton/Part III Project-Automotive/Carla Programs/OpticalCameraOutput/30HzSlow/"; 
        tspan_observer = [0:0.00825:29.99]; 
        tspan_sensors = [0:0.00825:29.99]; 
        tspan_kalman_filter = tspan_observer;
    case (4)
        FILEPATH = "F:/OpticalCameraOutput/30HzSlow2/" % CHANGE THIS TO YOUR LOCAL FILEPATH FOR THE SENSOR DATA
        timestep = 0.00825;
        timespan_camera = [0:(timestep*4):(4241*timestep)];
        tspan_observer = [0:timestep:(4241*timestep)];
        tspan_sensors = tspan_observer; 
        tspan_kalman_filter = tspan_observer;
        tspan_gnss = [0:0.0333:(4241*timestep)];
        tspan_ground_truth = tspan_observer;

end 
% Set the FIGUREPATH to where you want .eps figures output.
FIGUREPATH = "D:/Documents/OneDrive - University of Southampton/Part III Project-Automotive/misc";

% These are the default options.
enable_observer = 1;
enable_camera = 0;
camera_correct_orientation = 0;
camera_correct_velocity = 0;
enable_gnss = 1;
gnss_fail_start = 999;
gnss_fail_end = 1000;
enable_input_kf = 0;

% Setup ground truth 
ground_truth_file = fopen(strcat(FILEPATH, "ground_truth.txt"), "r");
format_spec_ground_truth = "X, %f, Y, %f, Z, %f\n";
ground_truth_size = [3 Inf];
ground_truth = fscanf(ground_truth_file, format_spec_ground_truth, ground_truth_size)';
fclose(ground_truth_file);
if(offset)
    ground_truth = ground_truth - ground_truth(1,:);
end

% Setup wheeled sensors
sensor_file = fopen(strcat(FILEPATH, "vehicle_sensors.txt"), "r");
format_spec_sensors = "SteeringAngle, %f, Speed, %f, Rotation, %f\n";
sensor_size = [3 Inf];
sensor_data = fscanf(sensor_file, format_spec_sensors, sensor_size)';
fclose(sensor_file);

sensor_data(:,1) = round(sensor_data(:,1) / 1.5) * 1.5 * pi / 180; 
% Round to nearest 1.5 degrees.
%sensor_data(:,3) = sensor_data(:,3) * pi / 180;

v = sensor_data(:,2);
n = 6;
m0=5*ones(n,1);


%% Perform Visual Odometry

run('VisualOdometry.m')


%% POST-VO SETUP 

run("KalmanFilter.m")
run("HighGainObserver.m")


%% High Gain Observer Only
close all

run('HighGainObserver.m')

fig1 = figure('Name','High Gain Observer Simulation','NumberTitle','off');
fig1.Position = [100, 100, 1200, 800];
offset = 0;
% xygraphpos = [0.1 0.6 0.8 0.35];
% subplot('position', xygraphpos);
% line1 = plot(ground_truth(:,1),ground_truth(:,2),'color','blue');
% line1.LineWidth = 2.0;
% hold on
% line2 = plot(x_hat_observer2(:,1),x_hat_observer2(:,4),':','color','red');
% line2.LineWidth = 2.0;
% hold off
% xlabel('$X [m]$','fontsize',16,'Interpreter','latex')
% ylabel('$Y [m]$','fontsize',16,'Interpreter','latex')
% axis equal
% %xlim([-1 40])
% %ylim([-1 40])
% title('Simulated Vehicle Trajectory')
% leg1 = legend({'$$Model Pos.$$','$$Est Pos.$$'},'Location','southwest','Interpreter','Latex');
% leg1.AutoUpdate = 'off';
% ax = gca;
% ax.FontWeight = "bold";
% ax.FontSize = 18;
% grid on

xgraphpos = [0.1 0.6 0.35 0.35];
subplot('position', xgraphpos);
line1 = plot(tspan_ground_truth, ground_truth(:,1),'color','blue');
line1.LineWidth = 2.0;
hold on 
line2 = plot(tspan_observer, x_hat_observer2(:,1),'color','red');
line2.LineWidth = 1.0;
hold off
ylabel('$X [m]$','fontsize',16,'Interpreter','latex')
xlabel('$t [s]$','fontsize',16,'Interpreter','latex')
%xlim([0 12.5])
%ylim([-10 80])
legend({'$$Gnd Truth.$$','$$\hat{z_1} (X).$$'},'Location','northeast','Interpreter','Latex')
ax = gca;
ax.FontWeight = "bold";
ax.FontSize = 18;
grid on

ygraphpos = [0.55 0.6 0.35 0.35];
subplot('position', ygraphpos);
line1 = plot(tspan_ground_truth, ground_truth(:,2),'color','blue');
line1.LineWidth = 2.0;
hold on 
line2 = plot(tspan_observer, x_hat_observer2(:,4),'color','red');
line2.LineWidth = 1.0;
hold off
ylabel('$Y [m]$','fontsize',16,'Interpreter','latex')
xlabel('$t [s]$','fontsize',16,'Interpreter','latex')
%xlim([0 12.5])
%ylim([-85 5])
legend({'$$Gnd Truth.$$','$$\hat{z_4} (Y).$$'},'Location','northwest','Interpreter','Latex')
ax = gca;
ax.FontWeight = "bold";
ax.FontSize = 18;
grid on

% rectangle('Position',[-0.5 -0.5 4 2])
% 
% line([-0.5, -15], [1.5, 12], 'Color', 'Black');
% line([3.5, 9], [1.5, 12], 'Color', 'Black');

% zoompos = [0.4 0.71 0.25 0.125];
% axzoom = axes('position',zoompos);
% box on
% line1 = plot(x_model(:,1),x_model(:,2),'color','blue');
% line1.LineWidth = 2.0;
% hold on
% line2 = plot(x_hat_observer2(:,1),x_hat_observer2(:,4),':','color','red');
% line2.LineWidth = 2.0;
% xlim([-0.5 3.5])
% ylim([-0.5 1.5])
% grid on

vgraphpos = [0.1 0.1 0.35 0.35];
subplot('position', vgraphpos);
line1 = plot(tspan_sensors,v,'color','blue');
line1.LineWidth = 2.0;
hold on 
line2 = plot(tspan_observer,v_hat,'color','red');
line2.LineWidth = 2.0;
ylabel('$V[m/s]$','fontsize',16,'Interpreter','latex')
%xlim([0 12.5])
%ylim([0 14])
leg4 = legend({'$V$','$\hat{V} = sqrt(\hat{z_5}^2 + \hat{z_2}^2)$'},'Location','northwest','Interpreter','Latex');
leg4.AutoUpdate = 'off';
ax = gca;
ax.FontWeight = "bold";
ax.FontSize = 18;
grid on

% rectangle('Position',[3 9.5 2.5 1.5])
% 
% line([3, 7], [9.5, 4], 'Color', 'Black');
% line([5.5, 13.5], [11, 8], 'Color', 'Black');
% 
% zoompos = [0.26 0.335 0.17 0.1];
% axzoom = axes('position',zoompos);
% box on
% line1 = plot(t,x_model(:,3),'color','blue');
% line1.LineWidth = 2.0;
% hold on 
% line2 = plot(t,v(:),':','color','red');
% line2.LineWidth = 2.0;
% xlim([3 5.5])
% ylim([9.5 11])
% grid on

psigraphpos = [0.55 0.1 0.35 0.35];
subplot('position', psigraphpos);
line1 = plot(tspan_sensors,AbsoluteAngleDeg(sensor_data(:,3)),'color','blue');
line1.LineWidth = 2.0;
hold on 
line2 = plot(t2,AbsoluteAngleDeg(Psi(:)),':','color','red');
line2.LineWidth = 2.0;
hold off
ylabel('$\psi [degree]$','fontsize',16,'Interpreter','latex')
%xlim([0 12.5])
%ylim([-10 370])
leg3 = legend({'$\psi$','$\hat{\psi} = atan2(\hat{z_5}, \hat{z_2})$'},'Location','northeast','Interpreter','Latex');
leg3.AutoUpdate = 'off';
ax = gca;
ax.FontWeight = "bold";
ax.FontSize = 18;
grid on

% rectangle('Position',[-0.5 -0.5 1.5 45])
% 
% line([-0.5, 0.6], [44.5, 160], 'Color', 'Black');
% line([1.0, 5], [44.5, 160], 'Color', 'Black');

% zoompos = [0.575 0.4 0.1 0.11];
% axzoom = axes('position',zoompos);
% box on
% line1 = plot(t,x_model(:,4)*180/pi,'color','blue');
% line1.LineWidth = 2.0;
% hold on 
% line2 = plot(t2,Psi*180/pi,':','color','red');
% line2.LineWidth = 2.0;
% xlim([-0.5 1])
% ylim([-0.5 44.5])
% grid on

% agraphpos = [0.1 0.1 0.35 0.15];
% subplot('position', agraphpos);
% line1 = plot(tspan_sensors, sensor_data(:,2),'color','blue');
% line1.LineWidth = 2.0;
% xlabel('$t [s]$','fontsize',16,'Interpreter','latex')
% ylabel('$a [m/s^2]$','fontsize',16,'Interpreter','latex')
% %xlim([0 12.5])
% %ylim([-1.1 1.3])
% ax = gca;
% ax.FontWeight = "bold";
% ax.FontSize = 18;
% grid on
% 
% dfgraphpos = [0.55 0.1 0.35 0.15];
% subplot('position', dfgraphpos);
% line1 = plot(tspan_sensors, sensor_data(:,1),'color','blue');
% line1.LineWidth = 2.0;
% xlabel('$t [s]$','fontsize',16,'Interpreter','latex')
% ylabel('$\delta_f [degree]$','fontsize',16,'Interpreter','latex')
% %xlim([0 12.5])
% %ylim([-1 13])
% ax = gca;
% ax.FontWeight = "bold";
% ax.FontSize = 18;
% grid on

%print(strcat(FIGUREPATH, "SimulatedObserver.eps"), '-depsc');


fig2 = figure('Name','High Gain Observer Errors','NumberTitle','off');
fig2.Position = [100, 100, 1200, 800];

ex1graphpos = [0.1 0.6 0.35 0.35];
subplot('position', ex1graphpos);
msre_x1 = 100 * abs(x_hat_observer2(:,1) - ground_truth(:,1))/norm(ground_truth(:,1),inf);
line2 = plot(t2,msre_x1,'color','red');
line2.LineWidth = 2.0;
ylabel('$X\:MSRE [\%]$','fontsize',16,'Interpreter','latex')
%xlim([0 12.5])
%ylim([0 15])
ax = gca;
ax.FontWeight = "bold";
ax.FontSize = 18;
grid on

% rectangle('Position',[4.5 -0.05 2 0.1])
% 
% line([4.5, 2.5], [0.05, 0.65], 'Color', 'Black');
% line([6.5, 10.5], [0.05, 0.65], 'Color', 'Black');
% 
% zoompos = [0.15 0.7 0.2 0.1];
% axzoom = axes('position',zoompos);
% box on
% line2 = plot(t,msre_x1,'color','red');
% line2.LineWidth = 2.0;
% xlim([4.5 6.5])
% ylim([-0.05 0.05])
% grid on


ex2graphpos = [0.55 0.6 0.35 0.35];
subplot('position', ex2graphpos);
msre_x2 = 100 * abs(x_hat_observer2(:,4) - ground_truth(:,2))/norm(ground_truth(:,2),inf);
line2 = plot(t2,msre_x2,'color','red');
line2.LineWidth = 2.0;
ylabel('$Y\:MSRE [\%]$','fontsize',16,'Interpreter','latex')
%xlim([0 12.5])
%ylim([0 15])
ax = gca;
ax.FontWeight = "bold";
ax.FontSize = 18;
grid on

% rectangle('Position',[4.5 -0.05 2 0.1])
% 
% line([4.5, 2.5], [0.05, 0.8], 'Color', 'Black');
% line([6.5, 10.5], [0.05, 0.8], 'Color', 'Black');
% 
% zoompos = [0.6 0.7 0.2 0.1];
% axzoom = axes('position',zoompos);
% box on
% line2 = plot(t,msre_x2,'color','red');
% line2.LineWidth = 2.0;
% xlim([4.5 6.5])
% ylim([-0.05 0.05])
% grid on

% ex3graphpos = [0.1 0.15 0.35 0.35];
% subplot('position', ex3graphpos);
% msre_x3 = 100 * abs(x_hat_observer2(:,3) - x_model(:,3))/norm(x_model(:,3),inf);
% line2 = plot(t,msre_x3,'color','red');
% line2.LineWidth = 2.0;
% xlabel('$t [s]$','fontsize',16,'Interpreter','latex')
% ylabel('$V\:MSRE [\%]$','fontsize',16,'Interpreter','latex')
% %xlim([0 12.5])
% %ylim([0 15])
% ax = gca;
% ax.FontWeight = "bold";
% ax.FontSize = 18;
% grid on

% rectangle('Position',[4.5 -0.1 2 2.6])
% 
% line([4.5, 2.5], [2.5, 22], 'Color', 'Black');
% line([6.5, 10.5], [2.5, 22], 'Color', 'Black');

% zoompos = [0.15 0.25 0.2 0.1];
% axzoom = axes('position',zoompos);
% box on
% line2 = plot(t,msre_x3,'color','red');
% line2.LineWidth = 2.0;
% xlim([4.5 6.5])
% ylim([-0.1 2.5])
% grid on


ex4graphpos = [0.1 0.15 0.8 0.35];
subplot('position', ex4graphpos);
msre_x4 = 100 * abs(AbsoluteAngleDeg(Psi(:)) - AbsoluteAngleDeg(sensor_data(:,3)))/norm(sensor_data(:,3),inf);
line2 = plot(t2,msre_x4,'color','red');
line2.LineWidth = 2.0;
hold on
msre_x5 = 100 * abs(AbsoluteAngleDeg(PsiNoBeta(:)) - AbsoluteAngleDeg(sensor_data(:,3)))/norm(sensor_data(:,3),inf);
line2 = plot(t2,msre_x5,'color','blue');
line2.LineWidth = 2.0;
xlabel('$t [s]$','fontsize',16,'Interpreter','latex')
ylabel('$\psi\:MSRE [\%]$','fontsize',16,'Interpreter','latex')
%xlim([0 12.5])
%ylim([0 15])
ax = gca;
ax.FontWeight = "bold";
ax.FontSize = 18;
grid on

% rectangle('Position',[4.5 -0.1 2 1.1])
% 
% line([4.5, 2.5], [1, 3.2], 'Color', 'Black');
% line([6.5, 10.5], [1, 3.2], 'Color', 'Black');
% 
% zoompos = [0.6 0.25 0.2 0.1];
% axzoom = axes('position',zoompos);
% box on
% line2 = plot(t,msre_x4,'color','red');
% line2.LineWidth = 2.0;
% xlim([4.5 6.5])
% ylim([-0.1 1.0])
% grid on


%% GNSS Sensor ONLY

run('KalmanFilter.m')

fig1 = figure('Name','GNSS Sensor','NumberTitle','off');
fig1.Position = [100, 100, 1000, 1200];

xygraphpos = [0.1 0.6 0.8 0.35];
subplot('position', xygraphpos);
line1 = plot(ground_truth(:,1),ground_truth(:,2),'color','blue');
line1.LineWidth = 2.0;
hold on 
line2 = plot(gnss_data(:,1),gnss_data(:,2),'color','red');
line2.LineWidth = 1.0;
xlabel('$X [m]$','fontsize',16,'Interpreter','latex')
ylabel('$Y [m]$','fontsize',16,'Interpreter','latex')
axis equal
%xlim([-10 80])
%xticks(-70:10:110);
ylim([-20 60])
title('Vehicle Trajectory - Ground Truth and GNSS Sensor Measurments')
leg1 = legend({'$$Gnd Truth.$$','$$GNSS.$$'},'Location','southeast','Interpreter','Latex');
leg1.AutoUpdate = 'off';
ax1 = gca;
ax1.FontWeight = "bold";
ax1.FontSize = 18;
ax1.YDir = "reverse";
grid on
% 
% rectangle('Position',[40 -0.5 10 5])
% 
% line([40, 29], [4.5, 20], 'Color', 'Black');
% line([50, 88], [4.5, 20], 'Color', 'Black');
% 
% zoompos = [0.4 0.7 0.4 0.175];
% axzoom = axes('position',zoompos);
% box on
% line1 = plot(ground_truth(:,1),ground_truth(:,2),'color','blue');
% line1.LineWidth = 2.0;
% hold on 
% line2 = plot(gnss_data(:,1),gnss_data(:,2),'color','red');
% line2.LineWidth = 2.0;
% xlim([40 50])
% ylim([-0.5 4.5])
% axzoom.YDir = "reverse";
% grid on


xgraphpos = [0.1 0.325 0.35 0.2];
subplot('position', xgraphpos);
line1 = plot(tspan_ground_truth, ground_truth(:,1),'color','blue');
line1.LineWidth = 2.0;
hold on 
line2 = plot(tspan_gnss, gnss_data(:,1)','color','red');
line2.LineWidth = 1.0;
hold off
ylabel('$X [m]$','fontsize',16,'Interpreter','latex')
xlabel('$t [s]$','fontsize',16,'Interpreter','latex')
%xlim([0 12.5])
ylim([-10 80])
legend({'$$Gnd Truth.$$','$$GNSS.$$'},'Location','northwest','Interpreter','Latex')
ax = gca;
ax.FontWeight = "bold";
ax.FontSize = 18;
grid on

ygraphpos = [0.55 0.325 0.35 0.2];
subplot('position', ygraphpos);
line1 = plot(tspan_ground_truth, ground_truth(:,2),'color','blue');
line1.LineWidth = 2.0;
hold on 
line2 = plot(tspan_gnss, gnss_data(:,2)','color','red');
line2.LineWidth = 1.0;
hold off
ylabel('$Y [m]$','fontsize',16,'Interpreter','latex')
xlabel('$t [s]$','fontsize',16,'Interpreter','latex')
%xlim([0 12.5])
ylim([-85 5])
legend({'$$Gnd Truth.$$','$$GNSS.$$'},'Location','northeast','Interpreter','Latex')
ax = gca;
ax.FontWeight = "bold";
ax.FontSize = 18;
grid on

dfgraphpos = [0.1 0.1 0.8 0.15];
subplot('position', dfgraphpos);
line1 = plot(tspan_ground_truth, sensor_data(:,1) * 180/pi,'color','red');
line1.LineWidth = 2.0;
xlabel('$t [s]$','fontsize',16,'Interpreter','latex')
ylabel('$\delta_f [degree]$','fontsize',16,'Interpreter','latex')
%xlim([0 12.5])
%ylim([-1 13])
ax = gca;
ax.FontWeight = "bold";
ax.FontSize = 18;
grid on

print(strcat(FIGUREPATH, "GNSSFigure.eps"), '-depsc');


%% Camera only

close all;

enable_observer = 0;
enable_camera = 1;
camera_correct_orientation = 1;
camera_correct_velocity = 1;
enable_gnss = 0;
gnss_fail_start = 12000;
gnss_fail_end = 14000;

run('KalmanFilter.m')

fig1 = figure('Name','Error State Extended Kalman Filter','NumberTitle','off');
fig1.Position = [100, 100, 1000, 1000];
        
%xygraphpos = [0.1 0.4 0.8 0.55];
%subplot('position', xygraphpos);

line1 = plot(ground_truth(:,1),ground_truth(:,2),'color','blue');
line1.LineWidth = 2.0;
hold on 
line2 = plot(pose_storage(1,:),pose_storage(2,:),':','color','red');
line2.LineWidth = 2.0;
hold off
xlabel('$X [m]$','fontsize',16,'Interpreter','latex')
ylabel('$Y [m]$','fontsize',16,'Interpreter','latex')
axis equal
%xlim([0 100])
%ylim([-5 1])
title('VO Estimated Vehicle Trajectory, with V and \psi Path Corrections')
legend({'$$Gnd Truth.$$','$$Est Pos.$$'},'Location','southeast','Interpreter','Latex')
ax = gca;
ax.FontWeight = "bold";
ax.FontSize = 18;
grid on
ax.YDir = "reverse";

% xgraphpos = [0.1 0.1 0.35 0.2];
% subplot('position', xgraphpos);
% line1 = plot(tspan_ground_truth, ground_truth(:,1),'color','blue');
% line1.LineWidth = 2.0;
% hold on 
% line2 = plot(tspan_kalman_filter, pose_storage(1,:),':','color','red');
% line2.LineWidth = 2.0;
% hold off
% ylabel('$X [m]$','fontsize',16,'Interpreter','latex')
% xlabel('$t [s]$','fontsize',16,'Interpreter','latex')
% %xlim([0 12.5])
% %ylim([-10 80])
% legend({'$$Gnd Truth.$$','$$EstPos.$$'},'Location','southeast','Interpreter','Latex')
% ax = gca;
% ax.FontWeight = "bold";
% ax.FontSize = 18;
% grid on
% 
% ygraphpos = [0.55 0.1 0.35 0.2];
% subplot('position', ygraphpos);
% line1 = plot(tspan_ground_truth, ground_truth(:,2),'color','blue');
% line1.LineWidth = 2.0;
% hold on 
% line2 = plot(tspan_kalman_filter, pose_storage(2,:),':','color','red');
% line2.LineWidth = 2.0;
% hold off
% ylabel('$Y [m]$','fontsize',16,'Interpreter','latex')
% xlabel('$t [s]$','fontsize',16,'Interpreter','latex')
% %xlim([0 12.5])
% %ylim([-5 85])
% legend({'$$Gnd Truth.$$','$$EstPos.$$'},'Location','east','Interpreter','Latex')
% ax = gca;
% ax.FontWeight = "bold";
% ax.FontSize = 18;
% grid on

print(strcat(FIGUREPATH, "KalmanFilterCameraOnly.eps"), '-depsc');


%% Camera and Observer

enable_observer = 1;
enable_camera = 0;
camera_correct_orientation = 0;
camera_correct_velocity = 0;
enable_gnss = 0;
gnss_fail_start = 999;
gnss_fail_end = 1000;

run('KalmanFilter.m')


fig1 = figure('Name','Error State Extended Kalman Filter','NumberTitle','off');
fig1.Position = [100, 100, 1000, 1200];
        
xygraphpos = [0.1 0.65 0.8 0.3];
subplot('position', xygraphpos);

line1 = plot(ground_truth(:,1),ground_truth(:,2),'color','blue');
line1.LineWidth = 2.0;
hold on 
line2 = plot(pose_storage(1,:),pose_storage(2,:),':','color','red');
line2.LineWidth = 2.0;
hold off
xlabel('$X [m]$','fontsize',16,'Interpreter','latex')
ylabel('$Y [m]$','fontsize',16,'Interpreter','latex')
axis equal
%xlim([0 100])
%ylim([-5 1])
title('Corrected Camera Estimated Vehicle Trajectory')
legend({'$$Gnd Truth.$$','$$Est Pos.$$'},'Location','southeast','Interpreter','Latex')
ax = gca;
ax.FontWeight = "bold";
ax.FontSize = 18;
grid on
ax.YDir = "reverse";

xgraphpos = [0.1 0.375 0.35 0.2];
subplot('position', xgraphpos);
line1 = plot(tspan_ground_truth, ground_truth(:,1),'color','blue');
line1.LineWidth = 2.0;
hold on 
line2 = plot(tspan_kalman_filter, pose_storage(1,:),':','color','red');
line2.LineWidth = 2.0;
hold off
ylabel('$X [m]$','fontsize',16,'Interpreter','latex')
xlabel('$t [s]$','fontsize',16,'Interpreter','latex')
%xlim([0 12.5])
%ylim([-10 80])
legend({'$$Gnd Truth.$$','$$EstPos.$$'},'Location','southeast','Interpreter','Latex')
ax = gca;
ax.FontWeight = "bold";
ax.FontSize = 18;
grid on

ygraphpos = [0.55 0.375 0.35 0.2];
subplot('position', ygraphpos);
line1 = plot(tspan_ground_truth, ground_truth(:,2),'color','blue');
line1.LineWidth = 2.0;
hold on 
line2 = plot(tspan_kalman_filter, pose_storage(2,:),':','color','red');
line2.LineWidth = 2.0;
hold off
ylabel('$Y [m]$','fontsize',16,'Interpreter','latex')
xlabel('$t [s]$','fontsize',16,'Interpreter','latex')
%xlim([0 12.5])
%ylim([-5 85])
legend({'$$Gnd Truth.$$','$$EstPos.$$'},'Location','northeast','Interpreter','Latex')
ax = gca;
ax.FontWeight = "bold";
ax.FontSize = 18;
grid on

xerrgraphpos = [0.1 0.1 0.35 0.2];
subplot('position', xerrgraphpos);
ground_truth_interp = interp1(tspan_ground_truth,ground_truth,tspan_kalman_filter);
line1 = plot(tspan_kalman_filter, abs(pose_storage(1,:)' - ground_truth_interp(:,1)),'color','red');
line1.LineWidth = 1.0;
hold off
xlabel('$t [s]$','fontsize',16,'Interpreter','latex')
ylabel('$Absolute Error [m]$','fontsize',16,'Interpreter','latex')
leg4 = legend({'$|\hat{x}_1-x_1|$'},'Location','southeast','Interpreter','Latex');
%xlim([0 12.5])
%ylim([0 15])
ax = gca;
ax.FontWeight = "bold";
ax.FontSize = 18;
grid on

yerrgraphpos = [0.55 0.1 0.35 0.2];
subplot('position', yerrgraphpos);
erry = abs(pose_storage(2,:)' - ground_truth_interp(:,2));
line1 = plot(tspan_kalman_filter, erry,'color','red');
line1.LineWidth = 1.0;
xlabel('$t [s]$','fontsize',16,'Interpreter','latex')
ylabel('$Absolute Error [m]$','fontsize',16,'Interpreter','latex')
leg4 = legend({'$|\hat{x}_2-x_2|$'},'Location','northeast','Interpreter','Latex');
leg4.AutoUpdate = 'off';
%xlim([0 12.5])
%ylim([0 15])
ax = gca;
ax.FontWeight = "bold";
ax.FontSize = 18;
grid on

print(strcat(FIGUREPATH, "KalmanFilterCameraObserver.eps"), '-depsc');



%% Camera, Observer and GNSS
close all
enable_observer = 0;
enable_camera = 1;
camera_correct_orientation = 1;
camera_correct_velocity = 1;
enable_gnss = 1;
gnss_fail_start = 15000;
gnss_fail_end = 24000;
offset = 1;
enable_input_kf = 1;

run('KalmanFilter.m')

fig1 = figure('Name','Error State Extended Kalman Filter','NumberTitle','off');
fig1.Position = [100, 100, 1000, 1200];
        
xygraphpos = [0.1 0.65 0.8 0.3];
subplot('position', xygraphpos);

line1 = plot(ground_truth(:,1),ground_truth(:,2),'color','blue');
line1.LineWidth = 2.0;
hold on 
line2 = plot(pose_storage(1,:),pose_storage(2,:),':','color','red');
line2.LineWidth = 2.0;
hold off
xlabel('$X [m]$','fontsize',16,'Interpreter','latex')
ylabel('$Y [m]$','fontsize',16,'Interpreter','latex')
axis equal
%xlim([0 100])
%ylim([-5 1])
title('Kalman Filter Estimated Vehicle Trajectory')
leg5 = legend({'$$Gnd Truth.$$','$$Est Pos.$$'},'Location','southeast','Interpreter','Latex');
leg5.AutoUpdate = 'Off';
ax = gca;
ax.FontWeight = "bold";
ax.FontSize = 18;
grid on
ax.YDir = "reverse";

rectangle('Position',[-5 -3 12 9])

line([-5, -45], [-3, 8], 'Color', 'Black');
line([7, -10], [6, 35], 'Color', 'Black');

zoompos = [0.425 0.7 0.2 0.15];
axzoom = axes('position',zoompos);
box on
line1 = plot(ground_truth(:,1),ground_truth(:,2),'color','blue');
line1.LineWidth = 2.0;
hold on
line2 = plot(pose_storage(1,:),pose_storage(2,:),':','color','red');
line2.LineWidth = 2.0;
xlim([-5 7])
ylim([-3 6])
ax1 = gca;
ax1.YDir = "reverse";
grid on

xgraphpos = [0.1 0.375 0.35 0.2];
subplot('position', xgraphpos);
line1 = plot(tspan_ground_truth, ground_truth(:,1),'color','blue');
line1.LineWidth = 2.0;
hold on 
line2 = plot(tspan_kalman_filter, pose_storage(1,:),':','color','red');
line2.LineWidth = 2.0;
hold off
ylabel('$X [m]$','fontsize',16,'Interpreter','latex')
xlabel('$t [s]$','fontsize',16,'Interpreter','latex')
%xlim([0 12.5])
%ylim([-10 80])
legend({'$$Gnd Truth.$$','$$EstPos.$$'},'Location','northeast','Interpreter','Latex')
ax = gca;
ax.FontWeight = "bold";
ax.FontSize = 18;
grid on

ygraphpos = [0.55 0.375 0.35 0.2];
subplot('position', ygraphpos);
line1 = plot(tspan_ground_truth, ground_truth(:,2),'color','blue');
line1.LineWidth = 2.0;
hold on 
line2 = plot(tspan_kalman_filter, pose_storage(2,:),':','color','red');
line2.LineWidth = 2.0;
hold off
ylabel('$Y [m]$','fontsize',16,'Interpreter','latex')
xlabel('$t [s]$','fontsize',16,'Interpreter','latex')
%xlim([0 12.5])
%ylim([-5 85])
legend({'$$Gnd Truth.$$','$$EstPos.$$'},'Location','northwest','Interpreter','Latex')
ax = gca;
ax.FontWeight = "bold";
ax.FontSize = 18;
grid on

xerrgraphpos = [0.1 0.1 0.35 0.2];
subplot('position', xerrgraphpos);
ground_truth_interp = interp1(tspan_ground_truth,ground_truth,tspan_kalman_filter);
line1 = plot(tspan_kalman_filter, abs(pose_storage(1,:)' - ground_truth_interp(:,1)),'color','red');
line1.LineWidth = 1.0;
hold off
xlabel('$t [s]$','fontsize',16,'Interpreter','latex')
ylabel('$Absolute Error [m]$','fontsize',16,'Interpreter','latex')
leg4 = legend({'$|\hat{x}_1-x_1|$'},'Location','northwest','Interpreter','Latex');
leg4.AutoUpdate = 'off';
%xlim([0 12.5])
%ylim([0 15])
ax = gca;
ax.FontWeight = "bold";
ax.FontSize = 18;
grid on

rectangle('Position',[1 0 2 0.5])

line([1, 6], [0.5, 4], 'Color', 'Black');
line([3, 28], [0, 2], 'Color', 'Black');

zoompos = [0.15 0.15 0.2 0.1];
axzoom = axes('position',zoompos);
box on
line1 = plot(tspan_kalman_filter, abs(pose_storage(1,:)' - ground_truth_interp(:,1)),'color','red');
line1.LineWidth = 1.0;
xlim([1 3])
ylim([0 0.5])
%ax1 = gca;
%ax1.YDir = "reverse";
grid on

yerrgraphpos = [0.55 0.1 0.35 0.2];
subplot('position', yerrgraphpos);
erry = abs(pose_storage(2,:)' - ground_truth_interp(:,2));
line1 = plot(tspan_kalman_filter, erry,'color','red');
line1.LineWidth = 1.0;
xlabel('$t [s]$','fontsize',16,'Interpreter','latex')
ylabel('$Absolute Error [m]$','fontsize',16,'Interpreter','latex')
leg4 = legend({'$|\hat{x}_2-x_2|$'},'Location','northeast','Interpreter','Latex');
leg4.AutoUpdate = 'off';
%xlim([0 12.5])
%ylim([0 15])
ax = gca;
ax.FontWeight = "bold";
ax.FontSize = 18;
grid on

rectangle('Position',[20 0 2 0.5])

line([20, 12], [0, 2], 'Color', 'Black');
line([22, 33], [0, 2], 'Color', 'Black');

zoompos = [0.65 0.15 0.2 0.1];
axzoom = axes('position',zoompos);
box on
line1 = plot(tspan_kalman_filter, abs(pose_storage(2,:)' - ground_truth_interp(:,2)),'color','red');
line1.LineWidth = 1.0;
xlim([20 22])
ylim([0 0.5])
%ax1 = gca;
%ax1.YDir = "reverse";
grid on

% rectangle('Position',[11 -0.1 2 2.6])
% 
% line([11, 5], [2.5, 13], 'Color', 'Black');
% line([13, 15], [2.5, 13], 'Color', 'Black');
% 
% zoompos = [0.6 0.13 0.15 0.1];
% axzoom = axes('position',zoompos);
% box on
% line1 = plot(tspan_observer, erry,'color','red');
% line1.LineWidth = 2.0;
% xlim([11 13])
% ylim([-0.1 2.5])
% grid on

print(strcat(FIGUREPATH, "KalmanFilterAll1.eps"), '-depsc');

%% Camera, Observer and GNSS, fail as t=4.25 and return at t=33.33

enable_observer = 0;
enable_camera = 1;
camera_correct_orientation = 1;
camera_correct_velocity = 1;
enable_gnss = 1;
gnss_fail_start = 510;
gnss_fail_end = 4000;
offset = 1;

run('KalmanFilter.m')

fig1 = figure('Name','Error State Extended Kalman Filter','NumberTitle','off');
fig1.Position = [100, 100, 1000, 1200];
        
xygraphpos = [0.1 0.65 0.8 0.3];
subplot('position', xygraphpos);

line1 = plot(ground_truth(:,1),ground_truth(:,2),'color','blue');
line1.LineWidth = 2.0;
hold on 
line2 = plot(pose_storage(1,:),pose_storage(2,:),':','color','red');
line2.LineWidth = 2.0;
hold off
xlabel('$X [m]$','fontsize',16,'Interpreter','latex')
ylabel('$Y [m]$','fontsize',16,'Interpreter','latex')
axis equal
%xlim([0 100])
%ylim([-90 5])
title('Kalman Filter Estimated Vehicle Trajectory with GPS Error')
leg6 = legend({'$$Gnd Truth.$$','$$Est Pos.$$'},'Location','southeast','Interpreter','Latex');
leg6.AutoUpdate = 'Off';
ax = gca;
ax.FontWeight = "bold";
ax.FontSize = 18;
grid on
ax.YDir = "reverse";

line([ground_truth(gnss_fail_start,1), -25], [ground_truth(gnss_fail_start,2), 15], 'Color', 'Black');
text(-24.8,15,"GNSS Failure",'FontSize',16,'FontWeight','bold');
line([ground_truth(gnss_fail_end,1), -50], [ground_truth(gnss_fail_end,2), 40], 'Color', 'Black');
text(-49.8,40,"GNSS Returns",'FontSize',16,'FontWeight','bold');

% rectangle('Position',[0 0 10 8])
% 
% line([0, 30], [10, 60], 'Color', 'Black');
% line([10, 60], [0, 20], 'Color', 'Black');
% % 
% zoompos = [0.45 0.72 0.2 0.16];
% axzoom = axes('position',zoompos);
% box on
% line1 = plot(ground_truth(:,1),ground_truth(:,2),'color','blue');
% line1.LineWidth = 2.0;
% hold on
% line2 = plot(pose_storage(1,:),pose_storage(2,:),':','color','red');
% line2.LineWidth = 2.0;
% xlim([0 10])
% ylim([0 8])
% ax1 = gca;
% ax1.YDir = "reverse";
% grid on


xgraphpos = [0.1 0.375 0.35 0.2];
subplot('position', xgraphpos);
line1 = plot(tspan_ground_truth, ground_truth(:,1),'color','blue');
line1.LineWidth = 2.0;
hold on 
line2 = plot(tspan_kalman_filter, pose_storage(1,:),':','color','red');
line2.LineWidth = 2.0;
hold off
ylabel('$X [m]$','fontsize',16,'Interpreter','latex')
xlabel('$t [s]$','fontsize',16,'Interpreter','latex')
%xlim([0 12.5])
%ylim([-10 80])
leg6 = legend({'$$Gnd Truth.$$','$$EstPos.$$'},'Location','southeast','Interpreter','Latex')
leg6.AutoUpdate = 'off';
ax = gca;
ax.FontWeight = "bold";
ax.FontSize = 18;
grid on

line([gnss_fail_start * timestep, 2], [ground_truth(gnss_fail_start,1), 10], 'Color', 'Black');
text(0.2,12,"GNSS Failure",'FontSize',16,'FontWeight','bold');
line([gnss_fail_end * timestep, 10], [ground_truth(gnss_fail_end,1), 23], 'Color', 'Black');
text(3,25,"GNSS Returns",'FontSize',16,'FontWeight','bold');

ygraphpos = [0.55 0.375 0.35 0.2];
subplot('position', ygraphpos);
line1 = plot(tspan_ground_truth, ground_truth(:,2),'color','blue');
line1.LineWidth = 2.0;
hold on 
line2 = plot(tspan_kalman_filter, pose_storage(2,:),':','color','red');
line2.LineWidth = 2.0;
hold off
ylabel('$Y [m]$','fontsize',16,'Interpreter','latex')
xlabel('$t [s]$','fontsize',16,'Interpreter','latex')
%xlim([0 12.5])
%ylim([-5 85])
leg7 = legend({'$$Gnd Truth.$$','$$EstPos.$$'},'Location','northeast','Interpreter','Latex');
leg7.AutoUpdate = "off";
ax = gca;
ax.FontWeight = "bold";
ax.FontSize = 18;
grid on

line([gnss_fail_start * timestep, 7], [ground_truth(gnss_fail_start,2), 0], 'Color', 'Black');
text(7.2,0,"GNSS Failure",'FontSize',16,'FontWeight','bold');
line([gnss_fail_end * timestep, 17], [ground_truth(gnss_fail_end,2), -10], 'Color', 'Black');
text(10.2,-10,"GNSS Returns",'FontSize',16,'FontWeight','bold');

xerrgraphpos = [0.1 0.1 0.35 0.2];
subplot('position', xerrgraphpos);
ground_truth_interp = interp1(tspan_ground_truth,ground_truth,tspan_kalman_filter);
errx = abs(pose_storage(1,:)' - ground_truth_interp(:,1));
line1 = plot(tspan_kalman_filter, errx,'color','red');
line1.LineWidth = 1.0;
hold off
xlabel('$t [s]$','fontsize',16,'Interpreter','latex')
ylabel('$Absolute Error [m]$','fontsize',16,'Interpreter','latex')
leg4 = legend({'$|\hat{x}_1-x_1|$'},'Location','northwest','Interpreter','Latex');
leg4.AutoUpdate = 'off';
%xlim([0 12.5])
ylim([0 10])
ax = gca;
ax.FontWeight = "bold";
ax.FontSize = 18;
grid on

line([gnss_fail_start * timestep, 5], [errx(gnss_fail_start), 5.5], 'Color', 'Black');
text(1,6,"GNSS Failure",'FontSize',16,'FontWeight','bold');
line([gnss_fail_end * timestep, 20], [errx(gnss_fail_end), 8], 'Color', 'Black');
text(17,8.6,"GNSS Returns",'FontSize',16,'FontWeight','bold');

yerrgraphpos = [0.55 0.1 0.35 0.2];
subplot('position', yerrgraphpos);
erry = abs(pose_storage(2,:)' - ground_truth_interp(:,2));
line1 = plot(tspan_kalman_filter, erry,'color','red');
line1.LineWidth = 1.0;
xlabel('$t [s]$','fontsize',16,'Interpreter','latex')
ylabel('$Absolute Error [m]$','fontsize',16,'Interpreter','latex')
leg4 = legend({'$|\hat{x}_2-x_2|$'},'Location','northeast','Interpreter','Latex');
leg4.AutoUpdate = 'off';
%xlim([0 12.5])
%ylim([0 15])
ax = gca;
ax.FontWeight = "bold";
ax.FontSize = 18;
grid on

% rectangle('Position',[4.5 -0.1 2 2.6])
% 
% line([4.5, 2.5], [2.5, 25], 'Color', 'Black');
% line([6.5, 10.5], [2.5, 25], 'Color', 'Black');
% 
% zoompos = [0.58 0.15 0.15 0.1];
% axzoom = axes('position',zoompos);
% box on
% line1 = plot(tspan_observer, erry,'color','red');
% line1.LineWidth = 2.0;
% xlim([4.5 6.5])
% ylim([-0.1 2.5])
% grid on

print(strcat(FIGUREPATH, "KalmanFilterGPSFail1.eps"), '-depsc');

%% All Systems

update = 1;
offset = 0;
enable_input_kf = 0;

if update
    tic;
    % Uncomment to run VO.
    %run('VisualOdometry.m');
    VO_Timing = toc
end

if update
    tic;
    run('HighGainObserver.m');
    Observer_Timing = toc
end

enable_observer = 0;
enable_camera = 1;
camera_correct_orientation = 0;
camera_correct_velocity = 0;
enable_gnss = 0;
gnss_fail_start = 12000;
gnss_fail_end = 14000;

if update
    run('KalmanFilter.m');
    VO_KF_Timing = timing;
    CamXY = pose_storage;
    CamError = CamXY(1:2,:) - ground_truth(:,1:2)';
    CamNorm = 1/4242 * sum(vecnorm(CamError))
end

enable_observer = 1;
enable_camera = 0;
camera_correct_orientation = 0;
camera_correct_velocity = 0;
enable_gnss = 0;
gnss_fail_start = 12000;
gnss_fail_end = 14000;

if update
    run('KalmanFilter.m');
    Observer_KF_Timing = timing;
    ObXY = pose_storage;
    ObError = ObXY(1:2,:) - ground_truth(:,1:2)';
    ObNorm = 1/4242 * sum(vecnorm(ObError))
end

enable_observer = 0;
enable_camera = 1;
camera_correct_orientation = 1;
camera_correct_velocity = 1;
enable_gnss = 0;
gnss_fail_start = 12000;
gnss_fail_end = 14000;

if update
    run('KalmanFilter.m');
    %run('KalmanFilter.m');
    %run('KalmanFilter.m');
    VO_Corr_Timing = timing;
    CamCorrObXY = pose_storage;
    CamCorrObError = CamCorrObXY(1:2,:) - ground_truth(:,1:2)';
    CamCorrObNorm = 1/4242 * sum(vecnorm(CamCorrObError))
end

enable_observer = 0;
enable_camera = 1;
camera_correct_orientation = 1;
camera_correct_velocity = 1;
enable_gnss = 1;
gnss_fail_start = 12000;
gnss_fail_end = 14000;

if update
    run('KalmanFilter.m');
    %run('KalmanFilter.m');
    %run('KalmanFilter.m');
    KF_Sys_Timing = timing;
    KFXY = pose_storage;
    KFError = KFXY(1:2,:) - ground_truth(:,1:2)';
    KFNorm = 1/4242 * sum(vecnorm(KFError))
end

% To give a fair start!
gnss_data(1,:) = [5,5,0];
gnss_interped_data = interp1(tspan_gnss, gnss_data, tspan_observer, "nearest");
GNSSError = gnss_interped_data(:,1:2)' - ground_truth(:,1:2)';
GNSSNorm = 1/4242 * sum(vecnorm(GNSSError))

fig1 = figure('Name','Error State Extended Kalman Filter','NumberTitle','off');
fig1.Position = [100, 100, 1000, 1000];
        
xygraphpos = [0.1 0.4 0.8 0.55];
subplot('position', xygraphpos);

line1 = plot(ground_truth(:,1),ground_truth(:,2),'color','blue');
line1.LineWidth = 2.0;
hold on 
line2 = plot(CamXY(1,:),CamXY(2,:),':','color','black');
line2.LineWidth = 2.0;
line3 = plot(ObXY(1,:),ObXY(2,:),':','color','green');
line3.LineWidth = 2.0;
line4 = plot(CamCorrObXY(1,:),CamCorrObXY(2,:),':','color','magenta');
line4.LineWidth = 2.0;
line5 = plot(KFXY(1,:),KFXY(2,:),':','color','red');
line5.LineWidth = 2.0;
hold off
xlabel('$X [m]$','fontsize',16,'Interpreter','latex')
ylabel('$Y [m]$','fontsize',16,'Interpreter','latex')
axis equal
%xlim([0 100])
%ylim([-5 1])
title('VO Estimated Vehicle Trajectory, with V and \psi Path Corrections')
leg5 = legend({'$$Ground~Truth.$$','$$VO~Est.$$', '$$Observer~Est.$$', '$$Corrected~VO~Est.$$', '$$Proposed~Sol~Est.$$'},'Location','northeast','Interpreter','Latex');
leg5.AutoUpdate = 0; 
ax = gca;
ax.FontWeight = "bold";
ax.FontSize = 18;
grid on
ax.YDir = "reverse";

line([5, 38], [5, 18], 'Color', 'Black');
text(40,20,"$t=0$",'Interpreter','latex','FontSize',16,'FontWeight','bold');

xgraphpos = [0.1 0.1 0.35 0.2];
subplot('position', xgraphpos);
line1 = plot(tspan_ground_truth, ground_truth(:,1),'color','blue');
line1.LineWidth = 2.0;
hold on 
line2 = plot(tspan_kalman_filter, CamXY(1,:),':','color','black');
line2.LineWidth = 2.0;
line3 = plot(tspan_kalman_filter, ObXY(1,:),':','color','green');
line3.LineWidth = 2.0;
line4 = plot(tspan_kalman_filter, CamCorrObXY(1,:),':','color','magenta');
line4.LineWidth = 2.0;
line5 = plot(tspan_kalman_filter, KFXY(1,:),':','color','red');
line5.LineWidth = 2.0;
hold off
ylabel('$X [m]$','fontsize',16,'Interpreter','latex')
xlabel('$t [s]$','fontsize',16,'Interpreter','latex')
%xlim([0 12.5])
%ylim([-10 80])
%legend({'$$Gnd Truth.$$','$$EstPos.$$'},'Location','southeast','Interpreter','Latex')
ax = gca;
ax.FontWeight = "bold";
ax.FontSize = 18;
grid on

ygraphpos = [0.55 0.1 0.35 0.2];
subplot('position', ygraphpos);
line1 = plot(tspan_ground_truth, ground_truth(:,2),'color','blue');
line1.LineWidth = 2.0;
hold on 
line2 = plot(tspan_kalman_filter, CamXY(2,:),':','color','black');
line2.LineWidth = 2.0;
line3 = plot(tspan_kalman_filter, ObXY(2,:),':','color','green');
line3.LineWidth = 2.0;
line4 = plot(tspan_kalman_filter, CamCorrObXY(2,:),':','color','magenta');
line4.LineWidth = 2.0;
line5 = plot(tspan_kalman_filter, KFXY(2,:),':','color','red');
line5.LineWidth = 2.0;
hold off
ylabel('$Y [m]$','fontsize',16,'Interpreter','latex')
xlabel('$t [s]$','fontsize',16,'Interpreter','latex')
%xlim([0 12.5])
%ylim([-5 85])
%legend({'$$Gnd Truth.$$','$$EstPos.$$'},'Location','east','Interpreter','Latex')
ax = gca;
ax.FontWeight = "bold";
ax.FontSize = 18;
grid on

print(strcat(FIGUREPATH, "AllSystems.eps"), '-depsc');

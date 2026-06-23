%
% Code adapted by William Jacques.
% Originally code courtesy of Hichem Bessafa.
%  
% H. Bessafa, C. Delattre, Z. Belkhatir, A. Zemouche and R. Rajamani, 
% "Nonlinear Observer Design Methods Based on High-Gain Methodology and LMIs with Application to Vehicle Tracking", 
% 2023 American Control Conference (ACC), San Diego, CA, USA, 2023,
% pp. 4735-4740, doi: 10.23919/ACC55779.2023.10156219.
%

% Wheelbase: length centre of gravity to front (lf) and rear (lr) wheels
lr = 1.415;
lf = 1.415;

deltaf0 = 0.01;
a0 = 1;

% Use smoothed gnss data from the Kalman filter.
y = [gnss_interped_data(:,1), gnss_interped_data(:,2)];
if(enable_input_kf)
    y = [pose_storage_gnss(1,:)', pose_storage_gnss(2,:)'];
end

if exist('gnss_fail_start','var') && exist('gnss_fail_end','var')
    he = min(gnss_fail_end, size(y,1));
    y(gnss_fail_start:he, 1) = y(max(gnss_fail_start-1,1), 1);
    y(gnss_fail_start:he, 2) = y(max(gnss_fail_start-1,1), 2);
end
%y = interp1(tspan_kalman_filter, y, tspan_sensors);

%y = [ground_truth(:,1), ground_truth(:,2)];

%tspan_sensors = tspan_sensors*1.2;
%v = v/ 1.2;
%figure;
%plot(y(:,1), y(:,2));
L_f = lf + lr;
n = 6;
a=[zeros(2,1) eye(2) ;zeros(1,n/2)];
A=blkdiag(a,a);
c=[1 zeros(1,n/2-1)];
C= blkdiag(c,c);
b=[zeros(n/2-1,1); 1];
B=blkdiag(b,b);
%Simulation
%   h=0.01;
%tspan = 0:h:30;


%LMI

Sigma1=[22,22, 22];
Sigma2=[-22,-22, -22];
lambda = 1e8;


%P= sdpvar(n/2,n/2) ; 
%Y=sdpvar(1,n/2); 
%Z=sdpvar(1,n/2); 
%M1= [ a'*P + P*a - c'*Y - Y'*c-Sigma1'*Z-Z'*Sigma1 + lambda * eye(n/2), zeros(n/2,n/2);
%    zeros(n/2,n/2), -P ];
%M2= [ a'*P + P*a - c'*Y - Y'*c-Sigma2'*Z-Z'*Sigma2 + lambda * eye(n/2), zeros(n/2,n/2);
%    zeros(n/2,n/2), -P ];
%const = [M1 <=0;M2<=0;Z>=10];
%objective = [-P(2,1),-P(3,2)];
%objective = [trace(P)];
%objective = [];
%if(enable_input_kf) 
%    objective = []; 
%end
%options = sdpsettings('solver', 'mosek','verbose',1,'debug',1);
%options = sdpsettings('solver', 'lmilab','verbose',1);
%diagnostic=optimize(const,objective,options);
%if diagnostic.problem ~= 0
    %clc
%    error(diagnostic.info)
%end

K = [2.2024; 4.7099; 1.8513];
M = [0.0147; 0.0213; 0.0134];


%clc
%K=value(P)\value(Y)';
%K=  [1.9633;
%    4.0861;         
%    1.4985];
%K = [100;
%     200;
%     100];
%M=value(P)\value(Z)';
K=[K zeros(n/2,1);zeros(n/2,1) K];
M=[M ;M];
%%
%Simulation du system transformé 
%z0=1*ones(n,1);
%[t,z] = ode45(@(t,z) CinematicModelTransformed(t,z,v), tspan, z0);

%Simulation du system 
%etat initial
x0=1*ones(4,1);
x0(3)=1;
x0(4)=.02;
%u=[0.3*ones(round(length(tspan)/2),1); zeros(floor(length(tspan)/2),1)];
%u=0.3*sin(tspan);

%Observateur Grand gain 
%calcul de thetha 
syms z1 z2 z3 z4 z5 z6 
f1= -1/(z2^2+z5^2)*(-z5*z3+z2*z6)*z6;
f2= 1/(z2^2+z5^2)*(-z5*z3+z2*z6)*z3;
J=jacobian([f1,f2],[z1, z2, z3, z4, z5, z6 ]);
syms f(z1, z2, z3, z4, z5, z6)
f(z1, z2, z3, z4, z5, z6)=norm(J);
z1max= 11;
z2max= 15;
z3max= 5;
z4max= z1max;
z5max= z2max;
z6max= z3max;
L=double(f(z1max, z2max,z3max,z4max, z5max, z6max));

%Theta0 = 2*L*max(eig(value(P)))/lambda; 
sat=[z1max,z2max,z3max,z4max,z5max,z6max];
Theta =3;
T=[];
for i=1:n/2
T=[T Theta^i];
end
T=diag([T T]);
%%
%Simulation Observateur
%etat intial d'observateur
vertex={[-z1max,z1max],[-z2max,z2max],[-z3max,z3max],[-z3max,z3max],[-z3max,z3max],[-z3max,z3max]};
m0=5*ones(n,1);

[t2,x_hat_observer2] = ode45(@(t2,x_hat_observer2) HighGainObserver_method2(t2,x_hat_observer2,A,B,C,T,K,M,y,v,tspan_sensors,vertex), tspan_sensors, m0);
%Calcul de l'angle PSi a partir des etat du system transformé
Psi = atan2(x_hat_observer2(:,5),x_hat_observer2(:,2));
v_hat = sqrt((x_hat_observer2(:,5).^2 + x_hat_observer2(:,2).^2)); 
Psi=NormlizeAngle(Psi(:));
%sensor_data(:,3)=NormlizeAngle(sensor_data(:,3));
%sensor_data(:,3) = sensor_data(:,3) * 180 / pi;
Beta = atan((lr/(lr+lf)) * tan(sensor_data(:,1)));
PsiNoBeta = Psi * 180 / pi;
Psi = (Psi - Beta') * 180 / pi;
%Psi = PsiNoBeta;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Kalman filter intuitive tutorial %
% (c) Alex Blekhman, (end of) 2006 %
% Contact: ablekhman at gmail.com  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The problem: Predict the position and velocity of a moving train 2 seconds ahead,
% having noisy measurements of its positions along the previous 10 seconds (10 samples a second). 

% Ground truth: The train is initially located at the point x = 0 and moves
% along the X axis with constant velocity V = 10m/sec, so the motion equation of the train is X =
% X0 + V*t. Easy to see that the position of the train after 12 seconds
% will be x = 120m, and this is what we will try to find.

% Approach: We measure (sample) the position of the train every dt = 0.1 seconds. But,
% because of imperfect apparature, weather etc., our measurements are
% noisy, so the instantaneous velocity, derived from 2 consecutive position
% measurements (remember, we measure only position) is innacurate. We will
% use Kalman filter as we need an accurate and smooth estimate for the velocity in
% order to predict train's position in the future.

% We assume that the measurement noise is normally distributed, with mean 0 and
% standard deviation SIGMA
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  

close all
clear all
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Ground truth %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Set true trajectory 
Nsamples=100;
dt = .1;
t=0:dt:dt*Nsamples;
Vtrue = 10;

% Xtrue is a vector of true positions of the train 
Xinitial = 0;
Xtrue = Xinitial + Vtrue * t;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Motion equations %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Previous state (initial guess): Our guess is that the train starts at 0 with velocity
% that equals to 50% of the real velocity
Xk_prev = [0; 
    .5*Vtrue];

% Current state estimate
Xk=[];

% Motion equation: Xk = Phi*Xk_prev + Noise, that is Xk(n) = Xk(n-1) + Vk(n-1) * dt
% Of course, V is not measured, but it is estimated
% Phi represents the dynamics of the system: it is the motion equation
Phi = [1 dt;
       0  1];

% The error matrix (or the confidence matrix): P states whether we should 
% give more weight to the new measurement or to the model estimate 
sigma_model = 1;
% P = sigma^2*G*G';
P = [sigma_model^2             0;
                 0 sigma_model^2];

% Q is the process noise covariance. It represents the amount of
% uncertainty in the model. In our case, we arbitrarily assume that the model is perfect (no
% acceleration allowed for the train, or in other words - any acceleration is considered to be a noise)
Q = [0 0;
     0 0];

% M is the measurement matrix. 
% We measure X, so M(1) = 1
% We do not measure V, so M(2)= 0
M = [1 0];

% R is the measurement noise covariance. Generally R and sigma_meas can
% vary between samples. 
sigma_meas = 100; % 1 m/sec
R = sigma_meas^2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Kalman iteration %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Buffers for later display
Xk_buffer = zeros(2,Nsamples+1);
Xk_buffer(:,1) = Xk_prev;
Z_buffer = zeros(1,Nsamples+1);

for k=1:Nsamples
    
    % Z is the measurement vector. In our
    % case, Z = TrueData + RandomGaussianNoise
    Z = Xtrue(k+1)+sigma_meas*randn;
    Z_buffer(k+1) = Z;
    
    % Kalman iteration
    P1 = Phi*P*Phi' + Q;
    S = M*P1*M' + R;
    
    % K is Kalman gain. If K is large, more weight goes to the measurement.
    % If K is low, more weight goes to the model prediction.
    K = P1*M'*inv(S);
    P = P1 - K*M*P1;
    
    Xk = Phi*Xk_prev + K*(Z-M*Phi*Xk_prev);
    Xk_buffer(:,k+1) = Xk;
    
    % For the next iteration
    Xk_prev = Xk; 
end;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Plot resulting graphs %%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Position analysis %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure;
plot(t,Xtrue,'g');
hold on;
plot(t,Z_buffer,'c');
plot(t,Xk_buffer(1,:),'m');
title('Position estimation results');
xlabel('Time (s)');
ylabel('Position (m)');
legend('True position','Measurements','Kalman estimated displacement');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Velocity analysis %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% The instantaneous velocity as derived from 2 consecutive position
% measurements
InstantaneousVelocity = [0 (Z_buffer(2:Nsamples+1)-Z_buffer(1:Nsamples))/dt];

% The instantaneous velocity as derived from running average with a window
% of 5 samples from instantaneous velocity
WindowSize = 5;
InstantaneousVelocityRunningAverage = filter(ones(1,WindowSize)/WindowSize,1,InstantaneousVelocity);

figure;
plot(t,ones(size(t))*Vtrue,'m');
hold on;
plot(t,InstantaneousVelocity,'g');
plot(t,InstantaneousVelocityRunningAverage,'c');
plot(t,Xk_buffer(2,:),'k');
title('Velocity estimation results');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('True velocity','Estimated velocity by raw consecutive samples','Estimated velocity by running average','Estimated velocity by Kalman filter');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Extrapolation 20 samples ahead %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

SamplesIntoTheFuture = 20;
Nlast = 10; % samples

% We take the last Nlast = 10 samples, and for each of these samples we try to see what would be the
% estimated position of the train at sample number Nsamples + SamplesIntoTheFuture
% if we took the position and the velocity that was known at that sample

TruePositionInTheFuture = Xinitial + (Nsamples + SamplesIntoTheFuture) * Vtrue * dt;

ProjectedPositionByRunningAverage = Xk_buffer(1,(Nsamples+1-Nlast):(Nsamples+1)) + ...
    ((SamplesIntoTheFuture+Nlast):-1:SamplesIntoTheFuture) .* dt .* InstantaneousVelocityRunningAverage((Nsamples+1-Nlast):(Nsamples+1));

ProjectedPositionByKalmanFilter = Xk_buffer(1,(Nsamples+1-Nlast):(Nsamples+1)) + ...
    ((SamplesIntoTheFuture+Nlast):-1:SamplesIntoTheFuture) .* dt .* Xk_buffer(2,(Nsamples+1-Nlast):(Nsamples+1));

figure;
plot(((Nsamples+1-Nlast):(Nsamples+1))*dt,ones(size(1:11))*TruePositionInTheFuture,'m');
hold on;
plot(((Nsamples+1-Nlast):(Nsamples+1))*dt,ProjectedPositionByRunningAverage,'c');
plot(((Nsamples+1-Nlast):(Nsamples+1))*dt,ProjectedPositionByKalmanFilter,'k');
title(['Extrapolation 20 samples ahead (at t = ' num2str((Nsamples + SamplesIntoTheFuture) * dt) ')']);
xlabel('Time of sample used for extrapolation (s)');
ylabel('Expected position (m)');
legend('True position','Estimated position by running average','Estimated position by Kalman filter');

% Note: In this example, we might have increased WindowSize to smoothen
% InstantaneousVelocityRunningAverage. But, as in real conditions the
% velocity of the tracked object is not constant, this will lead to poor
% results in the general case.

% Note: K is a measure of the convergence of the filter, so are P(1,1) and
% P(2,2). When the model is good and the data is consistent with the model,
% these should converge to 0.


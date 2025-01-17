clc;
clear all;

mu = 398600; %Standard gravitational parameter km3/s2
earthRadius = 6371; %km
orbitHeight = 600; %km
meanMotion = sqrt(mu/(earthRadius+orbitHeight)^3); % rad/s
n=meanMotion; %for simplicity

gravity = 9.81; %m/s2
dryMass = 3 ; % kg
fuelMass = 1; %kg
totalMass = dryMass+fuelMass;
massFlow = 0.01; %kg/s for a monopropellant spacecraft
maxThrust = 0.01; % kgkm/s2
maxAcc = maxThrust/totalMass;

%Implementing MPC
% Hill's Equations (State-Space Model)
% Continuous-time system dynamics
A = [0, 0, 0, 1, 0, 0; 
     0, 0, 0, 0, 1, 0; 
     0, 0, 0, 0, 0, 1;
     3*n^2, 0, 0, 0, 2*n, 0;
     0, 0, 0, -2*n, 0, 0;
     0, 0, -n^2, 0, 0, 0];
B = [0, 0, 0; 
     0, 0, 0; 
     0, 0, 0; 
     1, 0, 0; 
     0, 1, 0; 
     0, 0, 1];
C = eye(6); 
D = zeros(6, 3); 

% Discretization 
Ts = 3; % Sampling time (seconds)
sysc = ss(A, B, C, D); % Continuous-time state-space model
sysd = c2d(sysc, Ts); % Discrete-time state-space model
Ad = sysd.A;
Bd = sysd.B;

% Create the MPC Controller
mpc_controller = mpc(sysd, Ts);

% Define MPC Parameters
mpc_controller.PredictionHorizon = 30; % Predict 50 steps ahead
mpc_controller.ControlHorizon = 20; % Optimize for 20 steps
mpc_controller.Weights.ManipulatedVariables = [1, 1, 1]; % Input effort, Control input for minimizing fuel
mpc_controller.Weights.ManipulatedVariablesRate = [5, 5, 5]; % Input change rate, smoothens the trajectory
mpc_controller.Weights.OutputVariables = [1, 1, 1, 0.5, 0.5, 0.5]; % State weights - position with 1 is tracked better than velocity for better position safety margin

% Input Constraints (e.g., thrust limits)
mpc_controller.ManipulatedVariables(1).Min = -maxAcc; % Thrust x min (km/s^2)
mpc_controller.ManipulatedVariables(1).Max = maxAcc;  % Thrust x max (km/s^2)
mpc_controller.ManipulatedVariables(2).Min = -maxAcc; % Thrust y min (km/s^2)
mpc_controller.ManipulatedVariables(2).Max = maxAcc;  % Thrust y max (km/s^2)
mpc_controller.ManipulatedVariables(3).Min = -maxAcc; % Thrust z min (km/s^2)
mpc_controller.ManipulatedVariables(3).Max = maxAcc;  % Thrust z max (km/s^2)

% Output Constraint margin distance
mpc_controller.OutputVariables(2).Min = 0.001 ;% km for safety purposes
% Radial and tangential distance cannot be between 1 and -1 m
% Need to add constraint for that

%collision avoidance parameters
safeDistance = 0.05; %km
num_debris =3;

% Terminal Constraints for Position and Velocity
terminalPositionY = [0.001, 0.005]; % 1 to 5 meters (converted to km)
terminalVelocityY = [-0.00022, 0.00022]; % ≤ 0.22 m/s (converted to km/s)

% Define terminal constraints
term.Weight = [10, 10, 10, 5, 5, 5]; % Weight on terminal state (used higher weights for stricter constraints)
term.Min = [-Inf, terminalPositionY(1), -Inf, -Inf, terminalVelocityY(1), -Inf]; % Lower bounds
term.Max = [Inf, terminalPositionY(2), Inf, Inf, terminalVelocityY(2), Inf];     % Upper bounds
term.MinECR = [0, 0, 0, 0, 0, 0]; % Soft constraint penalties for violating lower bounds
term.MaxECR = [0, 0, 0, 0, 0, 0]; % Soft constraint penalties for violating upper bounds

% Apply terminal constraints to the MPC controller
setterminal(mpc_controller, term);

% Rocket Equation parameters
m0 = fuelMass; % Initial fuel mass (kg)
Isp = maxThrust/(massFlow * gravity); % Specific impulse (seconds)
ve = Isp * gravity; % Exhaust velocity (m/s)
fuel_history = []; % To track fuel usage

% Simulation Parameters
x0 = [0; 5; 0; 0; 0; 0]; % Initial state (5 km behind target)
ref = [0; 0; 0; 0; 0; 0]; % Target position and velocity

%initialize the MPCSTATE object
mpc_state=mpcstate(mpc_controller);

% Simulate Closed-Loop System
Tsim = 1500; % Total simulation time
x = x0; % Initialize state
u_history = []; % Input control history
x_history = x0';

% State vector of dynamic debris
debris_statevector =[0,0.08,0,0,0.0002,0.0006;       %debris 1
                     0,0.550,0.0002,0.000,0.00,0.000;%debris 2
                     0.005,0.05,0.05,0,0.002,0];     %no of debris X 6 dimension of array
debris_history=[];

for t = 0:Ts:Tsim

    %get debris position
    debris_pos = debris_statevector(:,1:3);

    %add collision avoidance constraint
    for i = 1:num_debris
        x_debris = debris_statevector(i, 1);
        y_debris = debris_statevector(i, 2);
        z_debris = debris_statevector(i, 3);
        vx_debris = debris_statevector(i, 4);
        vy_debris = debris_statevector(i, 5);
        vz_debris = debris_statevector(i, 6);

        [x_new, y_new, z_new, vx_new, vy_new, vz_new] = cal_hill(x_debris, y_debris, z_debris, vx_debris, vy_debris, vz_debris,n,t);
        debris_statevector(i, :) = [x_new, y_new, z_new, vx_new, vy_new, vz_new];

        % Calculate squared distance between chaser and debris (for all directions)
        distance_squared = (x(1) - x_new)^2 + (x(2) - y_new)^2 + (x(3) - z_new)^2;

        % Check if the distance is within the safe range
        if distance_squared < safeDistance^2

            % Define the collision avoidance constraint for all directions
            % position >= 50 + debrisdistance
            E = zeros(3,3); % Identity matrix for all three directions (thrust in x, y, z)
            F = eye(6); % Identity matrix for output variables (position in x, y, z)
            F(4,4)=0;
            F(5,5)=0;
            F(6,6)=0;
            G = safeDistance^2 - distance_squared; % Right-hand side constraint

            % Set the constraint for collision avoidance in all directions
            setconstraint(mpc_controller, E, F, G);
        end
    end

    % Compute MPC Control Action
    [u,mpcstate] = mpcmove(mpc_controller, mpc_state,x, ref);
    
    % Calculate Delta-V (change in velocity) for each direction
    deltaV_x = u(1) * Ts;  % Delta-V in the x direction (m/s)
    deltaV_y = u(2) * Ts;  % Delta-V in the y direction (m/s)
    deltaV_z = u(3) * Ts;  % Delta-V in the z direction (m/s)
    deltaV = sqrt(deltaV_x^2 + deltaV_y^2 + deltaV_z^2);  % Total Delta-V (m/s)
    
    % Fuel consumption based on Tsiolkovsky Rocket Equation
    mf = m0 * exp(-deltaV / ve);  % Final mass after fuel burn (kg)
    fuel_used = m0 - mf;  % Fuel used (kg)
    
    % Update initial mass for the next iteration (account for fuel burn)
    m0 = mf;

    % Apply Control Action (Update State)
    x = Ad * x + Bd * u;
    
    % Log Data
    u_history = [u_history; u'];
    x_history = [x_history; x'];
    fuel_history = [fuel_history; fuel_used];  % Log fuel usage
    debris_history = [debris_history;debris_statevector]; % log debris trajectory info
end

% Correct time vector
time = 0:Ts:Tsim; % Matches the number of states recorded in x_history
x_history = x_history(1:length(time), :); % Trim x_history to match time vector

%calculate fuel usage
total_fuel_used = sum(fuel_history);  % Total fuel used over the entire simulation time
disp(['Total Fuel Used: ', num2str(total_fuel_used), ' kg']);

figure;
subplot(4, 1, 1);
plot(time, x_history(:, 1:3)); % Plot position (x, y, z)
title('Relative Position (x, y, z)');
xlabel('Time (s)');
ylabel('Position (km)');
legend('x', 'y', 'z');
grid on;

subplot(4, 1, 2);
plot(time, x_history(:, 4:6)); % Plot velocity (vx, vy, vz)
title('Relative Velocity (vx, vy, vz)');
xlabel('Time (s)');
ylabel('Velocity (km/s)');
legend('vx', 'vy', 'vz');
grid on;

subplot(4, 1, 3); 
plot(time, u_history); % Plot control input 
title('Control Inputs (Thrust)');
xlabel('Time (s)');
ylabel('Thrust (km/s^2)');
legend('ux', 'uy', 'uz');
grid on;

subplot(4,1,4);
plot(time, fuel_history); % Plot fuel usage
title('Fuel Usage Over Time');
xlabel('Time (s)');
ylabel('Fuel Used (kg)');
grid on;

% Script file to do simulated system ID on the Beluga system, 7 state model
% Constructs model, simulates it with randomized inputs, then attempts to
% recover the parameter values of the model.

%% Construct model

% free parameters to estimate: (inital guesses)
m1          = 15;       % kg; comprises actual mass m and added mass m_i
m3          = 19;       % kg

J           = 2.5;      % kg*m^2

etaUp       = 0.0005;   % efficiency (dimensionless)
etaDown     = 0.0004;
wOffset     = 0.2;      % offset velocity (m/s)
KdVert      = 70;       % quadratic drag coefficient (kg/m)

Kt1         = 1.03;     % motor conversion (N/counts)
KOmega      = 7;        % drag-induced torque (kg*m^2/s)
Kd1         = 45;       % axial drag coefficient (kg/s)

% fixed parameters (assumed known)
r           = 0.35;     % m; thruster moment arm
KtetherVert = 1.1;      % tether weight/length (kg/s^2)
zOffset     = 1;        % tether buoyancy offset (m)

% model data
FileName        = 'vehicleModel';   % Name of .m model ODE file
Order           = [4 3 7];          % Model orders [ny nu nx]
Parameters      = [m1; m3; J; etaUp; etaDown; wOffset; KdVert;  % Initial parameters
                   Kt1; KOmega; Kd1;...         % unknown parameters
                   r; KtetherVert; zOffset];    % known parameters
InitialStates   = zeros(Order(3),1);            % Initial initial state
Ts              = 0;                            % continuous-time model

% construct model
model   = idnlgrey(FileName,Order,Parameters,InitialStates,Ts);

% set which parameters are fixed (known)
model.Parameters(11).Fixed = true;
model.Parameters(12).Fixed = true;
model.Parameters(13).Fixed = true;

% set parameter names, units, etc
set(model, 'InputName', {'u_t','u_{\phi}','u_z'}, 'InputUnit', {'counts','radians','counts'});
set(model, 'OutputName', {'x-position', 'y-position', 'z-position', '\theta'});
set(model, 'OutputUnit', {'m', 'm', 'm', 'radians'});

setpar(model, 'Name', {'m1', 'm3', 'J', 'etaUp', 'etaDown', 'wOffset', 'KdVert',...
                       'Kt1', 'KOmega', 'Kd1', 'r', 'KtetherVert', 'zOffset'});
setpar(model, 'Unit', {'kg', 'kg', 'kg*m^2', '1', '1', 'm/s', 'kg/m', 'N/count',...
                       'kg*m^2/s', 'kg/s', 'm', 'kg/s^2', 'm'});

% set parameter constraints (mostly that values be positive; if necessary)
setpar(model, 'Minimum', {0,0,0,0,0,0,0,0,0,0,0,0,0})
%% Generate inputs

u = generateInputs();

%% Before estimating the parameters, simulate the output of the system 
%  with the parameter guesses using the default differential equation solver 
%  (a Runge-Kutta 45 solver with adaptive step length adjustment).

% Set the absolute and relative error tolerances to small values (1e-6 and 1e-5, respectively).

model.Algorithm.SimulationOptions.AbsTol = 1e-6;
model.Algorithm.SimulationOptions.RelTol = 1e-5;

%% Simulate

y = sim(model,u);
simData = iddata(y,u);

%% Corrupt the initial guesses for parameters

% pull out the parameter values
parameterValuesTrue = cell2mat(getpar(model,'Value'));

% count the number of parameters for future reference
noParams = length(parameterValuesTrue);

% corrupt the values with 5% N(0,1) noise
parameterNoise = parameterValuesTrue.*0.05.*randn(size(parameterValuesTrue));
parameterNoise(11:13) = 0;      % don't perturb known parameters
parameterValuesCorrupted = parameterValuesTrue + parameterNoise;

setpar(model,'Value',num2cell(parameterValuesCorrupted));

disp([parameterValuesTrue parameterValuesCorrupted])

%% Compare the behavior of the system for the true and corrupted values of parameters

figure;
sim(model,u)
%compare(simData, model);       % having some issues with this command for
%some reason

%% Estimate the value of parameters

model = pem(simData, model, 'Display', 'Full');
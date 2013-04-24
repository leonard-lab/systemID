% Script file to do system ID on the Beluga system, 7 state model using
% real data. Requires data in an iddata object

%% Construct model

% free parameters to estimate: (inital guesses)
m1          = 15;       % kg; comprises actual mass m and added mass m_i
m3          = 19;       % kg

J           = 2.5;      % kg*m^2

etaUp       = 0.0005;   % efficiency (dimensionless)
etaDown     = 0.0004;
wOffset     = 0.2;      % offset velocity (m/s)
KdVert      = 70;       % quadratic drag coefficient (kg/m)

Kt1         = 0.172;     % motor conversion (N/counts)
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
% setpar(model, 'Minimum', {0,0,0,0,0,0,0,0,0,0,0,0,0})

%% Load the data

load realData.mat;     % replace this with the appropriate data file

% OPTIONAL
% set the initial state of the model equal to the empirically observed
% initial state (if necessary)
% initialState = [x, y, z, u, w, theta, theta_dot]
initialState = [-0.85495; -2.6563; 1.2589; 0; 0; 1.249; 0]; % replace with true values
setinit(model,'Value',mat2cell(initialState,ones(Order(3),1),1));

%% Before estimating the parameters, simulate the output of the system 
%  with the parameter guesses using the default differential equation solver 
%  (a Runge-Kutta 45 solver with adaptive step length adjustment).

% Set the absolute and relative error tolerances to small values (1e-6 and 1e-5, respectively).

model.Algorithm.SimulationOptions.AbsTol = 1e-6;
model.Algorithm.SimulationOptions.RelTol = 1e-5;


%% Set the initial guesses for parameters

% pull out the parameter values
parameterValuesDefault = getParameterVector(model);

% count the number of parameters for future reference
noParams = length(parameterValuesDefault);

disp('Default initial value for parameters')
disp(parameterValuesDefault)

% IF NECESSARY:
% If needed, can change the initial parameter guess by setting the 13 x 1
% vector of parameters parameterValues

% parameterValues = parameterValuesDefault;

%setpar(model,'Value',num2cell(parameterValues));

%disp([parameterValuesDefault parameterValues])

%% Estimate the value of parameters

model = pem(ts_data, model, 'Display', 'Full');

disp('Fitted parameter values:')
getParameterVector(model)

%% Simulate the model with the experimental data (replace simData with your experimental data object)

sim_output = sim(model, ts_data);     % only simulates the system given the experimental inputs

compare(ts_data, sim_output); % compares the experimental data with the model given the default parameter values

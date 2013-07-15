% Script file to do system ID on the Beluga system, 7 state model using
% real data. Requires data in an iddata object

%% Construct model

% free parameters to estimate: (inital guesses)
m1          = 6; % 95.2176;     %20  % kg; comprises actual mass m and added mass m_i
m3          = 4; %36.1737;      %4 % kg

J           = 2;    %2.5  % kg*m^2

eta3Up      = 0.1;  %0.0005 % efficiency (dimensionless)
eta3Down    = 0.1; %0.0004
eta1        = 0.1;    %0.2  % offset velocity (m/s)
Kd3         = 70; % 4.595*10^4;      %70 % quadratic drag coefficient (kg/m)

Kt          = 0.2;   %1.03/6  % motor conversion (N/counts)
KOmega      = 12;      %7  % drag-induced torque (kg*m^2/s)
Kd1         = 50; % 285.0417;     %45  % axial drag coefficient (kg/s)

% fixed parameters (assumed known)
r           = 0.35;     % m; thruster moment arm
Kg          = 0.9088;      % tether weight/length (kg/s^2)
zOffset     = 1.2640;        % tether buoyancy offset (m)

Kdz         = .05;

% model data
FileName        = 'vehicleModel';   % Name of .m model ODE file
Order           = [5 3 7];          % Model orders [ny nu nx]
Parameters      = [m1; m3; J; eta3Up; eta3Down; eta1; Kd3; 
                   Kt; KOmega; Kd1;...         
                   r; Kg; zOffset; Kdz];    
%Parameters = v;
InitialStates   = zeros(Order(3),1);            % Initial initial state
Ts              = 0;                            % continuous-time model

% construct model
model   = idnlgrey(FileName,Order,Parameters,InitialStates,Ts);

% set which parameters are fixed (known)
%model.Parameters(6).Fixed = true;
model.Parameters(11).Fixed = true;
%model.Parameters(12).Fixed = true;
%model.Parameters(13).Fixed = true;


% set parameter names, units, etc
set(model, 'InputName', {'u_t','u_{\phi}','u_z'}, 'InputUnit', {'counts','radians','counts'});
set(model, 'OutputName', {'x-position', 'y-position', 'z-position', 'sin(theta)', 'cos(theta)'});
set(model, 'OutputUnit', {'m', 'm', 'm', 'units', 'units'});

setpar(model, 'Name', {'m1', 'm3', 'J', 'eta3Up', 'eta3Down', 'eta1', 'Kd3',...
                       'Kt', 'KOmega', 'Kd1', 'r', 'Kg', 'zOffset', 'Kdz'});
setpar(model, 'Unit', {'kg', 'kg', 'kg*m^2', '1', '1', 'm/s', 'kg/m', 'N/count',...
                       'kg*m^2/s', 'kg/s', 'm', 'kg/s^2', 'm', 'N*m/count'});

% set parameter constraints (mostly that values be positive; if necessary)
setpar(model, 'Minimum', {0,0,0,0,0,0,0,0,0,0,0,0,0,0})

%% Load the data

%load realData.mat;     % replace this with the appropriate data file

% OPTIONAL
% set the initial state of the model equal to the empirically observed
% initial state (if necessary)
y_init = ts_data.OutputData(1,:);
initialState = [y_init(1); y_init(2); y_init(3); 0; 0; atan2(y_init(4), y_init(5)); 0]; % replace with true values
%initialState = [y_init(1); y_init(2); y_init(3); 0; 0; 2.1421; 0];
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

model.Algorithm.MaxIter = 25;

model = pem(ts_data, model, 'Display', 'Full');

disp('Fitted parameter values:')
v = getParameterVector(model)

%% Simulate the model with the experimental data (replace simData with your experimental data object)

sim_output = sim(model,ts_data);     % only simulates the system given the experimental inputs

try
    compare(ts_data, sim_output); % compares the experimental data with the model given the default parameter values
catch err
    Ts = mean(ts_data.SamplingInstants(2:end) - ts_data.SamplingInstants(1:end-1));
    temp_data = iddata(ts_data.OutputData, ts_data.InputData, Ts);
    sim_output = sim(model, temp_data);
    compare(temp_data, sim_output);
end
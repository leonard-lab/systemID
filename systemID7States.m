% Script file to do system ID on the Beluga system, 7 state model using
% real data. Requires data in an iddata object

fit = true;
%% Load the data

%load shemp_collection_2013_7_18.mat;     % replace this with the appropriate data file

Ne = size(ts_data, 4);
Nx = 7;

% OPTIONAL
% set the initial state of the model equal to the empirically observed
% initial state (if necessary)
InitialStates = zeros(Nx, Ne);
for i=1:Nx
    y_init = zeros(Ne,1);
    if any(i == [1, 2, 3, 6])
        for j=1:Ne
            data = getexp(ts_data, j);
            if i == 6
                y_init(j) = atan2(data.OutputData(1,4), data.OutputData(1,5));
            else
                y_init(j) = data.OutputData(1,i);
            end
        end
    end
    InitialStates(i,:) = y_init;
end


%% Construct model

% free parameters to estimate: (inital guesses)
m1          = 10; % 95.2176;     %20  % kg; comprises actual mass m and added mass m_i
m3          = 15; %36.1737;      %4 % kg

J           = 1.4;    %2.5  % kg*m^2

eta3Up      = 0.2;  %0.0005 % efficiency (dimensionless)
eta3Down    = 0.4; %0.0004
eta1        = 0.5;    %0.2  % offset velocity (m/s)
Kd3         = 59.8; % 4.595*10^4;      %70 % quadratic drag coefficient (kg/m)

Kt          = 0.2;   %1.03/6  % motor conversion (N/counts)
KOmega      = 3.2;      %7  % drag-induced torque (kg*m^2/s)
Kd1         = 50; % 285.0417;     %45  % axial drag coefficient (kg/s)

% fixed parameters (assumed known)
r           = 0.35;     % m; thruster moment arm
Kg          = 0.8;      % tether weight/length (kg/s^2)
zOffset     = 1.91;        % tether buoyancy offset (m)

Kdz         = 0.005;

% model data
FileName        = 'vehicleModel';   % Name of .m model ODE file
Order           = [5 3 7];          % Model orders [ny nu nx]
Parameters      = [m1; m3; J; eta3Up; eta3Down; eta1; Kd3; 
                   Kt; KOmega; Kd1;...         
                   r; Kg; zOffset; Kdz];    
%Parameters = v;
%InitialStates   = zeros(Order(3),1);            % Initial initial state
Ts              = 0;                            % continuous-time model

% construct model
model   = idnlgrey(FileName,Order,Parameters,InitialStates,Ts);

% set which parameters are fixed (known)
model.Parameters(2).Fixed = true;
model.Parameters(11).Fixed = true;

model.Parameters(8).Fixed = true;

model.Parameters(7).Fixed = true;
%model.Parameters(13).Fixed = true;


% set parameter names, units, etc
set(model, 'InputName', {'u_t','u_{\phi}','u_z'}, 'InputUnit', {'counts','radians','counts'});
set(model, 'OutputName', {'x-position', 'y-position', 'z-position', 'sin(\theta)', 'cos(\theta)'});
set(model, 'OutputUnit', {'m', 'm', 'm', 'units', 'units'});

setpar(model, 'Name', {'m1', 'm3', 'J', 'eta3Up', 'eta3Down', 'eta1', 'Kd3',...
                       'Kt', 'KOmega', 'Kd1', 'r', 'Kg', 'zOffset', 'Kdz'});
setpar(model, 'Unit', {'kg', 'kg', 'kg*m^2', '1', '1', 'm/s', 'kg/m', 'N/count',...
                       'kg*m^2/s', 'kg/s', 'm', 'kg/s^2', 'm', 'N*m/count'});

% set parameter constraints (mostly that values be positive; if necessary)
setpar(model, 'Minimum', {0,0,0,0,0,0,0,0,0,0,0,0,0,0})

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

if fit == true
    model.Algorithm.MaxIter = 25;

    model = pem(ts_data, model, 'Display', 'Full');

    disp('Fitted parameter values:')
    v = getParameterVector(model)
end
%% Simulate the model with the experimental data (replace simData with your experimental data object)

sim_output = sim(model,ts_data);     % only simulates the system given the experimental inputs

try
    compare(ts_data, sim_output); % compares the experimental data with the model given the default parameter values
catch err
    for i=1:Ne
        if Ne > 1
            times = ts_data.SamplingInstants{i};
        else
            times = ts_data.SamplingInstants;
        end
        Ts_mat(i) = mean(times(2:end) - times(1:end-1));
    end
    Ts = mean(Ts_mat);
    ts_data_const = iddata(ts_data.OutputData, ts_data.InputData, Ts);
    sim_output = sim(model, ts_data_const);
    compare(ts_data_const, sim_output);
end

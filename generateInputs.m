function u = generateInputs()

% Function to generate control inputs for the model. Returns an iddata
% object with three signals: u_t, u_phi, u_z

T  = 400;   % number of samples
nu = 3;     % number of control inputs

% u_t max (counts)
utMax = 45;

% u_phi min/max (radians)
uphiMin = -1;
uphiMax = 1;

% u_z min/max (counts)
uzMin = -45;
uzMax = 45;

% draw random inputs
u = idinput([T nu],'sine');

% u_t
u(:,1) = (utMax/2)*(u(:,1)+1);

% u_phi
u(:,2) = ((uphiMax-uphiMin)/2)*(u(:,2)+1);

% u_z
u(:,3) = ((uzMax-uzMin)/2)*(u(:,3)+1);

u = iddata([],u);

% set input names and units
u.InputName = {'u_t','u_{\phi}','u_z'};
u.InputUnit = {'counts','radians','counts'};
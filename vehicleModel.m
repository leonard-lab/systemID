function [dX, Y] = vehicleModel(~, X, U, m1, m3, J, eta3Up, eta3Down, eta1, ...
                        Kd3, Kt, KOmega, Kd1, r, Kg, zOffset, Kdz, varargin)

% Function [dX, Y] = vehicleModel(t, X, U, m1, m3, J, etaUp, etaDown, wOffset, ...
%                                 KdVert, Kt1, KOmega, Kd1, r, KtetherVert, zOffset, varargin)
% Computes the state and output of the vehicle model with 7 states. Assumes
% that v = body-2 (sideslip) velocity is 0.

% known parameters
%r
%KtetherVert
%zOffset

% Extract variables
z        = X(3);
u        = X(4);
w        = X(5);
theta    = X(6);
thetaDot = X(7);

% Extract control inputs
ut   = U(1);    % horizontal thruster input
uphi = U(2) * pi/180.0;    % horizontal thruster servo input (radians)
uz   = U(3);    % vertical thruster input

% Kinematics
xDot = cos(theta)*u;
yDot = sin(theta)*u;
zDot = w;

% Force model
% Body-1 (axial) force
%F1 = Kt1*ut - Kd1*u;
F1 = (1-eta1)*Kt*ut*cos(uphi)- Kd1*u*abs(u);

% Body-2 (sideslip) force
% F2 = 0; (by assumption)

% Body-3 (vertical) force
% logic to determine if actuator is pushing up or down (different
% efficiency coefficients)
if uz < 0       % want to descend
    eta = eta3Down;
elseif uz > 0   % want to ascend
    eta = eta3Up;
else            % uz = 0
    eta = 0;
end

% F3thrust = eta*(abs(uz)*(abs(uz)+11.25))/(abs(w) + wOffset);
F3thrust = (1-eta)*uz*Kt;
F3drag   = -Kd3*w*abs(w);
F3tether = Kg*(zOffset - z);

F3 = F3thrust + F3drag + F3tether;

% Torque model
Gamma = -1*(1-eta1)*Kt*ut*r*sin(uphi) - KOmega*thetaDot*abs(thetaDot) - Kdz*uz;

% Accelerations
uDot = F1/m1;
wDot = F3/m3;

thetaDotDot = Gamma/J;

% Output the derivative of the state
dX = [xDot yDot zDot uDot wDot thetaDot thetaDotDot]';

% Output the observation output: x, y, z, theta
Y  = [X(1); X(2); X(3); sin(X(6)); cos(X(6))];
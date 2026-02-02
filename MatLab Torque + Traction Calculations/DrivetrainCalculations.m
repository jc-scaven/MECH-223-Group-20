clear; close all; clc;

g = 9.81; %gravity (m/s^2)
rho = 1.225; %air density (kg/m^3)

% volumetric dimensions: w = 0.25m, L = 0.35m, h = 0.1m

rw = 0.03; %rear-wheel radius (m)
L = 0.28; %wheelbase (m)
Lf = 0.12; %COG distance from front axle (m)
A = 0.2 * 0.1; %frontal area for air resistance

G = 20; %Gearbox ratio
eta_d = 0.90; %estimated drivetrain efficiency

mu = 0.75; %tire-ground friction coefficient
Crr = 0.03; %rolling resistance coefficient

m_vehicle = 1; %empty vehicle mass

Cd = 0.32; %aero drag coefficient

T_stall = 17.29 / 1000; %stall torque (Nm)
omega_nl = 10679*2*pi/60; %no load speed / max rpm (rad/s)

Tm = @(omega) max(T_stall * (1 - omega/omega_nl), 0); %motor torque function, clamps to 0 if negative

omegam = @(v) G*v/rw; %motor angular velocity

F_torque = @(v) Tm(omegam(v))*G*eta_d/rw; %motor torque

m_max_torque = @(v, theta) F_torque(v) / (g*(sin(theta) + Crr*cos(theta))); %maximum mass carriable

slope_angle = 35 * pi / 180;

velocity = 0.5; %m/s

m = m_max_torque(velocity, slope_angle) - m_vehicle;

fprintf("Maximum mass carried is %.3f kg \n", m)

batteries = floor(max(m,0)) / 0.024;

fprintf("Maximum batteries carried is %.2f \n", batteries)

max_slope_gradient = mu*Lf/L - Crr;

fprintf("Slope gradient: %.3f\nMax slope gradient: %.3f\n", tan(slope_angle), max_slope_gradient)
canclimb = tan(slope_angle) <= mu*(Lf/L) - Crr;
fprintf("Can climb slope with current axle ratios, friction coefficient, rolling coefficient: %s\n", string(canclimb))
fprintf("Max grade if 4wd: %.3f\n", mu-Crr);


%% Anti-Sway Control, Kashirin Aleksandr
clear;
clc;

%% Model Data

M = 1000;               % Cart mass, kg
m = 12000;                % Load mass, kg
L = 5;                  % Reduced rope and load length, m
g = 9.81;               % Gravity constant, m/s^2 
k_cart = 0.0113;        % Rolling resistance coefficient   

% Calculated constants
c1 = 1 - m/(m + M);
c2 = -1/(L*(m + M));

%% Linearization

A = [-k_cart*(m + M)*g/M    0     0    m*g/M
              1             0     0      0
       k_cart*g/(L*c1)      0     0  -g/(L*c1) 
              0             0     1      0];
 
B = [1/M
      0
    c2/c1
      0];

C = [0 1 0 0
     1 0 0 0
     0 0 0 1
     0 0 1 0];
     
D = [0
     0
     0
     0];
 
sysC = ss(A,B,C,D,'StateName',{'X_dot' 'X' 'Theta_dot' 'Theta'}, ...
    'InputName',{'F_cart'}, 'OutputName', {'X' 'X_dot' 'Theta' 'Theta_dot'}');
       
%% MPC
Ts = 0.1;
load('goodmpc.mat');
mpc1.Model.Plant = d2d(mpc1.Model.Plant, Ts);
load('goodplant.mat');
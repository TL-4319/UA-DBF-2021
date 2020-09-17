%%  Tuan Luong
%   DBF Scoring analysis script
%   Assume propeller plane - Constant power craft
%   Assume rectangular wing planform

%% Set up
clc;
clear all;
close all;

%% Environmental Variables
g= 32.17;                                   %Gravitation acc (ft/s^2)
rho = 0.062;                                %Air density (lb/ft^3)

%% Payload Variables
w_mech = 0.5;                               %weight of mechanism (lb)
w_sensor = 0.5;                               %Sensor weight (lb)
w_carrier = 0.5;                              %Carrier weight (lb)
w_payload = w_sensor+w_carrier;             %Tot carrier+sensor (lb)
CD_sensor = 0.001;                           %Drag coef of sensor (for M3)
CD_carrier = 0.001;                          %Drag coef of sensor (for M2)
n_sensor =1;                                %Number of sensors carrier

%% Plane Variables
w_plane=5;                                  %Plane weight (lb)                                     %Cruise CL
CD_0= 0.02;             
CL_max= 1.2;
b=5;                                        %wing span (ft)
c=1;                                        %Chord (ft)
S=b*c;                                      %Wing area assuming rect wing
AR=b^2/S;                                   %Aspect ratio
e=1;
P_avail = 880;                              %Power avail (lb_ft/s)

%% Mission 1 Scores

% Flight speed solver
W1 = w_plane;
e_P1=1;
v1=10;
while e_P1 > 0.1
    P_req1=0.5*rho*v1^3*S*(CD_0+(4*W1*pi*AR*e)/(rho^2*v1^4*S^2));
    e_P1=abs(P_req1-P_avail);
    v1=v1+0.001;
end

n_max1 =0.5*rho*v1^2*CL_max*S/W1;            %Max load factor
V_stall=sqrt(2*W1/(rho*S*CL_max));           %V stall 
min_turn_rad1 = (2*W1)/(rho*g*CL_max*S);     %Min turn radius (ft)
turn_rad1=min_turn_rad1*1.2;                  %Normal turn radius (ft)
track1 = 2000+2*2*pi*turn_rad1;               %Min track length (ft)
t1=5*60;                                     %Allowable time(s)
dis_to_fly1 = track1*3;
time_to_fly1 = dis_to_fly1/v1;
if time_to_fly1>t1
    M1 = 0;
else
    M1=1;
end

%% Mission 2 Score
%Iterate over different number of payload
v2=15;
n_sensor=1;
  %Flight speed solver
for n_sensor = 1:20  
    W2=w_plane+w_mech+n_sensor*w_payload;
    CD_payload = CD_carrier*n_sensor;
    e_P2=2;
    while e_P2 > 0.1
        P_req2=0.5*rho*v2^3*S*(CD_0+CD_payload+(4*W2*pi*AR*e)/(rho^2*v2^4*S^2));
        e_P2=abs(P_req2-P_avail);
        v2=v2+0.001;
    end
v_mission2 (n_sensor) = v2;    
n_max2 =0.5*rho*v2^2*CL_max*S/W2;            %Max load factor
V_stall=sqrt(2*W2/(rho*S*CL_max));           %V stall 
min_turn_rad2 = (2*W2)/(rho*g*CL_max*S);     %Min turn radius (ft)
turn_rad2=min_turn_rad2*1.2;                  %Normal turn radius (ft)
track2 = 2000+2*2*pi*turn_rad2;               %Min track length (ft)                                %Allowable time(s)
dis_to_fly2 = track2*3;
time_to_fly2 (n_sensor) = dis_to_fly2/v2;
M2(n_sensor) = 1+n_sensor/time_to_fly2 (n_sensor);
n(n_sensor)=n_sensor;
v2=15;
end
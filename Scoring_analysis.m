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
w_sensor = 2;                               %Sensor weight (lb)
w_carrier = 2;                              %Carrier weight (lb)
w_payload = w_sensor+w_carrier;             %Tot carrier+sensor (lb)
CD_sensor = 0.01;                           %Drag coef of sensor (for M3)
CD_carrier = 0.01;                          %Drag coef of sensor (for M2)
n_sensor =1;                                %Number of sensors carrier

%% Plane Variables
w_plane=5;                                  %Plane weight (lb)                          
CD_0= 0.02;             
CL_max= 1.2;
b=5;                                        %wing span (ft)
c=1;                                        %Chord (ft)
S=b*c;                                      %Wing area assuming rect wing
AR=b^2/S;                                   %Aspect ratio
e=1;
P_avail = 0.21*0.8*550;                     %Power avail (lb_ft/s)

%% Mission 1 Scores

% Flight speed solver
W1 = w_plane;
e_P1=1;
v1=10;
while e_P1 > 0.01
    P_req1=0.5*rho*v1^3*S*(CD_0+(4*W1*pi*AR*e)/(rho^2*v1^4*S^2));
    e_P1=abs(P_req1-P_avail);
    v1=v1+0.001;
end

n_max1 =0.5*rho*v1^2*CL_max*S/W1;            %Max load factor
V_stall=sqrt(2*W1/(rho*S*CL_max));           %V stall 
min_turn_rad1 = (2*W1)/(rho*g*CL_max*S);     %Min turn radius (ft)
turn_rad1=min_turn_rad1*1.2;                  %Normal turn radius (ft)
track = 2000+2*2*pi*turn_rad1;               %Min track length (ft)
t1=5*60;                                     %Allowable time(s)
dis_to_fly1 = track*3;
time_to_fly1 = dis_to_fly1/v1;
if time_to_fly1>t1;
    M1 = 0;
else
    M1=1;
end

%% Mission 2 Score
%Iterate over different number of payload
i=1;
v2=1;
j=1;
n_sensor=1;
  %Flight speed solver
    W2=w_plane+w_mech+n_sensor*w_payload;
    i=n_sensor;
    CD_payload = CD_carrier*n_sensor;
    e_P2=1;
    while v2<60
        P_req2=0.5*rho*v2^3*S*(CD_0+CD_payload+(4*W2*pi*AR*e)/(rho^2*v2^4*S^2));
        P_req(j)= P_req2;
        v(j) = v2;
        e_P2=abs(P_req2-P_avail);
        v2=v2+0.01;
        j=j+1;
    end

plot (v,P_req)

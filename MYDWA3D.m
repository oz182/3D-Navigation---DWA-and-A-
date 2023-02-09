%% Configuration

obs = [1, 6, 3;
       4, 6, 8;
       7, 8, 1;
       3, 8, 9;
       7, 8, 6;
       1, 3, 7;
       4, 9, 2;
       2, 7, 9;
       1, 5, 8;
       8, 4, 8;
       7, 9, 9;
       8, 9, 6;
       4, 8, 0;
       2, 2, 2;
       4, 4, 5.5;
       4, 3, 3;
       3, 8, 3;
       4.5,2.5,4.5;]; % obstacles coordinates

% obs = [3, 3, 8;
%        3, 3, 0;
%        3, 3, 0.5;
%        3, 3, 1;
%        3, 3, 1.5;
%        3, 3, 2;
%        3, 3, 2.5;
%        3, 3, 3;
%        3, 3, 3.5;
%        3, 3, 4;
%        3, 3, 4.5;
%        3, 3, 5;
%        3, 3, 5.5;
%        3, 3, 6;
%        3, 3, 6.5;
%        3, 3, 7;
%        3, 3, 7.5;
%        3,3,8.5;
%        3,3,9]; % obstacles coordinates

obsR = 0.5;

obsPlotVol = obsR*ones((length(obs)),1);

area=[-1 11 -1 11 -1 11];

StartPoint = [1,1,0];
%GoalPoint = [8,8,2];
TargetPoint = [6,7,3];
global Arrive; Arrive = 1;

X_POS = 0; Y_POS = 0; Z_POS = 0; Vxy = 0; W = 0; Vz = 0; W_POS = 0; Wz_POS = pi/2; Wz = 0;
State_Vec = [X_POS Y_POS Z_POS W_POS Wz_POS Vxy W Wz]';

Max_Vx = 1.0; Max_Vy = 1.0; Max_Vz = 1.0; Ax = 0.2; Ay = 0.2; Az = 0.2; SpeedRes = 0.1; %For later use in more progress model
Max_V = 2.0; Acc = 0.2; V_res = 0.01; Max_W = deg2rad(80.0); Acc_W = deg2rad(90.0); W_res = deg2rad(1); Max_Wz = deg2rad(100.0); Acc_Wz = deg2rad(150.0);
Kinematics = [Max_V, Acc, V_res, Max_W, Acc_W, W_res Max_Wz Acc_Wz]; % Robot kinematics model parameters

DistFactor = 0.36;
VelFactor = 0.4; HeadFactor = 0.132; Predict_dt = 1; ZFactor = 0.1; %head=0.5, zfac=0.4 -> Original values for me to remember
EvalFactors = [DistFactor, VelFactor, HeadFactor, ZFactor, Predict_dt];

global dt; dt = 0.1;

%% Main Loop Simulation

Nstate = [];

[AstarPath, Rawpath] = AstarPlan(obs,StartPoint,TargetPoint,obsR);

vw = VideoWriter('DWA_Astar_drone6.avi');
vw.Quality = 100;
vw.FrameRate = 60;
open(vw);

for i = 1:5000
    
    %%%%%%%%%%%%%%%%%%
    %GoalPoint = [AstarPath(Arrive,:),TargetPoint(3)];   %Follow the a star
    %path
    GoalPoint = TargetPoint;
    %%%%%%%%%%%%%%%%%%%%%%
    
    [u, traj] = DynamicWindowAprroch(State_Vec, Kinematics, GoalPoint, EvalFactors, obs, obsR, Rawpath);
    
    State_Vec = f(State_Vec, u);
    
    Nstate = [Nstate; State_Vec'];
    
    %%%%%%%%%%%%%%%%%%%%%%% Astar dealing
%     if norm(State_Vec(1:2)-AstarPath(Arrive,:)) < 1
%         if (AstarPath(Arrive,:) == TargetPoint(1:2))
%             disp('Arrived Goal!!');
%             break;
%         else
%             disp('Moving along the Astar path')
%             Arrive = Arrive + 1;
%         end
%     end
    %%%%%%%%%%%%%%%%%%%%%
    
    if norm(State_Vec(1:3)-GoalPoint') < 1 
        disp('Arrive Goal!!');
        break;
    end
    
    %====Animation====
    hold off;
    ArrowLength=0.5;
    quiver3(State_Vec(1),State_Vec(2),State_Vec(3),ArrowLength*cos(State_Vec(1)),ArrowLength*sin(State_Vec(2)),ArrowLength*sin(State_Vec(3)),'ok');hold on;
    plot3(Nstate(:,1),Nstate(:,2),Nstate(:,3),'-b');hold on;
    %plot3(AstarPath(:,1),AstarPath(:,2),zeros(numel(AstarPath(:,1))));hold on;
    plot3(Rawpath(:,1),Rawpath(:,2),zeros(numel(Rawpath(:,1))), '-k');hold on;
    scatter3(GoalPoint(1),GoalPoint(2),GoalPoint(3),'*r');hold on;
    scatter3(TargetPoint(1),TargetPoint(2),TargetPoint(3),'*g');hold on;
    scatter3(obs(:,1), obs(:,2), obs(:,3),'*k');hold on;
    %bubblechart3(obs(:,1), obs(:,2), obs(:,3), obsPlotVol);hold on;
    if ~isempty(traj)
        for it=1:length(traj(:,1))/5
            ind=1+(it-1)*5;
            %plot3(traj(ind,:),traj(ind+1,:),traj(ind,:),'-g');hold on;
        end
    end
    axis(area);
    grid on;
    drawnow;
    
    frame = getframe(gcf);
    writeVideo(vw, frame);

end

close(vw);

%% DWA Functions

function [u, trajDB] = DynamicWindowAprroch(State_Vec, Kinematics, GoalPoint, EvalFactors, obs, obsR,Rawpath)

Vr=CalcDynamicWindow(State_Vec,Kinematics);
[EvalDB,trajDB]=Evaluation(State_Vec,Vr,GoalPoint,obsR,Kinematics,EvalFactors,obs,Rawpath);
EvalDB
if isempty(EvalDB)
    disp('no path to goal!!');
    u=[0;0;0];
    return;
end

EvalDB=NormalizeEval(EvalDB);

SumEval = [];
for i=1:length(EvalDB(:,1))
    SumEval = [SumEval; EvalFactors(1:4)*EvalDB(i,4:7)'];
end

EvalDB = [EvalDB SumEval];

[MaxValue, ind] = max(SumEval);
u = EvalDB(ind,1:3)';
end

function Vr = CalcDynamicWindow(State_Vec, Kinematics)
% calculation dynamic window
global dt

Vs = [0 Kinematics(1) -Kinematics(4) Kinematics(4) -Kinematics(7) Kinematics(7)]; % All the search space - minimum and maximum speed in every axis
Vd = [State_Vec(6)-Kinematics(2)*dt State_Vec(6)+Kinematics(2)*dt State_Vec(7)-Kinematics(5)*dt State_Vec(7)+Kinematics(5)*dt State_Vec(8)-Kinematics(8)*dt State_Vec(8)+Kinematics(8)*dt];

% The function might missed a Va exprassion.

Vtmp = [Vs;Vd]; %2 X 6 Each column is: minimum speed, maximum speed, minimum angular velocity, maximum angular velocity

Vr = [max(Vtmp(:,1)) min(Vtmp(:,2)) max(Vtmp(:,3)) min(Vtmp(:,4)) max(Vtmp(:,5)) min(Vtmp(:,6))];

end

function [EvalDB, trajDB] = Evaluation(State_Vec, Vr, GoalPoint, obsR, Kinematics, EvalFactors, obs, Rawpath)

EvalDB = [];
trajDB = [];

for Vx_t = Vr(1):Kinematics(3):Vr(2)
        for W_t = Vr(3):Kinematics(6):Vr(4)
            for Wz_t = Vr(5):Kinematics(6):Vr(6)
            
            [State_Vec_t, traj] = GenerateTrajectory(State_Vec,Vx_t,W_t,Wz_t,EvalFactors(5),Kinematics);
            [heading]=CalcHeadingEval(State_Vec_t,GoalPoint,Rawpath);
            dist = CalcDistEval(State_Vec_t,obs,obsR);
            vel = abs(Vx_t);
            Zhead = CalcDistfromZ(State_Vec_t,GoalPoint);
            StopDist = CalcBreakingDist(vel,Kinematics);
            
            if 1 == 1 %dist > StopDist
                EvalDB = [EvalDB;[Vx_t W_t Wz_t dist vel heading Zhead]]; 
                trajDB = [trajDB;traj];
            end
            
            end
        end
    %end
end


end

function EvalDB=NormalizeEval(EvalDB)

if sum(EvalDB(:,4))~=0
    EvalDB(:,4)=EvalDB(:,4)/sum(EvalDB(:,4));
end
if sum(EvalDB(:,5))~=0
    EvalDB(:,5)=EvalDB(:,5)/sum(EvalDB(:,5));
end
if sum(EvalDB(:,6))~=0
    EvalDB(:,6)=EvalDB(:,6)/sum(EvalDB(:,6));
end
if sum(EvalDB(:,7))~=0
    EvalDB(:,7)=EvalDB(:,7)/sum(EvalDB(:,7));
end
end

function [State_Vec,traj]=GenerateTrajectory(State_Vec,Vx_t,W_t,Wz_t,Predict_dt,Kinematics)
global dt
time = 0;
u = [Vx_t;W_t;Wz_t];
traj = State_Vec;

while time <= Predict_dt
  time = time + dt;
  State_Vec = f(State_Vec,u);
  traj = [traj State_Vec];
end
end

function [heading]=CalcHeadingEval(State_Vec,GoalPoint,Rawpath) %With Astar Algo

% heading to A star path evaluation function calculation 
% Input parameters: current position, A star path
% Output parameter: Heading parameter - score The heading of the current car and the heading relative to the A star path. The smaller the deviation, the higher the score. The maximum 180 points.
global Arrive;

HeadOrientation = rad2deg(State_Vec(4)); %Yaw_xy - heading of the robot

    if Arrive < (numel(Rawpath)/2)
        Goal = Rawpath(Arrive,:);
        if (abs(norm(State_Vec(1:2)'-Rawpath(Arrive,:))) < 0.5)
            Arrive = Arrive + 10;
        end
    else
        Goal = Rawpath((numel(Rawpath)/2),:);
    end

HeadOrienFromGoal = rad2deg(atan2((Goal(2) - State_Vec(2)),(Goal(1) - State_Vec(1)))); % Orintation needed for the direaction along the A star path

if HeadOrientation < HeadOrienFromGoal
    Target = HeadOrienFromGoal - HeadOrientation;
else
    Target = HeadOrientation - HeadOrienFromGoal;
end

heading = 180 - Target;
end

function State_Vec = f(State_Vec, u)
global dt;

%X_part = cos(State_Vec(4))*cos(rad2deg(State_Vec(5)));
%Y_part = sin(State_Vec(4))*cos(rad2deg(State_Vec(5)));

X_part = cos(State_Vec(4));
Y_part = sin(State_Vec(4));

Zangle = sin(State_Vec(5))+cos(State_Vec(5));
% Before the change it was just sin(state_vec(5))

%if State_Vec(3) > 2 %z value of goal
    %Zangle = -Zangle;
%end

F = [1 0 0 0 0 0 0 0
     0 1 0 0 0 0 0 0
     0 0 1 0 0 0 0 0
     0 0 0 1 0 0 0 0
     0 0 0 0 1 0 0 0
     0 0 0 0 0 0 0 0
     0 0 0 0 0 0 0 0
     0 0 0 0 0 0 0 0];
 
B = [dt*X_part               0             0
     dt*Y_part               0             0
     dt*Zangle               0             0
            0                dt            0
            0                0             dt
            1                0             0
            0                1             0
            0                0             1];
        
        
State_Vec = F*(State_Vec)+B*u;

end

function dist=CalcDistEval(State_Vec,obs,obsR)
 dist = 2;
 
 for io = 1:length(obs(:,1))
    if abs(obs(io,3) - State_Vec(3)) < 1 
        disttmp = norm(obs(io,1:2) - State_Vec(1:2)') - obsR;
        if dist>disttmp
            dist=disttmp;
        end
    end
    
end
 
end

function stopDist=CalcBreakingDist(vel,Kinematic)
global dt;
stopDist=0;
while vel>0
    stopDist=stopDist+vel*dt;
    vel=vel-Kinematic(2)*dt;
end
end

function Zhead=CalcDistfromZ(State_Vec,GoalPoint)
delta_y = GoalPoint(2) - State_Vec(2);
delta_x = GoalPoint(1) - State_Vec(1);
distSG = delta_x^2 + delta_y^2; %the distance on xy plane between two points

HeadOrientation = rad2deg(State_Vec(5)); %pitch_z - heading of the robot - z axis
%HeadOrienFromGoal = rad2deg(atan2((GoalPoint(3) - State_Vec(3)),(distSG))); % Orintation needed for the direaction of the goal
HeadOrienFromGoal = rad2deg(atan2((GoalPoint(3) - State_Vec(3)),(delta_x)));

if HeadOrientation < HeadOrienFromGoal
    Target = HeadOrienFromGoal - HeadOrientation;
else
    Target = HeadOrientation - HeadOrienFromGoal;
end

Zhead = 180 - Target;
end

function [Postpath,Rawpath] = AstarPlan(obs,start,goal,obsR)

    map = binaryOccupancyMap(10,10,10);

    setOccupancy(map, obs(:,1:2), ones(length(obs),1))
    inflate(map, obsR)
    %figure
    %show(map)
    
    planner = plannerAStarGrid(map);
    
    goal = goal*10;
    %start = [2 3];
    %goal = [90 40];
    
    Rawpath = (plan(planner,start(1:2),goal(1:2)))/10;
    jumps = (numel(Rawpath)/(numel(Rawpath)/10));
    Postpath = [Rawpath((2:jumps:end),:);goal(1:2)]/10;
    
    %show(planner)
    
end


% function [heading]=CalcHeadingEval(State_Vec,GoalPoint,Rawpath) %No Astar Algorothm.
% 
% % heading evaluation function calculation
% % Input parameters: current position, target position
% % Output parameter: Heading parameter - score The heading of the current car and the heading relative to the target point. The smaller the deviation, the higher the score. The maximum 180 points.
% 
% HeadOrientation = rad2deg(State_Vec(4)); %Yaw_xy - heading of the robot
% HeadOrienFromGoal = rad2deg(atan2((GoalPoint(2) - State_Vec(2)),(GoalPoint(1) - State_Vec(1)))); % Orintation needed for the direaction of the goal
% 
% if HeadOrientation < HeadOrienFromGoal
%     Target = HeadOrienFromGoal - HeadOrientation;
% else
%     Target = HeadOrientation - HeadOrienFromGoal;
% end
% 
% heading = 180 - Target;
% end

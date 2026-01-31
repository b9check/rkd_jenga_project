function [] = pick_place_sample()
% pick_place_sample
%
% pick and place example code; picks up an object at position "A" on
% the table and moves it to position "B".

%% First, we define a couple of helper functions.  You can break these out into
%% separate files if you wish.

% Define a simple function to actually send a series of points to the
% robot, where the 'trajectory' is a matrix of columns of joint angle
% commands to be sent to 'robot' at approximately 'frequency'.
% Note this also commands velocities, although you can choose to only command
% positions if desired, or to add torques to help compensate for gravity.
function [] = command_trajectory(robot, trajectory, frequency)
  %% Setup reusable structures to reduce memory use in loop
  cmd = CommandStruct();

  % Compute the velocity numerically
  trajectory_vel = diff(trajectory, 1, 2);

  % Command the trajectory
  for i = 1:(size(trajectory, 2) - 1)
    % Send command to the robot (the transposes on the trajectory
    % points turns column into row vector for commands).
    cmd.position = trajectory(:,i)';
    cmd.velocity = trajectory_vel(:,i)' * frequency;
    robot.set(cmd);

    % Wait a little bit to send at ~100Hz.
    pause(1 / frequency);
  end

  % Send the last point, with a goal of zero velocity.
  cmd.position = trajectory(:,end)';
  cmd.velocity = zeros(1, size(trajectory, 1));
  robot.set(cmd);
end

% Convenience function to use to hide the internal logic of starting the suction
function [] = pick(suction_cup)
  suction_cmd = IoCommandStruct();
  suction_cmd.e2 = 1;
  suction_cup.set(suction_cmd);
end

% Convenience function to use to hide the internal logic of stopping the suction
function [] = place(suction_cup)
  suction_cmd = IoCommandStruct();
  suction_cmd.e2 = 0;
  suction_cup.set(suction_cmd);
end

% Clear out old information to reduce problems with stale modules
HebiLookup.setLookupAddresses('*');
HebiLookup.clearModuleList();
HebiLookup.clearGroups();
pause(3);

% Connect to physical robot
robot = HebiLookup.newGroupFromNames('16384',{'base','shoulder','elbow','wrist1','wrist2'});
% Note -- this is how long particular commands that you send to the robot "last"
% before the robot goes limp. Here, we ensure they last for 1 second.
robot.setCommandLifetime(1);
% Load saved control gains, and set these on the robot. These can be tuned to
% improve accuracy, but you should be very careful when doing so.
gains = load('jenga_gains.mat');
robot.set('gains', gains.jenga_gains);

%% Connect to gripper, and initialize some settings properly
gripper = HebiLookup.newGroupFromNames('16384','gripper');
gripper.setCommandLifetime(0);
gripper.setFeedbackFrequency(100);

warning('Before continuing, ensure no persons or objects are within range of the robot!\nAlso, ensure that you are ready to press "ctrl-c" if the robot does not act as expected!');
disp('');
input('Once ready, press "enter" to continue...','s');

%% Get initial position
fbk = robot.getNextFeedback();
initial_thetas = fbk.position'; % (The transpose turns the feedback into a column vector)

%% Start logging
currentDir = fileparts(mfilename('fullpath'));
logFile = robot.startLog('file', fullfile(currentDir, 'robot_data'));

%% command frequency, in Hz
frequency = 100;

%% ROBOT MOVEMENT COMMANDS
%Build code robot
link_masses = [0; 0; 0; 0; 0];
joint_masses = [0; 0; 0; 0; 0];
dh = [0 pi/2 56.05 initial_thetas(1); 330.3 pi 67.025 initial_thetas(2); 254.1 pi 0 initial_thetas(3); 0 pi/2 54.475 initial_thetas(4) + pi/2; 0 0 213.75 initial_thetas(5)];
code_robot = Robot(dh, link_masses, joint_masses);

%Intermediate position thetas - constant
%position_1_approach = [0.2334 1.2 1.72 0.0226 0.0016]';
position_1_approach = code_robot.inverse_kinematics_analytical([0 530 -70 45*(pi/180) -25*(pi/180)]);
midpoint_a = [0.5750 1.2941 1.9712 -0.0805 -25*(pi/180)]';
%position_2_approach_a = [1.18 1.2449 1.85 -0.6575 -25*(pi/180)]';
%position_2_approach_b = [1.18 1.2449 1.85 -0.6575 65*(pi/180)]';
midpoint_b = [0.5750 1.2941 1.9712 -0.0805 65*(pi/180)]';
position_1 = [0.2060 0.95 1.6 -0.0801 0.0084]';


%GOAL POSITIONS: [X Y Z THETA THETA_5]
goal1 = [475 285 35 0 -34*(pi/180)];
goal2 = [445 290 40 0 -35*(pi/180)];
goal3 = [420 293 45 0 -35*(pi/180)];
goal4 = [445 240 65 0 63*(pi/180)];
goal5 = [445 272 65 0 63*(pi/180)];
goal6 = [445 307 63 0 63*(pi/180)];
goal7 = [475 285 70 0 -34*(pi/180)];
goal8 = [445 290 70 0 -35*(pi/180)];
goal9 = [420 290 70 0 -35*(pi/180)];
goal10 = [445 240 80 0 63*(pi/180)];
goal11 = [445 272 80 0 63*(pi/180)];
goal12 = [445 307 80 0 63*(pi/180)];
goal13 = [475 285 95 0 -34*(pi/180)];
goal14 = [445 290 95 0 -35*(pi/180)];
goal15 = [420 290 95 0 -35*(pi/180)];
goal16 = [445 240 110 0 63*(pi/180)];
goal17 = [445 272 110 0 63*(pi/180)];
goal18 = [445 307 110 0 63*(pi/180)];


g1 = code_robot.inverse_kinematics_analytical(goal1);
g2 = code_robot.inverse_kinematics_analytical(goal2);
g3 = code_robot.inverse_kinematics_analytical(goal3);
g4 = code_robot.inverse_kinematics_analytical(goal4);
g5 = code_robot.inverse_kinematics_analytical(goal5);
g6 = code_robot.inverse_kinematics_analytical(goal6);
g7 = code_robot.inverse_kinematics_analytical(goal7);
g8 = code_robot.inverse_kinematics_analytical(goal8);
g9 = code_robot.inverse_kinematics_analytical(goal9);
g10 = code_robot.inverse_kinematics_analytical(goal10);
g11 = code_robot.inverse_kinematics_analytical(goal11);
g12 = code_robot.inverse_kinematics_analytical(goal12);
g13 = code_robot.inverse_kinematics_analytical(goal13);
g14 = code_robot.inverse_kinematics_analytical(goal14);
g15 = code_robot.inverse_kinematics_analytical(goal15);
g16 = code_robot.inverse_kinematics_analytical(goal16);
g17 = code_robot.inverse_kinematics_analytical(goal17);
g18 = code_robot.inverse_kinematics_analytical(goal18);

goal_positions = {g1; g2; g3; g4; g5; g6; g7; g8; g9; g10; g11; g12; g13; g14; g15; g16; g17; g18};

%POSITION 2 APPROACH POSITIONS
d = 40;
a1 = code_robot.inverse_kinematics_analytical(goal1 + [0 0 d 0 0]);
a2 = code_robot.inverse_kinematics_analytical(goal2 + [0 0 d 0 0]);
a3 = code_robot.inverse_kinematics_analytical(goal3 + [0 0 d 0 0]);
a4 = code_robot.inverse_kinematics_analytical(goal4 + [0 0 d 0 0]);
a5 = code_robot.inverse_kinematics_analytical(goal5 + [0 0 d 0 0]);
a6 = code_robot.inverse_kinematics_analytical(goal6 + [0 0 d 0 0]);
a7 = code_robot.inverse_kinematics_analytical(goal7 + [0 0 d 0 0]);
a8 = code_robot.inverse_kinematics_analytical(goal8 + [0 0 d 0 0]);
a9 = code_robot.inverse_kinematics_analytical(goal9 + [0 0 d 0 0]);
a10 = code_robot.inverse_kinematics_analytical(goal10 + [0 0 d 0 0]);
a11 = code_robot.inverse_kinematics_analytical(goal11 + [0 0 d 0 0]);
a12 = code_robot.inverse_kinematics_analytical(goal12 + [0 0 d 0 0]);
a13 = code_robot.inverse_kinematics_analytical(goal13 + [0 0 d 0 0]);
a14 = code_robot.inverse_kinematics_analytical(goal14 + [0 0 d 0 0]);
a15 = code_robot.inverse_kinematics_analytical(goal15 + [0 0 d 0 0]);
a16 = code_robot.inverse_kinematics_analytical(goal16 + [0 0 d 0 0]);
a17 = code_robot.inverse_kinematics_analytical(goal17 + [0 0 d 0 0]);
a18 = code_robot.inverse_kinematics_analytical(goal18 + [0 0 d 0 0]);

approach_positions = {a1; a2; a3; a4; a5; a6; a7; a8; a9; a10; a11; a12; a13; a14; a15; a16; a17; a18};

%Go to position_1_approach from inital thetas
trajectory = trajectory_spline([initial_thetas position_1_approach], [0, 1], frequency);
command_trajectory(robot, trajectory, frequency);

%Run loop to place blocks at each goal position
count = 1;
change = 1;
for q = 1:length(goal_positions)
    %Go to position 1 and pick up block
    trajectory = trajectory_trap_vel([position_1_approach position_1], [0, 0.5], frequency,1/3);
    command_trajectory(robot, trajectory, frequency);
    pick(gripper);
    %pause(0.5);
    %Go to position 1 approach
    trajectory = trajectory_trap_vel([position_1 position_1_approach], [0, 0.5], frequency,1/3);
    command_trajectory(robot, trajectory, frequency);
    %Go to correct position_2_approach
    P2_approach = approach_positions{q};
    if mod(count,4) == 0
        change = change + 1;
    end
    if mod(change,2) == 0
        midpoint = (position_1_approach+P2_approach)/2;
        midpoint(end) = 65*(pi/180);
    else
        midpoint = (position_1_approach+P2_approach)/2;
        midpoint(end) = -25*(pi/180);
    end
    trajectory = trajectory_spline([position_1_approach midpoint P2_approach], [0, 0.75, 1.3], frequency);
    command_trajectory(robot, trajectory, frequency);
    %Go to position 2 and place
    position_2 = goal_positions{q};
    trajectory = trajectory_trap_vel([P2_approach position_2], [0, 0.5], frequency, 1/3);
    command_trajectory(robot, trajectory, frequency);
    place(gripper);
    %Go back to position 1 approach
    trajectory = trajectory_trap_vel([position_2 P2_approach], [0, 0.5], frequency, 1/3);
    command_trajectory(robot, trajectory, frequency);
    trajectory = trajectory_spline([P2_approach midpoint position_1_approach], [0, 0.5, 1], frequency);
    command_trajectory(robot, trajectory, frequency);
    %Update count of number of blocks placed
    count = count+1;
end


%{
% We keep the last joint equal to the first to ensure the block does not rotate
% as we move. Note this joint points in the opposite direction as the base. For
% position 2, we want to rotate 1/4 turn, so we add pi/2.
%position_1(5) = position_1(1);
%position_2(5) = position_2(1) + pi/2;

% Add a midpoint for the trajectory, so the robot does not just drag the piece
% across the table.
midpoint = [0.7005 1.1218 2.1636 0.3116 0]';
%midpoint(5) = position_1(5)*0.5 + position_2(5)*0.5;

% Create a set of "approach" angles that let us have a slow "final approach" to
% the actual pick and place location.  This can increase accuracy and reduce
% issues where straight-line-configuration-space trajectories make the end
% effector hit the table
position_1_approach = [0.2603 1.1993 1.8269 0.0699 pi/4]';
position_2_approach = [1.2603 1.1666 1.9437 -0.3164 pi/4]';
position_2_approach_1 = [1.1823 1.1966 1.9437 -0.3164 pi/4]';
position_2_approach_2 = [1.1223 1.2066 2.0437 -0.3164 pi/4]';


%position_2_approach(5) = position_2(1) + pi/2;

%% Moves the robot from the initial position to the first waypoint over 4
%% seconds.  We break this into 3 seconds to make most of the motion, and 1 for
%% the final approach.
trajectory = trajectory_spline([initial_thetas midpoint position_1_approach], [0, 2, 3], frequency);
command_trajectory(robot, trajectory, frequency);
trajectory = trajectory_spline([position_1_approach position_1], [0, 1], frequency);
command_trajectory(robot, trajectory, frequency);

%% Pick up the object at position 1.  We pause to let the robot stabilize before
%% moving. NOTE: If you pause more than the length of the command lifetime you
%% set above, then the robot "goes limp" because the previous position and/or
%% velocity and torque commands "expire".
pick(gripper);
pause(0.75);

%% Move to the second waypoint over 4 seconds, with special "retract" and
%% "approach" motions that are done more slowly.
trajectory = trajectory_spline([position_1 position_1_approach], [0, 1], frequency);
command_trajectory(robot, trajectory, frequency);
trajectory = trajectory_spline([position_1_approach midpoint position_2_approach], [0, 1, 2], frequency);
command_trajectory(robot, trajectory, frequency);
trajectory = trajectory_spline([position_2_approach position_2], [0, 1], frequency);
command_trajectory(robot, trajectory, frequency);

%% Place the object
place(gripper);
pause(0.75);

%% Move back to position 1.
trajectory = trajectory_spline([position_2 position_2_approach], [0, 0.5], frequency);
command_trajectory(robot, trajectory, frequency);
trajectory = trajectory_spline([position_2_approach midpoint position_1_approach], [0, 0.5, 1], frequency);
command_trajectory(robot, trajectory, frequency);
trajectory = trajectory_spline([position_1_approach position_1], [0, 1], frequency);
command_trajectory(robot, trajectory, frequency);

pick(gripper);
pause(0.75);

%% "approach" motions that are done more slowly.
trajectory = trajectory_spline([position_1 position_1_approach], [0, 1], frequency);
command_trajectory(robot, trajectory, frequency);
trajectory = trajectory_spline([position_1_approach midpoint position_2_approach_1], [0, 1, 2], frequency);
command_trajectory(robot, trajectory, frequency);
trajectory = trajectory_spline([position_2_approach_1 position_2_1], [0, 1], frequency);
command_trajectory(robot, trajectory, frequency);

%% Place the object
place(gripper);
pause(0.75);

%% Move back to position 1.
trajectory = trajectory_spline([position_2_1 position_2_approach_1], [0, 1], frequency);
command_trajectory(robot, trajectory, frequency);
trajectory = trajectory_spline([position_2_approach_1 midpoint position_1_approach], [0, 1, 2], frequency);
command_trajectory(robot, trajectory, frequency);
trajectory = trajectory_spline([position_1_approach position_1], [0, 1], frequency);
command_trajectory(robot, trajectory, frequency);

pick(gripper);
pause(0.75);

%% "approach" motions that are done more slowly.
trajectory = trajectory_spline([position_1 position_1_approach], [0, 1], frequency);
command_trajectory(robot, trajectory, frequency);
trajectory = trajectory_spline([position_1_approach midpoint position_2_approach_2], [0, 1, 2], frequency);
command_trajectory(robot, trajectory, frequency);
trajectory = trajectory_spline([position_2_approach_2 position_2_2], [0, 1], frequency);
command_trajectory(robot, trajectory, frequency);

%% Place the object
place(gripper);
pause(0.75);

%% Move back to position 1.
trajectory = trajectory_spline([position_2_2 position_2_approach_2], [0, 1], frequency);
command_trajectory(robot, trajectory, frequency);
trajectory = trajectory_spline([position_2_approach_2 midpoint position_1_approach], [0, 1, 2], frequency);
command_trajectory(robot, trajectory, frequency);
trajectory = trajectory_spline([position_1_approach position_1], [0, 1], frequency);
command_trajectory(robot, trajectory, frequency);
%}


%% Stop logging, and plot results
robot.stopLog();

hebilog = HebiUtils.convertGroupLog(fullfile(currentDir, 'robot_data.hebilog'));

% Plot angle data
figure();
subplot(3,1,1);
plot(hebilog.time, hebilog.positionCmd, 'k', 'LineWidth', 1)
hold on;
plot(hebilog.time, hebilog.position, 'r--', 'LineWidth', 1)
hold off;
title('Plot of joint positions during trajectory');
xlabel('t');
ylabel('\theta');
subplot(3,1,2);
plot(hebilog.time, hebilog.velocityCmd, 'k', 'LineWidth', 1)
hold on;
plot(hebilog.time, hebilog.velocity, 'r--', 'LineWidth', 1)
hold off;
title('Plot of joint velocities during trajectory');
xlabel('t');
ylabel('joint velocities');
subplot(3,1,3);
plot(hebilog.time, hebilog.torque, 'r--', 'LineWidth', 1)
title('Plot of joint torques during trajectory');
xlabel('t');
ylabel('\tau');

end

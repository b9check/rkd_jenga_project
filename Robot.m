classdef Robot
    %ROBOT Represents a general fixed-base kinematic chain.
    
    properties (SetAccess = 'immutable')
        dof
        link_masses
        joint_masses
        dh_parameters
    end
    
    methods
        %% Constructor: Makes a brand new robot with the specified parameters.
        function robot = Robot(dh_parameters, link_masses, joint_masses)
            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(dh_parameters, 2) ~= 4
                error('Invalid dh_parameters: Should be a dof x 4 matrix, is %dx%d.', size(dh_parameters, 1), size(dh_parameters, 2));
            end
            
            if size(link_masses, 2) ~= 1
                error('Invalid link_masses: Should be a column vector, is %dx%d.', size(link_masses, 1), size(link_masses, 2));
            end
            
            if size(joint_masses, 2) ~= 1
                error('Invalid joint_masses: Should be a column vector.');
            end
            
            robot.dof = size(dh_parameters, 1);
            
            if size(joint_masses, 1) ~= robot.dof
                error('Invalid number of joint masses: should match number of degrees of freedom. Did you forget the base joint?');
            end
            
            if size(link_masses, 1) ~= robot.dof
                error('Invalid number of link masses: should match number of degrees of freedom. Did you forget the base joint?');
            end
            
            robot.dh_parameters = dh_parameters;
            robot.link_masses = link_masses;
            robot.joint_masses = joint_masses;
        end
        
        % Returns the forward kinematic map for each frame, one for the base of
        % each link, and one for the end effector. Link i is given by
        % frames(:,:,i), and the end effector frame is frames(:,:,end).
        
        %% Foward Kinematics        
        function frames = forward_kinematics(robot)
            %Allocate a variable containing the homogeneous transforms for each frame
            frames = zeros(4,4,robot.dof);
            %Define initial parameters
            DH = robot.dh_parameters;
            H = 1;
            %Loop through each joint to make a homogeneous transform to that joint
            for row = 1:robot.dof
                %Define parameters from DH matrix
                a = DH(row,1);
                alpha = DH(row,2);
                d = DH(row,3);
                theta = DH(row,4);
                %Calculate homoegeneous transform from the previous joint to the current joint
                H1_i = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta)];
                H2_i = [sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta)];
                H3_i = [0 sin(alpha) cos(alpha) d];
                H4_i = [0 0 0 1];
                H_i =  [H1_i; H2_i; H3_i; H4_i];
                %Multiply by previous homogeneous transform to get full forward kinematic map
                H = H*H_i;
                %Assign to frames variable
                frames(:,:,row) = H;
            end
        end
        
            % The transform from the base of link 'i' to the base frame (H^0_i)
            % is given by the 4x4 matrix frames(:,:,i).
            
            % The transform from the end effector to the base frame (H^0_i) is
            % given by the 4x4 matrix frames(:,:,end).
        
        % Shorthand for returning the forward kinematics.
        function fk = fk(robot)
            fk = robot.forward_kinematics();
        end
        
        % Returns [x; y; z; psi; theta; phi] for the end effector given a
        % set of joint angles. Remember that psi is the roll, theta is the
        % pitch, and phi is the yaw angle.
        function ee = end_effector(robot, thetas)
            % Find the transform to the end-effector frame.
            frames = robot.fk(thetas);
            H_0_ee = frames(:,:,end);
            
            % Extract the components of the end_effector position and
            % orientation.
            
            % --------------- BEGIN STUDENT SECTION ----------------------------------

            % --------------- END STUDENT SECTION ------------------------------------
            
            % Pack them up nicely.
            ee = [x; y; z; psi; theta; phi];
        end
        
        % Shorthand for returning the end effector position and orientation.
        function ee = ee(robot, thetas)
            ee = robot.end_effector(thetas);
        end
        
        %% Jacobians
        
        function jacobians = jacobians_numerical(robot, thetas)
            % Returns the SE(3) Jacobian for each frame (as defined in the forward
            % kinematics map). Note that 'thetas' should be a column vector.
            
            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            %{
            if size(thetas, 1) ~= robot.dof || size(thetas, 2) ~= 1
                error('Invalid thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(thetas, 1), size(thetas, 2));
            end
            %}
            % Allocate a variable containing the Jacobian matrix from each frame
            % to the base frame.
            %jacobians = zeros(6,robot.dof,robot.dof);
            %epsilon = 0.001;
            
            % --------------- BEGIN STUDENT SECTION ----------------------------------
            syms t1 t2 t3 t4 t5

            %Initialize Jacobian
            jacob_angles = [0 0 0 0 1; 0 -1 1 -1 0; 1 0 0 0 0];
            dh = robot.dh_parameters;
            joints = [t1; t2; t3; t4; t5];
            
            %Get forward kinematic map and x, y, z
            frames = forward_kin(dh);
            H_ee = frames(:,:,end);   
            x = H_ee(1,4);
            y = H_ee(2,4);
            z = H_ee(3,4);
            
            %Define Jacobian
            J = sym(zeros(6, 5));
            J(4:6,1:robot.dof) = jacob_angles(:,1:robot.dof);
            
            for j = 1:robot.dof
                J(1,j) = diff(x,joints(j));
                J(2,j) = diff(y,joints(j));
                J(3,j) = diff(z,joints(j));
            end 

            jacobians = subs(J,joints,thetas);


            % --------------- END STUDENT SECTION ------------------------------------
        end
        
        function jacobians = jacobians_analytical(robot)
            % Returns the SE(3) Jacobian for each frame (as defined in the forward
            % kinematics map). Note that 'thetas' should be a column vector.
            
            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            %{
            if size(thetas, 1) ~= robot.dof || size(thetas, 2) ~= 1
                error('Invalid thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(thetas, 1), size(thetas, 2));
            end
            %}
            % Allocate a variable containing the Jacobian matrix from each frame
            % to the base frame.
            
            jacobians = zeros(6,robot.dof,robot.dof);

            % --------------- BEGIN STUDENT SECTION ----------------------------------
            % --------------- END STUDENT SECTION ------------------------------------
        end
        
        
        %% Inverse Kinematics
        
        function thetas = inverse_kinematics_graddescent(robot, initial_thetas, goal_position)
            % Returns the joint angles which minimize a simple squared-distance
            % cost function.
            
            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            %{
            if size(initial_thetas, 1) ~= robot.dof || size(initial_thetas, 2) ~= 1
                error('Invalid initial_thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(initial_thetas, 1), size(initial_thetas, 2));
            end
            %}
            % Allocate a variable for the joint angles during the optimization;
            % begin with the initial condition
            thetas = initial_thetas;
            
            % Step size for gradient update
            step_size = 0.0000005;
            
            % Once the norm (magnitude) of the computed gradient is smaller than
            % this value, we stop the optimization
            stopping_condition = 0.00005;
            
            % Also, limit to a maximum number of iterations.
            max_iter = 50000;
            num_iter = 0;
            
            % --------------- BEGIN STUDENT SECTION ----------------------------------
            % Run gradient descent optimization
            while (num_iter < max_iter)
                
                % Compute the gradient for either an [x;y;z] goal or an
                % [x; y; z; psi; theta; phi] goal, using the current value of 'thetas'.
                % TODO fill in the gradient of the squared distance cost function
                % HINT use the answer for theory question 2, the
                % 'robot.end_effector' function, and the 'robot.jacobians'
                % function to help solve this problem
            
                
                % Update 'thetas'
                % TODO
                
                % Check stopping condition, and return if it is met.
                % TODO
                
                num_iter = num_iter + 1;
            end
            % --------------- END STUDENT SECTION ------------------------------------
        end
        
        function cost = cost_function(robot, thetas, goal_position)
            % Cost function for fmincon
            current_pose = robot.ee(thetas);
            
            % --------------- BEGIN STUDENT SECTION ----------------------------------
            
            % --------------- END STUDENT SECTION ------------------------------------
            
        end
        
        function thetas = inverse_kinematics_numopt(robot, initial_thetas, goal_position)
            % Returns the joint angles which minimize a simple squared-distance
            % cost function. Using built in optimization (fmincon)
            
            
            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            %{
            if size(initial_thetas, 1) ~= robot.dof || size(initial_thetas, 2) ~= 1
                error('Invalid initial_thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(initial_thetas, 1), size(initial_thetas, 2));
            end
            %}
            % Allocate a variable for the joint angles during the optimization;
            % begin with the initial condition
            
            fun = @(thetas)robot.cost_function(thetas, goal_position);
            
            
            % --------------- BEGIN STUDENT SECTION ----------------------------------
            
            
            % --------------- END STUDENT SECTION ------------------------------------
        end
        
        function thetas = inverse_kinematics_analytical(robot, goal_position)
            % Returns the joint angles using an analytical approach to
            % inverse kinematics
            % Note: Kinematics Decoupling might be very useful for this
            % question
            
            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            %if size(initial_thetas, 1) ~= robot.dof || size(initial_thetas, 2) ~= 1
            %    error('Invalid initial_thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(initial_thetas, 1), size(initial_thetas, 2));
            %end

            %OFFSETS
            d1 = 56.05;
            d2 = 103.55;
            d3 = 330.3;
            d4 = 73.05;
            d5 = 254.10;
            d6 = 91;
            d7 = 74.05;
            d8 = 139.7; 

            %GOAL POSITIONS
            x = goal_position(1);
            y = goal_position(2);
            z = goal_position(3); %- d1;
            th = goal_position(4);
            th5 = goal_position(5);

            %CALCULATIONS
            c = sqrt(x^2+y^2);
            phi = atan2(y,x);
            gamma = asin((d2-d4+d6)/c);
            R = sqrt(c^2+(d2-d4+d6)^2);
            %disp('R');
            %disp(R);
            th1 = phi - gamma;
           

            R4 = R-(d8+d7)*cos(th);
            Z4 = z+(d8+d7)*sin(th);
            a = d2 - d4 + d6;

            th3 = acos( (R4^2+Z4^2-d3^2-d5^2) / (2*d3*d5) );
            th2 = -atan2(Z4,R4) + atan2((d5*sin(th3)),(d3+d5*cos(th3)));
            
            disp('geo th2');
            disp(th2*180/pi);
            
            th4 = pi - th - th2 - th3;
            %b = d3*cos(th2) + d4*cos(th2+th3) + (d7+d8)*cos(th2+th3+th4);

            disp('First atan2')
            disp(atan2(Z4,R4)*(180/pi))
            disp('Second atan2')
            disp((atan2((d5*sin(th3)), (d3+d5*cos(th3))))*(180/pi))
            disp('Z4')
            disp(Z4)
            disp('R')
            disp(R)
            disp('R4')
            disp(R4)


            %COORDINATE + SIGN CORRECTIONS
            th4 = -th4;
            th1 = -th1 + pi/2;
            th2 = pi/2 - th2;

            thetas = [th1; th2; th3; th4; th5];


           
        end
        
        function thetas = trajectory_to_thetas(robot, trajectory)
            thetas(:, 1) = robot.inverse_kinematics_analytical(trajectory(:, 1));
            for theta_col=2:size(trajectory, 2)
                thetas(:, theta_col) = robot.inverse_kinematics_analytical(trajectory(:, theta_col));
            end
        end
        
    end
end
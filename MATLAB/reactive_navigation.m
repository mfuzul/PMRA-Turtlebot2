% Path with API functions and tools
addpath('api');
addpath('tools');

% ------------------------ Program parameters -----------------------------
% V-REP
v_ip_addr = '127.0.0.1';    % V-REP is in this machine
v_port = 19999;             % Port is configured in V-REP model
    
% Real Turtle
r_ip_addr = '127.0.0.1';    % Check IP on real turtle's configuration 
r_port = 11311;             % Check port on real turtle's configuration

% Behaviour
defaultLinearVelocity        = 0.2;
defaultFastLinearVelocity    = 0.3;
defaultFastAngularVelocity   = pi;
defaultNormalAngularVelocity = pi / 2; 
defaultSlowAngularVelocity   = pi / 64;

% Reactive navigation constants
KA = 1;
KR = 4;

% Targets
targets = [3.8, 3.8; ];
% -------------------------------------------------------------------------

% Creating an instance of class 'VREP' and starting the simulation
myTurtle = VREP('vrep');
myTurtle.rosinit(v_ip_addr, v_port, r_ip_addr, r_port);

% ------------- Publishers, subscribers and services ----------------------
% Creating the publishers
kob_velocity_pub       = myTurtle.rospublisher('/mobile_base/commands/velocity');
resetodom_pub          = myTurtle.rospublisher('/mobile_base/commands/reset_odometry');
vrep_st_bar_pub        = myTurtle.rospublisher('/vrep/status_bar_message');
vrep_console_open_pub  = myTurtle.rospublisher('/vrep/aux_console/create');
vrep_console_print_pub = myTurtle.rospublisher('/vrep/aux_console/print');

% Creating the subscribers
odom_sub         = myTurtle.rossubscriber('/odom');
kinect_rgb_sub   = myTurtle.rossubscriber('/camera/rgb/image_color');
kinect_depth_sub = myTurtle.rossubscriber('/camera/depth/image_raw');
sim_time_sub     = myTurtle.rossubscriber('/simulation/sim_time');

% Creating the publishers's messages
status_bar_msg    = myTurtle.rosmessage(vrep_st_bar_pub);
velocity_msg      = myTurtle.rosmessage(kob_velocity_pub);
console_print_msg = myTurtle.rosmessage(vrep_console_print_pub);
console_open_msg  = myTurtle.rosmessage(vrep_console_open_pub);
% -------------------------------------------------------------------------

% Creating a V-REP's status bar notification
status_bar_msg.Data = 'Program: Reactive navigation';
myTurtle.send(vrep_st_bar_pub, status_bar_msg);

% ------------ V-REP's auxiliar console initialization --------------------
myTurtle.send(vrep_console_open_pub, console_open_msg);

sim_time_msg = myTurtle.receive(sim_time_sub);
initial_sim_time = sim_time_msg.Data;

disp('Initialized OK.');
console_print_msg.Data = 'Initialized OK.';    
myTurtle.send(vrep_console_print_pub, console_print_msg);
% -------------------------------------------------------------------------

for i = (1:size(targets, 1))
    target = targets(i, :); 

    % Reading the first x and y coordinate
    odom_msg  = myTurtle.receive(odom_sub);       
    [x, y]    = getKobukiPoseAndOrientation(odom_msg);

    % Computing the distance to the target point
    d = sqrt(sum(([x, y] - target) .^ 2));

    while d > 0.05 

        % Reading the current pose and orientation
        odom_msg      = myTurtle.receive(odom_sub);
        [x, y, theta] = getKobukiPoseAndOrientation(odom_msg);
        
        % Computing the distance to the target point
        d = sqrt(sum(([x, y] - target) .^ 2));

        % Showing the current pose in V-REP's console
        console_print_msg.Data = strcat('X: ', num2str(x),', Y: ', num2str(y),...
            ', Theta: ', num2str(theta),', d: ', num2str(d));    
        myTurtle.send(vrep_console_print_pub, console_print_msg);

        % Receiving the Kinect depth message
        kinect_depth_msg = myTurtle.receive(kinect_depth_sub);
        img = readImage(kinect_depth_msg);

        % Taking of the vision line
        vision_line = img(240, :);
        
        % Searching of the closest point
        [max_value, max_index] = max(vision_line);

        % Virtual repulsion force calculation
        if(max_value < 120)
            virtual_repulsion_force = 0;        
        else
            virtual_repulsion_force = double(KR * (max_value - 120));       
        end

        % Closest object handle
        object_angle_estim = deg2rad(((320 - max_index) * (28.5 / 320)));

        % Virtual repulsion vector in robot's reference
        frepx = virtual_repulsion_force * (cos(object_angle_estim + deg2rad(theta) + pi));
        frepy = virtual_repulsion_force * (sin(object_angle_estim + deg2rad(theta) + pi));

        % Virtual atraction force
        if (d < 0.5)
            virtual_atraction_force = KA * 0.5;
            if (i == size(targets, 1))
                velocity_msg.Linear.X = defaultLinearVelocity;                
            else
                velocity_msg.Linear.X = defaultFastLinearVelocity;
            end
        else
            virtual_atraction_force = KA * d;
            velocity_msg.Linear.X   = defaultFastLinearVelocity;
        end    

        % Virtual atraction vector in robot's reference
        targetx = virtual_atraction_force * (target(1, 1) - x);
        targety = virtual_atraction_force * (target(1, 2) - y); 

        % Resultant force
        ftotx = frepx + targetx;
        ftoty = frepy + targety;  
        
        myVectorx = 10 * cos(deg2rad(theta));
        myVectory = 10 * sin(deg2rad(theta));
        
        % Resultant angle       
        angle = atan2(ftoty, ftotx) - atan2(myVectory, myVectorx);
        
        % Si el angulo es mayor que pi, vamos en sentido contrario (camino
        % mas corto)
        if(abs(angle) > pi)
            angular_velocity = -angle;
        else
        	angular_velocity = angle;
        end
       
        if(angular_velocity > pi/2)
            angular_velocity = pi/2;
        elseif(angular_velocity < -pi/2)
            angular_velocity = -pi/2;        
        end        
        
        velocity_msg.Angular.Z = angular_velocity;
        myTurtle.send(kob_velocity_pub, velocity_msg);      
    end
end
% User stopping notification
sim_time_msg = myTurtle.receive(sim_time_sub);
disp(strcat('[', num2str(sim_time_msg.Data, '%.3f'),'] Stopping the turtle...'));
console_print_msg.Data = strcat('[', num2str(sim_time_msg.Data, '%.3f'),'] Stopping the turtle...');    
myTurtle.send(vrep_console_print_pub, console_print_msg);

% Stopping the turtle
velocity_msg.Linear.X  = 0;
velocity_msg.Angular.Z = 0; 
myTurtle.send(kob_velocity_pub, velocity_msg);
myTurtle.pause(1);
% -------------------------------------------------------------------------

% ------------------------ Program Ending ---------------------------------
% Notifying the program's ending to the user
sim_time_msg = myTurtle.receive(sim_time_sub);
disp(strcat('[', num2str(sim_time_msg.Data, '%.3f'),'] Program ended.'));
console_print_msg.Data = strcat('[', num2str(sim_time_msg.Data, '%.3f'),'] Program ended.');    
myTurtle.send(vrep_console_print_pub, console_print_msg);

% Displaying the simulation's elapsed time
sim_time_msg = myTurtle.receive(sim_time_sub);
elapsed_time = sim_time_msg.Data - initial_sim_time;
disp(strcat('Elapsed simulation time:', num2str(elapsed_time),' seconds.'));
console_print_msg.Data = strcat('Elapsed simulation time: ', num2str(elapsed_time),' seconds.');    
myTurtle.send(vrep_console_print_pub, console_print_msg);

myTurtle.pause(1);

% Closing the connection
myTurtle.rosshutdown;
% -------------------------------------------------------------------------
function dynamic_of_system(dhs, ls , masses, ratio , IL, IM ,q0 , qd0)
    % Initial joint angles and rates
    
    q0 = q0*(pi/180);
    [rows,~] = size(dhs);
    
    for i=1:rows
        L(i) = Link('alpha',dhs(i,2),'a', dhs(i,3), 'd', dhs(i,4), 'm', masses(i), 'r', [ls(i) 0 0]', 'I', [0 0 IL(i)], 'G', ratio(i), 'Jm', IM(i));
    end

    % Create the robot
    robot = SerialLink(L);
    robot.nofriction();
    robot.dyn();  
    %robot.edit("dyn");
    robot.gravity = [0 0 0];
    
    % plot of robot configaration
    figure
    plot(robot, q0)
    title(sprintf('Robot Configuration for Initial Joint Angles [%.0f %.0f %.0f] degrees', q0*(180/pi)))
    hold on
    
    % Simulation time and time step
    tspan = 3;
    % Perform the simulation
    [t, q, qd] = robot.fdyn(tspan, @torque_input,q0',qd0');
    
%     for i = 1:length(t)
%         fprintf('For Time %.2f seconds:\n', t(i));
%         fprintf('  Joint position q1 %.2f q2 %.2f q3 %.2f degrees\n', q(i,1)*(180/pi),q(i,2)*(180/pi),q(i,3)*(180/pi));
%         fprintf('  Joint position qd1 %.2f qd2 %.2f qd3 %.2f degrees\n', qd(i,1),qd(i,2),qd(i,3));
%         fprintf('\n'); % Add a blank line for readability
%     end
    
    
    %% Plot visualizing joint postions and joint velocities over time.
    figure
    plot(t, q*(180/pi), 'LineWidth', 2)
    title('Joint Position vs. Time')
    xlabel('Time(s)')
    ylabel('Position (degrees)')
    legend('q1', 'q2', 'q3')
    hold on
    
    
    figure
    plot(t, qd, 'LineWidth', 2)
    title('Joint Velocity vs. Time')
    xlabel('Time(s)')
    ylabel('Velocity (rad/s)')
    legend('qd1', 'qd2', 'qd3') 
    ylim([-0.01,0.04])
    hold on
    q_at_3 = q(49,:)*(180/pi);
    
    % plot of robot configaration
    figure
    plot(robot,q_at_3)
    title(sprintf('Robot Configuration at 3 seconds for Joint Angles [%.0f %.0f %.0f] degrees', q_at_3))
    hold off
end 
function tau = torque_input(~,~,~,~,~,~)
    tau = [10 5 1];
end

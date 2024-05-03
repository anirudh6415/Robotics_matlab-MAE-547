function impedance_entrance(xd1, xd2, K_D, K_P, a1, a2, friction, IL1, IL2, mL1, mL2, mm2, L1, L2, kr1, kr2, Im1, Im2, q1, q2,xd_dot, xd_doubledot, xr, stiffness, mdx, mdy)

    xd = [xd1 xd2];
    qi = [q1 q2];
    options=simset('SrcWorkspace','current','DstWorkspace','current');
    sim(['impedance_control_test3'],10, options)

    % plot desired vs actual end-effector position and contact force
    figure('Name','Indirect force control through impedance control');

    % plot xe xd at x axis
    subplot(4,1,1);
    plot(ans.tout,ans.xe(:,1))
    hold on;
    plot(ans.tout,xd(1)*ones(length(ans.tout)))
    xlabel('s')
    ylabel('mm')
    title('desired vs actual end-effector at x direction');

    % plot ye yd at y axis
    subplot(4,1,2);
    plot(ans.tout,ans.xe(:,2))
    hold on;
    plot(ans.tout,xd(2)*ones(length(ans.tout)))
    xlabel('s')
    ylabel('mm')
    title('desired vs actual end-effector at y direction');

    % end effector contact force
    subplot(4,1,3); 
    plot(ans.tout,ans.f(:,1))
    xlabel('s')
    ylabel('N')
    title('contact force at x direction');

    subplot(4,1,4); 
    plot(ans.tout,ans.f(:,2))
    xlabel('s')
    ylabel('N')
    title('contact force at y direction');                          
end
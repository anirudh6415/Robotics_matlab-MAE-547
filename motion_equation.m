% Input sequence for motion attributes:
% kr ml mm IL Im tau type
% For type, 0: revolute 1: prismatic
function re = motion_equation(dhs, motion_attributes)
    
    syms q1 q2 q3 q4 q5
    syms qd1 qd2 qd3 qd4 qd5
    syms qdd1 qdd2 qdd3 qdd4 qdd5

    [n, cols] = size(motion_attributes);
    
    % DH parameter constants
    alpha = dhs(:,2);
    a = dhs(:,3);
    theta = dhs(:,1);
    d = dhs(:,4);
    
    % Motion parameters extract
    data = motion_attributes;
    disp(data);
    kr = data(:, 1);  % Extract lengths column
    ml = data(:, 2);   % Extract masses column
    mm = data(:, 3);   % Extract ratios column
    IL = data(:, 4);       % Extract IL column
    Im = data(:, 5);       % Extract IM column
    tau = data(:, 6);       % Extract q0 column
    types = data(:, 7);
    
    % Gravity
    g0 = [0 0 9.81];
    
%     % Motion parameters extract

%     motion_attributes
%     motion_attributes = motion_attributes';
%     kr = cell2mat(motion_attributes(1,:))'
%     ml = cell2mat(motion_attributes(2,:))'
%     mm = cell2mat(motion_attributes(3,:))'
%     IL = cell2mat(motion_attributes(4,:))';
%     Im = cell2mat(motion_attributes(5,:))';
%     tau = cell2mat(motion_attributes(6,:))';
%     types = cell2mat(motion_attributes(7,:))';
%     
    switch n
    case 1
        ml = ml(1);
        mm = mm(1);
        kr = kr(1);
        IL = IL(1);
        Im = Im(1);
        q = q1;
        qd = qd1;
        qdd = qdd1;
    case 2
        ml = [ml(1) ml(2)];
        mm = [mm(1) mm(2)];
        kr = [kr(1) kr(2)];
        IL = [IL(1) IL(2)];
        Im = [Im(1) Im(2)];
        q = [q1; q2];
        qd = [qd1; qd2];
        qdd = [qdd1; qdd2];
    case 3
        ml = [ml(1) ml(2) ml(3)];
        mm = [mm(1) mm(2) mm(3)];
        kr = [kr(1) kr(2) kr(3)];
        IL = [IL(1) IL(2) IL(3)];
        Im = [Im(1) Im(2) Im(3)];
        q = [q1; q2; q3];
        qd = [qd1; qd2; qd3];
        qdd = [qdd1; qdd2; qdd3];
    case 4
        ml = [ml(1) ml(2) ml(3) ml(4)];
        mm = [mm(1) mm(2) mm(3) mm(4)];
        kr = [kr(1) kr(2) kr(3) kr(4)];
        IL = [IL(1) IL(2) IL(3) IL(4)];
        Im = [Im(1) Im(2) Im(3) Im(4)];
        q = [q1; q2; q3; q4];
        qd = [qd1; qd2; qd3; qd4];
        qdd = [qdd1; qdd2; qdd3; qdd4];
    otherwise
        ml = [ml(1) ml(2) ml(3) ml(4) ml(5)];
        mm = [mm(1) mm(2) mm(3) mm(4) mm(5)];
        kr = [kr(1) kr(2) kr(3) kr(4) kr(5)];
        IL = [IL(1) IL(2) IL(3) IL(4) IL(5)];
        Im = [Im(1) Im(2) Im(3) Im(4) Im(5)];
        q = [q1; q2; q3; q4; q5];
        qd = [qd1; qd2; qd3; qd4; qd5];
        qdd = [qdd1; qdd2; qdd3; qdd4; qdd5];
    end


    % I use cosd and sind, instead of cos and sin, because cosd and sind can
    % input degree directly, for example, cosd(90) = 0;
    A1 = eye(4);
    for i = 1:n
    A = [cosd(theta(i)) -sind(theta(i))*cosd(alpha(i))  sind(theta(i))*sind(alpha(i)) a(i)*cosd(theta(i));
         sind(theta(i))  cosd(theta(i))*cosd(alpha(i)) -cosd(theta(i))*sind(alpha(i)) a(i)*sind(theta(i));
         0               sind(alpha(i))                 cosd(alpha(i))                d(i);
         0               0                              0                             1];
     T{i} = A1*A;
     A1 = T{i};
     R{i} = A(1:3,1:3);
     if i == 1
         Rm{i} = eye(3);
     else
         Rm{i} = R{i-1};
     end
    end

    % Calculate P0 P1 P2...
    for i = 1:n+1
        if i == 1
            P(:,1) = [0;0;0];
        else
            psave = T{i-1}(1:3,4);
            P = [P psave];
        end
    end

    % Calculate PL1 PL2 PL3...
    for i = 1:n
        PL{i} = 0.5*(P(:,i) + P(:,i+1));
    end

    % Calculate z0 z1 z2...
    for i = 1:n
        if i == 1
            z(:,1) = [0;0;1];
        else
            zsave = T{i-1}(1:3,3);
            z = [z zsave];
        end
    end

    %Calculate JpL
    %for prismatic joint, JpL is z0, z1 or z2. 
    %for revolute joint, JpL is cross product of z0 and (PL{i}-P0)
    for i = 1:n
        for j = 1:i
            types
            switch types(j)
                case 2
                    JpL{i}(:,j) = cross(z(:,j),(PL{i}-P(:,j)));
                otherwise
                    JpL{i}(:,j) = z(:,j);
            end
        end
        for k = 1:(n-i)
            JpL{i}(:,i+k) = zeros(3,1);
        end
    end



    %Calculate JoL
    %for prismatic joint, JoL is 0. 
    %for revolute joint, JoL is z0
    for i = 1:n
        for j = 1:i
            switch types(j)
                case 2
                    JoL{i}(:,j) = z(:,j);
                otherwise
                    JoL{i}(:,j) = zeros(3,1);
            end
        end
        for k = 1:(n-i)
            JoL{i}(:,i+k) = zeros(3,1);
        end
    end

    %Calculate Jpm
    for i = 1:n
        if i == 1
            Jpm{i} = zeros(3,n);
        else
            for j = 1:i-1
                switch types(j)
                    case 2
                        Jpm{i}(:,j) = cross(z(:,j),(PL{i-1}-P(:,j)));
                    otherwise
                        Jpm{i}(:,j) = z(:,j);
                end
            end
            for k = 0:(n-i)
                Jpm{i}(:,i+k) = zeros(3,1);
            end
        end
    end

    %Calculate Jom
    for i = 1:n
        for j = 1:i
            if j < i
                Jom{i}(:,j) = JoL{i}(:,j);
            else
                Jom{i}(:,i) = kr(i)*z(:,i);
            end
        end
        for k = 1:(n-i)
            Jom{i}(:,i+k) = zeros(3,1);
        end
    end

    BB = zeros(n);
    for i = 1:n
        B{i} =  ml(i)*JpL{i}'*JpL{i} + JoL{i}'*R{i}*IL(i)*R{i}'*JoL{i}... 
                + mm(i)*Jpm{i}'*Jpm{i} + Jom{i}'*Rm{i}*Im(i)*Rm{i}'*Jom{i};
        BB = BB + B{i};
    end

    C = sym(zeros(n,n));
    for i = 1:n
        for j = 1:n
            s = 0;
            for k = 1:n
                c{k}(i,j) = 1/2*(diff(BB(i,j),q(k))+diff(BB(i,k),q(j))-diff(BB(j,k),q(i)))*qd(k);
                s = s + c{k}(i,j);
            end
            C(i,j) = s;
        end
    end
    for i = 1:n
        gg{i} = 0;
        for j = 1:n
            gg{i} = gg{i} + ml(j)*g0*JpL{j}(:,i) + mm(j)*g0*Jpm{j}(:,i);
        end
        if i == 1
            g1 = [gg{i}];
        else
            g1 = [g1; gg{i}];
        end

    end
    tau = BB*qdd + C*qd + g1;

    switch n
        case 1
            tau1 = tau(1,1)
            re = [tau1]
        case 2
            tau1 = tau(1,1)
            tau2 = tau(2,1)
            re = [tau1 tau2]
        case 3
            tau1 = tau(1,1)
            tau2 = tau(2,1)
            tau3 = tau(3,1)
            re = [tau1 tau2 tau3]
        case 4
            tau1 = tau(1,1)
            tau2 = tau(2,1)
            tau3 = tau(3,1)
            tau4 = tau(4,1)
            re = [tau1 tau2 tau3 tau4]
        otherwise
            tau1 = tau(1,1)
            tau2 = tau(2,1)
            tau3 = tau(3,1)
            tau4 = tau(4,1)
            tau5 = tau(5,1)
            re = [tau1 tau2 tau3 tau4 tau5]
    end

end
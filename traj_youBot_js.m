% Demo program for trajectory generator of youBot Arm
function traj_youBot_js()

    clc; clear all; close all;

    % start state [x y z pitch roll]
      % start state [x y z pitch roll]
    PoseA = [35 17 20 -10 20];
    VelocityA = [0 0 0 0 0]; % so far no support for other value

    % end state [x y z pitch roll]
    PoseE = [25 10 30 -25 0];
    VelocityE = [0 0 0 0 0];

    % time window and step size
    TA = 0;
    TE = 5;
    TDelta = 0.1;

    % global varibles
    global d1 a1 a2 a3 d5

    d1 = 14.7;
    a1 = 3.3;
    a2 = 15.5;
    a3 = 13.5;
    d5 = 21.75;

    k_Arm1 = 1;
    k_Arm3 = -1;

    %=============== Graphicial setup====================
    ax = createStage([-6 10 -10 10 -2 12], [50 20]);
    set(ax, 'OuterPosition', [0 0 0.5 1]);
    set(gcf,'units','normalized','outerposition',[0 0 1 1])

    camlight();
    linkColors = {[.4 .4 .4], [.5 1 .5], [.5 .5 1], [.5 1 1], [1 .5 .5], [1 1 1]};

    robot = youBot(d1, a1, a2, a3, d5);
    robot.colorLinks(linkColors{1}, linkColors{2}, linkColors{3}, linkColors{4}, linkColors{5}, linkColors{6});
    robot.setTransparency(1);
    robot.hideOrigins();
    
    Theta_ini = ik_Youbot(PoseA,k_Arm1,k_Arm3);
    robot.setJoins(Theta_ini);

    % control UI
    hArmConfig1 = uicontrol('Style', 'checkbox', 'String', 'k_arm1 = -1', 'Position', [5, 5, 100, 20], 'Value', 0);
    hArmConfig3 = uicontrol('Style', 'checkbox', 'String', 'k_arm3 = -1', 'Position', [120, 5, 100, 20], 'Value', 1);
    hPlayBtn = uicontrol('Style', 'pushbutton', 'String', 'Play', 'Position', [240, 0, 80, 40], 'Callback', @play);

    % Path of the TCP in 3D view
    hPlot = line('XData', [], 'YData', [], 'Color', 'k', 'Marker', '.', 'LineStyle', 'none');

    % Diagram preparation
    axTheta = axes('OuterPosition', [0.5 2/3 0.5 1/3], 'XLim', [TA TE]);
    hTheta1 = line('XData', TA, 'YData', 0, 'Color', linkColors{1}, 'Marker', '.');
    hTheta2 = line('XData', TA, 'YData', 0, 'Color', linkColors{2}, 'Marker', '.');
    hTheta3 = line('XData', TA, 'YData', 0, 'Color', linkColors{3}, 'Marker', '.');
    hTheta4 = line('XData', TA, 'YData', 0, 'Color', linkColors{4}, 'Marker', '.');
    hTheta5 = line('XData', TA, 'YData', 0, 'Color', linkColors{5}, 'Marker', '.');
    legend('\theta_1','\theta_2','\theta_3','\theta_4', '\theta_5');
    ylabel('\theta in deg');
    grid on;
    axThetaDot = axes('OuterPosition', [0.5 1/3 0.5 1/3], 'XLim', [TA TE]);
    hThetaDot1 = line('XData', TA, 'YData', 0, 'Color', linkColors{1}, 'Marker', '.');
    hThetaDot2 = line('XData', TA, 'YData', 0, 'Color', linkColors{2}, 'Marker', '.');
    hThetaDot3 = line('XData', TA, 'YData', 0, 'Color', linkColors{3}, 'Marker', '.');
    hThetaDot4 = line('XData', TA, 'YData', 0, 'Color', linkColors{4}, 'Marker', '.');
    hThetaDot5 = line('XData', TA, 'YData', 0, 'Color', linkColors{5}, 'Marker', '.');
    legend('\omega_1','\omega_2','\omega_3','\omega_4', '\omega_5');
    ylabel('\omega in deg/s');
    grid on;
    axTheta2Dot = axes('OuterPosition', [0.5 0 0.5 1/3], 'XLim', [TA TE]);
    hTheta2Dot1 = line('XData', TA, 'YData', 0, 'Color', linkColors{1}, 'Marker', '.');
    hTheta2Dot2 = line('XData', TA, 'YData', 0, 'Color', linkColors{2}, 'Marker', '.');
    hTheta2Dot3 = line('XData', TA, 'YData', 0, 'Color', linkColors{3}, 'Marker', '.');
    hTheta2Dot4 = line('XData', TA, 'YData', 0, 'Color', linkColors{4}, 'Marker', '.');
    hTheta2Dot5 = line('XData', TA, 'YData', 0, 'Color', linkColors{5}, 'Marker', '.');
    legend('\alpha_1','\alpha_2','\alpha_3','\alpha_4', '\alpha_5');
    ylabel('\alpha in deg^2/s');
    grid on;

    function play(hObject,eventdata)
%         set(hArmConfig1,'Enable','off');
%         set(hArmConfig3,'Enable','off');

        % Speicher für Datenreihen
        % Format: [t x y z theta1 theta2 theta3 theta4 theta5 omega1 omega2 
        %          omega3 omega4 omega5 alpha1 alpha2 alpha3 alpha4 alpha5]
        recData = [];

        % read the arm configuration
        if get(hArmConfig1, 'Value') == 1
            k_Arm1 = -1;
        else k_Arm1 = 1;
        end

        if get(hArmConfig3, 'Value') == 1
            k_Arm3 = -1;
        else k_Arm3 = 1;
        end
        
        % Calculate coefficents
        coeff = genTrajJS(PoseA, VelocityA, PoseE, VelocityE); 

        %  animate the trajectory
        animate(TA:TDelta:TE,@animStep,TDelta);
        
        disp('Maximum angular rates:');
        fprintf('omega1: %9.2f deg/s\nomega2: %9.2f deg/s\nomega3: %9.2f deg/s\nomega4: %9.2f deg/s\nomega5: %9.2f deg/s\n', max(abs(recData(:, 10:14))) * 180 / pi);
		disp('Maximum angular accelerations:');
        fprintf('alpha1: %9.2f deg^2/s\nalpha2: %9.2f deg^2/s\nalpha3: %9.2f deg^2/s\nalpha4: %9.2f deg^2/s\nalpha5: %9.2f deg^2/s\n', max(abs(recData(:, 15:19))) * 180 / pi);

%         set(hArmConfig1,'Enable','on');
%         set(hArmConfig3,'Enable','on');

        function animStep(t)
            Theta = zeros(1,5);
            ThetaDot = zeros(1,5);
            Theta2Dot =zeros(1,5);
            % actual joint angles, velocities and accelerations
            for i=1:5
                Theta(i) = [1, t, t^2, t^3]*coeff(:,i);
                ThetaDot(i) = [0, 1, 2*t, 3*t^2]*coeff(:,i);			
                Theta2Dot(i) = [0, 0, 2, 6*t]*coeff(:,i);
            end
            
            % Refresh 3D view
            robot.setJoins(Theta);
            tcp = robot.getTcp();
            
            % record data for this step
            recData = [recData; t, tcp, Theta, ThetaDot, Theta2Dot];
            
            % draw the trajectory of tcp
            set(hPlot, 'XData', recData(:, 2), 'YData', recData(:, 3), 'ZData', recData(:, 4));
            
            % plot angle, angular velocity and angular acceleratuin
            arrayfun(@(h, i)set(h, 'XData', recData(:, 1), 'YData', recData(:, i) * 180 / pi), ...
					 [hTheta1, hTheta2, hTheta3, hTheta4, hTheta5, hThetaDot1, hThetaDot2, hThetaDot3, hThetaDot4, hThetaDot5, hTheta2Dot1, hTheta2Dot2, hTheta2Dot3, hTheta2Dot4, hTheta2Dot5],...
					 5:19);
        end


    end

    function A=genTrajJS(startP,startV,endP,endV)
        % calculate joint angles at the start point
        jointArrayA = ik_Youbot(startP,k_Arm1,k_Arm3);        
        % solution valid check
        if numel(jointArrayA)~=5
            return
        end

        robot.setJoins(jointArrayA(1), jointArrayA(2), jointArrayA(3), jointArrayA(4), jointArrayA(5));

        % calculate joint angles at the start point
        jointArrayE = ik_Youbot(endP,k_Arm1,k_Arm3);
        % solution valid check
        if numel(jointArrayE)~=5
            return
        end

        % Calculate coefficients of the trajectories for each joint
        A = zeros(4,5);
        B = [1, TA, TA^2, TA^3;...
            0, 1, 2*TA, 3*TA^2;...
            1, TE, TE^2, TE^3;...
            0, 1, 2*TE, 3*TE^2];
        inverseB = inv(B);
        for i=1:5
           A(:,i) = inverseB*[jointArrayA(i);0;jointArrayE(i);0]; 
    %        A(:,i) = [startP(i);startV(i);endP(i);endV(i)];/B;
        end

    end

    function [theta] = ik_Youbot(desiredpose,jointconfig1,jointconfig3)
        gx=desiredpose(1);
        gy=desiredpose(2);
        gz=desiredpose(3);
        gphi=desiredpose(4)*pi/180;
        ggamma=desiredpose(5)*pi/180;
        T_B_T = T_shift(gx, gy, gz) * T_rot('xz', gphi, ggamma);  

        % first joint
        j1 = atan2(gy,gx);
        if jointconfig1 == 1
            pt_x = sqrt(gx^2 + gy^2)-a1;            
        else
            pt_x = sqrt(gx^2 + gy^2)+a1;              

            if j1<0
                j1= j1+pi;
            else
                j1= j1-pi;
            end
        end
        pt_y = gz-d1;

        % check if the goal positon can be reached
        if sqrt(pt_x^2+pt_y^2)>(a2+a3+d5)
            errordlg('Out of work space!');
            return
        end

        % third joint
        pw_x = pt_x - d5*cos(gphi);
        pw_y = pt_y - d5*sin(gphi);

        % check if the goal position can be reached at all
        if sqrt(pw_x^2 + pw_y^2)>(a2+a3) || sqrt(pw_x^2 + pw_y^2)<abs(a2-a3)
            errordlg('goal position cannot be reached!');
            theta=[];
            return
        end

        alpha = atan2(pw_y, pw_x);

        j3_cos = (pw_x^2 + pw_y^2 - a2^2 - a3^2)/(2*a2*a3);
        if j3_cos > 0.9999999
            j3 = 0;
        elseif j3_cos < -0.9999999
            j3 = pi;
        else
            j3 = atan2(sqrt(1-j3_cos^2), j3_cos);
        end
        j3 = jointconfig3*j3;

        % second joint
        beta_cos = (pw_x^2 + pw_y^2 + a2^2 - a3^2)/(2*a2*sqrt(pw_x^2 + pw_y^2));
        if beta_cos > 0.9999999
            beta = 0;
        elseif beta_cos < -0.9999999
            beta = pi;
        else
            beta = atan2(sqrt(1-beta_cos^2), beta_cos);
        end
        if j3<0
            j2 = alpha + beta;
        else
            j2 = alpha - beta;
        end

        % fourth joint determines the pitch of the gripper
        j4 = gphi-j2-j3;

        % fifth joint, determines the roll of the gripper (= wrist angle)
%         j5_cos = T_B_T(2,2)*cos(j1)-T_B_T(12)*sin(j1);
%         if j5_cos > 0.9999999
%             j5 = 0;
%         elseif j3_cos < -0.9999999
%             j5 = pi;
%         else
%             j5 = atan2((T_B_T(2,1)*cos(j1)-T_B_T(1,1)*sin(j1)), (T_B_T(2,2)*cos(j1)-T_B_T(12)*sin(j1)));
%         end  
        j5 = ggamma;

        if jointconfig1 == -1
            j2 = pi-j2;
            j3 = -j3;
            j4 = -j4;
            j5 = -j5;
        end

        theta = [j1, j2, j3, j4, j5];
        
%         solution_valid = isSolutionValid(jointArrayA);
%         if ~solution_valid 
%             theta = [];
%             errordlg('Joint angle out of range!');
%         end
    end

    function solution_valid = isSolutionValid(theta)
        max_angles = [169, 155, 151, 102.5, 167.5].*pi/180;
        min_angles = [-169, 0 -146, -102.5, -167.5].*pi/180;
        
        solution_valid = true;
        if numel(theta)~=5
            solution_valid = false;
            return
        end
        for i=1:5
            if theta(i)<min_angles(i) ||theta(i)>max_angles(i)
                solution_valid = false;
                return
            end
        end
    end

end


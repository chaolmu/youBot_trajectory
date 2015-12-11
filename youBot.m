function robot = youBot(d1, a1, a2, a3, d5)
    tcp = [0 0 0];	
    width = 4;
    
    % for debug
%     d1 = 14.7;
%     a1 = 3.3;
%     a2 = 15.5;
%     a3 = 13.5;
%     d5 = 21.75;
%     ax = createStage([-60 60 -60 60 -30 70], [-38 29]);
%     set(ax, 'CameraUpVector', [1 0 0], 'CameraPosition', [12, 15, -3], 'CameraTarget', [1 0 5]);
    

	robot = struct();
	robot.org0 = triade(T_unity(), [], 4, 0.1);
	robot.joint0 = createObject(geoCylinder(5.5, 7.2));
%     robot.joint0.setWireframeColor([0 0.2 0]);


	%t = linspace(-pi, 0, 17);
	robot.org1 = triade(T_unity(), [], 4, 0.1);
	robot.link1 = createObject(transform(T_shift(-1.5, -d1, -0.95*width/2), geoBox(3, d1, 0.95*width)));
%     robot.link1.setWireframeColor([0 0.2 0]);
    robot.joint1 = createObject(transform(T_shift(0, 0, -width/2), geoCylinder(2, width)));
    
	robot.org2 = triade(T_unity(), [], 4, 0.1);
    robot.link2 = createObject(transform(T_shift(-a2, -1.5, -0.95*width/2), geoBox(a2, 3, 0.95*width)));
%     robot.link2.setWireframeColor([0 0.2 0]);
    robot.joint2 = createObject(transform(T_shift(0, 0, -width/2), geoCylinder(2, width)));

	robot.org3 = triade(T_unity(), [], 4, 0.1);
	robot.link3 = createObject(transform(T_shift(-a3, -1.5, -0.95*width/2), geoBox(a3, 3, 0.95*width)));
%     robot.link3.setWireframeColor([0 0.2 0]);
    robot.joint3 = createObject(transform(T_shift(0, 0, -width/2), geoCylinder(2, width)));

	robot.org4 = triade(T_unity(), [], 6, 0.1);
    robot.link4 = createObject(transform(T_shift(-1.5, -0.9*width/2, 0), geoBox(3, 0.9*width, d5-3-width)));
%     robot.link4.setWireframeColor([0 0.2 0]);
    robot.joint4 = createObject(transform(T_shift(0, 0, -(width+3)), geoCylinder(2.15, width)));
    
    robot.org5 = triade(T_unity(), [], 4, 0.1);
    link5_1 = createObject(transform(T_shift(-0.5, -3, -3), geoBox(1, 6, 1)));
    link5_2 = createObject(transform(T_shift(-0.5, 1, -2), geoBox(1, 1, 2)));
    link5_3 = createObject(transform(T_shift(-0.5, -2, -2), geoBox(1, 1, 2)));
    robot.gripper = createObjectGroup(link5_1,link5_2,link5_3);
%     robot.gripper.setWireframeColor([0 0.2 0]);
    
    
    robot.setJoins = @setJoins;

	robot.getTcp = @getTcp;
	function [out] = getTcp()
		out = tcp;
	end
	robot.setTransparency = @setTransparency;
	function setTransparency(t)
		robot.link1.setTransparency(t);
 		robot.link2.setTransparency(t);
 		robot.link3.setTransparency(t);
 		robot.link4.setTransparency(t);
 		robot.gripper.setTransparency(t);
        robot.joint0.setTransparency(t);
        robot.joint1.setTransparency(t);
        robot.joint2.setTransparency(t);
        robot.joint3.setTransparency(t);
        robot.joint4.setTransparency(t);
	end
	robot.colorLinks = @colorLinks;
	function colorLinks(varargin)
		if nargin < 1, return; end
		robot.joint0.setFaceColor(varargin{1 + mod(0, nargin)});		
		robot.org0.setColor(varargin{1 + mod(0, nargin)});
		robot.link1.setFaceColor(varargin{1 + mod(0, nargin)});
		robot.org1.setColor(varargin{1 + mod(1, nargin)});
        robot.joint1.setFaceColor(varargin{1 + mod(1, nargin)});
		robot.link2.setFaceColor(varargin{1 + mod(1, nargin)});
		robot.org2.setColor(varargin{1 + mod(2, nargin)});
        robot.joint2.setFaceColor(varargin{1 + mod(2, nargin)});
		robot.link3.setFaceColor(varargin{1 + mod(2, nargin)});		
		robot.org3.setColor(varargin{1 + mod(3, nargin)});
        robot.joint3.setFaceColor(varargin{1 + mod(3, nargin)});
		robot.link4.setFaceColor(varargin{1 + mod(3, nargin)});		
		robot.org4.setColor(varargin{1 + mod(4, nargin)});
        robot.joint4.setFaceColor(varargin{1 + mod(5, nargin)});
		robot.gripper.setFaceColor(varargin{1 + mod(5, nargin)});		
		robot.org5.setColor(varargin{1 + mod(5, nargin)});
	end
	robot.showOrigins = @()cellfun(@(o)o.show(), {robot.org0, robot.org1, robot.org2, robot.org3, robot.org4, robot.org5});
	robot.hideOrigins = @()cellfun(@(o)o.hide(), {robot.org0, robot.org1, robot.org2, robot.org3, robot.org4, robot.org5});
	
	setJoins(0, 155*(pi/180), -146*(pi/180), 102.5*(pi/180), 0);
	
	function setJoins(theta1, theta2, theta3, theta4, theta5)
		if nargin == 1 && numel(theta1) == 5
			theta5 = theta1(5);
			theta4 = theta1(4);
			theta3 = theta1(3);
			theta2 = theta1(2);
			theta1 = theta1(1);
		end
		T_1_0 = T_dh(theta1, d1, a1, pi/2);
		T_2_1 = T_dh(theta2, 0, a2, 0);
		T_3_2 = T_dh(theta3, 0, a3, 0);
		T_4_3 = T_dh(theta4 + pi/2, 0, 0, pi/2);
		T_5_4 = T_dh(theta5, d5, 0, 0);

		robot.org1.place(T_1_0);
		robot.link1.place(T_1_0);
        robot.joint1.place(T_1_0);
		T_2_0 = T_1_0 * T_2_1;
		robot.org2.place(T_2_0);
		robot.link2.place(T_2_0);
        robot.joint2.place(T_2_0);
		T_3_0 = T_2_0 * T_3_2;
		robot.org3.place(T_3_0);
		robot.link3.place(T_3_0);
        robot.joint3.place(T_3_0);
		T_4_0 = T_3_0 * T_4_3;
 		robot.org4.place(T_4_0);
        robot.link4.place(T_4_0);
     
		T_5_0 = T_4_0 * T_5_4;
		robot.org5.place(T_5_0);
		robot.gripper.place(T_5_0);
        robot.joint4.place(T_5_0);
		
		tcp = transform(T_5_0, [0 0 0]);
	end
end
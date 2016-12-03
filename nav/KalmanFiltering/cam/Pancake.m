%%%%%%%%%%%%%%%%%%%%%%%%
% Cameron Seward
% Attempt 1 : 12-2-2016
%%%%%%%%%%%%%%%%%%%%%%%%

classdef Pancake < handler
   properties (Constant = true)
        stateDim = 3; %State [phi, theta, d]
        controlDim = 2; %Control [v, omega]
        noiseDim = 2; %Noise [n_v, n_omega]
        
        zeroNoise = [0;0]; % zero noise vector
   end
   
   properties
       R = 12 * 0.0254; %radius of pipe in meters
   end
   
   methods
       function robot = Pancake()
            robot.dt = 0.1;
            robot.Q_w = 0.001 * eye(2);
       end
       
       function next_x(robot, x, u, w)
           phi = x(1);
           sqrt_dt = sqrt(robot.dt);
           
           delta_phi = u(2)*robot.dt + w(2)*sqrt_dt;
           delta_theta = (u(1)*sin(phi)*robot.dt)/robot.R + (w(1)*((sin(phi)*sqrt_dt)/robot.R));
           delta_d = u(1)*cos(phi)*robot.dt + w(1)*cos(phi)*sqrt_dt;
           
           x_next = x + [delta_phi; delta_theta; delta_d];
       end
   
       function F = getStateTranstionJacobian(robot, x, u)
       end
       
       function L = getProcessNoiseJacobian(robot, x)
       end
       
       function w = genProcessNoise(robot, x, u)
       end
end
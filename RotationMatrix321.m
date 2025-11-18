% Contributors: Cole Nightingale, Katya Korobov, Spencer Batten, Brady Hormuth
% Course number: ASEN 3801
% File name: RotationMatrix321.m
% Created: 8/26/25
% Purpose: Contains the function definition for the RotationMatrix321
%   function, which creates a rotation matrix given a set of Euler angles
%   in the 3-2-1 rotation sequence.

function DCM = RotationMatrix321(attitude321)
    % Use the Euler angles for the 3-2-1 rotation sequence to calculate the associated DCM% % Inputs:
    %
    % Inputs:
    %   attitude321 : 3x1 vector with Euler angles in the form [phi, theta, psi]'
    %
    % Outputs:
    %   DCM : 3x3 rotation matrix calculated from the input angles
    
    % Unpack angles
    phi = attitude321(1);
    theta = attitude321(2);
    psi = attitude321(3);

    % First rotation matrix (about 3rd axis)
    R3 = [cos(psi), sin(psi), 0;
         -sin(psi), cos(psi), 0;
             0,           0,      1;];
    % Second rotation matrix (about 2nd axis)
    R2 = [cos(theta), 0, -sin(theta);
              0,     1,      0;
          sin(theta), 0, cos(theta)];
    
    % Third rotation matrix (about 1st axis)
    R1 = [1,      0,          0;
          0, cos(phi), sin(phi);
          0, -sin(phi), cos(phi)];
    
    % Combined rotation matrix (DCM)
    DCM = R1 * R2 * R3;
end
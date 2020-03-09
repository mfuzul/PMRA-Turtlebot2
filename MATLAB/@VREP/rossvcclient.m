function [client, returnCode] = rossvcclient(robot, servicename)
%ROSSVCCLIENT creates a service client that connects to a service server
%
%   [client, returnCode] = rossvcclient(robot, servicename)
%
%   'returnCode' will always be set to '0' (No errors)

% -------------------------------------------------------------------------
% Ivan Fernandez-Vega
% B.Sc. Degree in Computer Engineering
% University of Malaga
% June, 2016
% -------------------------------------------------------------------------
%    This program is free software: you can redistribute it and/or modify
%    it under the terms of the GNU General Public License as published by
%    the Free Software Foundation, either version 3 of the License, or
%    (at your option) any later version.
%
%    This program is distributed in the hope that it will be useful,
%    but WITHOUT ANY WARRANTY; without even the implied warranty of
%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%    GNU General Public License for more details.
%
%    You should have received a copy of the GNU General Public License
%    along with this program.  If not, see <http://www.gnu.org/licenses/>.
% -------------------------------------------------------------------------

    if robot.operating_mode == 0 % V-REP        
        client = struct('ServerName', servicename);
        returnCode = 0;
        
    elseif (robot.operating_mode == 1 || robot.operating_mode == 2) % ROS or MIXED 
        client = rossvcclient(servicename); 
        returnCode = 0;
        
    end
end

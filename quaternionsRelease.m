angleArr120   = [10, 20, 30, 40, 50, 60];   % array with each joint angle
rotAxis120    = [3, 2, 2, 1, 2, 1];         % rotation axis: 1 = roll
                                            %   2 = pitch, 3 = yaw
axisOffsets = [ 0, 0, 290       ;  ...      % offsets from joint, in the
                0, 0, 270       ;  ...      %   format [x,y,z], referenced
                134, 0, 70      ; ...       %   to each previous joint's
                302-134, 0, 0 ; ...         %   origin
                72, 0, 0        ; ...
    ];

% Calculate the end effector position quat (ee) and end effector rotation
%   (eeRot) quaternion.
[ee, eeRot] = calculateDual(angleArr120, rotAxis120, axisOffsets);

% Extract the parts, for pretty printing
[x, y, z, t] = parts(ee);
fprintf("Final link position: %.2f %.2f %.2f %.2f\n", ...
        x, y, z, t);


% Calculates direct kinematics of a manipulator constructed with revolute
%   joints.
% @param angleArr angle array containing the joint angles found
% @param rotAxis an array representing the axis around which rotation is
%   done
% @param axisOffsets a 3xN matrix, containing the offsets from each joint
%   to the previous one, for N - 1 joints.
% @param plotPoints contains plot points to be displayed, may be left blank
% @return [eePos, eeRot] eePos = position quaternion, eeRot orientation
%   quaternion
function [eePos, eeRot] = calculateDual(angleArr, rotAxis, axisOffsets, plotPoints) 
    [r,c] = size(angleArr);             % perform a sanity check, to avoid                                 
                                        % malformed input
    if ~((r > 1) || (c > 1))
        disp("this function operates on an angle array")
        EE = NaN;
        return;
    end

    finalLinkT = quaternion(0, 0, 0, 0);        % final translation quat
    finalLinkR = quaternion(1, 0, 0, 0);        % final orientation quat
    offset = 1;                                 % used in printing
    
    for i = 1 : (c - 1)                         % for each joint
        switch rotAxis(i)                       % convert axis-angle to
            case 1                              % quaternion
                rot = rpy(-deg2rad(angleArr(i)), 0, 0);
            case 2
                rot = rpy(0, -deg2rad(angleArr(i)), 0);
            case 3
                rot = rpy(0, 0, -deg2rad(angleArr(i)));
        end

        link = quaternion(0, ...                % construct link
            axisOffsets(i, 1), ...              %   (vector magnitude part)
            axisOffsets(i, 2), ...
            axisOffsets(i, 3));

        rotLoc = rot * finalLinkR;              % construct local rotation
        rotated =conj(rotLoc) * link * rotLoc;  % and rotate the link

        finalLinkT = finalLinkT + rotated;      % translate by given coords

        [~, y, z, t] = parts(finalLinkT);       % plot the points and links
        plotPoints(i + offset, 1) = y;
        plotPoints(i + offset, 2) = z;
        plotPoints(i + offset, 3) = t;

        finalLinkR = rot * finalLinkR;          % and finally, take into
    end                                         % account the final orient.

    fprintf("Final link orientation: %.2f %.2f %.2f\n", ...
        getRPYFromQuaternion(finalLinkR));

    [x, y, z, t] = parts(finalLinkR);
    fprintf("Final rot q: %.2f, %.2f, %.2f, %.2f\n", x, y, z, t);
    
    plot3(plotPoints(:,1), plotPoints(:,2), plotPoints(:,3),'LineWidth',8, 'Marker', 'o');
    
    eePos = finalLinkT;
    eeRot = finalLinkR;
end


% Converts from degrees to radians
% @param deg the value to be converted, in degrees
% @return rad the converted value, in radians
function rad = deg2rad(deg)

    rad = pi() / 180 * deg;

end

% Returns the conjugated quaternion
% @param quat quaternion to be conjugated
% @param conjugate the conjugated quaternion
function conjugate = conj(quat)
    [q1, q2, q3, q4] = parts(quat);
    
    conjugate = quaternion(q1, -q2, -q3, -q4);

end

% Transforms an axis-angle rotation to quaternion
% @param u the value around axis x
% @param u the value around axis y
% @param u the value around axis z
% @return r the resulting quaternion
function r = rpy(u, v, w)
    q0 = cos(u / 2) * cos(v / 2) * cos(w / 2) + sin(u / 2) * sin(v / 2) * sin(w / 2);
    q1 = sin(u / 2) * cos(v / 2) * cos(w / 2) - cos(u / 2) * sin(v / 2) * sin(w / 2);
    q2 = cos(u / 2) * sin(v / 2) * cos(w / 2) + sin(u / 2) * cos(v / 2) * sin(w / 2);
    q3 = cos(u / 2) * cos(v / 2) * sin(w / 2) - sin(u / 2) * sin(v / 2) * cos(w / 2);
    r = quaternion(q0, q1, q2, q3);
end
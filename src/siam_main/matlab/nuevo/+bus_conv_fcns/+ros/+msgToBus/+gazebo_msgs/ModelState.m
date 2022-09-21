function slBusOut = ModelState(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    slBusOut.ModelName_SL_Info.ReceivedLength = uint32(strlength(msgIn.ModelName));
    currlen  = min(slBusOut.ModelName_SL_Info.ReceivedLength, length(slBusOut.ModelName));
    slBusOut.ModelName_SL_Info.CurrentLength = uint32(currlen);
    slBusOut.ModelName(1:currlen) = uint8(char(msgIn.ModelName(1:currlen))).';
    currentlength = length(slBusOut.Pose);
    for iter=1:currentlength
        slBusOut.Pose(iter) = bus_conv_fcns.ros.msgToBus.geometry_msgs.Pose(msgIn.Pose(iter),slBusOut(1).Pose(iter),varargin{:});
    end
    slBusOut.Pose = bus_conv_fcns.ros.msgToBus.geometry_msgs.Pose(msgIn.Pose,slBusOut(1).Pose,varargin{:});
    currentlength = length(slBusOut.Twist);
    for iter=1:currentlength
        slBusOut.Twist(iter) = bus_conv_fcns.ros.msgToBus.geometry_msgs.Twist(msgIn.Twist(iter),slBusOut(1).Twist(iter),varargin{:});
    end
    slBusOut.Twist = bus_conv_fcns.ros.msgToBus.geometry_msgs.Twist(msgIn.Twist,slBusOut(1).Twist,varargin{:});
    slBusOut.ReferenceFrame_SL_Info.ReceivedLength = uint32(strlength(msgIn.ReferenceFrame));
    currlen  = min(slBusOut.ReferenceFrame_SL_Info.ReceivedLength, length(slBusOut.ReferenceFrame));
    slBusOut.ReferenceFrame_SL_Info.CurrentLength = uint32(currlen);
    slBusOut.ReferenceFrame(1:currlen) = uint8(char(msgIn.ReferenceFrame(1:currlen))).';
end

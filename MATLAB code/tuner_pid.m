clc

%tuner and system identification

data = iddata(out.angle_out,out.force_in,out.tout(2)-out.tout(1));

% Abort procedure
function abort_exp(obj,step)
curr_pos_msg=obj.getCartesian();
curr_pos=[curr_pos_msg.X curr_pos_msg.Y curr_pos_msg.Z];
curr_ori=[curr_pos_msg.Q0 curr_pos_msg.Qx curr_pos_msg.Qy curr_pos_msg.Qz];
obj.moveCartesianFullOrientation(curr_pos+[step -step step],curr_ori);
end
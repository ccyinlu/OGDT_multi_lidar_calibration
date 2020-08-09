function pose = loadPoseYaml(filePath)
    % load the 6dof pose file [roll pitch yaw x y z] angles in degree
    pose_struct = YAML.read(filePath);

    roll = pose_struct.roll * pi / 180;
    pitch = pose_struct.pitch * pi / 180;
    yaw = pose_struct.yaw * pi / 180;

    x = pose_struct.x;
    y = pose_struct.y;
    z = pose_struct.z;

    pose = [roll pitch yaw x y z];
end

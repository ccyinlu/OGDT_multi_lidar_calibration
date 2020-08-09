function pose = pose_from_transform_matrix(transform_matrix)
    % pose corresponds to [roll pitch yaw x y z]
    eul = rotm2eul(transform_matrix(1:3, 1:3));
    roll = eul(3);
    pitch = eul(2);
    yaw = eul(1);
    x = transform_matrix(1, 4);
    y = transform_matrix(2, 4);
    z = transform_matrix(3, 4);

    pose = [roll pitch yaw x y z];
end
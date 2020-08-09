function transform_matrix = transform_matrix_from_pose(pose)
    % pose corresponds to [roll pitch yaw x y z]
    transform_matrix = eye(4);
    rot = eul2rotm([pose(3) pose(2) pose(1)]);
    vec = [pose(4) pose(5) pose(6)];
    transform_matrix(1:3, 1:3) = rot;
    transform_matrix(1:3, 4) = vec';
end
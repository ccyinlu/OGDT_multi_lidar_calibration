function transform_matrix = transform_matrix_from_pose_quat(pose)
    % pose corresponds to [x y z qx qy qz qw]
    transform_matrix = eye(4);
    rot = quat2rotm([pose(7) pose(4) pose(5) pose(6)]); % [qw qx qy qz]
    vec = [pose(1) pose(2) pose(3)];
    transform_matrix(1:3, 1:3) = rot;
    transform_matrix(1:3, 4) = vec';
end
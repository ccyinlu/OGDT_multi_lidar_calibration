function [error_xyz, error_xy] = forward_backward_projection_error(gt_6dof, est_6dof, control_points)
  % inputs:
  % gt_6dof: 1x6 [roll pitch yaw x y z] [rad rad rad m m m]
  % est_6dof: 1x6 [roll pitch yaw x y z] [rad rad rad m m m]
  % control_points: Nx3

  % output:
  % error_xyz: Nx1
  % error_xy: Nx1

  % form the transform matrix
  gt_transform_matrix = eye(4);
  gt_transform_rot = eul2rotm([gt_6dof(3) gt_6dof(2) gt_6dof(1)]);
  gt_transform_matrix(1:3, 1:3) = gt_transform_rot;
  gt_transform_matrix(1, 4) = gt_6dof(4);
  gt_transform_matrix(2, 4) = gt_6dof(5);
  gt_transform_matrix(3, 4) = gt_6dof(6);

  est_transform_matrix = eye(4);
  est_transform_rot = eul2rotm([est_6dof(3) est_6dof(2) est_6dof(1)]);
  est_transform_matrix(1:3, 1:3) = est_transform_rot;
  est_transform_matrix(1, 4) = est_6dof(4);
  est_transform_matrix(2, 4) = est_6dof(5);
  est_transform_matrix(3, 4) = est_6dof(6);

  % transform the control points from the parent to child
  control_points_hom = [control_points ones(size(control_points, 1), 1)];
  control_points_backward_hom = (inv(gt_transform_matrix) * control_points_hom')';

  control_points_backward_forward_hom = (est_transform_matrix * control_points_backward_hom')';
  control_points_backward_forward = control_points_backward_forward_hom(:, 1:3);

  control_points_error = control_points_backward_forward - control_points;

  error_xyz = sqrt(control_points_error(:, 1).^2 + control_points_error(:, 2).^2 + control_points_error(:, 3).^2);
  error_xy = sqrt(control_points_error(:, 1).^2 + control_points_error(:, 2).^2);
end
function deltaZRP_child2parent = estimate_deltaZRP_from_mountZRP(mountZRP_child, mountZRP_parent)
  % construct transformation
  % input: roll, pitch with uinit degree
  % mountZRP_child: input 1 x 3 [mount_z mount_roll mount_pitch] [m degree degree]
  % mountZRP_parent: input 1 x 3 [mount_z mount_roll mount_pitch] [m degree degree]

  child_preset_yaw = 92.0000; % degree
  % child_preset_yaw = 0; % degree
  child_preset_x = -2.0000; % meters
  child_preset_y = 1.3550; % meters

  parent_preset_yaw = 2.0000; % degree
  % parent_preset_yaw = 0; % degree
  parent_preset_x = 1.0000; % meters
  parent_preset_y = 0.0000; % meters

  T_mount2child = eye(4);
  R_mount2child = eul2rotm([child_preset_yaw/180*pi mountZRP_child(3)/180*pi mountZRP_child(2)/180*pi]);
  V_mount2child = [child_preset_x child_preset_y mountZRP_child(1)]';
  T_mount2child(1:3, 1:3) = R_mount2child;
  T_mount2child(1:3, 4) = V_mount2child;

  T_mount2parent = eye(4);
  R_mount2parent = eul2rotm([parent_preset_yaw/180*pi mountZRP_parent(3)/180*pi mountZRP_parent(2)/180*pi]);
  V_mount2parent = [parent_preset_x parent_preset_y mountZRP_parent(1)]';
  T_mount2parent(1:3, 1:3) = R_mount2parent;
  T_mount2parent(1:3, 4) = V_mount2parent;

  T_child2parent = inv(T_mount2parent) * T_mount2child;
  R_child2parent = T_child2parent(1:3, 1:3);
  V_child2parent = T_child2parent(1:3, 4);

  eul_child2parent = rotm2eul(R_child2parent);

  deltaZ_child2parent = V_child2parent(3);
  deltaR_child2parent = eul_child2parent(3) / pi * 180;
  deltaP_child2parent = eul_child2parent(2) / pi * 180;

  deltaZRP_child2parent = [deltaZ_child2parent deltaR_child2parent deltaP_child2parent];
end
function deltaZRP_child2parent = estimate_deltaZRP_from_groundPlane(ground_plane_child, ground_plane_parent)
  % input: 
  % ground_plane_child: input 1 x 4 [a b c d] normalized vector for the child
  % ground_plane_child: input 1 x 4 [a b c d] normalized vector for the parent
  % output:
  % deltaZRP_child2parent: 1 x 3 [z roll pitch] [meter degree degree]

  childNormals = [ground_plane_child(1) ground_plane_child(2) ground_plane_child(3)];
  parentNormals = [ground_plane_parent(1) ground_plane_parent(2) ground_plane_parent(3)];
  initExtrinsicParamsQ = eul2quat([89.9057*pi/180 2.7975*pi/180 -1.0031*pi/180]);
  extrinsicParamsQ = estimatePitchRollByCoNormalCeresMex(double(parentNormals), ...
                                                        double(childNormals), ...
                                                        double(initExtrinsicParamsQ), ...
                                                        false);

  extrinsicParamsEuler = quat2eul([
    extrinsicParamsQ(1) ...
    extrinsicParamsQ(2) ...
    extrinsicParamsQ(3) ...
    extrinsicParamsQ(4) ...
  ]); % [yaw pitch roll]

  deltaZRP_child2parent = zeros(1, 3);
  deltaZRP_child2parent(1) = 0;
  deltaZRP_child2parent(2) = extrinsicParamsEuler(3) * 180 / pi;
  deltaZRP_child2parent(3) = extrinsicParamsEuler(2) * 180 / pi;
end
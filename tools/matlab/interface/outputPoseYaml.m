function outputPoseYaml(pose, yamlFile)
    % save the pose to yaml file
    fid = fopen(yamlFile, 'w');
    str = sprintf('#relative pose, orientation in degree\n');
    fprintf(fid, str);
    str = sprintf('x: %.4f\n', pose(4));
    fprintf(fid, str);
    str = sprintf('y: %.4f\n', pose(5));
    fprintf(fid, str);
    str = sprintf('z: %.4f\n', pose(6));
    fprintf(fid, str);
    str = sprintf('roll: %.4f\n', pose(1) * 180 / pi);
    fprintf(fid, str);
    str = sprintf('pitch: %.4f\n', pose(2) * 180 / pi);
    fprintf(fid, str);
    str = sprintf('yaw: %.4f\n', pose(3) * 180 / pi);
    fprintf(fid, str);
    fclose(fid);
end
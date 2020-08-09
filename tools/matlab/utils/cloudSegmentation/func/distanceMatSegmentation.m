function distanceMatFull = distanceMatSegmentation(points, planeModel, pointType, horizontal_res)
    % segment the points according to the distance matrix

    if isequal(pointType, 'hesai_p40p')
        vertical_theta = [7.14299997649533 ...
                        6.14199999369268 ...
                        5.13599998302528 ...
                        4.12799999379798 ...
                        3.11499999759304 ...
                        2.10199999817724 ...
                        1.76700000074352 ...
                        1.42100000085773 ...
                        1.08699999931673 ...
                        0.751999999400975 ...
                        0.406000000141502 ...
                        0.0709999999638067 ...
                        -0.268000000278996 ...
                        -0.606000000378927 ...
                        -0.945000000723587 ...
                        -1.28399999783447 ...
                        -1.62100000004531 ...
                        -1.95999999838307 ...
                        -2.29799999910524 ...
                        -2.63600000254155 ...
                        -2.97400000194410 ...
                        -3.31100000267718 ...
                        -3.64799999962323 ...
                        -3.98599999721783 ...
                        -4.31900000176996 ...
                        -4.66099999339903 ...
                        -4.99499999970970 ...
                        -5.32600000224023 ...
                        -5.66700000201380 ...
                        -5.99999999161288 ...
                        -7.00099998042811 ...
                        -7.99700000286255 ...
                        -8.98699999592648 ...
                        -9.97100001148963 ...
                        -10.9479999800399 ...
                        -11.9169999834595 ...
                        -12.8739999942060 ...
                        -13.8250000366970 ...
                        -14.7660000283462 ...
                        -15.6960000066476];

        vertical_theta = vertical_theta/180*pi;
    elseif isequal(pointType, 'prescan_p40p')
        vertical_theta = [12.6749995304425 ...
                        12.0250000966667 ...
                        11.3749995623743 ...
                        10.7250002693310 ...
                        10.0750001292309 ...
                        9.42499975514755 ...
                        8.77500042464323 ...
                        8.12499945908174 ...
                        7.47499942750013 ...
                        6.82500032350010 ...
                        6.17500022246597 ...
                        5.52500003050543 ...
                        4.87500007595364 ...
                        4.22500001997206 ...
                        3.57499999086806 ...
                        2.92500015468267 ...
                        2.27499996486830 ...
                        1.62499999782561 ...
                        0.974999982567533 ...
                        0.325000020108150 ...
                        -0.324999994757377 ...
                        -0.975000007740665 ...
                        -1.62500002641223 ...
                        -2.27499999917517 ...
                        -2.92500009514571 ...
                        -3.57500001134124 ...
                        -4.22500004865302 ...
                        -4.87500014258578 ...
                        -5.52499996938786 ...
                        -6.17500032217496 ...
                        -6.82500036297083 ...
                        -7.47499942851741 ...
                        -8.12499963238950 ...
                        -8.77500046520763 ...
                        -9.42499966169597 ...
                        -10.0750001569357 ...
                        -10.7250001568748 ...
                        -11.3749995850157 ...
                        -12.0250000801206 ...
                        -12.6749994499319];

        vertical_theta = vertical_theta/180*pi;
    end

    % calc the distance between the points and the plane
    plane_func_params = planeModel.Parameters;
    plane_normal_norm = norm(plane_func_params(1:3));

    points_extend = [points(:, 1:3) ones(size(points, 1), 1)];
    distance = abs(points_extend * plane_func_params') / plane_normal_norm;

    % calc the vertical angle and horizontal angle
    v_angle = atan2(points(:, 3), sqrt(points(:, 1).*points(:, 1) + points(: ,2) .* points(:, 2)));
    if(isequal(pointType,'hesai_p40p'))
        h_angle = atan2(points(: ,1), -points(: ,2));
    elseif (isequal(pointType,'prescan_p40p'))
        h_angle = atan2(points(: ,1), points(: ,2));
    end

    h_angle_index = floor((h_angle + pi )/(2*pi)*(2*pi/horizontal_res));
    h_angle_index_min = min(h_angle_index);
    h_angle_index_max = max(h_angle_index);

    h_angle_index = h_angle_index - h_angle_index_min + 1;
    h_angle_num = h_angle_index_max - h_angle_index_min + 1;

    v_angle_index = floor(zeros(length(v_angle), 1));

    for i = 1 : length(v_angle)

        v_index = 1;
        v_angle_res_min = 1000000;
        for j = 1 : length(vertical_theta)
            v_angle_res = abs(v_angle(i) - vertical_theta(j));
            if(v_angle_res < v_angle_res_min)
                v_angle_res_min = v_angle_res;
                v_index = j;
            else
                break;
            end
        end

        v_angle_index(i) = v_index;
    end

    v_angle_num = length(vertical_theta);

    distanceMatFull = NaN(v_angle_num, h_angle_num, 6);

    for i = 1 : size(points, 1)
        cur_row_index = v_angle_index(i);
        cur_col_index = h_angle_index(i);
        cur_x = points(i, 1);
        cur_y = points(i, 2);
        cur_z = points(i, 3);
        cur_d = distance(i);
        cur_intensity = points(i, 4);
        distanceMatFull(cur_row_index, cur_col_index, 1) = cur_x;
        distanceMatFull(cur_row_index, cur_col_index, 2) = cur_y;
        distanceMatFull(cur_row_index, cur_col_index, 3) = cur_z;
        distanceMatFull(cur_row_index, cur_col_index, 4) = cur_d;
        distanceMatFull(cur_row_index, cur_col_index, 5) = i;
        distanceMatFull(cur_row_index, cur_col_index, 6) = cur_intensity;
    end

end
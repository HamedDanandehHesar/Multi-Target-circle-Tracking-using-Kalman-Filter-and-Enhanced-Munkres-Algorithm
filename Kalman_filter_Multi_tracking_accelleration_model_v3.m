% ===============================
% Initial settings
% ===============================
clear
close all
rng(200); % Set seed for generating random numbers
prompt = {'Enter starting frame number (default=1):'};
dlgtitle = 'Enter frame number';
dims = [1 35];
definput = {'1'};
answer = inputdlg(prompt, dlgtitle, dims, definput);

if isempty(answer)
    % User pressed Cancel or closed the dialog
    start_frame = 1;
else
    start_frame = str2double(answer{1});
    if isnan(start_frame)
        start_frame = 1; % fallback to default if input is not a number
    end
end

if isempty(start_frame)
    start_frame = 1;
end

euclidean_dist_thresh = 100; % Distance threshold for assignment
max_track_strikes = 10; % Maximum number of frames without detection before object removal
initial_estimate_covariance = [60, 20,20 60,20,20]; % Initial covariance
initial_x_dot = 0; % Initial velocity x
initial_y_dot = 0; % Initial velocity y
initial_x_dot_dot = 0; % Initial acceleration x
initial_y_dot_dot = 0; % Initial acceleration y
sensor_noise = 10; % Sensor noise
dt = 1/30; % Time interval


% Kalman filter Initialization Parameters
P0 = diag(initial_estimate_covariance);
dt2 = dt^2;
dt3 = dt^3 / 2;
dt4 = dt^4 / 4;
Q =  50*[dt4, dt3, dt2,0,0,0;
    dt3, dt2, dt, 0,0,0;
    dt2, dt, 1, 0,0,0;
    0, 0, 0, dt4, dt3, dt2;
    0,0,0,dt3, dt2, dt;
    0,0,0,dt2, dt, 1];

R = diag([sensor_noise sensor_noise]);
A =  [1, dt,0.5*dt2, 0,0,0;
    0, 1, dt, 0,0,0;
    0, 0, 1, 0,0,0;
    0, 0, 0, 1, dt,0.5*dt2;
    0,0,0,0, 1, dt;
    0,0,0,0,0,1];

H = [1, 0, 0, 0,0,0;
    0, 0, 0, 1,0,0,];
% ===========================
% Get settings from user
% ===========================
color = rand(3000,3)*255; % Random color for each object (you can also define fixed colors)

images_folder = uigetdir('', 'Please select images folder');
if images_folder == 0
    error('Operation canceled. Application stopped.');
end

output_folder = uigetdir('', 'Select output folder to save images');
if output_folder == 0
    error('Operation canceled. Application stopped.');
end

trajectory_length =10;% input('Enter the length of the displayed trajectory (number of frames): ');
if isempty(trajectory_length)
    trajectory_length = 50;
end




% % ===============================
% % Load detection data (assume: detections is a cell variable)
% % ===============================
% load(('detections.mat'), 'detections');
% % detections must be a cell where detections{frame} is an Nx2 matrix of points [x,y]

% =================================
% Prepare image list
% ===============================
image_files = dir(fullfile(images_folder, '*.jpg'));
% Sort by filename number Assuming filenames are numeric
file_nums = zeros(length(image_files),1);
for i=1:length(image_files)
    [~,name,~] = fileparts(image_files(i).name);
    file_nums(i) = str2double(name);
end
[~, idx] = sort(file_nums);
image_files = image_files(idx);

if isempty(image_files)
    error('No jpg images found in the imported folder.');
end

% Read first image to set dimensions
frame = imread(fullfile(images_folder, image_files(1).name));
frame_size = size(frame);

% ==============================
% Initialize tracking variables (important)
% ==============================
all_P_update = {}; %
all_P_predict = {}; %
all_x_update = {};
all_x_predict = {};
all_track_ids = [];
all_track_strikes = [];
all_track_classes = {};
id_count = 0;
all_last_positions={};
trajectories = containers.Map('KeyType','int32','ValueType','any'); % Trajectories

% ===============================
% Prepare CSV file
% ===============================
csv_path = fullfile(output_folder, 'Kalman_filter_accel_trajectories.csv');
fid_csv = fopen(csv_path, 'w');
fprintf(fid_csv, 'Frame,Track_ID,Class,X,Y\n');

% ================================
% Image Processing Loop
% ==================================
for frame_idx = start_frame:length(image_files)
    % Reading Image
    img_name = image_files(frame_idx).name;
    frame = imread(fullfile(images_folder, img_name));
    frame_gray = rgb2gray(frame);
    frame_binary = imbinarize(frame_gray);
    s = regionprops(frame_binary,'centroid');
    current_detections = cat(1,s.Centroid);

    % Display frame number on image
    frame = insertText(frame, [10 10], sprintf('Frame: %d', frame_idx), 'FontSize', 18, 'BoxColor', 'yellow', 'BoxOpacity', 0.6);

    % Draw detection boxes (circles on points)
    for d = 1:size(current_detections,1)
        frame = insertShape(frame, 'Circle', [current_detections(d,1), current_detections(d,2), 10], 'Color', 'red', 'LineWidth', 2);
    end

    % Prediction for all targets
    for i = 1:length(all_x_update)
        X0 = all_x_update{i};
        P00 = all_P_update{i};
        if all_track_strikes(i) ==0
            % Predict target motions
            X0 = A*X0;
            P00 = A*P00*A'+Q;
        else
            dt_1 = 1/5;
            A_1 =  [1, dt_1, dt_1*dt_1/2, 0,0,0;
                0, 1, dt_1, 0,0,0;
                0, 0, 1, 0,0,0;
                0, 0, 0, 1,dt_1, dt_1*dt_1/2;
                0,0,0, 0, 1, dt_1;
                0,0,0,0, 0, 1];
            X0 = A_1*X0;
            P00 = A_1*P00*A_1'+Q;
            all_x_update{i} = X0;
            all_P_update{i} = P00;

        end

        all_x_predict{i} = X0;
        all_P_predict{i} = P00;
    end

    % If there is a detection
    if ~isempty(current_detections)
        if isempty(all_x_predict)

            for d = 1:size(current_detections,1)
                det_center_x = current_detections(d,1);
                det_center_y = current_detections(d,2);
                X0 = [det_center_x;initial_x_dot;initial_x_dot_dot;det_center_y;initial_y_dot;initial_y_dot_dot];

                all_x_update{end+1} = X0;
                all_x_predict{end+1} = X0;
                all_P_predict{end+1} = P0;
                all_P_update{end+1} = P0;
                all_track_ids(end+1) = id_count;
                all_track_strikes(end+1) = 0;
                all_track_classes{end+1} = 'target'; 
                id_count = id_count + 1;
                all_last_positions{end+1} = current_detections(d,:);
            end
        else
            % --------- Improved Data Association and Track Management ---------

            all_track_mean_position = zeros(length(all_x_update),2);
            for i=1:length(all_x_update)
                X0 = all_x_update{i};
                mean_pos = X0([1,4])';

                mean_veloc = X0([2,5])';

                %                 all_track_mean_position(i,:) = mean_pos+mean_veloc*dt;
                %                 all_track_mean_position(i,:) = mean_pos;
                if all_track_strikes(i)==0
                    all_track_mean_position(i,:) = mean_pos+(mean_pos-all_last_positions{i})*dt;
                    %                             all_track_mean_position(i,:) = mean_pos+mean_veloc*dt;

                else
                    dt_1 = 1/30;

                    all_track_mean_position(i,:) =mean_pos+(mean_pos-all_last_positions{i})*dt_1;
                    %                             all_track_mean_position(i,:) = mean_pos+mean_veloc*dt_1;

                end

                all_last_positions{i} = mean_pos;


            end

            detection_center_positions = current_detections;

            % Hungarian/Munkres assignment
            if ~isempty(all_track_mean_position) && ~isempty(detection_center_positions)
                %     The cost matrix now includes a large penalty for assignments where the detection is not
                % in the predicted direction of the track's velocity.
                %
                %     This helps the Munkres algorithm maintain label consistency after targets cross or come close.
                direction_angle_cosine_thresh = 0.05;
                direction_penalty = euclidean_dist_thresh/4;

                cost_matrix = zeros(size(all_track_mean_position,1), size(detection_center_positions,1));
                for i = 1:size(all_track_mean_position,1)
                    mean_pos = all_track_mean_position(i,:)';
                    X0 = all_x_update{i};
                    mean_veloc = X0([2,5]);
                    for j = 1:size(detection_center_positions,1)
                        det_pos = detection_center_positions(j,:)';
                        eucl_dist = norm(mean_pos - det_pos);
                        to_det_vec = det_pos - mean_pos;
                        if norm(mean_veloc) > 1e-3 && norm(to_det_vec) > 1e-3
                            cos_theta = dot(mean_veloc, to_det_vec) / (norm(mean_veloc)*norm(to_det_vec));
                        else
                            cos_theta = 1;
                        end
                        if cos_theta < direction_angle_cosine_thresh
                            %                         cost_matrix(i,j) = eucl_dist + direction_penalty;
                            cost_matrix(i,j) = eucl_dist + direction_penalty*(1-cos_theta);

                        else
                            cost_matrix(i,j) = eucl_dist;
                        end
                    end
                end


                % manual implementation of Munkres algorithm----
                [row_ind, col_ind] = munkres(cost_matrix,euclidean_dist_thresh);
                assignments = [row_ind col_ind];
                assigned_tracks = assignments(:,1);
                unassigned_tracks = setdiff(1:size(all_track_mean_position,1), row_ind);
                unassigned_detections = setdiff(1:size(detection_center_positions,1), col_ind);
                %-------------
                % Update assigned tracks
                for i = assigned_tracks'
                    [r,~,~] = find(assignments(:,1)==i);
                    det_idx = assignments(r,2);
                    track_idx = i;
                    if det_idx > 0 && cost_matrix(track_idx,det_idx) <= euclidean_dist_thresh
                        all_track_strikes(track_idx) = 0;
                        X0 = all_x_predict{track_idx};
                        P00 = all_P_predict{track_idx};
                        z = detection_center_positions(det_idx,:)';


                        y = z - H*X0;        % Innovation (measurement residual)
                        S = H*P00*H' + R;  % Innovation covariance
                        K = P00 *H'*inv(S);  % Kalman gain
                        X0 = X0+K*y;
                        P00 = P00+(eye(6)-K*H)*P00;
                        all_x_update{track_idx} = X0;
                        all_P_update{track_idx} = P00;

                    else
                        all_track_strikes(track_idx) = all_track_strikes(track_idx) + 1;
                    end
                end

                % Try to re-associate unassigned tracks with close detections (track recovery)
                if ~isempty(unassigned_tracks)

                    for i = unassigned_tracks
                        % Only attempt re-association if strikes < max_track_strikes
                        if all_track_strikes(i) < max_track_strikes
                            % Find the nearest unassigned detection
                            dists = cost_matrix(i, unassigned_detections);
                            [min_dist, min_idx] = min(dists);
                            if ~isempty(min_dist) && min_dist <= euclidean_dist_thresh
                                det_idx = unassigned_detections(min_idx);
                                % Re-associate this track with the detection
                                all_track_strikes(i) = 0;
                                X0 = all_x_predict{track_idx};
                                P00 = all_P_predict{track_idx};
                                z = detection_center_positions(det_idx,:)';


                                y = z - H*X0;        % Innovation (measurement residual)
                                S = H*P00*H' + R;  % Innovation covariance
                                K = P *H'*inv(S);  % Kalman gain
                                X0 = X0+K*y;
                                P00 = P00+(eye(6)-K*H)*P00;
                                all_x_update{track_idx} = X0;
                                all_P_update{track_idx} = P00;
                                % Remove this detection from unassigned_detections
                                unassigned_detections(min_idx) = [];
                            else
                                % No close detection found, increase strike
                                all_track_strikes(i) = all_track_strikes(i) + 1;
                            end
                        else
                            all_track_strikes(i) = all_track_strikes(i) + 1;
                        end
                    end
                end

                % Remove tracks with too many strikes
                i = 1;
                while i <= length(all_x_update)
                    if all_track_strikes(i) >= max_track_strikes
                        fprintf('Track %d removed!\n', all_track_ids(i));
                        all_x_update(i) = [];
                        all_x_predict(i) = [];
                        all_P_update(i) = [];
                        all_P_predict(i) = [];
                        all_track_ids(i) = [];
                        all_track_strikes(i) = [];
                        all_track_classes(i) = [];
                    else
                        i = i + 1;
                    end
                end

                % Create new tracks for remaining unassigned detections
                if ~isempty(unassigned_detections)
                    for idx = unassigned_detections
                        det_center_x = detection_center_positions(idx,1);
                        det_center_y = detection_center_positions(idx,2);
                        X0 = [det_center_x;initial_x_dot;initial_y_dot_dot;det_center_y;initial_y_dot;initial_y_dot_dot];

                        all_x_update{end+1} = X0;
                        all_x_predict{end+1} = X0;
                        all_P_predict{end+1} = P0;
                        all_P_update{end+1} = P0;
                        all_track_ids(end+1) = id_count;
                        all_track_strikes(end+1) = 0;
                        all_track_classes{end+1} = 'target'; % کلاس فرضی

                        id_count = id_count + 1;
                        all_last_positions{end+1} = detection_center_positions(idx,:);

                    end
                end
            end
            % --------- End Improved Data Association and Track Management ---------
        end
    end

    % Update trajectories and save to CSV
    current_positions = [];
    for i = 1:length(all_x_update)
        X0 = all_x_update{i};

        track_id = all_track_ids(i);
        track_class = all_track_classes{i};
        mean_pos = X0([1,4]);
        x = round(mean_pos(1));
        y = round(mean_pos(2));

        if ~isKey(trajectories, track_id)
            trajectories(track_id) = zeros(trajectory_length, 2);
        end
        traj = trajectories(track_id);
        traj = [traj(2:end,:); [x, y]];
        trajectories(track_id) = traj;

        % Write to CSV
        fprintf(fid_csv, '%d,%d,%s,%d,%d\n', frame_idx, track_id, track_class, x, y);
    end

    % Draw particles and paths
    for i = 1:length(all_x_update)
        X0 = all_x_update{i};

        track_id = all_track_ids(i);
        track_class = all_track_classes{i};


        if isKey(trajectories, track_id)
            traj = trajectories(track_id);
            for k = 2:size(traj,1)
                if sum(traj(k-1,:))
                    frame = insertShape(frame, 'Line', [traj(k-1,:), traj(k,:)], 'Color', color(track_id+1,:), 'LineWidth', 2);
                end
            end

        end

        mean_pos = X0([1,4]);
        frame = insertShape(frame, 'FilledCircle', [mean_pos(1), mean_pos(2), 5], 'Color', 'white');
        frame = insertText(frame, [mean_pos(1), mean_pos(2)-20], sprintf('%s %d', track_class, track_id), 'FontSize', 14, 'BoxColor', color(track_id+1,:)/255, 'TextColor', 'green');
    end


% Save output image
imwrite(frame, [output_folder '\' num2str(frame_idx) '.jpg']);

% Show frame
figure(1)
imshow(frame);
title(sprintf('Frame %d', frame_idx));
drawnow;
pause(0.001)
% Press q to exit (requires active Figure window)
key = get(gcf,'CurrentCharacter');
if key == 'q'
break;
end
end

fclose(fid_csv);

% ============================
% Helper functions
% ===============================
% (add munkres function here or make sure they are in your MATLAB path)
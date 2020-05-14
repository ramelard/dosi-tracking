% % for data before paper_data/ was collected
% squareSize = 7.4;  % mm
% origin_to_sensorX = -30;  % mm
% origin_to_sensorY = 31;  % mm
% probe_depth = 67.3;  % mm

% % for paper_data/
% squareSize = 6.5;  % mm
% origin_to_sensorX = -34;  % mm
% origin_to_sensorY = 28;  % mm
% probe_depth = 67.3;  % mm

fps = 30; %Set frames per second of the video

%Specify video data directory
data_dir = 'D:\Work\RoblyerLab\trackDOSI\data\hand1';

%Set pattern parameters
if strfind(data_dir, 'linear stage') > 0
  squareSize = 7.4;  % mm
  origin_to_sensorX = -30;  % mm
  origin_to_sensorY = 31;  % mm
  probe_depth = 67.3;  % mm
else
  squareSize = 6.5;  % mm
  origin_to_sensorX = -34;  % mm
  origin_to_sensorY = 28;  % mm
  probe_depth = 67.3;  % mm
end

% Load calibration camera params
[file,path] = uigetfile('*.mat','Choose cameraParams.mat');
load(fullfile(path, file))
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Image to show the probe path on
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[file,path] = uigetfile('*','Choose baseline image for overlay');

%%
%Get list of video files in data directory
D = dir([data_dir, '/fc2*.avi']);
%Start with video 1
Didx = 1;
%Set a large number of frames (assumes a 2 minute acquisition)
nframes = fps*120;
%Read in the video file
vin = VideoReader(fullfile(D(Didx).folder, D(Didx).name));
%Get the frame
frame = readFrame(vin);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%I'm not sure what this does. Maybe sets theta 0?
%theta_axis isn't used anywhere else
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure, imshow(frame)
% text(10,20,'Select orientation axis (black to white)','color','r','fontweight','bold')
% axis_pts = getline;
% axis_pts(:,2) = size(frame,1) - axis_pts(:,2);
% close;
% theta_axis = pi - atan2(-diff(axis_pts(:,2)), -diff(axis_pts(:,1)));
% theta_axis = rad2deg(theta_axis);
theta_axis = 0;
vin.CurrentTime = 0;  % reset

%Sets time0 to the frame where the lasers saturate the camera
%fprintf(1,'Finding sync start time...')
%[vid_t0] = sync_t0(vin, theta_axis);
%fprintf('Done\n')


hwait = waitbar(0,'');
hfig = figure;
%M0 = [];
%Mopt = [];
%K = cameraParams.IntrinsicMatrix';

%Location of the segmented checkerboard
board_rect = [];
rotationVectors = [];
translationVectors = [];
prevCameraParams = [];
%2D projection of the detector path
sensor_route = zeros(2,nframes);
%3d location of detector path
sensor3D = zeros(3,nframes);
fprintf(1,'Reading and processing frames...')
tic;
sensor_mask = true(1, nframes);
rotationVectors0 = [];
translationVectors0 = [];
%Iterate through all the frames
for i = 1:nframes
  waitbar(i/nframes,hwait,sprintf('%u/%u',i,nframes));
  %Videos are split into multiple files if this is true move onto the next
  %file
  if vin.CurrentTime >= vin.Duration
    Didx = Didx + 1;
    if Didx > numel(D) %Are we finished?
      nframes = i-1; %Set the number of frames to the actual number
      %Truncate sensor route vectors
      sensor_route = sensor_route(:, 1:nframes);
      sensor3D = sensor3D(:, 1:nframes);
%       rotationVectors = rotationVectors(1:nframes, :);
%       translationVectors = translationVectors(1:nframes, :);
      sensor_mask = sensor_mask(1:nframes);
      %Quit the loop
      break;
    end
    %If we're not done, read in the next file
    vin = VideoReader(fullfile(D(Didx).folder, D(Didx).name));
  end
  %Convert frame to grayscale
  frame = rgb2gray(im2double(vin.readFrame()));
  try
     %Get probe location
    [newCameraParams, board_rect, checkerboard_pts] = ...
      track_probe(squareSize, frame, cameraParams, board_rect, prevCameraParams, theta_axis);
    
    if isempty(rotationVectors0)
      rotationMatrix0 = newCameraParams.RotationMatrices;
      rotationVectors0 = newCameraParams.RotationVectors;
      translationVectors0 = newCameraParams.TranslationVectors;
    end
    
%     % Transform coordinate system
%     rotationVectors(i,:) = newCameraParams.RotationVectors - rotationVectors0;
%     translationVectors(i,:) = newCameraParams.TranslationVectors - translationVectors0;
%     newCameraParams = cameraParameters('IntrinsicMatrix',newCameraParams.IntrinsicMatrix, ...
%                                        'RadialDistortion',newCameraParams.RadialDistortion, ...
%                                        'TangentialDistortion',newCameraParams.TangentialDistortion, ...
%                                        'RotationVectors',rotationVectors(i,:), ...
%                                        'TranslationVectors',translationVectors(i,:), ...
%                                        'WorldPoints',newCameraParams.WorldPoints, ...
%                                        'EstimateSkew',newCameraParams.EstimateSkew, ...
%                                        'NumRadialDistortionCoefficients',newCameraParams.NumRadialDistortionCoefficients, ...
%                                        'EstimateTangentialDistortion',newCameraParams.EstimateTangentialDistortion);
    %Get the new rotation and translation vectors
    rotationVectors(i,:) = newCameraParams.RotationVectors;
    translationVectors(i,:) = newCameraParams.TranslationVectors;
    prevCameraParams = newCameraParams;
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %I'm unsure what errors can arise
  %My best guess is that the checkerboard was no longer found
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  catch err
    disp(err)
    warning('Copying motion vectors for t=%g (i=%u)', vin.CurrentTime, i)
%     if i > 1
%       rotationVectors(i,:) = rotationVectors(i-1,:);
%       translationVectors(i,:) = translationVectors(i-1,:);
%     else
%       rotationVectors(i,:) = [0 0 0];
%       translationVectors(i,:) = [0 0 0];
%     end
    board_rect = [];  % lost track of the board
    prevCameraParams = [];
%       newCameraParams = cameraParameters('IntrinsicMatrix',cameraParams.IntrinsicMatrix, ...
%                                          'RadialDistortion',cameraParams.RadialDistortion, ...
%                                          'TangentialDistortion',cameraParams.TangentialDistortion, ...
%                                          'RotationVectors',rotationVectors(i,:), ...
%                                          'TranslationVectors',translationVectors(i,:), ...
%                                          'WorldPoints',cameraParams.WorldPoints, ...
%                                          'EstimateSkew',cameraParams.EstimateSkew, ...
%                                          'NumRadialDistortionCoefficients',cameraParams.NumRadialDistortionCoefficients, ...
%                                          'EstimateTangentialDistortion',cameraParams.EstimateTangentialDistortion);
    %For this frame only the sensor mask is set to false
    sensor_mask(i) = false;
    
    %Show the bad figure
    figure(hfig);
    imshow(undistortImage(frame, cameraParams)), hold on
    %   plot(sensor_route(1,i), sensor_route(2,i), 'or','markersize',10)
    plot(checkerboard_pts(1,:), checkerboard_pts(2,:), 'xr')
    plot(checkerboard_pts(1,1), checkerboard_pts(2,1), 'xg')
    text(10,10,sprintf('t=%.2fs',i/fps),'color','r')
    hold off
  
    continue;
  end
  
  %Calculate the sensor position for this frame
  
  sensor_worldcoord = [origin_to_sensorX origin_to_sensorY probe_depth];
%   sensor_worldcoord = [0 0 0];
%   if i == 1
%       sensor3D(:,i) = newCameraParams.RotationMatrices' * sensor_worldcoord' + newCameraParams.TranslationVectors(:);
%   else
      %Transformation to coordinates at time 0
      M = [rotationMatrix0' zeros(3,1); 0 0 0 1] + [zeros(3,3) translationVectors0(:); 0 0 0 0];
      %Transformation matrix based on new position
      P_A = newCameraParams.RotationMatrices' * sensor_worldcoord' + newCameraParams.TranslationVectors(:);
      %Calculate sensor position in world coordinates (i think)
      P_B = pinv(M) * [P_A; 1];
      sensor3D(:,i) = P_B(1:3);
%   end
  
  
%   rmatrix_transformed = rotationVectorToMatrix(newCameraParams.RotationVectors - rotationVectors0);
%   tvector_transformed = newCameraParams.TranslationVectors - translationVectors0;
%   sensor3D(:,i) = rmatrix_transformed' * [0 0 0]' + tvector_transformed(:);
%   sensor3D(:,i) = newCameraParams.RotationMatrices' * sensor_worldcoord' + newCameraParams.TranslationVectors(:);
  
  %Calculate probe position in sensor coordinates for display
  sensor_route(:,i) = worldToImage(newCameraParams, ...
    newCameraParams.RotationMatrices, newCameraParams.TranslationVectors, sensor_worldcoord);
  
  %Display the frame with the checkerboard points
  figure(hfig);
  imshow(undistortImage(frame, cameraParams)), hold on
  rectangle('position', [sensor_route(1,i)-25 sensor_route(2,i)-25 50 50], 'curvature', [1 1], 'facecolor','r')
%   plot(sensor_route(1,i), sensor_route(2,i), 'or','markersize',10)
  plot(checkerboard_pts(1,:), checkerboard_pts(2,:), 'or', 'markersize',10,'linewidth',2)
  plot(checkerboard_pts(1,1), checkerboard_pts(2,1), 'og', 'markersize',10,'linewidth',2)
  text(10,20,sprintf('t=%.2fs',i/fps),'color','r')
  
  % Overlay perimeter of probe (values were taken from 3D model).
  R = newCameraParams.RotationMatrices;
  T = newCameraParams.TranslationVectors;
  p1 = worldToImage(newCameraParams, R, T, [0 0 0]);
  p2 = worldToImage(newCameraParams, R, T, [-38 0 0]);
  p3 = worldToImage(newCameraParams, R, T, [-38 45 0]);
  p4 = worldToImage(newCameraParams, R, T, [-38 45 probe_depth]);
  plot([p1(1) p2(1) p3(1) p4(1)], [p1(2) p2(2) p3(2) p4(2)], '.-g', 'linewidth', 2, 'markersize', 20)
  hold off
  
end
close(hwait)
el = toc;
fprintf(1,'Done (%.2f)\n',el)
fprintf(1, 'FPS: %.2f \n', nframes/el)

%Show probe route (omit points where checkerboard was lost)
fullphantom = imread(fullfile(path, file));
fullphantom = undistortImage(fullphantom, cameraParams);
figure;
imshow(fullphantom)
hold on
p=plot(sensor_route(1,sensor_mask),sensor_route(2,sensor_mask),'linewidth',2);
npts = sum(sensor_mask);
%Arbitrary colors
colorgradient = [uint8(jet(npts)*255) uint8(ones(npts,1))].';
drawnow;
set(p.Edge, 'ColorBinding','interpolated', 'ColorData',colorgradient)

filename = sprintf('track_probe_variables-%s.mat', datestr(now, 'mmdd_HHMM'));
disp(sprintf('Saving tracking variables to %s', filename))
savepath = sprintf('%s/%s', data_dir, filename);
save(savepath)



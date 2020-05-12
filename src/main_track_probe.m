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

fps = 30;

data_dir = 'D:/Research Data/2018 DOSI tracking/hand1';

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

[file,path] = uigetfile('*','Choose baseline image for overlay');

%%
D = dir([data_dir, '/fc2*.avi']);
Didx = 1;
nframes = fps*120;

vin = VideoReader(fullfile(D(Didx).folder, D(Didx).name));

frame = readFrame(vin);
figure, imshow(frame)
text(10,20,'Select orientation axis (black to white)','color','r','fontweight','bold')
axis_pts = getline;
axis_pts(:,2) = size(frame,1) - axis_pts(:,2);
close;
theta_axis = pi - atan2(-diff(axis_pts(:,2)), -diff(axis_pts(:,1)));
theta_axis = rad2deg(theta_axis);
vin.CurrentTime = 0;  % reset

fprintf(1,'Finding sync start time...')
[vid_t0] = sync_t0(vin, theta_axis);
fprintf('Done\n')

hwait = waitbar(0,'');
hfig = figure;
M0 = [];
Mopt = [];
K = cameraParams.IntrinsicMatrix';
board_rect = [];
rotationVectors = [];
translationVectors = [];
prevCameraParams = [];
sensor_route = zeros(2,nframes);
sensor3D = zeros(3,nframes);
fprintf(1,'Reading and processing frames...')
tic;
sensor_mask = true(1, nframes);
rotationVectors0 = [];
translationVectors0 = [];
for i = 1:nframes
  waitbar(i/nframes,hwait,sprintf('%u/%u',i,nframes));
  
  if vin.CurrentTime >= vin.Duration
    Didx = Didx + 1;
    if Didx > numel(D)
      nframes = i-1;
      sensor_route = sensor_route(:, 1:nframes);
      sensor3D = sensor3D(:, 1:nframes);
%       rotationVectors = rotationVectors(1:nframes, :);
%       translationVectors = translationVectors(1:nframes, :);
      sensor_mask = sensor_mask(1:nframes);
      break;
    end
    vin = VideoReader(fullfile(D(Didx).folder, D(Didx).name));
  end
  
  frame = rgb2gray(im2double(vin.readFrame()));
  try
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
    
    rotationVectors(i,:) = newCameraParams.RotationVectors;
    translationVectors(i,:) = newCameraParams.TranslationVectors;
    prevCameraParams = newCameraParams;
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
    sensor_mask(i) = false;
    
    figure(hfig);
    imshow(undistortImage(frame, cameraParams)), hold on
    %   plot(sensor_route(1,i), sensor_route(2,i), 'or','markersize',10)
    plot(checkerboard_pts(1,:), checkerboard_pts(2,:), 'xr')
    plot(checkerboard_pts(1,1), checkerboard_pts(2,1), 'xg')
    text(10,10,sprintf('t=%.2fs',i/fps),'color','r')
    hold off
  
    continue;
  end
  
  
  sensor_worldcoord = [origin_to_sensorX origin_to_sensorY probe_depth];
%   sensor_worldcoord = [0 0 0];
%   if i == 1
%       sensor3D(:,i) = newCameraParams.RotationMatrices' * sensor_worldcoord' + newCameraParams.TranslationVectors(:);
%   else
      M = [rotationMatrix0' zeros(3,1); 0 0 0 1] + [zeros(3,3) translationVectors0(:); 0 0 0 0];
      P_A = newCameraParams.RotationMatrices' * sensor_worldcoord' + newCameraParams.TranslationVectors(:);
      P_B = pinv(M) * [P_A; 1];
      sensor3D(:,i) = P_B(1:3);
%   end
  
  
%   rmatrix_transformed = rotationVectorToMatrix(newCameraParams.RotationVectors - rotationVectors0);
%   tvector_transformed = newCameraParams.TranslationVectors - translationVectors0;
%   sensor3D(:,i) = rmatrix_transformed' * [0 0 0]' + tvector_transformed(:);
%   sensor3D(:,i) = newCameraParams.RotationMatrices' * sensor_worldcoord' + newCameraParams.TranslationVectors(:);
  sensor_route(:,i) = worldToImage(newCameraParams, ...
    newCameraParams.RotationMatrices, newCameraParams.TranslationVectors, sensor_worldcoord);
  
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
fprintf(1,'Done (%.2f)\n',toc)

fullphantom = imread(fullfile(path, file));
fullphantom = undistortImage(fullphantom, cameraParams);
figure;
imshow(fullphantom)
hold on
p=plot(sensor_route(1,sensor_mask),sensor_route(2,sensor_mask),'linewidth',2);
npts = sum(sensor_mask);
colorgradient = [uint8(jet(npts)*255) uint8(ones(npts,1))].';
drawnow;
set(p.Edge, 'ColorBinding','interpolated', 'ColorData',colorgradient)

filename = sprintf('track_probe_variables-%s.mat', datestr(now, 'mmdd_HHMM'));
disp(sprintf('Saving tracking variables to %s', filename))
savepath = sprintf('%s/%s', data_dir, filename);
save(savepath)



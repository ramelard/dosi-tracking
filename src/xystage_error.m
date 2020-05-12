% Calculate the 3D estimation error for a pre-programmed XY stage path.
% The estimated sensor coordinates are projected onto a plane of best fit,
% and errors are calculated as the smallest difference between the
% coordinate and XY path.
function [err,xest,xtrue] = xystage_error(sensor3D, camera_tilt)
  if ~exist('camera_tilt','var') || isempty(camera_tilt)
    camera_tilt = questdlg('Which data is this?','Data Confirmation','Flat','Angled','Flat');
  end

  assert(any(size(sensor3D) == 3));
  if size(sensor3D, 1) == 3
    sensor3D = sensor3D';
  end
  
  % Generate xy stage points (8cm x 1cm, 8 scans)
  path_points = [0  0;  80 0; ...
                 80 10; 0  10; ...
                 0  20; 80 20; ...
                 80 30; 0  30; ...
                 0  40; 80 40; ...
                 80 50; 0  50; ...
                 0  60; 80 60; ...
                 80 70; 0  70; ...
                 0  80; 80 80;];
  path_mm = [];
  for i = 1:size(path_points, 1)-1
    x = linspace(path_points(i,1), path_points(i+1,1), 801);
    y = linspace(path_points(i,2), path_points(i+1,2), 801);
    path_mm = [path_mm; x(:) y(:)];
  end
  
  % Manually determined to flatten the plane, so we can compare to
  % path_points.
  if strcmpi(camera_tilt, 'flat')
    theta = [-0.0068 0 0.0184];  % xystage
  elseif strcmpi(camera_tilt, 'angled')
    theta = [-0.0111 -0.397 0];  % xystage-angled
  else
    error('Unsupported camera tilt type %s. Expected flat or angled.', camera_tilt)
  end
  sensor3D = rotate_points(sensor3D, theta);
  sensor3D(:,1:2) = -sensor3D(:,1:2);
  
%   % Fit a plane to the data: ax + by + cz = 0
%   % nvec is [a b c]'
%   nvec = [sensor3D(:,1:2) ones(size(sensor3D,1),1)] \ sensor3D(:,3);
%   nvec = nvec./norm(nvec);
%   
%   % Project data onto plane. Take a point well into the data collection
%   % (beginning could have been erroneous).
%   % Plane is defined by point in plane P(a,b,c) and normal vector nvec
%   a = sensor3D(100,1);
%   b = sensor3D(100,2);
%   c = sensor3D(100,3);
%   d = nvec(1);
%   e = nvec(2);
%   f = nvec(3);
%   
% %   % Dot product method should be the same as:
% %   t = (a*d-a*sensor3D(:,1)+b*e-b*sensor3D(:,2)+c*f-c*sensor3D(:,3))./(a^2+b^2+c^2);
% %   pt_proj = [sensor3D(:,1)+t*a, sensor3D(:,2)+t*b, sensor3D(:,3)+t*c];
% 
%   pt_proj = zeros(size(sensor3D));
%   npts = size(sensor3D, 1);
%   for i = 1:npts
%     q = sensor3D(i,:);
%     pt_proj(i,:) = q - dot(q-[a b c], nvec) * nvec';
%   end
%   % Shift so it starts at (0,0,0).
% %   pt_proj2 = bsxfun(@minus, pt_proj, [pt_proj(1,1:2) 0]);
%   pt_proj2 = bsxfun(@minus, pt_proj, pt_proj(1,:));
%   pt_proj2(:,1:2) = -pt_proj2(:,1:2);
  
  pt_proj2 = sensor3D - sensor3D(1,:);  % start at origin
  
  D = pdist2(pt_proj2, [path_mm zeros(size(path_mm, 1), 1)]);
  [err,minidx] = min(D, [], 2);
  
  xest = pt_proj2;
  xtrue = [path_mm(minidx,:) zeros(numel(minidx),1)];
end


% Theta found by making first trajectory coordinates constant along x axis
% xystage theta = [0 0 0.0184]
% xystage-angled theta = [-0.0111 -0.397 0]
function sensor3D_rotated = rotate_points(sensor3D, theta_rad)
  tx = -mean(sensor3D(:,1));
  ty = -mean(sensor3D(:,2));
  tz = -mean(sensor3D(:,3));
  
  T = [1 0 0 tx; 0 1 0 ty; 0 0 1 tz; 0 0 0 1];
  theta = rad2deg(theta_rad);
  R = rotx(theta(1)) * roty(theta(2)) * rotz(theta(3));
  R = [R ones(size(R,1), 1); 0 0 0 1];
      
  x = [sensor3D ones(size(sensor3D,1), 1)]';
  xhat = pinv(T)*R*T*x;
  
  sensor3D_rotated = xhat(1:3,:)';
  
  figure, plot3(x(1,:), x(2,:), x(3,:), 'r')
  grid on, hold on
  plot3(xhat(1,:), xhat(2,:), xhat(3,:), 'b')
end
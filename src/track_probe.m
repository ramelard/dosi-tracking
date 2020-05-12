function [newCameraParams, board_rect_out, checkerboard_pts] = ...
    track_probe(squareSize, frame, calibrationParams, prev_board_rect, prevCameraParams, theta_axis)
  
  if nargin < 4
    prev_board_rect = [];
  end
  if nargin < 5
    prevCameraParams = [];
  end
  if nargin < 6
    theta_axis = 0;
  end
  
  frame = undistortImage(frame, calibrationParams);
%   frame = imrotate(frame, theta_axis, 'bicubic', 'crop');
  
  if isempty(prev_board_rect)
    % If no previous board ROI given, look for checkerboard in whole image.
    [Pcam, boardSize, imagesUsed] = detectCheckerboardPoints(frame);
  else
    % Enlarge previous ROI
    rect = prev_board_rect;
    w = rect(3);
    h = rect(4);
    rect = round([rect(1)-0.5*w rect(2)-0.5*h w*2 h*2]);
    rect(1) = max(rect(1), 1);
    rect(2) = max(rect(2), 1);
    if rect(1)+rect(3) > size(frame,2)
      rect(3) = size(frame,2) - rect(1);
    end
    if rect(2)+rect(4) > size(frame,1)
      rect(4) = size(frame,1) - rect(2);
    end
    
    frame_crop = imcrop(frame,rect);
    [Pcam, boardSize, imagesUsed] = detectCheckerboardPoints(frame_crop);
    if imagesUsed == false
      error('Could not detect checkerboard');
    end
    
    % Add points back to Pcam based on crop offset.
    Pcam = bsxfun(@plus, Pcam, rect(1:2)-1);
  end
  
  if prod(boardSize) ~= 7*8
    % With newer Matlab versions, use MinCornerMetric to try to get the 
    % proper size.
    error('Detected wrong sized checkerboard.')
  end
  
  % Update detected board ROI for starting position next time
  minX = min(Pcam(:,1)); 
  minY = min(Pcam(:,2)); 
  maxX = max(Pcam(:,1)); 
  maxY = max(Pcam(:,2));
  board_rect_out = [minX minY maxX-minX maxY-minY];
  
%   Pcam = unrotate_checkerboard_points(Pcam, theta_axis, frame);
  
  Pworld = generateCheckerboardPoints(boardSize, squareSize);
  % Explicitly set z=0, since it is planar.
  Pworld = [Pworld zeros(size(Pworld,1),1)];
  
  % Make rows coordinates and columns instances, and homog coords
  Pworld = [Pworld ones(size(Pworld,1),1)]';
  Pcam = [Pcam ones(size(Pcam,1),1)]';

  % Least squares optimization seems to fail when the checkerboard is
  % approx parallel to the camera (small rotation), so compute extrinsics
  % explicitly ("or true" for now)
  %if isempty(prevCameraParams) || ...
  %    all(abs(rad2deg(prevCameraParams.RotationVectors(1:2))) < 5)
    K = calibrationParams.IntrinsicMatrix';
    [H, validIdx] = computeHomographies(Pcam', Pworld');
    if ~validIdx
      warning('Did not find valid homography')
      return
    end
    % Equivalent to calling extrinsics()
    [rvecs, tvecs] = computeExtrinsics(K, H);
    % I don't think we need to do non-lin least squares because
    % conventionally that's done to find the lens distortion, but that has
    % already been found.
  
    % newCameraParams
    prevCameraParams = cameraParameters('IntrinsicMatrix', calibrationParams.IntrinsicMatrix, ...
      'RotationVectors', rvecs, ...
      'TranslationVectors', tvecs, 'WorldPoints', Pworld(1:2,:)', ...
      'WorldUnits', 'mm', 'EstimateSkew', calibrationParams.EstimateSkew,...
      'NumRadialDistortionCoefficients', calibrationParams.NumRadialDistortionCoefficients,...
      'EstimateTangentialDistortion', calibrationParams.EstimateTangentialDistortion,...
      'RadialDistortion', calibrationParams.RadialDistortion);
  %end
%   else
    K = prevCameraParams.IntrinsicMatrix';
    R = prevCameraParams.RotationMatrices';  % transpose?
    T = prevCameraParams.TranslationVectors(:);
    M = @(x) K * eye(3,4) * [x(1) x(2) x(3) x(10); ...
                           x(4) x(5) x(6) x(11); ...
                           x(7) x(8) x(9) x(12); ...
                           0    0     0     1];
    M0 = K * eye(3,4) * [R T; 0 0 0 1];
    
    clear x;
    % For some reason, cameraParameter's Rmatrix is the transpose of
    % vectorToMatrix.
%     Rx = @(x) vision.internal.calibration.rodriguesVectorToMatrix(x)';
    Rx = @(x) rotationVectorToMatrix(x);
    % Same as:
%     Rx = @(x) rotx(rad2deg(x(1)))*roty(rad2deg(x(2)))*rotz(rad2deg(x(3)))';
    Tx = @(x) x(:);
    
    z = Pcam(1:2,:)';
    zhat = @(x) worldToImage(calibrationParams, Rx(x(1:3)), Tx(x(4:6)), Pworld(1:3,:)');
    fun = @(x) (z-zhat(x)).^2;
%     fun = @(x) (Pcam(1:2,:)' - worldToImage(prevCameraParams, Rx(x(1:3)), Tx(x(4:6)), Pworld([1,2,4],:)'));
    options.Algorithm = 'levenberg-marquardt';
    options.Display = 'none';
    x0 = [prevCameraParams.RotationVectors(:)' prevCameraParams.TranslationVectors(:)'];
    xopt = lsqnonlin(fun,x0,[],[],options);
    
    rvec = xopt(1:3);
    tvec = xopt(4:6);
    
    newCameraParams = cameraParameters('IntrinsicMatrix', calibrationParams.IntrinsicMatrix, ...
      'RotationVectors', rvec, ...
      'TranslationVectors', tvec, 'WorldPoints', Pworld(1:2,:)', ...
      'WorldUnits', 'mm', 'EstimateSkew', calibrationParams.EstimateSkew,...
      'NumRadialDistortionCoefficients', calibrationParams.NumRadialDistortionCoefficients,...
      'EstimateTangentialDistortion', calibrationParams.EstimateTangentialDistortion,...
      'RadialDistortion', calibrationParams.RadialDistortion);
%   end
   
  
  if nargout > 2
    checkerboard_pts = Pcam;
  end
   
%    % Apply Levenberg-Marquardt
%    % from : estimateCameraParameters():
%    %   errors = refine(cameraParams, imagePoints, calibrationParams.shouldComputeErrors);
%    
% %    E = @(M) sum( sum((Pcam-M*Pworld).^2) );
%     fun = @(M) (Pcam-M*Pworld);
%    
%     options.Algorithm = 'levenberg-marquardt';
%     options.Display = 'iter';
%     options.MaxFunEvals = 10000;
%     options.MaxIter = 10000;
%     Mopt = lsqnonlin(fun,M0,[],[],options);
%     
%     if nargout > 1
%       M0_out = M0;
%     end
end



% Compute projective transformation from worldPoints to imagePoints
function H = computeHomography(imagePoints, worldPoints)
  H = fitgeotrans(worldPoints, imagePoints, 'projective');
  H = (H.T)';
  H = H / H(3,3);
end


% Compute homographies for all images
function [homographies, validIdx] = computeHomographies(points, worldPoints)
  w1 = warning('Error', 'MATLAB:nearlySingularMatrix'); %#ok
  w2 = warning('Error', 'images:maketform:conditionNumberofAIsHigh'); %#ok

  numImages = size(points, 3);
  validIdx = true(numImages, 1);
  homographies = zeros(3, 3, numImages);
  for i = 1:numImages
      try    
          homographies(:, :, i) = ...
              computeHomography(double(points(:, 1:2, i)), worldPoints(:,1:2));
      catch 
          validIdx(i) = false;
      end
  end
  warning(w1);
  warning(w2);
  homographies = homographies(:, :, validIdx);
  if ~all(validIdx)
      warning(message('vision:calibrate:invalidHomographies', ...
          numImages - size(homographies, 3), numImages));
  end
end


% Compute translation and rotation vectors for all images
function [rotationVectors, translationVectors] = computeExtrinsics(A, homographies)
  numImages = size(homographies, 3);
  rotationVectors = zeros(3, numImages);
  translationVectors = zeros(3, numImages); 
  Ainv = inv(A);
  for i = 1:numImages
      H = homographies(:, :, i);
      h1 = H(:, 1);
      h2 = H(:, 2);
      h3 = H(:, 3);
      lambda = 1 / norm(Ainv * h1); %#ok

      % 3D rotation matrix
      r1 = lambda * Ainv * h1; %#ok
      r2 = lambda * Ainv * h2; %#ok
      r3 = cross(r1, r2);
      R = [r1,r2,r3];

      rotationVectors(:, i) = vision.internal.calibration.rodriguesMatrixToVector(R);

      % translation vector
      t = lambda * Ainv * h3;  %#ok
      translationVectors(:, i) = t;
  end

  rotationVectors = rotationVectors';
  translationVectors = translationVectors';
end


function Pcam2 = unrotate_checkerboard_points(Pcam, theta, frame)
  assert(size(Pcam,2) == 2);
  
  tx = size(frame,1)/2 + 1;  % positive to right
  ty = size(frame,2)/2 + 1;  % positive down
  T = [1 0 tx; 0 1 ty; 0 0 1];
  T2 = [1 0 -tx; 0 1 -ty; 0 0 1];
  R = rotz(theta);
  Pcam2 = T*R*T2*[Pcam ones(size(Pcam,1), 1)]';
  Pcam2 = Pcam2(1:2,:)';
end
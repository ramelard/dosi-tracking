% % Adapted from showExtrinsics()
% intmat = readNPY('../data/cameraParams.npz/cameraMatrix.npy');
% distCoefs = readNPY('../data/cameraParams.npz/distortionCoefs.npy');
% cameraParams = cameraParameters('IntrinsicMatrix', intmat',...
% 'RadialDistortion',distCoefs(1:2),...
% 'TangentialDistortion',distCoefs(3:4),...
% 'EstimateSkew',logical(0),...
% 'NumRadialDistortionCoefficients', 2,...
% 'EstimateTangentialDistortion',logical(0));
% 
% left = min(cameraParams.WorldPoints(:,1));
% top = min(cameraParams.WorldPoints(:,2));
% right = max(cameraParams.WorldPoints(:,1));
% bottom = max(cameraParams.WorldPoints(:,2));

accFile = fullfile(dataDir,'accuracyHoriz_Small05_OPs_frommatlab.txt');
accFile = fullfile(dataDir,'accuracyHoriz_Small05_OPs.txt');
A=csvread(accFile,1,0);
%Extrinsics
rotVecs = A(:,[7:9]);
transVecs = A(:,[10:12]);


left = 0;
top = 0;
right = 4.75*4;
bottom = 4.75*2;
% right = 14.5*3;
% bottom = 14.5*3;

% World coordinates of checker board
boardW = [left top 0; left bottom 0; right bottom 0; right top 0;]';

figure; hold on;
for i = 1:size(rotVecs,1)
    rotVeci = rotVecs(i,:);
    R = rotationVectorToMatrix(rotVeci);
    T = transVecs(i,:)';

    if i == 1
        color = 'r';
    else
        color = 'b';
    end
    alpha = 0.5;

    % Camera coordinates of checkerboard
    tformi = rigid3d(R, T');
    boardC = tformi.transformPointsForward(boardW')';

    cX = boardC(1,:);
    cZ = boardC(3,:);
    cY = boardC(2,:);
    
    if i > 2
%         delete(h1);
%         delete(h2);
    end
    
    
    h1 = patch(cX,cY,cZ,'r');
    h2 = plot3(cX(1),cY(1),cZ(1),'or');
    text(cX(1),cY(1),cZ(1),num2str(i))
    
    if i == 1
        % Plot axes on first checkerboard
        X = [0 0 0; diag([200, 200, 10])];
        
        boardC = tformi.transformPointsForward(boardW')';
        Xc = tformi.transformPointsForward(X);
        % Plot each axis line segment
        for j = 2:4
            line(Xc([1,j],1), Xc([1,j],2), Xc([1,j],3), 'color', 'k', 'linewidth', 2)
        end
    end
    
%     xlim([-80 120])
%     ylim(yl)
%     zlim(zl)
%     view([a,b])
    grid on

    
    set(h1,'FaceColor',color,'FaceAlpha',alpha, ...
            'EdgeColor','k','LineWidth',1.0);
          
%           pause(0.5)
end     
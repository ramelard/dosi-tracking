dataDir = '..\data\22Oct2020\Accuracy';
accFile = fullfile(dataDir,'accuracyHoriz_Small05_OPs.txt');
% accFile = fullfile(dataDir,'accuracyHoriz_Small05_OPs_frommatlab.txt');
% accFile = fullfile(dataDir,'accuracyHoriz_Large02_OPs.txt');
A=csvread(accFile,1,0);
% Camera coordinates
x = A(:,2);
y = A(:,3);
z = A(:,4);
% Image coordinates
xPx = round(A(:,5));
yPx = round(A(:,6));
% Extrinsics
rotVecs = A(:,[7:9]);
transVecs = A(:,[10:12]);
% Optical properties
mua = A(:,13);
mus = A(:,14);
% Intrinsics. NOTE: OpenCV expresses it differently than Matlab. They are
% transposed relative to each other, and may define the centre point
% differently (1,1 vs 0,0).
camMat=readNPY('../data/cameraParams.npz/cameraMatrix.npy');

% Get the first extrinsic matrix to transform others back to.
worldOrgIdx = 1;
rotVec1 = rotVecs(worldOrgIdx,:);
rotMat1 = rotationVectorToMatrix([rotVec1(1),rotVec1(2),rotVec1(3)]);
transVec1 = transVecs(worldOrgIdx,:);
tform0 = rigid3d(rotMat1, transVec1);
% How far was measurement point from origin
measToObjOrigin = [7.5,4,16];

% World coordinates
u = zeros(size(x)); v = zeros(size(x)); w=zeros(size(x));
for i = 1:length(x)
    if ~isnan(x(i))
        xw = [x(i), y(i), z(i)]';
        
        rotVeci = rotVecs(i,:);
        rotMati = rotationVectorToMatrix(rotVeci);
        transVeci = transVecs(i,:);

        % To get from our current world coordinates to the original
        % checkerboard's world coordinates, we need to:
        %   1) World_i -> Camera (extrinsics forward transform)
        %   2) Camera -> World_0 (extrinsics inverse transform)
        % Matlab expresses transformation matrices in the transpose of what
        % is typical, so use built-in functions to do the work for us.
        tformi = rigid3d(rotMati, transVeci);
        origin_camcoords = tformi.transformPointsForward([0 0 0]);  % use origin for this
        wc = tform0.transformPointsInverse(origin_camcoords);
    end
    u(i) = wc(1); v(i) = wc(2); w(i) = wc(3);
end

%True coordinates measured with the translation stage (mm)
trueX = zeros(1,length(u));
trueY = zeros(1,length(u));
trueZ = zeros(1,length(u));
i = 1;
while (i-1)*31 + 31 < length(u)
    trueX((i-1)*31+1:(i-1)*31 + 31) = 0:5:150;
    trueY((i-1)*31+1:(i-1)*31 + 31) = ones(1,31) * 25.4*(i-1);
    i = i + 1;
end

figure
plot3(trueX,trueY,trueZ,'o')
hold on
plot3(u,v,w,'*')
%plot3(x,y,z,'s')
xlabel('X (mm)')
ylabel('Y (mm)')
zlabel('Z (mm)')
legend('True position','Recovered position')
title('3D accuracy (World coords)')
set(gca,'fontsize',18)
grid on;

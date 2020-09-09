dataDir = '..\data\01Sep2020\';
accFile = fullfile(dataDir,'accuracyHoriz02_OPs.txt');
A=csvread(accFile,1,0);
%camera coordinates
x = A(:,2);
y = A(:,3);
z = A(:,4);
%image coordinates
xPx = round(A(:,5));
yPx = round(A(:,6));
%Extrinsics
rotVecs = A(:,[7:9]);
transVecs = A(:,[10:12]);
%Intrinsics
camMat=readNPY('../data/cameraParams.npz/cameraMatrix.npy');
%Get the first extrinsic matrix
worldOrgIdx = 1;
rotVec1 = rotVecs(worldOrgIdx,:);
rotMat1 = rotationVectorToMatrix([rotVec1(1),rotVec1(2),rotVec1(3)]);
transVec1 = transVecs(worldOrgIdx,:);
%How far was measurement point from origin
measToObjOrigin = [7.5,4,16];
%Transformation to coordinates at time 0
M = [rotMat1, zeros(3,1); 0 0 0 1] + [zeros(3,3),transVec1(:); 0 0 0 0];

%World coordinates
%Coords using \ method
u3 = zeros(size(x)); v3 = zeros(size(x)); w3=zeros(size(x));
%Coords using pinv method
u4 = zeros(size(x)); v4 = zeros(size(x)); w4=zeros(size(x));
for i = 1:length(x)
    if ~isnan(x(i))
        %wc = inv(M)* [x3(i),y3(i),z3(i),1]';
        thisRotMat = rotationVectorToMatrix(rotVecs(i,:));
        P_A = thisRotMat * measToObjOrigin' + transVecs(i,:)';
        P_B = pinv(M) * [P_A;1];
        wc3 = P_B;
        c = [x(i),y(i),z(i)]' - [x(worldOrgIdx),y(worldOrgIdx),z(worldOrgIdx)]';
        wc2 = rotMat1\c;
    else
        wc2 = [NaN, NaN,NaN];
    end
    u3(i) = wc2(1); v3(i) = wc2(2); w3(i) = wc2(3);
    u4(i) = wc3(1); v4(i) = wc3(2); w4(i) = wc3(3);
end
%Optical properties, mostly random for this, the lasers weren't on
mua = A(:,13);
mus = A(:,14);

%Number of horizontal rows collected
numRows = 8;
%True coordinates measured with the translation stage (mm)
trueX = zeros(1,length(u3));
trueY = zeros(1,length(u3));
trueZ = zeros(1,length(u3));
for i = 1:numRows
    trueX((i-1)*31+1:(i-1)*31 + 31) = 0:-5:-150;
    trueY((i-1)*31+1:(i-1)*31 + 31) = ones(1,31) * -25.4*(i-1);
end
trueDistFromStart = sqrt((trueX-trueX(1)).^2 + (trueY-trueY(1)).^2 + (trueZ-trueZ(1)).^2);
measDistFromStart = sqrt((u3-u3(1)).^2 + (v3-v3(1)).^2 + (w3-w3(1)).^2);
%Plotting
figure
plot3(u3,v3,w3,'o')
hold on
plot3(u4,v4,w4,'*')

figure
plot(trueDistFromStart,'o')
hold on
plot(measDistFromStart,'*')

figure
plot3(trueX,trueY,trueZ,'o')
hold on
plot3(u4,v4,w4,'*')
%plot3(x,y,z,'s')
xlabel('X (mm)')
ylabel('Y (mm)')
zlabel('Z (mm)')
legend('True position','Recovered position')
title('3D accuracy (World coords)')
set(gca,'fontsize',18)

figure
plot(trueX,trueY,'o')
hold on
plot(u3,v3,'*')
xlabel('X (mm)')
ylabel('Y (mm)')
title('2D accuracy (World coords.)')
legend('True position','Recovered position')
set(gca,'fontsize',18)


figure
plot(trueX,trueY,'o')
hold on
plot(x-x(1),y-y(1),'s')
xlabel('X (mm)')
ylabel('Y (mm)')
title('2D accuracy (Camera coords.)')
legend('True position','Recovered position')
set(gca,'fontsize',18)

%%
%The above obviously didn't work. Let's try a new tack where I try to go
%from image coordinates (2D) to world coordinates

%Let's just see if I can recover xPx and yPx from x, y, z (ie image coords
%from cam coords)

newXPx = zeros(size(xPx));
newYPx = zeros(size(yPx));
newZPx = zeros(size(yPx));

for p = 1:length(x)
    newPt = camMat * [x(p);y(p);z(p)];
    
    newXPx(p) = newPt(1); newYPx(p)= newPt(2); newZPx(p) = newPt(3);
end

figure
plot(xPx,yPx,'o')
hold on
plot(newXPx,newYPx,'*')

%Great, so that worked. No let's try going the other direction. From xPx to
%x,y, and z (ie image coords to camera coords)

newX = zeros(size(x));
newY = zeros(size(y));
newZ = zeros(size(z));

for p = 1:length(x)
    newPt = camMat \ [newXPx(p);newYPx(p);newZPx(p)];
    newX(p)=newPt(1) ;newY(p)=newPt(2);newZ(p)=newPt(3);
end

figure
plot3(x,y,z,'o')
hold on
plot3(newX,newY,newZ,'*')

%So that worked too, but we need to know all three parts of the projected
%coordinates, no big deal. At this point I haven't used any extrinsic
%matrices.

M2 = [rotMat1,transVec1'];
xw = zeros(size(x));
yw = zeros(size(y));
zw = zeros(size(z));
for p = 1:length(newX)
    newPt = rotMat1 \ [newX(p),newY(p),newZ(p)]';
    xw(p) = newPt(1); yw(p) = newPt(2); zw(p) = newPt(3);
end

figure
plot3(xw,yw,zw,'o')
hold on
plot3(trueX,trueY,trueZ,'*')

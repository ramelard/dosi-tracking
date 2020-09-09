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
%Calculate distance
trueDistFromStart = sqrt((trueX-trueX(1)).^2 + (trueY-trueY(1)).^2 + (trueZ-trueZ(1)).^2);
measDistFromStart = sqrt((u3-u3(1)).^2 + (v3-v3(1)).^2 + (w3-w3(1)).^2);
%Fit plane to world data
DM = [u3, v3, ones(size(w3))];
B = DM\w3;
%First I need to rotate the plane to the z=0 axis
%Normal to best fit plane
normal0 = [-B(1),-B(2),1];
normal = normal0/norm(normal0);
%Normal to z=0 plane
normalZ = [0,0,1];
%Axis-angle representation
rot_angle = acos(dot(normalZ,normal)); %Rotation angle to rotate plane onto z axis
rot_axis = cross(normalZ,normal); %Axis to rotate around
rot_axis = rot_axis/norm(rot_axis); %Make it a unit vector
axAng = rot_axis*rot_angle; %Set up the rotation vector
rot_mat = rotationVectorToMatrix(axAng); %Calculate rotation matrix
%Rotate the points
rotPts = zeros(numel(u3),3);
for i = 1:numel(u3)
    rotPts(i,:) = rot_mat * [u3(i);v3(i);w3(i)];
end
%Also set up a rectangle for visualization
rectX = [min(u3(:)),min(u3(:)),max(u3(:)),max(u3(:))];
rectY = [min(v3(:)),max(v3(:)),max(v3(:)),min(v3(:))];
rectZ = B(1) * rectX + B(2)*rectY + B(3);
rotRect = zeros(length(rectX),3);
for p = 1:length(rectX)
    rotRect(p,:) = rot_mat * [rectX(p);rectY(p);rectZ(p)];
end
%Translate the points in the minus Z direction so the best fit plane is at
%z=0
rotPts(:,3) = rotPts(:,3) - rotRect(1,3);
%Rotate the points about the z axis so the measurements are aligned with
%the x axis
pts2D = rotPts(:,1:2);
lastLine = rotPts(218:248,:); %Use the last line b/c it's the straightest
lm=polyfit(lastLine(:,1),lastLine(:,2),1);
ang2D = atan(lm(1)); %angle to rotate around
rotMat2D = rotationVectorToMatrix([0,0,ang2D]); %Rotation matrix
%Calculate final points
finPts = zeros(numel(u3),3);
for i = 1:numel(u3)
    finPts(i,:) = rotMat2D * [rotPts(i,1),rotPts(i,2),rotPts(i,3)]';
end
%%
%Plot the final points
figure
hold on
plot3(trueX,trueY,trueZ,'o')
plot3(finPts(:,1),finPts(:,2),finPts(:,3),'*')
xlabel('X (mm)')
ylabel('Y (mm)')
zlabel('Z (mm)')
title('Recovered points via rotation')
legend('True','Estimated')
% DM = [rotPts(:,1), rotPts(:,2), ones(size(w3))];
% B = DM\w3;
figure
plot3(u3,v3,w3,'o')
hold on
plot3(rotPts(:,1),rotPts(:,2),rotPts(:,3),'*')
axis equal
%Plotting
%Compare the \ and pinv method
figure
plot3(u3,v3,w3,'o')
hold on
plot3(u4,v4,w4,'*')

%Check out best fit plane
figure
plot3(u3,v3,w3,'o')
hold on
h=fill3(rectX,rectY,rectZ,'r');
set(h,'facealpha',0.5)
plot3(rotPts(:,1),rotPts(:,2),rotPts(:,3),'*')
k=fill3(rotRect(:,1),rotRect(:,2),rotRect(:,3),'b');
set(k,'facealpha',0.5);
plot3([0,30*normal(1)],[0,30*normal(2)],[B(3),B(3)+30*normal(3)],'linewidth',2)
axis equal

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

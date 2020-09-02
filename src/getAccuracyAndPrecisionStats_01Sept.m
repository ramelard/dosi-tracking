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

%Origin of world coords
worldOrgIdx = 1;
rotVec1 = rotVecs(worldOrgIdx,:);
rotMat1 = rotationVectorToMatrix([rotVec1(1),rotVec1(2),rotVec1(3)]);
transVec1 = transVecs(worldOrgIdx,:);
%World coordinates
u3 = zeros(size(x)); v3 = zeros(size(x)); w3=zeros(size(x));
for i = 1:length(x)
    if ~isnan(x(i))
        %wc = inv(M)* [x3(i),y3(i),z3(i),1]';
        c = [x(i),y(i),z(i)]' - [x(worldOrgIdx),y(worldOrgIdx),z(worldOrgIdx)]';
        wc2 = rotMat1\c;
    else
        wc2 = [NaN, NaN,NaN];
    end
    u3(i) = wc2(1); v3(i) = wc2(2); w3(i) = wc2(3);
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

%Plotting
figure
plot3(trueX,trueY,trueZ,'o')
hold on
plot3(u3,v3,w3,'*')
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



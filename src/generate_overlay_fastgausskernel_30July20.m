% Pre: load data file into matrix A.
A=csvread('..\data\inclusion2xa_OPs.txt',1,0);
bgIm = imread('..\data\inclusion2xa_img.tif');
grayBG = rgb2gray(bgIm);
[imHeight,imWidth] = size(grayBG);
% Hack for now: use X,Y points as pixel points.
x = round(A(:,5));
y = round(A(:,6));

mua = A(:,7);
mus = A(:,8);

[X,Y] = meshgrid(1:size(bgIm,2), 1:size(bgIm,1));

% 2D Gaussian kernel sigmas
sigma = [4 4];
spreadImWidth = 40*sigma(2);
spreadImHeight = 40*sigma(1);
[sX,sY] = meshgrid(1:spreadImWidth, 1:spreadImHeight);
%[spreadImWidth,spreadImHeight] = size(sX);
distFromCtr = sqrt((sX - round(spreadImHeight/2)).^2 + (sY-round(spreadImWidth/2)).^2);
spreadIm = zeros(size(sX));
spreadIm(round(spreadImWidth/2),round(spreadImHeight/2)) = 1;
spreadIm = imgaussfilt(spreadIm,sigma,'filtersize',spreadImWidth-1);
spreadIm(distFromCtr > (9 * sigma(1))) = 0;
spreadIm = spreadIm./max(spreadIm(:));

figure
imagesc((spreadIm))

% figure;
singleIm = zeros(size(X));
sumWtIm = zeros(size(X));
sumWtMuaIm = zeros(size(X));
tic
for i = 1:numel(x)
  if ~isnan(x(i)) && ~isnan(y(i))
      xi = x(i);
      yi = y(i);
      mua_i = mua(i);
      
      
      cropLeftFlag = 0;
      cropRightFlag = 0;
      cropTopFlag = 0;
      cropBottomFlag = 0;
      leftEdge = round(xi-spreadImWidth/2);
      if leftEdge < 1
          leftEdge = 1;
          cropLeftFlag =1;
      end
      rightEdge = leftEdge+spreadImWidth-1;
      if rightEdge > imWidth
          rightEdge = imWidth;
          cropRightFlag =1 ;
      end
      topEdge = round(yi-spreadImHeight/2);
      if topEdge < 1
          topEdge = 1;
          cropTopFlag = 1;
      end
      bottomEdge = topEdge + spreadImHeight-1;
      if bottomEdge > imHeight
          bottomEdge = imHeight;
          cropBottomFlag = 1;
      end
      
      singleIm(topEdge:bottomEdge,leftEdge:rightEdge) = spreadIm;
    %   if mod(i,10) == 0
    %     imshow(mua_wi, [])
    %     drawnow;
    %   end

      sumWtIm = sumWtIm + singleIm;
      sumWtMuaIm = sumWtMuaIm + singleIm*mua_i;
      singleIm(topEdge:bottomEdge,leftEdge:rightEdge) = 0;

%       imagesc(sumWtMuaIm./sumWtIm, [0 0.02]);
%       %set(gcf,'position',[839   178   726   703]);
%       drawnow;
%       pause(0.1)
  end
end
%mua_map = mua_map + 1;

mua_map = sumWtMuaIm./sumWtIm;
mua_map(isnan(mua_map(:))) = 0;
toc
map = colormap();
nonZeroIdx = find(mua_map > 0);
minv = min(mua_map(nonZeroIdx));
maxv = max(mua_map(nonZeroIdx));
ncol = size(map,1);
s = round(1+(ncol-1)*(mua_map-minv)/(maxv-minv));
rgb_image = ind2rgb(s,map);
alphaMap = zeros(size(mua_map));
alphaMap(nonZeroIdx) = 0.75;
%red = cat(3, ones(size(X)), zeros(size(X)), zeros(size(X)));
%c = imfuse(bgIm,mua_map,'colorchannels',[2,0,1]);
figure
imshow(bgIm);
hold on
h = imshow(rgb_image);
set(h,'AlphaData',alphaMap);

%hold on
%imshow(mua_map, [0.01, 0.02], 'colormap',jet)

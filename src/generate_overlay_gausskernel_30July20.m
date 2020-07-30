% Pre: load data file into matrix A.
A=csvread('..\data\inclusion2xa_OPs.txt',1,0);
bgIm = imread('..\data\inclusion2xa_img.tif');
grayBG = rgb2gray(bgIm);
% Hack for now: use X,Y points as pixel points.
x = round(A(:,5));
y = round(A(:,6));

mua = A(:,7);
mus = A(:,8);

[X,Y] = meshgrid(1:size(bgIm,2), 1:size(bgIm,1));

mua_map = zeros(size(X));
% Track the denominator of weighted average so we can update it.
sumwi = zeros(size(X));

% 2D Gaussian kernel sigmas
sigma = [20 20];
% figure;
tic
for i = 1:numel(x)
  if ~isnan(x(i)) && ~isnan(y(i))
      xi = x(i);
      yi = y(i);
      mua_i = mua(i);

      % TODO: Parzen window method?
      % Generate spatial weights to spread the reading according to Gaussian kernel.
      mua_wi = zeros(size(X));
      mua_wi(yi,xi) = 1;
      mua_wi = imgaussfilt(mua_wi, sigma);
      mua_wi = mua_wi ./ max(mua_wi(:));

    %   if mod(i,10) == 0
    %     imshow(mua_wi, [])
    %     drawnow;
    %   end

      % Update mua map.
      % Before: mua = sum(wi * mua_i)/sum(wi)
      % Update: (sum(mua * sum(wi)) + new_mua*mua_wi)/sum(wi + mua_wi)
      mua_map = (mua_map.*sumwi + mua_wi*mua_i)./(sumwi + mua_wi);
      sumwi = sumwi + mua_wi;
      mua_map(isnan(mua_map)) = 0;

    %   imshow(mua_map, [0 0.02]);
    %   set(gcf,'position',[839   178   726   703]);
    %   drawnow;
    %   pause(0.5)
  end
end
toc

map = colormap('jet');
nonZeroIdx = find(mua_map > 0);
minv = min(mua_map(nonZeroIdx));
maxv = max(mua_map(nonZeroIdx));
ncol = size(map,1);
s = round(1+(ncol-1)*(mua_map-minv)/(maxv-minv));
rgb_image = ind2rgb(s,map);

%red = cat(3, ones(size(X)), zeros(size(X)), zeros(size(X)));
%c = imfuse(bgIm,mua_map,'colorchannels',[2,0,1]);
figure
imshow(bgIm);
hold on
h = imshow(rgb_image);
set(h,'AlphaData',mua_map.*(1/max(mua_map(:))));

%hold on
%imshow(mua_map, [0.01, 0.02], 'colormap',jet)

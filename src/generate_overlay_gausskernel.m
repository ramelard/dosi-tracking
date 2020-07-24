% Pre: load data file into matrix A.

% Hack for now: use X,Y points as pixel points.
x = round(A(:,1));
y = round(A(:,2));

mua = A(:,6);
mus = A(:,7);

[X,Y] = meshgrid(1:max(x), 1:max(y));

mua_map = zeros(size(X));
% Track the denominator of weighted average so we can update it.
sumwi = zeros(size(X));

% 2D Gaussian kernel sigmas
sigma = [3 3];
% figure;
for i = 1:numel(x)
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

figure, imshow(mua_map, [0.01, 0.02], 'colormap',jet)

% DOSI starts recording when lasers have been turned on. So analyse the
% frames for when the reflectance from the lasers have saturated.
% Note: alters vin.CurrentTime to be vid_t0
function [vid_t0] = sync_t0(vin, theta_axis)
  if nargin < 2
    theta_axis = 0;
  end

  found_dosi_t0 = false;
  laser_rect = [];
  laser_avg = [];
  idx = 1;
  hfig = figure;
  while ~found_dosi_t0
    frame = rgb2gray(im2double(vin.readFrame()));

    if isempty(laser_rect)
      figure, imshow(frame);
      text(10,20,'Select illumination ROI','color','r','fontweight','bold')
      laser_rect = getrect;
      close;
      
      roi = imcrop(frame(:,:,1), laser_rect);
      maxval = max(roi(:))*1.03;
    end
    
    frame_crop = imcrop(frame(:,:,1), laser_rect);
    laser_avg(idx) = sum(frame_crop(:) > maxval);
%     laser_avg(idx) = mean2(imcrop(frame(:,:,1), laser_rect));
    if idx > 10
      thresh = pi/180;
      theta = atan2(laser_avg(idx) - laser_avg(idx-5), 5/30);
      figure(hfig);
      subplot(1,2,2)
      title(sprintf('\\theta=%.2f, min=%.3f (ratio=%.3f)', ...
        rad2deg(theta), min(laser_avg(1:idx)), laser_avg(idx)/min(laser_avg(1:idx))))
      found_dosi_t0 = (laser_avg(idx) > 1.05*min(laser_avg(1:idx))) && (theta < thresh);
    end
    idx = idx + 1;
    
    figure(hfig);
    subplot(1,2,1), imshow(frame), hold on
    rectangle('position',laser_rect,'edgecolor','r')
    hold off
    drawnow;
    subplot(1,2,2), plot(laser_avg)
  end
  close;
  
  vin.CurrentTime = vin.CurrentTime - 6/vin.FrameRate;
  vid_t0 = vin.CurrentTime;
%   vid_t0 = (idx-6) / vin.FrameRate;
%   vin.CurrentTime = vid_t0;
end
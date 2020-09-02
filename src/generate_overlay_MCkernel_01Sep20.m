% Pre: load data file into matrix A.
sampNames = {'inclusion2xa','inclusion5xa','kneeMeas01','armMeas02','breastPhantom01','breastPhantom02','legMeas01'};
for r = 1:length(sampNames)
    sampName = sampNames{r};
    A=csvread(sprintf('D:/googleDrive/dDOSI/trackDOSI/data/01Sep2020/%s_OPs.txt',sampName),1,0);
    bgIm = imread(sprintf('D:/googleDrive/dDOSI/trackDOSI/data/01Sep2020/%s_cleanFrame.png',sampName));

    grayBG = rgb2gray(bgIm);
    %bgIm = grayBG;
    [imHeight,imWidth] = size(grayBG);
    % Hack for now: use X,Y points as pixel points.
    x = A(:,2);
    y = A(:,3);
    z = A(:,4);
    xPx = round(A(:,5));
    yPx = round(A(:,6));
    rotVecs = A(:,[7:9]);
    transVecs = A(:,[10:12]);
    mua = A(:,23);
    mua_vec = A(:,[13:2:23]);
    mus_vec = A(:,[14:2:24]);
    mus = A(:,24);
    dx = diff(x);
    dy = diff(y);
    dpx = diff(xPx);
    dpy = diff(yPx);
    
    for j = 1:2
        if j == 1
            useMC = 0;
        else
            useMC = 1;
        end

        c = polyfit(dx(~isnan(dx)),dpx(~isnan(dx)),1);
        pxPerMM = c(1);
%         figure
%         plot(dx,dpx,'o','markerfacecolor','b')
%         hold on
%         plot(dy,dpy,'o','markerfacecolor','r')
%        
%         plot([min([dx;dy]),max([dx;dy])],c(1)*[min([dx;dy]),max([dx;dy])]+c(2),'k--','linewidth',1.5)
%         title('Camera calibration')
%         xlabel('Difference in cam coords (mm)')
%         ylabel('Difference in pixel coords (px)')
%         legend('X','Y','Fit','Location','northwest')
 
        %Image coordinate meshgrid
        [X,Y] = meshgrid(1:size(bgIm,2), 1:size(bgIm,1));
        if useMC
            if exist('probIm','var') == 0
                probIm = generateWeightMap(0.001,10,0.9);
            end
            %Now this image has the same scale as the camera image
            probRescale = imresize(probIm,0.5*pxPerMM,'bilinear');
            spreadIm = probRescale./max(probRescale(:));
        else
        % 2D Gaussian kernel sigmas
            sigma = [7 7];
            spreadImWidth = 10*sigma(2);
            spreadImHeight = 10*sigma(1);
            [sX,sY] = meshgrid(1:spreadImWidth, 1:spreadImHeight);
            %[spreadImWidth,spreadImHeight] = size(sX);
            distFromCtr = sqrt((sX - round(spreadImHeight/2)).^2 + (sY-round(spreadImWidth/2)).^2);
            spreadIm = zeros(size(sX));
            spreadIm(round(spreadImWidth/2),round(spreadImHeight/2)) = 1;
            spreadIm = imgaussfilt(spreadIm,sigma,'filtersize',spreadImWidth-1);
            spreadIm(distFromCtr > (4 * sigma(1))) = 0;
            spreadIm = spreadIm./max(spreadIm(:));
        end
        spreadImWidth = size(spreadIm,2);
        spreadImHeight = size(spreadIm,1);
        %Okay, probIm is made of pixels that are 0.5 mm on a side
        % 2D Gaussian kernel sigmas
        % sigma = [4 4];
        % spreadImWidth = 40*sigma(2);
        % spreadImHeight = 40*sigma(1);
        % [sX,sY] = meshgrid(1:spreadImWidth, 1:spreadImHeight);
        % %[spreadImWidth,spreadImHeight] = size(sX);
        % distFromCtr = sqrt((sX - round(spreadImHeight/2)).^2 + (sY-round(spreadImWidth/2)).^2);
        % spreadIm = zeros(size(sX));
        % spreadIm(round(spreadImWidth/2),round(spreadImHeight/2)) = 1;
        % spreadIm = imgaussfilt(spreadIm,sigma,'filtersize',spreadImWidth-1);
        % spreadIm(distFromCtr > (9 * sigma(1))) = 0;
        % spreadIm = spreadIm./max(spreadIm(:));

        % figure
        % imagesc((spreadIm))

         %figure;
        lams = [690,730,785,808,830,850];
        numChroms = 2;
        chromFile = 'D:\googleDrive\dDOSI\trackDOSI\code\chromophores_ZijlstraKouVanVeen.txt';
        C = dlmread(chromFile,'\t',1,0);
        coefs = zeros(length(lams),numChroms);
        for l = 1:length(lams)
           coefs(l,:)= C(find(C(:,1) == lams(l)),2:(numChroms+1));
        end
        singleIm = zeros(size(X));
        sumWtIm = zeros(size(X));
        sumWtMuaIm = zeros(size(X));
        sumWtOxyIm = zeros(size(X));
        sumWtDeoxyIm = zeros(size(X));
        tic
        for i = 1:numel(xPx)
          if ~isnan(xPx(i)) && ~isnan(yPx(i))
              xi = xPx(i);
              yi = yPx(i);
              mua_i = mua(i);
              mua_vec_i = mua_vec(i,:);
              chroms_i = coefs \ mua_vec_i';
              roti = rotVecs(i,:);
              zRotMat = rotationVectorToMatrix([roti]);
              xAxInCam = zRotMat \ [1,0,0]';
              rotZ = atan2(xAxInCam(2),xAxInCam(1));
              rotSpreadIm = imrotate(spreadIm,rad2deg(rotZ),'crop');

              cropLeftFlag = 0;
              cropRightFlag = 0;
              cropTopFlag = 0;
              cropBottomFlag = 0;
              leftEdge = round(xi-spreadImWidth/2);
              %repIm = rotSpreadIm;
              if leftEdge < 1
                  leftEdge = 1;
                  cropLeftFlag =1;
                  %repIm = rotSpreadIm
              end
              rightEdge = leftEdge+spreadImWidth-1;
              if rightEdge > imWidth
                  rightEdge = imWidth;
                  leftEdge = imWidth - spreadImWidth+1;
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
                  topEdge = imHeight - spreadImHeight+1;
                  cropBottomFlag = 1;
              end

              singleIm(topEdge:bottomEdge,leftEdge:rightEdge) = rotSpreadIm;
            %   if mod(i,10) == 0
            %     imshow(mua_wi, [])
            %     drawnow;
            %   end

              sumWtIm = sumWtIm + singleIm;
              sumWtMuaIm = sumWtMuaIm + singleIm*mua_i;
              sumWtOxyIm = sumWtOxyIm + singleIm*chroms_i(1);
              sumWtDeoxyIm = sumWtDeoxyIm + singleIm*chroms_i(2);
              singleIm(topEdge:bottomEdge,leftEdge:rightEdge) = 0;

              %imagesc(sumWtMuaIm./sumWtIm,[0,0.05]);
        %       imagesc(sumWtIm,[0,8.5]);
        %       set(gcf,'position',[839   178   726   703]);
        %       drawnow;
        %       pause(0.1)
          end
        end
        %mua_map = mua_map + 1;

        mua_map = sumWtMuaIm./sumWtIm;
        oxy_map = sumWtOxyIm./sumWtIm;
        deoxy_map = sumWtDeoxyIm./sumWtIm;
        mua_map(isnan(mua_map(:))) = 0;
        oxy_map(isnan(oxy_map(:))) = 0;
        deoxy_map(isnan(deoxy_map(:))) = 0;
        sat_map = oxy_map./(oxy_map+deoxy_map);
        toc

        map = colormap('jet');
        nonZeroIdx = find(mua_map > 0);
        minv = min(mua_map(nonZeroIdx));
        maxv = max(mua_map(nonZeroIdx));
        maxv = maxv*.95;
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
        colormap('jet');
        colorbar
        caxis([minv,maxv])
        %plot(xPx,yPx,'w.')
        title('Absorption @ 850 nm')
        set(h,'AlphaData',alphaMap);
        if useMC
            print(sprintf('../data/01Sep2020/%s_MCWeights_overlay.png',sampName),'-dpng')
        else
            print(sprintf('../data/01Sep2020/%s_gaussWeights_overlay.png',sampName),'-dpng')
        end
        
        
        map = colormap('jet');
        nonZeroIdx = find(mua_map > 0);
        minv = min(sat_map(nonZeroIdx));
        maxv = max(sat_map(nonZeroIdx));
        maxv = maxv*.95;
        ncol = size(map,1);
        s = round(1+(ncol-1)*(sat_map-minv)/(maxv-minv));
        rgb_image = ind2rgb(s,map);
        alphaMap = zeros(size(sat_map));
        alphaMap(nonZeroIdx) = 0.75;
        %red = cat(3, ones(size(X)), zeros(size(X)), zeros(size(X)));
        %c = imfuse(bgIm,mua_map,'colorchannels',[2,0,1]);
        figure
        imshow(bgIm);
        hold on
        h = imshow(rgb_image);
        colormap('jet');
        colorbar
        caxis([minv,maxv]*100)
        %plot(xPx,yPx,'w.')
        title('Saturation')
        set(h,'AlphaData',alphaMap);
        if useMC
            print(sprintf('../data/01Sep2020/%s_MCWeights_sat_map.png',sampName),'-dpng')
        else
            print(sprintf('../data/01Sep2020/%s_gaussWeights_sat_map.png',sampName),'-dpng')
        end
        
         map = colormap('jet');
        nonZeroIdx = find(mua_map > 0);
        total_map = oxy_map + deoxy_map;
        minv = min(total_map(nonZeroIdx));
        maxv = max(total_map(nonZeroIdx));
        maxv = maxv*.95;
        ncol = size(map,1);
        s = round(1+(ncol-1)*(total_map-minv)/(maxv-minv));
        rgb_image = ind2rgb(s,map);
        alphaMap = zeros(size(total_map));
        alphaMap(nonZeroIdx) = 0.75;
        %red = cat(3, ones(size(X)), zeros(size(X)), zeros(size(X)));
        %c = imfuse(bgIm,mua_map,'colorchannels',[2,0,1]);
        figure
        imshow(bgIm);
        hold on
        h = imshow(rgb_image);
        colormap('jet');
        colorbar
        caxis([minv,maxv]*1000)
        %plot(xPx,yPx,'w.')
        title('Total Hb')
        set(h,'AlphaData',alphaMap);
        if useMC
            print(sprintf('../data/01Sep2020/%s_MCWeights_totalHb.png',sampName),'-dpng')
        else
            print(sprintf('../data/01Sep2020/%s_gaussWeights_totalHb.png',sampName),'-dpng')
        end
        
        map = colormap('jet');
        minv1 = min(oxy_map(nonZeroIdx));
        maxv1 = max(oxy_map(nonZeroIdx))*.95;
        minv2 = min(deoxy_map(nonZeroIdx));
        maxv2 = max(deoxy_map(nonZeroIdx))*.95;
        
        maxv = max([maxv1,maxv2]);
        minv = min([minv1,minv2]);
        
        ncol = size(map,1);
        s = round(1+(ncol-1)*(oxy_map-minv)/(maxv-minv));
        rgb_image = ind2rgb(s,map);
        alphaMap = zeros(size(oxy_map));
        alphaMap(nonZeroIdx) = 0.75;
        %red = cat(3, ones(size(X)), zeros(size(X)), zeros(size(X)));
        %c = imfuse(bgIm,mua_map,'colorchannels',[2,0,1]);
        figure
        imshow(bgIm);
        hold on
        h = imshow(rgb_image);
         colormap('jet');
        colorbar
        caxis([minv,maxv]*1000)
        %plot(xPx,yPx,'w.')
        set(h,'AlphaData',alphaMap);
        title('Oxy-hemoglobin')
        if useMC
            print(sprintf('../data/01Sep2020/%s_MCWeights_oxy.png',sampName),'-dpng')
        else
            print(sprintf('../data/01Sep2020/%s_gaussWeights_oxy.png',sampName),'-dpng')
        end
        
%         minv = min(deoxy_map(nonZeroIdx));
%         maxv = max(deoxy_map(nonZeroIdx));
%         maxv = maxv*.95;
        ncol = size(map,1);
        s = round(1+(ncol-1)*(deoxy_map-minv)/(maxv-minv));
        rgb_image = ind2rgb(s,map);
        alphaMap = zeros(size(deoxy_map));
        alphaMap(nonZeroIdx) = 0.75;
        %red = cat(3, ones(size(X)), zeros(size(X)), zeros(size(X)));
        %c = imfuse(bgIm,mua_map,'colorchannels',[2,0,1]);
        figure
        imshow(bgIm);
        hold on
        h = imshow(rgb_image);
        colormap('jet');
        colorbar
        caxis([minv,maxv]*1000)
        %plot(xPx,yPx,'w.')
        set(h,'AlphaData',alphaMap);
        title('Deoxy-hemoglobin')
        if useMC
            print(sprintf('../data/01Sep2020/%s_MCWeights_deoxy.png',sampName),'-dpng')
        else
            print(sprintf('../data/01Sep2020/%s_gaussWeights_deoxy.png',sampName),'-dpng')
        end
        
        %hold on
        %imshow(mua_map, [0.01, 0.02], 'colormap',jet)
        h=figure;
        set(h,'Position',[100,100,1024,512])
        subplot(121)
        imagesc(sumWtIm)
        axis equal
        title('Weights')
        subplot(122)
        imagesc(rgb_image)
        axis equal
        title('Weighted Absorption')
        if useMC
            print(sprintf('../data/01Sep2020/%s_MCWeights_weights.png',sampName),'-dpng')
        else
            print(sprintf('../data/01Sep2020/%s_gaussWeights_weights.png',sampName),'-dpng')
        end
    end
end


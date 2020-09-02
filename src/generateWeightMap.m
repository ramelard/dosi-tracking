%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MCXLAB - Monte Carlo eXtreme for MATLAB/Octave by Qianqina Fang
%
% In this example, we show the most basic usage of MCXLAB.
%
% This file is part of Monte Carlo eXtreme (MCX) URL:http://mcx.sf.net
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function prob = generateWeightMap(mua,mus,g)
%     addpath(genpath('C:\Users\Matthew\Documents\MCXStudio\MATLAB'));
%     addpath(genpath('C:\Users\Matthew\Documents\MCXStudioMCXSuite\mcx\utils'));
%     addpath(genpath('C:\Users\Matthew\Documents\MCXStudioMCXSuite\mmc\matlab'));
    addpath(genpath('D:\Work\RoblyerLab\mcxlab'))
    clear cfg cfgs
    cfg.nphoton=1e9;
    cfg.vol=uint8(ones(120,120,120));
    cfg.srcpos=[60 50 0];
    cfg.srcdir=[0 0 1];
    cfg.gpuid=1;
    cfg.unitinmm = 0.5;
    cfg.maxdetphoton = 1e8;
    % cfg.gpuid='11'; % use two GPUs together
    cfg.autopilot=1;
    cfg.issrcfrom0=1;
    cfg.prop=[0 0 1 1;mua mus g 1.4];
    cfg.tstart=0;
    cfg.tend=5e-9;
    cfg.tstep=5e-9;
    % calculate the flux distribution with the given config
    cfg.detpos=[60 70 0 2];
    %cfg.savedetflag='dsp';
    [~, detp, ~ , seeds]=mcxlab(cfg);

    newcfg=cfg;
    newcfg.seed=seeds.data;
    newcfg.outputtype='jacobian';
    newcfg.detphotons=detp.data;
    [flux2, detp2, vol2, seeds2]=mcxlab(newcfg);
    jac=sum(flux2.data,4);

    prob = sum(flux2.data,3);
    prob(log(prob)<-6) = 0;
    % figure
    % imagesc(log(prob))
end



function [t_dosi, dosi] = read_dosi_asc(folder)
  file_time = dir([folder, '/*_TIME.asc']);
  file_dosi = dir([folder, '/*_dBMU.asc']);
  file_chromophores = dir([folder, '/*_dBSUM.asc']);
  
  if isempty(file_time) || isempty(file_dosi)
    warning('Could not find DOSI files.')
    t_dosi = [];
    mua = [];
    return;
  end
  
  % There are 3 rows (wavelengths) for each time point (727nm, 787nm, 832nm)
  Mdosi = tdfread(fullfile(file_dosi.folder, file_dosi.name));
%   Mtime = tdfread(fullfile(file_time.folder, file_time.name));
  Mtime = read_time_file(fullfile(file_time.folder, file_time.name));
  
  dosi = struct();
  % Take only one wavelength's mua and mus for now.
  dosi.mua = Mdosi.mua(2:3:end);
  dosi.mus = Mdosi.mus(2:3:end);
  dosi.thc = [];
  dosi.hbo2 = [];
  dosi.hb = [];
  dosi.so2 = [];
  if ~isempty(file_chromophores)
    HbO2_row1 = 4;
    Hb_row1 = 5;
    TCH_row1 = 6;
    O2sat_row1 = 7;
    Mchrom = tdfread(fullfile(file_chromophores.folder, file_chromophores.name));
    dosi.thc = Mchrom.value(TCH_row1:7:end);
    dosi.hbo2 = Mchrom.value(HbO2_row1:7:end);
    dosi.hb = Mchrom.value(Hb_row1:7:end);
    dosi.so2 = Mchrom.value(O2sat_row1:7:end);
  end
  
  fmt = 'mm:ss.SSS';
  try 
    datetime(Mtime.time{1},'InputFormat',fmt);
  catch err
    fmt = 'HH:mm:ss.SSS';
  end
  
  t0 = datetime(Mtime.time{1},'InputFormat',fmt);
  t_dosi = zeros(numel(Mtime.time), 1);
  for i = 2:numel(t_dosi)
    tcur = datetime(Mtime.time{i},'InputFormat',fmt);
    delta = tcur - t0;
    t_dosi(i) = milliseconds(delta) / 1000;
  end
end


% tdfread() has a bug with str2num. Create our own.
function Mstruct = read_time_file(filename)
  fid = fopen(filename);
  
  headers = fgets(fid);
  data = textscan(fid, '%s\t%s');
  Mstruct.position = data{1};
  Mstruct.time = data{2};
  
  fclose(fid);
end
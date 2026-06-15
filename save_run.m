function filepath = save_run(simin, simout, label)
% Saves simin and simout to data/<timestamp>_<label>.mat
%   label — short string identifying the run, e.g. 'link2_free_swing'

    folder = fullfile(fileparts(mfilename('fullpath')), 'data');
    if ~exist(folder, 'dir')
        mkdir(folder);
    end

    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    filename  = fullfile(folder, sprintf('%s_%s.mat', timestamp, label));
    save(filename, 'simin', 'simout');
    fprintf('Saved: %s\n', filename);
    filepath = filename;
end

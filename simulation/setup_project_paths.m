function paths = setup_project_paths()
% 自动设置项目路径，避免绝对路径和模型遮蔽问题

thisFile = mfilename('fullpath');
projectRoot = fileparts(thisFile);

paths.root = projectRoot;
paths.library = fullfile(projectRoot, 'LIBRARY');
paths.motion_case = fullfile(projectRoot, 'motion_case');
paths.simulink = fullfile(projectRoot, 'SIMULINK');

addpath(paths.root);

if isfolder(paths.library)
    addpath(paths.library);
else
    warning('LIBRARY 文件夹不存在：%s', paths.library);
end

if isfolder(paths.motion_case)
    addpath(paths.motion_case);
else
    warning('motion_case 文件夹不存在：%s', paths.motion_case);
end

if isfolder(paths.simulink)
    addpath(paths.simulink);
else
    warning('SIMULINK 文件夹不存在：%s', paths.simulink);
end

end
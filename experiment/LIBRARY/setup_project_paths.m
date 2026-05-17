function paths = setup_project_paths()
% setup_project_paths
% 自动设置项目路径。
% 该文件位于 simulation/LIBRARY/ 下，因此项目根目录是 LIBRARY 的上一级。

thisFile = mfilename('fullpath');
libraryDir = fileparts(thisFile);
projectRoot = fileparts(libraryDir);

paths.root = projectRoot;
paths.library = libraryDir;
paths.motion_case = fullfile(projectRoot, 'motion_case');
paths.simulink = fullfile(projectRoot, 'SIMULINK');

% 加入路径
addpath(paths.root);
addpath(paths.library);

% if isfolder(paths.motion_case)
%     addpath(paths.motion_case);
% else
%     warning('motion_case 文件夹不存在：%s', paths.motion_case);
% end
% 
% if isfolder(paths.simulink)
%     addpath(paths.simulink);
% else
%     warning('SIMULINK 文件夹不存在：%s', paths.simulink);
% end
% 
% end
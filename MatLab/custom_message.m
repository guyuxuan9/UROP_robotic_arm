%% This will create a folder called custom in the current working directory. 
folderPath = fullfile(pwd,"custom");
%% Copy the custom package into the folder. Then run the following.
ros2genmsg(folderPath)

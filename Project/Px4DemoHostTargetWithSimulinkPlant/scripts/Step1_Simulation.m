%   Copyright 2020 The MathWorks, Inc.

project = simulinkproject;
projectRoot = project.RootFolder;
open_system(fullfile(projectRoot,'models','QuadcopterSimulation.slx'));
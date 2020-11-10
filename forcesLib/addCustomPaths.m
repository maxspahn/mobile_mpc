function addCustomPaths()
    pathForces = '';
    pathCasadi = '';

    forcesPath = genpath(pathForces);
    casadiPath = genpath(pathCasadi);
    addpath(forcesPath);
    addpath(casadiPath);

end


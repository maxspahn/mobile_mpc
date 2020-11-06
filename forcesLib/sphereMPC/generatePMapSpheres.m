function pMap = generatePMapSpheres(N, nbSpheres)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
pMap = struct();
pMap.dt1 = 1;
pMap.dt2 = 2;
pMap.r = 3;
pMap.L = 4;
pMap.refPath = [5, 4 + 3 * N - 1];
pMap.desOrient = 5 + 3 * N;
pMap.desArmConfig = [pMap.desOrient + 1, pMap.desOrient + 7];
pMap.weights = [pMap.desArmConfig(2) + 1, pMap.desArmConfig(2) + 7];
pMap.safetyMarginBase = pMap.weights(2) + 1;
pMap.safetyMarginArm = pMap.weights(2) + 2;

pMap.nbMovingObstacles = 5;
pMap.nbStaticSpheres = nbSpheres;
pMap.dimStaticSphere = 4;
pMap.nbInfPlanesEachSphere = 0;
pMap.nbInfPlanes = pMap.nbInfPlanesEachSphere * 4;
pMap.dimInfPlane = 4;
pMap.dimMovingObstacle = 7;
%nbInfPlanes = pMap.dimInfPlane * pMap.nbInfPlanesEachSphere;
%pMap.infPlanes = [pMap.safetyMarginArm + 1, pMap.safetyMarginArm + nbInfPlanes * 4];
pMap.movingObstacles = [pMap.safetyMarginArm + 1, pMap.safetyMarginArm + pMap.nbMovingObstacles * 7];
pMap.staticSpheres = [pMap.movingObstacles(2) + 1, pMap.movingObstacles(2) + pMap.nbStaticSpheres * pMap.dimStaticSphere];
end


axis equal;
hold on;
% PlaceObject('Orange_Ripe.ply',[0,-1,0]);
% % PlaceObject('Orange_OverRipe.ply',[0.1,-1,0]);
% % PlaceObject('Orange_UnRipe.ply',[0.2,-1,0]);
% % PlaceObject('HalfSizedRedGreenBrick.ply',[0.3,-1,-0.02]);
% % PlaceObject('treeNormal.ply',[0,-1,0]);
% % PlaceObject('GripRightLink0.ply',[0,1,0]);


robotFunctions = RobotFunctions();

QA = Panda(transl(0,0,0));

robotFunctions.FormPointCloud(QA);

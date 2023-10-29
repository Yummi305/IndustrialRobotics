axis equal;
hold on;
% PlaceObject('Orange_Ripe.ply',[0,-1,0]);
% % PlaceObject('Orange_OverRipe.ply',[0.1,-1,0]);
% % PlaceObject('Orange_UnRipe.ply',[0.2,-1,0]);
% % PlaceObject('HalfSizedRedGreenBrick.ply',[0.3,-1,-0.02]);
% PlaceObject('treeNormal.ply',[0,-1,0]);
% PlaceObject('GripRightLink0.ply',[0,1,0]);
% PlaceObject('ColouredPandaLink0.ply',[0,1.5,0]);
% PlaceObject('ColouredPandaLink7.ply',[2,1,0]);
QA = LinearUR3(transl(2,2,0));
PlaceObject('cone.ply',[0,0,0]);
PlaceObject('lightCurtain_NegX.ply',[0,0,0.24]);



% robotFunctions = RobotFunctions();
% 
% QA = LinearUR3(transl(0,0,0));
% 
% QA.model.teach();

%robotFunctions.FormPointCloud(QA);

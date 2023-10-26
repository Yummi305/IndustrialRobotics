classdef FruitLoopSystem
    properties
        
    end

    methods (Static)
        function Environment

            objFunc = ObjectFunctions();

            % Read environment textures
             floor = imread ('animal_crossing__grass.jpg'); % Floor Image
            %  wall = imread ('forest.jpg'); % Wall Image
            
            % Floor
             surf([-6.5,-6.5;7,7],[-5,5;-5,5],[0.012,0.012;0.012,0.012],'CData',floor,'FaceColor','texturemap'); 
            
             hold on
            % Walls 
            %  surf([-4,2;-4,2],[2,2;2,2],[0,0;4,4],'CData',wall,'FaceColor','texturemap'); % Back wall
            %  surf([2,2;2,2],[-3,2;-3,2],[4,4;0,0],'CData',wall,'FaceColor','texturemap'); % Side wall
            
            %% Farm Environment
            % Tree Location Container
            tree_position = [-0.4, 0.6, 0; 
                             -1.2, 0.6, 0;
                              0.4, 0.6, 0];
            
            %% Initialise Farm
            
            % Safety Equipment
            % Guard rails to secure the zone from external hazards
            PlaceObject('fence.ply',[-1.4,1,0.01]);
            PlaceObject('fence.ply',[-0.15,1,0.01]);
            PlaceObject('fence.ply',[1.1,1,0.01]);
            
            % Safety cones
            PlaceObject('cone.ply',[-1.2,0,0.01]);
            PlaceObject('cone.ply',[0.5,0,0.01]);
            PlaceObject('cone.ply',[0.5,-1.4,0.01]);
            PlaceObject('cone.ply',[-1.2,-1.4,0.01]);
            
            PlaceObject('fireExtinguisherElevated.ply', [-1.8,0.65,0.01]);
            PlaceObject('emergencyStopButton.PLY',[-2,-1.8,0.01]);
            
            % Unsorted Mandarins crate
            PlaceObject('fruit_crate_unsorted.ply',[-0.7,-0.35,0.01]);
            
            % Ripe Mandarins crate
            PlaceObject('fruit_crate_ripe.ply',[-1.1,-0.6,0.01]);
            
            % Overripe Mandarins crate
            PlaceObject('fruit_crate_over_ripe.ply',[-1.1,-0.9,0.01]);
            
            % Trees
            objFunc.PlaceManyObjects('treeMandarin.ply',tree_position, false, 0, 0);
            
        end

        function [tree1_pos, tree1_obj, tree1_verts, tree1_picked, tree2_pos, tree2_obj, tree2_verts, tree2_picked, tree1_crate_pos, tree2_crate_pos, tree1_above_crate, tree2_above_crate, tree1_sorted_crate_pos, tree2_sorted_crate_pos, tree1_above_sorted_crate, tree2_above_sorted_crate] = Mandarins()
            objFunc = ObjectFunctions();

            %% Tree 1 Mandarins
            % Tree 1 Mandarin Initial Locations
            tree1_pos = [-0.4, 0.3, 0.5; 
                         -0.5, 0.3, 0.55; 
                         -0.6, 0.3, 0.51;
                         -0.6, 0.33, 0.4];
            
            % Store Mandarin objects and vertices
            tree1_obj = cell(1, size(tree1_pos, 1));
            tree1_verts = cell(1, size(tree1_pos, 1));
            
            % Tree 1 Mandarin picked off tree location
            tree1_picked = tree1_pos;
            tree1_picked(:, 2) = tree1_picked(:, 2) - 0.6; 
            
            
            %% Tree 2 Mandarins
            % Tree 2 Mandarin Initial Locaitons
            tree2_pos = [-0.95, 0.38, 0.42;
                         -0.9, 0.38, 0.48;
                         -1, 0.38, 0.51;
                         -1.1, 0.3, 0.39];
            
            % Store Mandarin objects and vertices
            tree2_obj = cell(1, size(tree2_pos, 1));
            tree2_verts = cell(1, size(tree2_pos, 1));
            
            % Tree 2 Mandarin picked off tree location
            tree2_picked = tree2_pos;
            tree2_picked(:, 2) = tree2_picked(:, 2) - 0.1;
            
            %% Unsorted Crate Positions
            % Tree 1 Mandarin Crate Locations
            tree1_crate_pos = [-0.55,-0.3,0.04; 
                               -0.65,-0.3,0.04; 
                               -0.75,-0.3,0.04; 
                               -0.85,-0.3,0.04];
            
            % Tree 2 Mandarin Crate Locations
            tree2_crate_pos = [-0.85,-0.37,0.04
                               -0.75,-0.37,0.04; 
                               -0.65,-0.37,0.04; 
                               -0.55,-0.37,0.04];
            
            % Tree 1 Mandarin above crate
            tree1_above_crate = tree1_crate_pos;
            tree1_above_crate(:, 3) = tree1_above_crate(:, 3) + 0.8;
            
            % Tree 2 Mandarin above crate
            tree2_above_crate = tree2_crate_pos;
            tree2_above_crate(:, 3) = tree2_above_crate(:, 3) + 0.8;
            
            %% Sorted Crate Positions
            % Tree 1 Mandarin Sorted Crate Locations
            tree1_sorted_crate_pos = [-1.2,-0.65,0.04; 
                                      -1.1,-0.65,0.04; 
                                      -1.0,-0.65,0.04; 
                                      -1.1,-0.82,0.04];
            
            % Tree 2 Mandarin Sorted Crate Locations
            tree2_sorted_crate_pos = [-1.2,-0.52,0.04
                                      -1.1,-0.52,0.04; 
                                      -1.0,-0.52,0.04; 
                                      -1.0,-0.82,0.04];
            
            % Tree 1 Mandarin above sorted crate
            tree1_above_sorted_crate = tree1_sorted_crate_pos;
            tree1_above_sorted_crate(:, 3) = tree1_above_sorted_crate(:, 3) + 0.8;
            
            % Tree 2 Mandarin above sorted crate
            tree2_above_sorted_crate = tree2_sorted_crate_pos;
            tree2_above_sorted_crate(:, 3) = tree2_above_sorted_crate(:, 3) + 0.8;
            
            %% Grow Fruit
            % If testing QA only, fast forward mandarin position to unsorted crate.
                % Else, spawn mandarin on tree.
                [tree1_obj,tree1_verts] = objFunc.GrowMandarins('Mandarin_ripe.ply','Mandarin_OverRipe.ply',tree1_pos,tree1_obj,tree1_verts);
                % Tree 2's Mandarins
                [tree2_obj,tree2_verts] = objFunc.GrowMandarins('Mandarin_ripe.ply','Mandarin_OverRipe.ply',tree2_pos,tree2_obj,tree2_verts);
    
        end
    end
end

%% Additional Notes
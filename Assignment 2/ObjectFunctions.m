classdef ObjectFunctions
    % Class containing functions that facilitate brick placement.
    methods (Static)
        %% Update Brick Container
        function container = UpdateBrickLocation(container, transform)
            % Update brick location container positions
            for j = 1:size(container)
                container(j,:) = container(j,:) + transform;
            end
        end
        %% Place Objects (bricks)
        function [objectArray, vertices] = PlaceManyObjects(objectName, positionMatrix, boolOriginalBrick, objectArray, vertices)
            % Place object in desired location
            for i = 1:size(positionMatrix, 1)
                % Load the object mesh and store verts objectMesh in bricks.
                if boolOriginalBrick == true
                    x = positionMatrix(i,1);
                    y = positionMatrix(i,2);
                    z = positionMatrix(i,3);
                    objectArray{i} = PlaceObject(objectName);
                    vertices{i} = get(objectArray{i}, 'Vertices');
                    % Compute the transformation
                    vertsTransform = [vertices{i}, ones(size(vertices{i}, 1), 1)] * transl([x,y,z])';
                    % Update the object's position and orientation
                    set(objectArray{i}, 'Vertices', vertsTransform(:, 1:3));
                else
                    objectTemp = PlaceObject(objectName, positionMatrix(i,:));
                    % Compute the transformation
                    verts = [get(objectTemp, 'Vertices'), ones(size(get(objectTemp, 'Vertices'), 1), 1)];
                    % Update the object's position and orientation
                    set(objectTemp, 'Vertices', verts(:, 1:3));
                end
            end
        end

        %% Grow Oranges
        function [objectArray, vertices] = GrowMandarins(ripeOrange,OverRipeOrange, positionMatrix, objectArray, vertices)
            % Place object in desired location
            for i = 1:size(positionMatrix, 1)
                % Load the object mesh and store verts objectMesh in bricks.
                    x = positionMatrix(i,1);
                    y = positionMatrix(i,2);
                    z = positionMatrix(i,3);
                    if i == size(positionMatrix, 1)
                        objectArray{i} = PlaceObject(OverRipeOrange);
                    else
                        objectArray{i} = PlaceObject(ripeOrange);
                    end
                    vertices{i} = get(objectArray{i}, 'Vertices');
                    % Compute the transformation
                    vertsTransform = [vertices{i}, ones(size(vertices{i}, 1), 1)] * transl([x,y,z])';
                    % Update the object's position and orientation
                    set(objectArray{i}, 'Vertices', vertsTransform(:, 1:3));
            end
        end
    end
end
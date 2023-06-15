addpath("classes/world-generator/")

% Filenames
wc_file = 'world_test.wc';
sdf_file = 'world_test.sdf';

% Create the world and the generator
world = World([0 0 0], [550, 550]);
generator = BuildingGenerator([10,50], [10,50], [10,50]);

%Set init pos
x = int32(0 - world.sx/2 + generator.sx(2));
y = int32(0 - world.sy/2 + generator.sy(2));

% Open wc_file and write world size in the first line
delete(wc_file); delete(sdf_file);
file = fopen(wc_file, 'w');
fprintf(file, '%d %d %d\n', world.sx, world.sy, world.sz);

max_x = 0;
%Loop to generate the buildings
while x < int32(world.sx/2 - generator.sx(2))
    % Set Y init pos (row of buildings)
    y = int32(0 - world.sy/2 + generator.sy(2));
    while y < int32(world.sy/2 - generator.sy(2))
        % New building
        building = generator.generateBuilding();
        % Save the maximum x size of the buildings in the row
        if max_x < building.sx
            max_x = building.sx;
        end
        % Set building position
        building = building.setPos([x, int32(y+building.sy/2), 0]);
        world = world.addBuilding(building);
        % Write the building in the file
        fprintf(file, '%d %d %d %d %d %d\n', x, int32(y+building.sy/2), 0, building.sx, building.sy, building.sz);
        %Increment Y with building size + street size (randomly)
        y = y + building.sy + randi([world.streets_size(1), world.streets_size(2)]);
    end
    
    %Increment X with max_x (max size in x of the buildings in the row) and
    %the street size (randomly)
    x = x + max_x + randi([world.streets_size(1), world.streets_size(2)]);
    % Reset max_x
    max_x = 0;
end
fclose(file);

%Geenerate SDF
sdf = world.generateSDF();

%Save SDF code
f = fopen(sdf_file,"w");
fprintf(f, sdf);
fclose(f);
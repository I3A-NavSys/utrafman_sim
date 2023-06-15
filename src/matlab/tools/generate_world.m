addpath("classes/world-generator/")

% Nombres de los archivos de salida
file_name = 'world_test.wc';
export_file = 'world_test.sdf';

% Crear el mundo y un generador de edificios
world = World([0 0 0], [1000, 1000]);
generator = BuildingGenerator([10,50], [10,50], [10,50]);

% Establecer la posición inicial de los edificios
x = int32(0 - world.sx/2 + generator.sx(2));
y = int32(0 - world.sy/2 + generator.sy(2));

% Abrir el archivo y escribir las propiedades del mundo (primera línea)
file = fopen(file_name, 'w');
fprintf(file, '%d %d %d\n', world.sx, world.sy, world.sz);

max_x = 0;
% Bucle de generación de edificios
while x < int32(world.sx/2 - generator.sx(2))
    % Establecer y en la posición inicial
    y = int32(0 - world.sy/2 + generator.sy(2));
    while y < int32(world.sy/2 - generator.sy(2))
        % Generar un nuevo edificio
        building = generator.GenerateBuilding();
        % Si el tamaño en x es el máximo, almacenar el valor
        if max_x < building.sx
            max_x = building.sx;
        end
        % Establecer la posición del edificio y agregarlo al mundo
        building = building.SetPos([x, int32(y+building.sy/2), 0]);
        world = world.AddBuilding(building);
        % Escribir el edificio en el archivo
        fprintf(file, '%d %d %d %d %d %d\n', x, int32(y+building.sy/2), 0, building.sx, building.sy, building.sz);
        % Incrementar y con el componente y del edificio y un valor aleatorio entre el tamaño de las calles
        y = y + building.sy + randi([world.streets_size(1), world.streets_size(2)]);
    end
    
    % Incrementar x con el max_x de los edificios y un valor aleatorio entre el tamaño de las calles
    x = x + max_x + randi([world.streets_size(1), world.streets_size(2)]);
    % Restablecer max_x
    max_x = 0;
end
fclose(file);

sdf = "<?xml version='1.0'?><sdf version='1.7'><world name='default'>";

sdf = strcat(sdf, world.GenerateSDF());

%for building in world.buildings:
for building = world.buildings
    sdf = strcat(sdf, building.GenerateSDF());
end
plugin = "<!-- World god (to spawn and remove drones and models) --> <plugin name='UTRAFMAN_God' filename='libUTRAFMAN_God.so' /> </world></sdf>";
sdf = strcat(sdf, plugin);

f = fopen(export_file,"w");
fprintf(f, sdf);
fclose(f);
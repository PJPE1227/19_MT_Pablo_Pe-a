%examples of all the functions used during the tests

%% algoritmos base/pruebas con funciones de costo varias post-barrido

%funciones de costo: Sphere, Beale, Ackley, CrossTray, Booth, threeHump,
%Easom, Michalewicz, Rosenbrock y Eggcrate

[~, ~, ~, ~, ~, ~] = Bat_Algorithm(40, 200, 2, 0.7, 0.6, "Sphere", 1, 1.0);

[~, ~, ~, ~, ~, ~] = Firefly_Algorithm(20, 200, 0.6, 1.0, 0.01, 1.0, 2, "Ackley", 1, 1.0);

%% generacion de paths con penalización artificial

%mapas disponibles: A-I

[~, ~, ~, ~, ~] = Bat_Algorithm_Path(40, 200, 10, 0.7, 0.6, [-10, 10], [-10, 10], [-9, -9], [9, 9], "Map E", 0);

[~, ~, ~, ~, ~] = Firefly_Algorithm_Path(20, 200, 0.6, 1.0, 0.01, 1.0, 10, [-10, 10], [-10, 10], [-9, -9], [9, 9], "Map A", 0);

%% experimentos con APFs

%mapas disponibles: APFmapA-I

%metodos para generar APFs: Choset y Kim

[~, ~, ~, ~, ~, ~, ~] = Bat_Algorithm_APF(40, 200, 2, 0.7, 0.6, "APFmapA", 1, 1.0,[-9, -9], [9, 9], "Choset");

[~, ~, ~, ~, ~, ~, ~] = Firefly_Algorithm_APF(20, 200, 0.6, 1.0, 0.01, 1.0, 2, "APFmapH", 1, 1.0,[-9, -9], [9, 9], "Choset");

%% generacion de paths con penalización artificial y repulsion de APFs

%mapas disponibles: A-I

[~, ~, ~, ~, ~, ~, ~] = Bat_Algorithm_Path(40, 200, 10, 0.7, 0.6, 0, [-10, 10], [-10, 10], [-9, -9], [9, 9], "Map A", 1);

[~, ~, ~, ~, ~, ~, ~] = Firefly_Algorithm_Path(20, 200, 0.6, 1.0, 0.01, 1.0, 10, 0, [-10, 10], [-10, 10], [-9, -9], [9, 9], "Map A", 1);

%% generacion de subPaths para resolver problemas mas complejos

%mapas disponibles: A-I

[~, trackBats, ~, ~] = Bat_Algorithm_subPaths(80, 400, 8, 0.7, 0.6, 0, [-9,-4], [9,4], "Map I", 3, 0);

[~, trackFlies, ~, ~] = Firefly_Algorithm_subPaths(20, 200, 0.6, 1.0, 0.01, 1.0, 6, 0, [-9,-4], [9,4], "Map I", 3, 0);

%% resolucion de otros problemas de optimización

%mapas disponibles: A-L
 [t, ~, B, A, radius, ~, pointsBat] = Bat_Algorithm_Sensors(40, 300, 8, 0.7, 0.6, [-10,10], [-10,10], "Map L", 5, 5);

 [~, ~, B, A, r, ~, pointsFlies] = Firefly_Algorithm_Sensors(20, 150, 0.6, 1.0, 0.01, 1.0, 10, [-10,10], [-10,10], "Map L", 5, 5);
 
 
% Chemin vers le fichier Excel
chemin_fichier = 'camera_euler_nicococo_9.csv';

% Importer les données brutes depuis le fichier Excel
[data, ~, ~] = xlsread(chemin_fichier);
T = readtable(chemin_fichier,'NumHeaderLines',5);

% Extraire les données de la première colonne (premier attribut de T) et de la sixième colonne (sixième attribut de T)
colonne_1 = T(:, 1).Variables*0.01 - 1.2 ;% Première colonne
colonne_6 = -T(:, 7).Variables*0.001 +0.03; % Sixième colonne
colonne_7 = T(:, 8).Variables*0.001 + 0.053;
colonne_8 = T(:, 9).Variables*0.001 - 0.05;
% Tracer la sixième colonne en fonction de la première colonne

% xlabel('Nom de l\'axe X'); % Ajouter une étiquette à l'axe X
% ylabel('Nom de l\'axe Y'); % Ajouter une étiquette à l'axe Y
% title('Tracé de la 6ème colonne en fonction de la 1ère colonne'); % Ajouter un titre au graphique
% grid on; % Activer la grille

% Ouvrir le fichier JSON
chemin_fichier_json = 'camera_data.json';
donnees_json = fileread(chemin_fichier_json);

% Convertir le fichier JSON en une structure de données MATLAB
donnees_matlab = jsondecode(donnees_json);

camera_meas = zeros(1, 7);
% Créer une variable avec une ligne par clé
noms_des_cles = fieldnames(donnees_matlab);
for i = 1:numel(noms_des_cles)

    a = donnees_matlab.(noms_des_cles{i});
    b = a{2,1};
    camera_meas(i,1) = a{1,1};
    camera_meas(i,2) = b(1);
    camera_meas(i,3) = b(2);
    camera_meas(i,4) = b(3);
    camera_meas(i,5) = b(4);
    camera_meas(i,6) = b(5);
    camera_meas(i,7) = b(6);
end
camera_meas;
figure
plot(colonne_1, -colonne_7, 'b-');
hold on; % Maintenir le tracé actuel
plot(camera_meas(:,1), camera_meas(:,2), 'r--')
grid on;

figure
plot(colonne_1, colonne_6, 'b-');
hold on; % Maintenir le tracé actuel
plot(camera_meas(:,1), camera_meas(:,3), 'r--')
grid on;

figure
plot(colonne_1, colonne_8, 'b-');
hold on; % Maintenir le tracé actuel
plot(camera_meas(:,1), camera_meas(:,4), 'r--')
grid on;

%-----------------------axe z ----------------------------------------
erreurs_mesures_z = [];
courbe2 = colonne_8;
courbe1 = camera_meas(:,4);
temps1 = colonne_1;
temps2 = camera_meas(:,1);
abs_erreur_z = 0;
% Pour chaque mesure dans la première courbe
for i = 1:length(temps2)
    % Trouver l'index de la mesure la plus proche dans la deuxième courbe
    [~, index_plus_proche] = min(abs(temps1 - temps2(i)));
    
    difference = courbe1(i) - courbe2(index_plus_proche);
    
    % Gérer les valeurs dans un intervalle cyclique (-180 à 180)
    if difference > 180
        difference = difference - 360; % Ajuster la différence
    elseif difference < -180
        difference = difference + 360; % Ajuster la différence
    end

    % Calculer l'erreur absolue entre les deux valeurs mesurées
    erreur_mesure = abs(difference);
    if ~isnan(erreur_mesure)
        if erreur_mesure > abs_erreur_z
            abs_erreur_z = erreur_mesure;
            time_max_error_z = temps2(i);
        end
        % Ajouter l'erreur à la liste des erreurs
        erreurs_mesures_z = [erreurs_mesures_z, erreur_mesure];
    end
end

% Calculer l'erreur moyenne des mesures
erreurs_mesures_z;
mean_error_z = mean(erreurs_mesures_z)
abs_erreur_z
time_max_error_z

%--------------axe y-----------------------------------------------
erreurs_mesures_y = [];
courbe2 = colonne_6;
courbe1 = camera_meas(:,3);
temps1 = colonne_1;
temps2 = camera_meas(:,1);
abs_erreur_y = 0;
% Pour chaque mesure dans la première courbe
for i = 1:length(temps2)
    % Trouver l'index de la mesure la plus proche dans la deuxième courbe
    [~, index_plus_proche] = min(abs(temps1 - temps2(i)));
    
    difference = courbe1(i) - courbe2(index_plus_proche);
    
    % Gérer les valeurs dans un intervalle cyclique (-180 à 180)
    if difference > 180
        difference = difference - 360; % Ajuster la différence
    elseif difference < -180
        difference = difference + 360; % Ajuster la différence
    end

    % Calculer l'erreur absolue entre les deux valeurs mesurées
    erreur_mesure = abs(difference);
    if ~isnan(erreur_mesure)
        if erreur_mesure > abs_erreur_y
            abs_erreur_y = erreur_mesure;
            time_max_error_y = temps2(i);
        end
        % Ajouter l'erreur à la liste des erreurs
        erreurs_mesures_y = [erreurs_mesures_y, erreur_mesure];
    end
end

% Calculer l'erreur moyenne des mesures
erreurs_mesures_y;
mean_error_y = mean(erreurs_mesures_y)
abs_erreur_y
time_max_error_y

%---------------------------------------------axe x----------------------
erreurs_mesures_x = [];
courbe2 = -colonne_7;
courbe1 = camera_meas(:,2);
temps1 = colonne_1;
temps2 = camera_meas(:,1);
abs_erreur_x = 0;
% Pour chaque mesure dans la première courbe
for i = 1:length(temps2)
    % Trouver l'index de la mesure la plus proche dans la deuxième courbe
    [~, index_plus_proche] = min(abs(temps1 - temps2(i)));
    
    difference = courbe1(i) - courbe2(index_plus_proche);
    
    % Gérer les valeurs dans un intervalle cyclique (-180 à 180)
    if difference > 180
        difference = difference - 360; % Ajuster la différence
    elseif difference < -180
        difference = difference + 360; % Ajuster la différence
    end

    % Calculer l'erreur absolue entre les deux valeurs mesurées
    erreur_mesure = abs(difference);
    if ~isnan(erreur_mesure)
        if erreur_mesure > abs_erreur_x
            abs_erreur_x = erreur_mesure;
            time_max_error_x = temps2(i);
        end
        % Ajouter l'erreur à la liste des erreurs
        erreurs_mesures_x = [erreurs_mesures_x, erreur_mesure];
    end
end

% Calculer l'erreur moyenne des mesures
erreurs_mesures_x;
mean_error_x = mean(erreurs_mesures_x)
abs_erreur_x
time_max_error_x

figure
plot3(camera_meas(:,2), camera_meas(:,3), camera_meas(:,4), 'LineWidth', 2);
hold on;
plot3(-colonne_7, colonne_6, colonne_8,'r--', 'LineWidth', 2);

%---------------------------------------------total----------------------
erreurs_mesures_total = [];
courbe2_x = -colonne_7;
courbe1_x = camera_meas(:,2);
courbe2_y = colonne_6;
courbe1_y = camera_meas(:,3);
courbe2_z = colonne_8;
courbe1_z = camera_meas(:,4);
temps1 = colonne_1;
temps2 = camera_meas(:,1);
abs_erreur_total = 0;
% Pour chaque mesure dans la première courbe
for i = 1:length(temps2)
    % Trouver l'index de la mesure la plus proche dans la deuxième courbe
    [~, index_plus_proche] = min(abs(temps1 - temps2(i)));

    difference = courbe1(i) - courbe2(index_plus_proche);

    difference = sqrt((courbe1_x(i) - courbe2_x(index_plus_proche))^2 + (courbe1_y(i) - courbe2_y(index_plus_proche))^2 + (courbe1_z(i) - courbe2_z(index_plus_proche))^2);
    
    % Gérer les valeurs dans un intervalle cyclique (-180 à 180)
    if difference > 180
        difference = difference - 360; % Ajuster la différence
    elseif difference < -180
        difference = difference + 360; % Ajuster la différence
    end

    % Calculer l'erreur absolue entre les deux valeurs mesurées
    erreur_mesure = abs(difference);
    if ~isnan(erreur_mesure)
        if erreur_mesure > abs_erreur_total
            abs_erreur_total = erreur_mesure;
            time_max_error_total = temps2(i);
        end
        % Ajouter l'erreur à la liste des erreurs
        erreurs_mesures_total = [erreurs_mesures_total, erreur_mesure];
    end
end

% Calculer l'erreur moyenne des mesures
erreurs_mesures_total;
mean_error_total = mean(erreurs_mesures_total)
abs_erreur_total
time_max_error_total


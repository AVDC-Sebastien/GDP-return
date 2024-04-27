chemin_fichier = 'Nicofrenchie_2.csv';

% Importer les données brutes depuis le fichier Excel
[data, ~, ~] = xlsread(chemin_fichier);
T = readtable(chemin_fichier,'NumHeaderLines',5)

% Extraire les données de la première colonne (premier attribut de T) et de la sixième colonne (sixième attribut de T)
colonne_1 = T(:, 1).Variables*0.01 - 1.42 % Première colonne
colonne_5 = T(:, 6).Variables % Sixième colonne
colonne_4 = T(:, 7).Variables
colonne_8 = T(:, 8).Variables*0.1 - 11

chemin_fichier_json = 'lidar_data.json';
donnees_json = fileread(chemin_fichier_json);

% Convertir le fichier JSON en une structure de données MATLAB
donnees_matlab = jsondecode(donnees_json);

lidar_meas = zeros(1, 2);
% Créer une variable avec une ligne par clé
noms_des_cles = fieldnames(donnees_matlab);
a_t = donnees_matlab.(noms_des_cles{1});
t = a_t(1);
for i = 1:numel(noms_des_cles)

    a = donnees_matlab.(noms_des_cles{i});
    
    lidar_meas(i,1) = a(1)-t;
    lidar_meas(i,2) = a(2);
   
end
lidar_meas

figure
plot(colonne_1, colonne_8, 'b-');
hold on; % Maintenir le tracé actuel
plot(lidar_meas(:,1), lidar_meas(:,2), 'r--')
grid on;

erreurs_mesures = [];
courbe2 = colonne_8;
courbe1 = lidar_meas(:,2);
temps1 = colonne_1;
temps2 = lidar_meas(:,1);
abs_erreur = 0
% Pour chaque mesure dans la première courbe
for i = 1:length(temps2)
    % Trouver l'index de la mesure la plus proche dans la deuxième courbe
    [~, index_plus_proche] = min(abs(temps1 - temps2(i)));
    
    % Calculer l'erreur entre les deux valeurs mesurées
    erreur_mesure = abs(courbe1(i) - courbe2(index_plus_proche));
    if erreur_mesure > abs_erreur
        abs_erreur = erreur_mesure;
    end
    % Ajouter l'erreur à la liste des erreurs
    erreurs_mesures = [erreurs_mesures, erreur_mesure];
end

% Calculer l'erreur moyenne des mesures
mean_error = mean(erreurs_mesures)
abs_erreur
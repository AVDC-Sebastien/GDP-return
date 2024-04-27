% Chemin vers le fichier Excel
chemin_fichier = 'camera_euler_nicococo_8.csv';

% Importer les données brutes depuis le fichier Excel
[data, ~, ~] = xlsread(chemin_fichier);
T = readtable(chemin_fichier,'NumHeaderLines',5)

% Extraire les données de la première colonne (premier attribut de T) et de la sixième colonne (sixième attribut de T)
colonne_1 = T(:, 1).Variables*0.01 +0.8 -3.5 +1.4% Première colonne
colonne_6 = -T(:, 7).Variables*0.001 + 0.03 -0.5 % Sixième colonne
colonne_7 = T(:, 8).Variables*0.001 + 0.073
colonne_8 = T(:, 9).Variables*0.001 - 0.08
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
plot(colonne_1, colonne_6, 'b-');
hold on; % Maintenir le tracé actuel
plot(camera_meas(:,1), camera_meas(:,3), 'r--')
grid on;
figure
plot(colonne_1, -colonne_7, 'b-');
hold on; % Maintenir le tracé actuel
plot(camera_meas(:,1), camera_meas(:,2), 'r--')
grid on;
figure
plot(colonne_1, -colonne_6, 'b-');
hold on; % Maintenir le tracé actuel
plot(camera_meas(:,1), camera_meas(:,4), 'r--')
grid on;
figure
plot3(camera_meas(:,2), camera_meas(:,3), camera_meas(:,4), 'LineWidth', 2);
hold on;
plot3(-colonne_7, colonne_6, colonne_8,'r--', 'LineWidth', 2);


% Chemin vers le fichier Excel
chemin_fichier = 'imu_nicoco.csv';

% Importer les données brutes depuis le fichier Excel
[data, ~, ~] = xlsread(chemin_fichier);
T = readtable(chemin_fichier,'NumHeaderLines',5);

% Extraire les données de la première colonne (premier attribut de T) et de la sixième colonne (sixième attribut de T)
colonne_1 = T(:, 1).Variables*0.01 - 1.5;  % Première colonne
colonne_2 = [];
colonne_3 = [];
colonne_4 = [];
for i=1:numel(colonne_1)
    q1 =T(i, 3).Variables;
    q2 =T(i, 4).Variables;
    q3 =T(i, 5).Variables;
    q4 =T(i, 6).Variables;
    quat = [q1,q2,q3,q4];
    eul = quat2eul(quat, "ZYX");
    if rad2deg(eul(1,1))<0
        colonne_2(i) = rad2deg(eul(1,1)) +180;
    else
        colonne_2(i) = rad2deg(eul(1,1)) - 180;
    end
    
    colonne_3(i) = rad2deg(eul(1,2));
    colonne_4(i) = rad2deg(eul(1,3));

end





% Ouvrir le fichier JSON
chemin_fichier_json = 'imu_data.json';
donnees_json = fileread(chemin_fichier_json);

% Convertir le fichier JSON en une structure de données MATLAB
donnees_matlab = jsondecode(donnees_json);

camera_meas = zeros(1, 7);
% Créer une variable avec une ligne par clé
noms_des_cles = fieldnames(donnees_matlab);
for i = 1:numel(noms_des_cles)
    i;
    c = donnees_matlab.(noms_des_cles{1});
    a = donnees_matlab.(noms_des_cles{i});
    b = a{2,1};
    camera_meas(i,1) = a{1,1}-c{1,1};

    camera_meas(i,4) = b(1,3);
    
    camera_meas(i,3) = b(1,2);

    if camera_meas(i,1)>23.34 && (b(1,1)<-150 || b(1,1)>150 )

        if b(1,1)<-150
           camera_meas(i,2) = b(1,1) + 180; 
        else
            camera_meas(i,2) = b(1,1) - 180; 
        end
        
    else
        
        camera_meas(i,2) = b(1,1);
    end
    % camera_meas(i,5) = b(4);
    % camera_meas(i,6) = b(5);
    % camera_meas(i,7) = b(6);
end

figure
plot(colonne_1, colonne_3, 'b-');
hold on; % Maintenir le tracé actuel
plot(camera_meas(:,1), camera_meas(:,4), 'r--')
grid on;

figure
plot(colonne_1, -colonne_2, 'b-');
hold on; % Maintenir le tracé actuel
plot(camera_meas(:,1), camera_meas(:,3), 'r--')
grid on;

figure
plot(colonne_1, -colonne_4, 'b-');
hold on; % Maintenir le tracé actuel
plot(camera_meas(:,1), camera_meas(:,2), 'r--')
grid on;


%-----------------------axe z ----------------------------------------
erreurs_mesures_z = [];
courbe2 = -colonne_4;
courbe1 = camera_meas(:,2);
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

%--------------axe yè-----------------------------------------------
erreurs_mesures_y = [];
courbe2 = -colonne_2;
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
courbe2 = colonne_3;
courbe1 = camera_meas(:,4);
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


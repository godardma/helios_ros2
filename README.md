# Helios Ros2

Dossier présent sur l'ordi embarqué dans workspaceRos2/src (pas encore en fait)

WORK IN PROGRESS, actuellement une simulation fonctionnelle est proposée

## Auteur :

:student: Maël GODARD <mael.godard@ensta-bretagne.org> (FISE 2023)

## Git Structure :

* :file_folder: [/helios_ros2](helios_ros2) : **dossier contenant les sources**
* :file_folder: [/logs](logs) : **dossier contenant les logs de mission**
* :file_folder: [/launch](launch) : **dossier contenant les launcher**
* :file_folder: [/path](path) : **dossier contenant les fichiers de path à suivre**
* :file_folder: [/rviz2_config](rviz2_config) : **dossier contenant la config rviz2 conseillée**
* :spiral_notepad: [/setup.py](setup.py)    **fichier de setup ROS2 python**
* :spiral_notepad: [/README.md](README.md)

## Technologies :

* Ubuntu 20.04
* Python
* ROS2 foxy


## Lancement

* Cloner le repo dans un workspace ROS2 Foxy
* Dans un premier terminal, lancer rviz2 et ouvrir la config jointe dans le dossier [/rviz2_config](rviz2_config)
* Lancer la simulation :
```bash
colcon build --packages-select helios_ros2
. install/setup.bash
ros2 launch helios_ros2 helios.launch.py
```

## Logs

Une écriture de logs est implémentée. Le nom du fichier peut être choisi depuis le launcher et le chemin à suivre et la trajectoire se retrouvent dans le dossier [logs](logs).

Pour exploiter ces logs, se placer dans le dossier helios_ros2 et exécuter la commande :
```bash
python3 helios_ros2/create_gpx.py logs_test
```
Où "logs_test" est le nom du fichier de logs choisi. Les fichiers de logs associés seront convertis du format txt au format gpx et pourront être affichés sur Géoportail.

## Notes

Les positions (Pose) sont données dans le repère Lambert93 à un offset près (voir ref dans le launch file)

Il est possible de préparer plusieurs missions à l'avance. Pour celà il suffit d'enregistrer les waypoints dans un fichier texte au format suivant:

* Un point par ligne
* Un point est défini par sa longitude et sa latitude (à Guerledan de l'ordre de -3° et 48°) , séparées par une virgule (et sans espace)
* Les latitudes et longitudes sont en degrés décimaux

Les fichiers sont ensuite à stocker dans le dossier [path](path) et leur nom est à renseigner sous "pathfile_name" dans le [launcher](launch) (un seul par launch)

Pour que l'ajout du mesh se fasse correctement, il est nécessaire de lancer le launcher depuis la racine du workspace ROS2 contenant le package (i.e. depuis le dossier workspaceRos2 si le package helios_ros2 se trouve dans le dossier workspaceRos2/src)

## Connexion en ssh au helios

Se connecter au reseau munu_ubnt, puis dans un terminal :

```bash
ssh s100@10.43.20.223
#mot de passe : s100
```

## Utilisation

Ouvrir le port du recepteur GNSS et envoyer les corrections rtk dans un terminal:

```bash
narval_supply_control.py -e usbl
send-rtk-corrections
```

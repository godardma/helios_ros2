# Helios Ros2

Dossier présent sur l'ordi embarqué dans workspaceRos2/src

WORK IN PROGRESS

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

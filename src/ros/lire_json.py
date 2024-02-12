#!/usr/bin/env python3
import json

# Spécifiez le chemin complet du fichier JSON
chemin_fichier = '/home/yoann/catkin_ws/src/test/src/bonding.json'

# Charger les données depuis le fichier JSON
with open(chemin_fichier, 'r') as fichier:
    donnees = json.load(fichier)

# Variables pour les dimensions du magnét
longueur_magnet = donnees["Magnet"]["length"]
largeur_magnet = donnees["Magnet"]["width"]
epaisseur_magnet = donnees["Magnet"]["thickness"]

# Variables pour les données de picking
premier_magnet = donnees["picking"]["first_magnet"]
espacement_vertical = donnees["picking"]["spacing"]["vertical"]
espacement_horizontal = donnees["picking"]["spacing"]["horizontal"]
nombre_de_lignes = donnees["picking"]["number_of_row"]
magnets_par_ligne = donnees["picking"]["magnet_per_row"]

# Variables pour les données de placement
x_placement = donnees["placing"]["x"]
y_placement = donnees["placing"]["y"]
z_placement = donnees["placing"]["z"]

premier_magnet = [premier_magnet['x'], premier_magnet['y'], premier_magnet['z']]
# Afficher les variables dans le terminal
# print("Dimensions du magnét :")
# print(f"Longueur : {longueur_magnet}")
# print(f"Largeur : {largeur_magnet}")
# print(f"Épaisseur : {epaisseur_magnet}")

# print("\nDonnées de picking :")
print(premier_magnet)
# print(type(premier_magnet))
# print(f"Espacement vertical : {espacement_vertical}")
# print(f"Espacement horizontal : {espacement_horizontal}")
# print(f"Nombre de lignes : {nombre_de_lignes}")
# print(f"Magnets par ligne : {magnets_par_ligne}")

# print("\nDonnées de placement :")
# print(f"Coordonnée x : {x_placement}")
# print(f"Coordonnée y : {y_placement}")
# print(f"Coordonnée z : {z_placement}")


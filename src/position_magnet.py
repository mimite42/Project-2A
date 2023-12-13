#!/usr/bin/env python

import json
import rospy
from std_msgs.msg import Float32MultiArray

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

coordonnees = []

for i in range(nombre_de_lignes):
    deplacement_y = premier_magnet["y"] + i * (espacement_vertical + longueur_magnet)

    for k in range(magnets_par_ligne):
        num_magnet = i * magnets_par_ligne + k
        x = premier_magnet["x"] + k * (largeur_magnet + espacement_horizontal)
        y = deplacement_y
        z = premier_magnet["z"]
        c = [x, y, z]
        coordonnees.append(c)

print(coordonnees)

# Initialiser le nœud ROS
rospy.init_node('position_magnet', anonymous=True)

# Créer un éditeur pour publier sur le topic "pos_mark"
pos_mark_pub = rospy.Publisher('pos_mark', Float32MultiArray, queue_size=10)

# Taux de publication
rate = rospy.Rate(1)  # 1 Hz

while not rospy.is_shutdown():
    # Créer un message Float32MultiArray et publier la liste de coordonnées
    pos_mark_msg = Float32MultiArray(data=sum(coordonnees, []))
    pos_mark_pub.publish(pos_mark_msg)
    
    rate.sleep()


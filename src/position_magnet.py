import json
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Charger les données depuis le fichier JSON
with open('bonding.json', 'r') as fichier:
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


#### VISUALISATION 2D #####

fig, ax = plt.subplots()

x = [coordonnees[i][0] for i in range(len(coordonnees))]
y = [coordonnees[i][1] for i in range(len(coordonnees))]


for i in range(len(coordonnees)):
    rectangle = patches.Rectangle((x[i] - largeur_magnet/2, y[i] - longueur_magnet/2), largeur_magnet, longueur_magnet, linewidth=1, edgecolor='r', facecolor='none')
    ax.add_patch(rectangle)
    ax.text(x[i] - largeur_magnet/2, y[i] + longueur_magnet/2, str(i + 1), fontsize=8, color='r')
    
plt.plot(x, y, 'o')

ax.set_xlim(-25, 200) 
ax.set_ylim(-25, 225) 
 
plt.show()


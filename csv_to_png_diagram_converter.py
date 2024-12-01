import sys
import csv
import matplotlib.pyplot as plt

try:
    filename = sys.argv[1]
except Exception as e:
    print(f"Erreur lors de la lecteur du paramètre du script...")
    raise e

# Extraction des données contenues dans le nom du fichier (nom du maillage, facteur de décimation et nature du diagramme à créer (valences ou angles dièdres))
path_name, decimation_temp, operation = filename.split('-')
mesh_name = path_name.split('/')[1]
decimation_factor = decimation_temp[:1] + ',' + decimation_temp[1:]

# Ouvrir le fichier CSV et lire les en-têtes
with open(filename, newline='', encoding='utf-8') as csvfile:
    file_content = csv.reader(csvfile)
    # Lecture de l'en-tête et extraction des titres des axes des abcisses et et des ordonnées
    x_col, y_col = next(file_content)

    # Lecture des données contenues dans le fichier .csv
    x_data = []
    y_data = []
    # Ajout des valeurs dans chaque colonne du diagramme en barres
    for row in file_content:
        x_data.append(row[0])
        y_data.append(int(row[1]))

    # Création du diagramme en barres
    plt.figure(figsize=(12, 9))
    plt.bar(x_data, y_data, color='skyblue')

    # Ajout des valeurs au-dessus de chaque barre
    for i, valeur in enumerate(y_data):
        plt.text(i, valeur + 0.5, str(valeur), ha='center', va='bottom', fontsize=10)       
    
    plt.xlabel(x_col)
    plt.ylabel(y_col)

    sub_operations = operation[:-4].rsplit('_', 1)  
    # S'il s'agit d'un diagramme sur les valences
    if "valency" in sub_operations:
        # S'il s'agit du diagramme des valences du maillage après application de l'algorithme de décimation
        if "decimated" in sub_operations:
            plt.title(f"Nombre de sommets en fonction de leur valence pour le maillage '{mesh_name}'\naprès application d'un algorithme de décimation en utilisant un facteur de décimation de {decimation_factor}.")
        else:
            plt.title(f"Nombre de sommets en fonction de leur valence pour le maillage '{mesh_name}'.")
    # Sinon il s'agit d'un diagramme sur les angles dièdres
    else:
        plt.xticks(rotation=90)
        # S'il s'agit du diagramme des angles dièdres du maillage après application de l'algorithme de décimation
        if "decimated" in sub_operations:
            plt.title(f"Nombre de sommets en fonction de leur angle dièdre pour le maillage '{mesh_name}'\naprès application d'un algorithme de décimation en utilisant un facteur de décimation de {decimation_factor}.")
        else:
            plt.title(f"Nombre de sommets en fonction de leur angle dièdre pour le maillage '{mesh_name}'.")

    # Sauvegarde du diagramme au format .png
    plt.savefig(filename[:-4] + ".png", format='png', dpi=300)
    print(f"Sauvegarde du diagramme {filename[:-4]}.png.")

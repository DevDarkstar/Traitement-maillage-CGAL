#!/bin/bash

# Vérification si l'utilisateur a renseigné le paramètre de facteur de décimation lors de l'exécution du script
if [[ $# -lt 1 || $# -gt 2 ]]; then
    echo "Erreur dans l'exécution du script bash..."
    echo "Vous devez renseigner au moins le facteur de décimation à appliquer sur les maillages..."
    echo "La commande d'exécution du script doit être "$0" <facteur_décimation> ou "$0" <facteur_décimation> <nom_fichier>"
    exit 1
fi

# Récupération du facteur de décimation renseigné par l'utilisateur
DECIMATION_FACTOR=$1
# Nom du dossier contenant CGAL (pour vérifier si CGAL est déjà installé)
DIR_NAME="CGAL-5.6.1"
# Nom du dossier contenant les objets 3D à traiter
MESH_DIR="objets_3D"
# Nom du dossier contenant les fichiers de compilation du programme
BUILD_DIR="build"
# Nom du dossier qui contiendra l'ensemble des diagrammes au format .csv issus des traitements sur les différents maillages 3D
DIAGRAMS_DIR="diagrammes"
# Dossier qui contiendra les maillages, au format .obj, contenant les cartes de courbures
MESHES_DIR="maillages"
# Dossier contenant l'environnement virtuel de python (utilisé pour générer des diagrammes depuis des fichiers .csv en utilisant Matplotlib)
PYTHON_VENV="env"

# Vérification si CGAL est bien installé
if [[ ! -d "$DIR_NAME" ]]; then
    echo "$DIR_NAME n'a pas été trouvé. Vérifiez que $DIR_NAME est bien installé dans votre dossier."
    exit 1
fi

# Création du dossier "diagrammes" qui accueillera les diagrammes obtenus au format .csv
if [[ ! -d "$DIAGRAMS_DIR" ]]; then
    mkdir "$DIAGRAMS_DIR"
fi

# Création du dossier "maillages"
if [[ ! -d "$MESHES_DIR" ]]; then
    mkdir "$MESHES_DIR"
fi

# Nous nous rendons dans le dossier "build" du projet
cd "$BUILD_DIR"

# Exécution du programme pour chaque maillage présent dans le dossier "objets_3D" ou pour un maillage en particulier selon le choix de l'utilisateur
if [[ $# -eq 1 ]]; then
    for file in ../$MESH_DIR/*; do
        ./main $file $DECIMATION_FACTOR
    done
else
    ./main ../$MESH_DIR/$2 $DECIMATION_FACTOR
fi

# Nous remontons d'un cran dans l'arborescence du projet
cd ..

# Conversion de tous les fichiers au format .csv en diagramme exploitable par l'utilisateur
for file in $DIAGRAMS_DIR/*.csv; do
    # Transformation du fichier .csv en diagramme en barres au format .png en utilisant le module matplotlib de python
    python3 csv_to_png_diagram_converter.py "$file"
    # et suppression du fichier .csv après la création de l'image
    if [[ $? -eq 0 ]]; then
        rm -f "$file"
    fi
done

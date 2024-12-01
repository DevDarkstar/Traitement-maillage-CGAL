#!/bin/bash

# Vérification si l'utilisateur a renseigné le paramètre de facteur de décimation lors de l'exécution du script
if [[ $# -ne 1 ]]; then
    echo "Vous devez renseigner le facteur de décimation à appliquer sur les maillages..."
    echo "La commande d'exécution du script doit être du type "$0" <facteur_décimation>."
    exit 1
fi

# Récupération du facteur de décimation renseigné par l'utilisateur
DECIMATION_FACTOR=$1
# Chemin absolu du dossier dans lequel est exécuté le script courant
SCRIPT_DIR=$(dirname "$(realpath "$0")")
# Nom du dossier contenant CGAL (pour vérifier si CGAL est déjà installé)
DIR_NAME="CGAL-5.6.1"
# Nom du dossier contenant ls fichiers de compilation
BUILD_DIR="build"
# Nom du dossier contenant les objets 3D à traiter
MESH_DIR="objets_3D"
# Nom du dossier qui contiendra l'ensemble des diagrammes au format .csv issus des traitements sur les différents maillages 3D
DIAGRAMS_DIR="diagrammes"
# Dossier qui contiendra les maillages, au format .obj, contenant les cartes de courbures
MESHES_DIR="maillages"

# Vérification si CGAL est bien installé
if [[ ! -d "$DIR_NAME" ]]; then
    echo "$DIR_NAME n'a pas été trouvé. Vérifiez que $DIR_NAME est bien installé dans votre dossier."
    exit 1
fi

# Préparation des fichiers 3D pour traitements
for file in $MESH_DIR/*; do
    # Si le fichier est une archive au format .zip
    if [[ "$file" == *.zip ]]; then 
        # On extrait le fichier contenant l'objet 3D de l'archive
        unzip -q "$file" -d "$MESH_DIR"
        # et nous supprimons l'archive
        rm -f "$file"
    fi
done

# Création du dossier "diagrammes" qui accueillera les diagrammes obtenus au format .csv
if [[ ! -d "$DIAGRAMS_DIR" ]]; then
    mkdir "$DIAGRAMS_DIR"
fi

# Création du dossier "maillages"
if [[ ! -d "$MESHES_DIR" ]]; then
    mkdir "$MESHES_DIR"
fi

# Compilation du programme
echo "Compilation du programme..."

# Création du dossier "build" si non présent afin d'y stocker tous les fichiers de compilation
if [[ ! -d "$BUILD_DIR" ]]; then
    mkdir "$BUILD_DIR"
fi

# Se rendre dans le dossier "build"
cd "$BUILD_DIR" || exit

# Exécution du cmake
cmake -DCGAL_DIR=$SCRIPT_DIR/$DIR_NAME -DCMAKE_BUILD_TYPE=Release ..

# Vérification si le cmake s'est bien exécuté
if [[ $? -ne 0 ]]; then
    echo "Erreur lors de l'exécution du cmake"
    exit 1
fi

# Compilation du programme
make

echo "Compilation du programme réussie."

# Exécution du programme pour chaque maillage présent dans le dossier "objets_3D"
for file in "../"$MESH_DIR/*; do
    ./main $file $DECIMATION_FACTOR
done

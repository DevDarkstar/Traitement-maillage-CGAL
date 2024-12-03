#!/bin/bash

# Chemin absolu du dossier dans lequel est exécuté le script courant
SCRIPT_DIR=$(dirname "$(realpath "$0")")
# URL menant à l'archive CGAL à télécharger
URL="https://github.com/CGAL/cgal/releases/download/v5.6.1/CGAL-5.6.1.tar.xz"
# Nom de l'archive CGAL
FILE="CGAL-5.6.1.tar.xz"
# Nom du dossier contenant CGAL (pour vérifier si CGAL est déjà installé)
DIR_NAME="CGAL-5.6.1"
# Nom du dossier contenant les fichiers de compilation
BUILD_DIR="build"
# Nom du dossier contenant les objets 3D à traiter
MESH_DIR="objets_3D"

# Nous vérifions si CGAL n'est pas déjà installé
if [[ ! -d "$DIR_NAME" ]]; then

    echo "Installation de CGAL..."

    # Téléchargement de l'archive CGAL dans le répertoire où le script est situé
    wget -O "$SCRIPT_DIR/$FILE" "$URL"

    # Nous vérifions si nous avons bien pu télécharger l'archive
    if [[ $? -ne 0 ]]; then
        echo "Échec du téléchargement de $FILE."
        exit 1
    fi

    # Décompression de l'archive
    tar -xf "$FILE"

    # Vérification de la décompression de l'archive contenant CGAL
    if [[ $? -ne 0 ]]; then
        echo "Échec de la décompression de $FILE."
        exit 1
    fi

    echo "Installation de CGAL réussie"

# Sinon CGAL est déjà installé
else
    echo "$DIR_NAME est déjà installé"
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

# Compilation du programme
echo "Compilation du programme..."

# Création du dossier "build" si non présent afin d'y stocker tous les fichiers de compilation
if [[ ! -d "$BUILD_DIR" ]]; then
    mkdir "$BUILD_DIR"
fi

# Déplacement dans le dossier de compilation "build"
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

# Vérification si la compilation s'est bien déroulée
if [[ $? -ne 0 ]]; then
    echo "Erreur lors de l'exécution du make"
    exit 1
fi

echo "Compilation du programme réussie."

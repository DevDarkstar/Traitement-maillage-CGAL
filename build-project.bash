#!/bin/bash

# Chemin absolu du dossier dans lequel est exécuté le script courant
SCRIPT_DIR=$(dirname "$(realpath "$0")")
# Nom du dossier contenant CGAL (pour vérifier si CGAL est déjà installé)
DIR_NAME="CGAL-5.6.1"
# Nom du dossier contenant ls fichiers de compilation
BUILD_DIR="build"

# Vérification si CGAL est bien installé
if [[ ! -d "$DIR_NAME" ]]; then
    echo "$DIR_NAME n'a pas été trouvé. Vérifiez que CGAL est bien installé dans votre dossier."
    exit 1
fi

echo "Compilation du programme..."

# Création du dossier "build" si non présent afin d'y stocker tous les fichiers de compilation
if [ ! -d "$BUILD_DIR" ]; then
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

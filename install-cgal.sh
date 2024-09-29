#!/bin/bash

# Chemin absolu du dossier dans lequel est exécuté le script courant
SCRIPT_DIR=$(dirname "$(realpath "$0")")
# URL menant à l'archive CGAL à télécharger
URL="https://github.com/CGAL/cgal/releases/download/v5.6.1/CGAL-5.6.1.tar.xz"
# Nom de l'archive CGAL
FILE="CGAL-5.6.1.tar.xz"
# Nom du dossier contenant CGAL (pour vérifier si CGAL est déjà installé)
DIR_NAME="CGAL-5.6.1"

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

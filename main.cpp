#include "SurfaceMesh.hpp"

//cmake -DCGAL_DIR=/home/richard/Documents/CGAL-5.6 -DCMAKE_BUILD_TYPE=Release ..

int main(int argc, char* argv[]){
    // Récupération du nom du fichier .obj
    std::string objFilePath;
    if (argc != 2) {
        std::cerr << "Vous devez passer le nom du fichier .obj à lire à l'execution du programme.";
        return 1;
    } else {
        objFilePath = argv[1];
    }

    // Création d'un maillage de type SurfaceMesh
    SurfaceMesh surface_mesh(objFilePath);
    // Affichage du nombre de sommets et de faces du maillage
    surface_mesh.displaySurfaceMeshInfos();
    // Calcul de la valence de chaque sommet du maillage
    surface_mesh.generateVerticesValency();

    return 0;
}
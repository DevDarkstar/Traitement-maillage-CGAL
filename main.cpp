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
    surface_mesh.computeVerticesValency();
    // Exportation des valences des sommets au format CSV
    surface_mesh.exportVerticesValencyAsCSV("../valency.csv");
    // Calcul des angles dièdres des faces adjacentes entre elles
    surface_mesh.computeDihedralAngles();
    //surface_mesh.displayDihedralAnglesInfos();
    // Exportation des valeurs des angles dièdres en fonction de leur nombre d'occurrences au format CSV
    surface_mesh.exportDihedralAnglesAsCSV("../dihedral_angles.csv");
    // Calcul de l'aire des faces du maillage
    surface_mesh.computeAreaOfFaces();
    // Calcul de l'approximation de la courbure gaussienne à chaque sommet du maillage
    surface_mesh.computeGaussianCurvature();
    // Exportation du maillage avec un code couleur associé aux courbures gaussiennes de chaque sommet dans un fichier OBJ
    surface_mesh.exportGaussianCurvatureAsOBJ("../mesh_with_gaussian_curvature.obj");

    return 0;
}
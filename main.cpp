#include "SurfaceMesh.hpp"

//cmake -DCGAL_DIR=/home/richard/Documents/CGAL-5.6 -DCMAKE_BUILD_TYPE=Release ..

int main(int argc, char* argv[]){
    // Récupération du nom du fichier .obj
    std::string objFilePath;
    float decimation_factor;
    if (argc != 3) {
        std::cerr << "Vous devez passer le nom du fichier .obj à lire à l'execution du programme, ainsi que le facteur de décimation.";
        return 1;
    } else {
        objFilePath = argv[1];
        float value = std::stof(argv[2]);
        if (value < 0 || value > 1) {
            std::cerr << "Le facteur de décimation est un nombre décimal compris entre 0 et 1.";
        }
        else 
            decimation_factor = value;
    }

    // Création d'un maillage de type SurfaceMesh
    SurfaceMesh surface_mesh(objFilePath, decimation_factor);
    // Affichage du nombre de sommets et de faces du maillage
    surface_mesh.displaySurfaceMeshInfos();
    // Calcul de la valence de chaque sommet du maillage
    std::tuple<std::map<vertex_descriptor, int>, int> vertex_valency_mesh = surface_mesh.computeVerticesValency();
    //surface_mesh.displayValencyInfos(std::get<0>(vertex_valency_mesh));
    // Exportation des valences des sommets au format CSV
    surface_mesh.exportVerticesValencyAsCSV(vertex_valency_mesh, "../valency.csv");
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
    surface_mesh.exportGaussianCurvatureAsOBJ("../gaussian_curvature.obj", true);

    //Application d'un algorithme de décimation au maillage
    surface_mesh.triangulated_surface_mesh_simplification();

    //Réapplication de l'ensemble des opérations précédentes sur le maillage résultant
    // Affichage du nombre de sommets et de faces du maillage
    surface_mesh.displaySurfaceMeshInfos();
    // Calcul de la valence de chaque sommet du maillage
    std::tuple<std::map<vertex_descriptor, int>, int> vertex_valency_decimated_mesh = surface_mesh.computeVerticesValency();
    //surface_mesh.displayValencyInfos(std::get<0>(vertex_valency_decimated_mesh));
    // Exportation des valences des sommets au format CSV
    surface_mesh.exportVerticesValencyAsCSV(vertex_valency_decimated_mesh, "../valency_decimated.csv");
    // Calcul des angles dièdres des faces adjacentes entre elles
    surface_mesh.computeDihedralAngles();
    //surface_mesh.displayDihedralAnglesInfos();
    // Exportation des valeurs des angles dièdres en fonction de leur nombre d'occurrences au format CSV
    surface_mesh.exportDihedralAnglesAsCSV("../dihedral_angles_decimated.csv");
    // Calcul de l'aire des faces du maillage
    surface_mesh.computeAreaOfFaces();
    // Calcul de l'approximation de la courbure gaussienne à chaque sommet du maillage
    surface_mesh.computeGaussianCurvature();
    // Exportation du maillage avec un code couleur associé aux courbures gaussiennes de chaque sommet dans un fichier OBJ
    surface_mesh.exportGaussianCurvatureAsOBJ("../gaussian_curvature_decimated.obj", false);

    return 0;
}
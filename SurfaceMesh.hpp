#ifndef SURFACEMESH_HPP
#define SURFACEMESH_HPP

#include <string>
#include <vector>
#include <map>
#include <tuple>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_count_ratio_stop_predicate.h>
#include <CGAL/boost/graph/generators.h>

//Simplification des noms des classes CGAL
typedef CGAL::Simple_cartesian<double>          Kernel;
typedef Kernel::Point_3                         Point_3;
typedef CGAL::Surface_mesh<Point_3>             Surface_mesh;
typedef Surface_mesh::Vertex_index              vertex_descriptor;
typedef Surface_mesh::Face_index                face_descriptor;
typedef Surface_mesh::Edge_index                edge_descriptor;
typedef Surface_mesh::Halfedge_index            halfedge_descriptor;
typedef Kernel::Vector_3                        Vector_3;

class SurfaceMesh{
    public:
    SurfaceMesh(const std::string& filepath, float decimation_factor); // Constructeur de la classe permettant de construire un objet de type Surface_mesh
    void displaySurfaceMeshInfos() const; // Affichage du nombre de sommets et de faces de l'attribut de la classe Surface_mesh
    std::tuple<std::map<vertex_descriptor, int>, int> computeVerticesValency(); // Calcul de la valence des sommets du maillage (retourne un tuple contenant la table associant à chaque sommet sa valence correspondante, ainsi que la valeur de valence maximum)
    void displayValencyInfos(const std::map<vertex_descriptor, int>& vertex_valency) const; // Affiche dans la console le nom des sommets du maillage avec la valence qui leur est associée (pour débuggage)
    void exportVerticesValencyAsCSV(const std::tuple<std::map<vertex_descriptor, int>, int>& data, const std::string csvFilename); // Exportation des valences des sommets du maillage au format CSV
    std::tuple<std::map<face_descriptor, std::vector<double>>, std::map<face_descriptor, Vector_3>> computeDihedralAngles(); // Calcul des angles dièdres entre les faces adjacentes entre elles
    void displayDihedralAnglesInfos(const std::map<face_descriptor, std::vector<double>>& dihedral_angles) const; // Affiche dans la console le nom des faces du maillage et, pour chacune d'entre elles, la liste des faces d'indice plus elevé qui lui sont associées ainsi que la valeur de l'angle dièdre (en degré) entre ces deux faces
    void exportDihedralAnglesAsCSV(std::map<face_descriptor, std::vector<double>>& dihedral_angles, const std::string csvFilename); // Exportation des valeurs des angles dièdres en fonction du nombre d'occurrences au format CSV
    std::map<face_descriptor, double> computeFaceArea(std::map<face_descriptor, Vector_3>& face_normal); // Calcul de l'aire des faces du maillage
    void displayFaceAreaInfos(const std::map<face_descriptor, double>& face_area); // Affiche la l'identifiant des faces du maillage et l'aire qui leur est associée
    std::map<vertex_descriptor, double> computeGaussianCurvature(std::map<face_descriptor, double>& face_area, std::map<face_descriptor, Vector_3>& face_normal); // Calcul de l'approximation de la courbure gaussienne à chaque sommet du maillage
    void exportGaussianCurvatureAsOBJ(std::map<vertex_descriptor, double>& vertex_gaussian_curvature, const std::string objFilename, bool is_decimated); // Exportation du maillage avec un code couleur associé aux courbures gaussiennes de chaque sommet dans un fichier OBJ
    void triangulated_surface_mesh_simplification(); // Application de l'algorithme de décimation sur la surface mesh contenue dans l'attribut m_surface_mesh
    std::map<vertex_descriptor, int> getIndicesRemapping(); // Permet de redéfinir les indices associés aux sommets de la surface mesh après l'algorithme de décimation
    void readPlyFile(const std::string& filepath, bool preload_into_memory); // Lecture d'un fichier au format .ply en utilisant tinyply
    void readObjFile(const std::string& filepath); // Lecture d'un fichier au format .obj

    private:
    // Objet CGAL construit par la classe
    Surface_mesh m_surface_mesh;
    // Facteur de décimation utilisé dans l'algorithme de décimation du maillage
    float m_decimation_factor;
    // Nom du maillage
    std::string m_mesh_name;
    // Valence maximum et minimum des sommets du maillage (utilisés pour le fichier CSV)
    //int m_min_valency;
    //int m_max_valency;
    // Propriétés associées au maillage créé
    //std::map<face_descriptor, std::vector<vertex_descriptor>> m_face_vertices; // Associe à chaque face du maillage les sommets lui appartenant
    //std::map<vertex_descriptor, int> m_vertex_valency; // Associe à chaque sommet du maillage la valence correspondante
    //std::map<face_descriptor, Vector_3> m_face_normal; // Associe à chaque face du maillage sa normale correspondante
    //std::map<face_descriptor, std::vector<face_descriptor>> m_adjacent_faces; // Associe à chaque face du maillage la liste des faces (avec un indice plus élevé) qui lui sont adjacentes
    //std::map<face_descriptor, std::vector<double>> m_dihedral_angles; // Associe à chaque face du maillage la liste des angles dièdres entre la face courante et les faces qui lui sont adjacentes
    //std::map<face_descriptor, double> m_face_area; // Associe à chaque face du maillage son aire correspondante
    //std::map<vertex_descriptor, double> m_vertex_gaussian_curvature; // Associe à chaque sommet du maillage une approximation de sa courbure gaussienne
};

#endif
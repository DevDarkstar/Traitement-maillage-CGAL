#include "SurfaceMesh.hpp"
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <iterator>
#include <cmath>
#include <chrono>
#include "tinyply.h"
#include "plyUtils.hpp"

using namespace tinyply;
namespace SMS = CGAL::Surface_mesh_simplification;

SurfaceMesh::SurfaceMesh(const std::string& filepath, float decimation_factor) : m_decimation_factor(decimation_factor) {
    //Récupération de l'extension du fichier à lire
    std::string extension = filepath.substr(filepath.find_last_of('.') + 1);
    //Si le fichier a pour extension .obj
    if(extension == "obj"){
        this->readObjFile(filepath);
    }
    //sinon si le fichier a pour extension .ply
    else if(extension == "ply"){
        this->readPlyFile(filepath, true);
    }
    //sinon nous levons une exception
    else{
        throw std::runtime_error("Le fichier lu doit avoir pour extension .obj ou .ply...");
    }
}

void SurfaceMesh::displaySurfaceMeshInfos() const {
    // Affichage du nombre de sommets et de faces du maillage
    // Première solution en utilisant les fonctions number_of_vertices et number_of_faces
    std::cout << "Nombre de sommets de de faces du maillage :" << std::endl;
    std::cout << "Nombres de sommets : " << this->m_surface_mesh.number_of_vertices() << std::endl;
    std::cout << "Nombres de faces : " << this->m_surface_mesh.number_of_faces() << std::endl;
    std::cout << std::endl;

    // Deuxième solution en récupérant la taille des tableaux contenant les sommets (fonction vertices() de Surface_mesh)
    // et les faces (fonction faces() de Surface_mesh)
    // std::cout << "Nombre de sommets de de faces en utilisant les tailles des tableaux de sommets et de faces :" << std::endl;
    // std::cout << "Nombres de sommets : " << m_surface_mesh.vertices().size() << std::endl;
    // std::cout << "Nombres de faces : " << m_surface_mesh.faces().size() << std::endl;
}

std::tuple<std::map<vertex_descriptor, int>, int> SurfaceMesh::computeVerticesValency() {
    // Calcul de la valence de chaque sommet du maillage. 
    //m_vertex_valency.clear();
    // Création d'une table associant à un sommet du maillage sa valence
    std::map<vertex_descriptor, int> vertex_valency;
    // Initialisation de la map
    for(const auto& vertex : this->m_surface_mesh.vertices()){
        vertex_valency[vertex] = 0;
    }
    // Ainsi qu'une variable contenant la valence maximale dans le maillage (utilisées pour la création du fichier CSV)
    int max_valency = -10000;
    // La valence pour un sommet donné correspond au nombre
    // de sommets qui sont directement voisins de ce sommet. Une autre façon de définir la valence d'un sommet va être de calculer le
    // nombre d'arêtes possédant ce sommet comme membre. C'est la stratégie utilisée ici.
    // Nous commençons par parcourir les arêtes du maillage
    for (const auto& edge : m_surface_mesh.edges()) {
        // récupération d'une demi-arête associée à cette arête
        halfedge_descriptor he = m_surface_mesh.halfedge(edge);
        // récupération des deux sommets associés à cette demi-arête
        vertex_descriptor v1 = m_surface_mesh.source(he);
        vertex_descriptor v2 = m_surface_mesh.target(he);

        // Incrémentation de la valence de ces deux sommets dans la propriété du maillage sur les valences et mise à jour de la valeur de valence maximale
        max_valency = std::max(max_valency, vertex_valency[v1]++);
        max_valency = std::max(max_valency, vertex_valency[v2]++);
    }

    return std::make_tuple(vertex_valency, max_valency);
}

void SurfaceMesh::displayValencyInfos(const std::map<vertex_descriptor, int>& vertex_valency) const {
    // Affichage des valences des sommets du maillage
    for (const auto& p : vertex_valency) {
        std::cout << "Valence de " << p.first << " : " << p.second << std::endl;
    }
}

void SurfaceMesh::exportVerticesValencyAsCSV(const std::tuple<std::map<vertex_descriptor, int>, int>& data, const std::string csvFileName) {
    // Comme l'objectif est de pouvoir exporter les données de sorte à pouvoir réaliser un histogramme, nous allons préparer ces dernières
    // de sorte à pouvoir afficher un histogramme représentant le nombre de sommets en fonction de leur valence
    // Extraction des données contenues dans le tuple
    std::map<vertex_descriptor, int> vertex_valency;
    int max_valency;

    std::tie(vertex_valency, max_valency) = data;

    // Création d'un tableau contenant pour chaque indice i, le nombre de sommets ayant une valence égale à i
    std::vector<int> valency_data (max_valency + 1, 0);

    // Nous parcourons la table associant un sommet du maillage à sa valence
    for (const auto& p : vertex_valency) {
        // incrémentation du nombre de sommets avec la valence obtenue à l'indice i de vertex_valency
        valency_data[p.second]++;
    }

    // Création du fichier d'exportation
    // Ouverture du fichier en mode écriture
    std::ofstream csvOutputFile(csvFileName);

    // Si le fichier ne s'est pas créé correctement
    if (!csvOutputFile.is_open()) {
        throw std::runtime_error("Impossible d'ouvrir le fichier CSV.");
    // sinon il est ouvert et nous pouvons écrire à l'intérieur
    } else {
        // Nous commençons par écrire l'en-tête de ce fichier à savoir le titre des deux colonnes des données à exporter (valence et nombre de sommets)
        csvOutputFile << "Valence,Nombre de sommets\n";
        // Remplissage du fichier avec les données de valence
        for (int i = 0; i < valency_data.size(); i++) {
            //std::cout << "test" << std::endl;
            csvOutputFile << i << "," << valency_data[i] << "\n";
        }

        // Fermeture du fichier CSV
        csvOutputFile.close();
        std::cout << "Exportation des données de valence réussie." << std::endl;
    }
}

std::tuple<std::map<face_descriptor, std::vector<double>>, std::map<face_descriptor, Vector_3>> SurfaceMesh::computeDihedralAngles() {
    // Nous allons avoir besoin pour le calcul des angles dièdres entre faces adjacentes des coordonnées des sommets du maillage (contenue dans la structure de données
    // interne de la Surface Mesh de CGAL) ainsi que de la liste des sommets associés à une face (table de propriété m_face_vertices de la classe)
    // Récupération de la table de propriété des coordonnées des sommets du maillage
    Surface_mesh::Property_map<vertex_descriptor, Point_3> vertices_coordinates = this->m_surface_mesh.property_map<vertex_descriptor, Point_3>("v:point").first;

    // Les valeurs des angles dièdres (calculés ici en degré), seront stockées dans la table suivante où à chaque face seront associées les valeurs des angles diédres obtenus
    // entre la face courante et les faces adjacentes.
    // Afin d'éviter les doublons, la stratégie va être d'associer à chaque face les valeurs des angles dièdres entre cette face et ses faces adjacentes avec un indice
    // strictement supérieur
    std::map<face_descriptor, std::vector<double>> dihedral_angles;
    // Nous créons également une autre table associant à une face donnée sa normale stockée sous la forme d'un Vector_3
    std::map<face_descriptor, Vector_3> face_normal;

    // Remplissage de la table des normales aux faces en associant à chaque face du maillage le vecteur normal (de type Vector_3) correspondant
    for (const auto& face : this->m_surface_mesh.faces()) {
        // Récupération de la liste des coordonnées des sommets associés à cette face
        std::vector<Point_3> vertex_coordinates;
        for(const auto& vertex : vertices_around_face(this->m_surface_mesh.halfedge(face), this->m_surface_mesh)){
            vertex_coordinates.push_back(vertices_coordinates[vertex]);
        }
        //const std::vector<vertex_descriptor>& vertices = this->m_face_vertices[face];

        // Calcul de deux vecteurs de cette face
        Vector_3 v1 = vertex_coordinates[1] - vertex_coordinates[0];
        Vector_3 v2 = vertex_coordinates[2] - vertex_coordinates[0];

        // Calcul du vecteur normal à cette face
        Vector_3 normal = CGAL::cross_product(v1, v2);

        // Stockage de la normale à cette face dans la table de propriété associée
        face_normal[face] = normal;
    }

    // Calcul des angles dièdres entre faces adjacentes
    // A chaque face, sera associé la liste des angles dièdres des faces, ayant un indice strictement plus élevé à cette dernière, qui lui sont adjacentes

    // Remplissage de la table de propriété
    // Parcours des faces du maillage
    for(const auto& face : this->m_surface_mesh.faces()) {
        // Récupération du vecteur normal à la face courante
        Vector_3 normal = face_normal[face];
        // Calcul de la norme de ce vecteur
        double norm = CGAL::sqrt(normal.squared_length());

        // Puis, parcours des faces adjacentes à la face courante en utilisant ses demi-arêtes
        for(const auto& adjacent_face : faces_around_face(this->m_surface_mesh.halfedge(face), this->m_surface_mesh))
        {
            // Si la face adjacente a un indice strictement supérieur à la face courante
            if(adjacent_face.idx() > face.idx())
            {
                // Récupération de la normale de la face adjacente
                Vector_3 adjacent_normal = face_normal[adjacent_face];
                // Calcul de la norme de ce vecteur
                double adjacent_norm = CGAL::sqrt(adjacent_normal.squared_length());
                // Calcul de l'angle dièdre entre cette face et la face courante en degrés (compris entre 0 et 180°)
                double dihedral_angle = std::acos((normal * adjacent_normal) / (norm * adjacent_norm)) * 180 / CGAL_PI;
                // Ajout de l'angle dièdre à la table de propriété
                dihedral_angles[face].push_back(dihedral_angle);
            }
        }
    }

    return std::make_tuple(dihedral_angles, face_normal);
}

void SurfaceMesh::displayDihedralAnglesInfos(const std::map<face_descriptor, std::vector<double>>& dihedral_angles) const {
    // Affichage des angles dièdres des faces adjacentes
    for(const auto& p : dihedral_angles){
        std::cout << "Valeur des angles dièdres des faces adjacentes à " << p.first << " ";
        for(const auto& angle : p.second){
            std::cout << angle << "°, ";
        }
        std::cout << std::endl;
    }
}

void SurfaceMesh::exportDihedralAnglesAsCSV(std::map<face_descriptor, std::vector<double>>& dihedral_angles, const std::string csvFileName) {
    // Nous allons nous servir pour créer le fichier CSV de la table de propriété de la classe m_dihedral_angles contenant l'ensemble des angles dièdres entre faces adjacentes

    // Comme l'objectif est de pouvoir exporter les données de sorte à pouvoir réaliser un histogramme, nous allons préparer ces dernières
    // de sorte à pouvoir afficher un histogramme représentant le nombre d'occurences ayant une certaine valeur d'angle
    // De plus, afin de limiter la taille de l'histogramme, nous allons arrondir les valeurs d'angles obtenues à l'entier le plus proche
    // Création d'un tableau contenant les descriptions d'intervalles et qui seront utilisés pour créer le fichier csv
    std::vector<std::string> intervals = {"[0;5[","[5;10[","[10;15[","[15;20[","[20;25[","[25;30[","[30;35[","[35;40[","[40;45[","[45;50[","[50;55[","[55;60[","[60;65[","[65;70[","[70;75[","[75;80[","[80;85[","[85;90[",
    "[90;95[","[95;100[","[100;105[","[105;110[","[110;115[","[115;120[","[120;125[","[125;130[","[130;135[","[135;140[","[140;145[","[145;150[","[150;155[","[155;160[","[160;165[","[165;170[","[170;175[","[175;180]"};
    // Création d'un tableau où, pour un indice i, est associé le nombre de faces possédant un angle dièdre avec une face adjacente compris dans un intervalle situé à l'indice i dans le tableau intervals ci-dessus 
    std::vector<int> dihedral_angles_data(intervals.size(), 0);

    //Initialisation des valeurs pour les données des angles dièdres
    for(int i = 0; i < intervals.size(); i++){
        dihedral_angles_data[i] = 0;
    }

    // Nous parcourons la liste des faces du maillage
    for (const auto& face : this->m_surface_mesh.faces()) {
        // récupération de la liste des angles dièdres associés à cette face
        std::vector<double> angles = dihedral_angles[face];
        // Parcours de la liste de ces angles
        for(int i = 0; i < angles.size(); i++) {
            // arrondissement de la valeur de l'angle dièdre à l'entier le plus proche
            int rounded_angle = static_cast<int>(std::round(angles[i]));
            // incrémentation du nombre d'occurrences avec cette valeur d'angle dans l'intervalle correspondant
            dihedral_angles_data[rounded_angle / 5]++;
        }     
    }

    // Création du fichier d'exportation
    // Ouverture du fichier en mode écriture
    std::ofstream csvOutputFile(csvFileName);

    // Si le fichier ne s'est pas créé correctement
    if (!csvOutputFile.is_open()) {
        throw std::runtime_error("Impossible d'ouvrir le fichier CSV.");
    // sinon il est ouvert et nous pouvons écrire à l'intérieur
    } else {
        // Nous commençons par écrire l'en-tête de ce fichier à savoir le titre des deux colonnes des données à exporter (angle dièdre et nombre d'occurrences)
        csvOutputFile << "Intervalle angle dièdre,Nombre d'occurrences\n";
        // Remplissage du fichier avec les données des angles dièdres
        std::vector<std::string>::iterator it_keys = intervals.begin();
        std::vector<int>::iterator it_values = dihedral_angles_data.begin();
        for(; it_keys != intervals.end() && it_values != dihedral_angles_data.end(); ++it_keys, ++it_values) {
            csvOutputFile << *it_keys << "," << *it_values << "\n";
        }

        // Fermeture du fichier CSV
        csvOutputFile.close();
        std::cout << "Exportation des données des angles dièdres réussie." << std::endl;
    }
}

std::map<face_descriptor, double> SurfaceMesh::computeFaceArea(std::map<face_descriptor, Vector_3>& face_normal) {
    // Création d'une table associant à une face son aire
    std::map<face_descriptor, double> face_area;

    // Calcul de l'aire des faces (le résultat sera stocké dans la table de propriété face_area)
    // Parcours de la liste des faces du maillage
    for (const auto& face : m_surface_mesh.faces()) {
        // L'aire d'une face se calcule en faisant la norme du produit vectoriel de deux vecteurs appartenant à cette face, le tout multiplié par 1/2
        // Autrement dit, nous pouvons également la calculer en faisant la norme du produit vectoriel du vecteur normal à la face, le tout multiplié par 1/2
        // Aire = 1 / 2 * || v1 ^ v2 || = 1 / 2 * || N || où N = v1 ^ v2
        // Récupération de la normale à la face courante
        Vector_3 normale = face_normal[face];

        // Calcul et stockage de l'aire de la face dans la table face_area
        face_area[face] = 0.5 * CGAL::sqrt(normale.squared_length());
    }
    return face_area;
}

void SurfaceMesh::displayFaceAreaInfos(const std::map<face_descriptor, double>& face_area) {
    // Affichage de l'aire des faces du maillage
    for(const auto& p : face_area) {
        std::cout << "Face " << p.first << " : aire = " << p.second << " m²" << std::endl;
    }
}

std::map<vertex_descriptor, double> SurfaceMesh::computeGaussianCurvature(std::map<face_descriptor, double>& face_area, std::map<face_descriptor, Vector_3>& face_normal) {
    //m_vertex_gaussian_curvature.clear();
    // La technique utilisée ici est une approche en approximant la courbure gaussienne de chaque sommet du maillage en utilisant 
    // les normales et les aires des faces
    // Comme nous allons utiliser l'aire des faces dans ce calcul et que le résultat de la courbure y est directement lié,
    // nous devons donc normaliser les courbures gaussiennes résultantes afin de pouvoir généraliser le code couleur de la carte de courbure gaussienne
    // à des maillages de tailles diverses
    std::map<vertex_descriptor, double> vertex_gaussian_curvature;

    // Calcul des approximations de courbure gaussienne (le résultat sera stocké dans la table de propriété vertex_gaussian_curvature)
    // Parcours de l'ensemble des sommets du maillage 
    for (const auto& vertex : this->m_surface_mesh.vertices()) {
        /*if(m_surface_mesh.is_border(vertex)){
            std::cout << "Sommet de bord" << std::endl;
        } else {
            std::cout << "Pas sommet de bord" << std::endl;
        }*/
        // Création d'une variable pour la somme des angles entre normales
        double angle_sum = 0.0;
        // Création d'une variable pour la somme des aires des faces ayant ce sommet comme membre
        double area_sum = 0.0;
        // Récupération d'une demi-arête ayant ce sommet comme target
        halfedge_descriptor he_target = this->m_surface_mesh.halfedge(vertex);
        // Parcours de l'ensemble des demi-arêtes autour de ce sommet
        for (const auto& he : this->m_surface_mesh.halfedges_around_target(he_target)) {
            // Si l'arête contenant cette demi-arête n'est pas une arête de bord du maillage
            if (!this->m_surface_mesh.is_border(this->m_surface_mesh.edge(he))) {
                // Récupération des deux faces ayant cette arête en commun
                face_descriptor f1 = this->m_surface_mesh.face(he);
                face_descriptor f2 = this->m_surface_mesh.face(this->m_surface_mesh.opposite(he));

                // Ajout de l'aire de la deuxième face à la somme des aires
                area_sum += face_area[f1];

                // Récupération des normales à ces deux faces
                Vector_3 n1 = face_normal[f1];
                Vector_3 n2 = face_normal[f2];
                // Calcul de la norme de ces deux vecteurs
                double n1_norm = CGAL::sqrt(n1.squared_length());
                double n2_norm = CGAL::sqrt(n2.squared_length());
                // Calcul de l'angle entre les deux normales en radian
                double angle = std::acos((n1 * n2) / (n1_norm * n2_norm));
                // Ajout du résultat à la somme des angles obtenus
                angle_sum += angle;
            }
            else {
                face_descriptor face = this->m_surface_mesh.face(this->m_surface_mesh.opposite(he));
                area_sum += face_area[face];
            }
        }
        //std::cout << vertex << " somme des angles : " << angle_sum << ", somme des aires : " << area_sum << std::endl;
        // Calcul de l'approximation de la courbure gaussienne pour ce sommet
        double gaussian_curvature = (2 * CGAL_PI - angle_sum) / area_sum;
        // et ajout du résultat à la table de propriété
        if(std::isnan(gaussian_curvature)) {
            gaussian_curvature = 0.0;
        }
        vertex_gaussian_curvature[vertex] = gaussian_curvature;
    }

    // Création d'une variable contenant la somme des courbures gaussiennes des sommets
    double sum = 0.0;

    for (const auto& vertex : this->m_surface_mesh.vertices()) {
        sum += vertex_gaussian_curvature[vertex];
    }

    double average_area = sum / this->m_surface_mesh.number_of_faces();

    // Normalisation des valeurs 
    for (const auto& vertex : this->m_surface_mesh.vertices()) {
        vertex_gaussian_curvature[vertex] /= average_area;
    }

    // Affichage des résultats
    /*for (vertex_descriptor vertex : m_surface_mesh.vertices()) {
        std::cout << "Sommet " << vertex << ", courbure gaussienne: " << m_vertex_gaussian_curvature[vertex] << std::endl;
    }*/
    return vertex_gaussian_curvature;
}

void SurfaceMesh::exportGaussianCurvatureAsOBJ(std::map<vertex_descriptor, double>& vertex_gaussian_curvature, const std::string objFileName, bool is_decimated) {
    // Construction d'un fichier obj stockant à la fois les coordonnées des sommets et les couleurs de chacun d'entre eux
    // Sur Blender par exemple, les couleurs des sommets peuvent être voir en passant en mode "Vertex Paint"
    
    // Récupération de la table de propriété des coordonnées des sommets du maillage
    Surface_mesh::Property_map<vertex_descriptor, Point_3> vertices_coordinates = m_surface_mesh.property_map<vertex_descriptor, Point_3>("v:point").first;
    // Création du fichier d'exportation
    // Ouverture du fichier en mode écriture
    std::ofstream objOutputFile(objFileName);

    // Si le fichier ne s'est pas créé correctement
    if (!objOutputFile.is_open()) {
        throw std::runtime_error("Impossible d'ouvrir le fichier OBJ.");
    // sinon il est ouvert et nous pouvons écrire à l'intérieur
    } else {
        // Nous commençons par écrire l'en-tête de ce fichier
        objOutputFile << "# OBJ file generated with gaussian curvature infos\n# https://github.com/DevDarkstar/Traitement-maillage-CGAL\no Mesh\n";
        // Remplissage du fichier avec les coordonnées des sommets du maillage ainsi que les couleurs déterminées selon la courbure gaussienne
        for (vertex_descriptor v : m_surface_mesh.vertices()) {
            // Récupération des coordonnées du sommet
            Point_3 coordinates = vertices_coordinates[v];
            // écriture des coordonnées dans le fichier OBJ
            objOutputFile << "v " << coordinates.x() << " " << coordinates.y() << " " << coordinates.z() << " ";

            // Récupération de la courbure gaussienne de ce sommet
            double gaussian_curvature = vertex_gaussian_curvature[v];
            // et stockage de la couleur associée à ce sommet en angle de la courbure
            // Si la valeur est supérieur strictement à 0.9, le sommet sera blanc
            if (gaussian_curvature > 5) {
                objOutputFile << 1 << " " << 1 << " " << 1 << "\n";
            }
            // Sinon si la valeur est supérieur strictement à 0.8 et le sommet sera de couleur magenta
            else if (gaussian_curvature > 3) {
                objOutputFile << 1 << " " << 0 << " " << 1 << "\n";
            }
            // Sinon si la valeur est supérieur strictement à 0.7 et le sommet sera de couleur rouge
            else if (gaussian_curvature > 1.5) {
                objOutputFile << 1 << " " << 0 << " " << 0 << "\n";
            }
            // Sinon si la valeur est supérieur strictement à 0.6 et le sommet sera de couleur orange
            else if (gaussian_curvature > 1) {
                objOutputFile << 1 << " " << 0.5 << " " << 0 << "\n";
            }
            // Sinon si la valeur est supérieur strictement à 0.5 et le sommet sera de couleur jaune
            else if (gaussian_curvature > 0.75) {
                objOutputFile << 1 << " " << 1 << " " << 0 << "\n";
            }
            // Sinon si la valeur est supérieur strictement à 0.4 et le sommet sera de couleur vert clair
            else if (gaussian_curvature > 0.5) {
                objOutputFile << 0.5 << " " << 1 << " " << 0 << "\n";
            }
            // Sinon si la valeur est supérieur strictement à 0.3 et le sommet sera de couleur verte
            else if (gaussian_curvature > 0.25) {
                objOutputFile << 0 << " " << 1 << " " << 0 << "\n";
            }
            // Sinon si la valeur est supérieur strictement à 0.2 et le sommet sera de couleur cyan
            else if (gaussian_curvature > 0.125) {
                objOutputFile << 0 << " " << 1 << " " << 1 << "\n";
            }
            // Sinon si la valeur est supérieur strictement à 0.1 et le sommet sera de couleur bleue
            else if (gaussian_curvature > 0.075) {
                objOutputFile << 0 << " " << 0 << " " << 1 << "\n";
            }
            // Sinon le sommet sera de couleur noire
            else {
                objOutputFile << 0 << " " << 0 << " " << 0 << "\n";
            }
        }
        // Si nous utilisons les indices des sommets contenus dans la surface mesh (cas avant l'algorithme de décimation)
        if (!is_decimated) {
            // Remplissage du fichier obj avec les indices des sommets des faces du maillage
            for (face_descriptor f : m_surface_mesh.faces()) {
                // Récupération des sommets composants cette face
                //std::vector<vertex_descriptor> vertices = m_face_vertices[f];
                objOutputFile << "f ";
                // Parcours des sommets appartenant à la face
                for(const auto& vertex : vertices_around_face(this->m_surface_mesh.halfedge(f), this->m_surface_mesh)){
                    // Et écriture de leur indice dans le fichier
                    objOutputFile << vertex.idx() + 1 << " ";
                }             
                objOutputFile << "\n";
            }
        }
        // Sinon nous utilisons un autre système d'indices des sommets (cas après l'algorithme de décimation)
        else {
            // Récupération des nouveaux indices des sommets restants après l'algorithme de décimation
            std::map<vertex_descriptor, int> vertex_indices = this->getIndicesRemapping();

            for(face_descriptor f : m_surface_mesh.faces()) {
                // Et écriture de leur indice dans le fichier en utilisant les nouvelles valeurs contenues dans vertex_indices
                objOutputFile << "f ";
                // Pour ce faire, nous reparcourons les sommets composant une face en utilisant cette fois le méthode vertices_around_face
                for(vertex_descriptor v : vertices_around_face(m_surface_mesh.halfedge(f), m_surface_mesh)) {
                    objOutputFile << vertex_indices[v] << " ";
                }
                objOutputFile << "\n";
            }
        }

        // Fermeture du fichier OBJ
        objOutputFile.close();
        std::cout << "Création du fichier OBJ relatif aux courbures gaussiennes réussie." << std::endl;
    }
}

void SurfaceMesh::triangulated_surface_mesh_simplification(){

    if(!CGAL::is_triangle_mesh(m_surface_mesh))
    {
        throw std::runtime_error("Le maillage n'est pas composé que de faces triangulaires...");
    }

    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    // In this example, the simplification stops when the number of undirected edges
    // drops below 10% of the initial count
    SMS::Edge_count_ratio_stop_predicate<Surface_mesh> stop(m_decimation_factor);
    int r = SMS::edge_collapse(m_surface_mesh, stop);
    std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
    std::cout << "\nFinished!\n" << r << " edges removed.\n" << m_surface_mesh.number_of_edges() << " final edges.\n";
    std::cout << "Time elapsed: " << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() << "ms" << std::endl;
    std::cout << std::endl;
}

std::map<vertex_descriptor, int> SurfaceMesh::getIndicesRemapping(){
    std::map<vertex_descriptor, int> remapping;

    // Pour ré-indexer le maillage, nous allons parcourir l'ensemble des sommets du maillage
    // et associer pour chacun d'entre eux un indice qui ira de 1 au nombre de sommets restants du maillage
    int new_index = 1;
    for (vertex_descriptor v : m_surface_mesh.vertices()) {
        remapping[v] = new_index++;
    }

    return remapping;
}

//Lecture d'un fichier PLY dont le nom est passé en paramètre
void SurfaceMesh::readPlyFile(const std::string& filepath, bool preload_into_memory)
{
    std::unique_ptr<std::istream> file_stream;
    std::vector<uint8_t> byte_buffer;

    try
    {
        // For most files < 1gb, pre-loading the entire file upfront and wrapping it into a 
        // stream is a net win for parsing speed, about 40% faster.
        //Le preloading permet de stocker l'ensemble du fichier en RAM pour aller plus vite. A priori pour les fichiers < 1Go
        if (preload_into_memory)
        {
            byte_buffer = read_file_binary(filepath);
            file_stream.reset(new memory_stream((char*)byte_buffer.data(), byte_buffer.size()));
        }
        else
        {
            file_stream.reset(new std::ifstream(filepath, std::ios::binary));
        }

		//Si l'ouverture ne s'est pas réalisée
        if (!file_stream || file_stream->fail()) throw std::runtime_error("file_stream failed to open " + filepath);

        file_stream->seekg(0, std::ios::end);
        const float size_mb = file_stream->tellg() * float(1e-6);
        file_stream->seekg(0, std::ios::beg);

        PlyFile file;
        file.parse_header(*file_stream);

        // Because most people have their own mesh types, tinyply treats parsed data as structured/typed byte buffers. 
        // See examples below on how to marry your own application-specific data structures with this one. 
        std::shared_ptr<PlyData> vertices, normals, colors, texcoords, faces, tripstrip;
        
        //Parmi toutes les données lues, nous ne conserverons que les coordonnées des sommets et les indices des sommets des faces
        //Préparation des structures de sommets {x,y,z}
        try { vertices = file.request_properties_from_element("vertex", { "x", "y", "z" }); }
        catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

		//Pour les faces (indices de sommets)
        try { faces = file.request_properties_from_element("face", { "vertex_indices" }, 3); }
        catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }
        
		//Lecture du fichier
		file.read(*file_stream);

        //Affichage du nombre d'éléments
        if (vertices)   std::cout << "\tRead " << vertices->count  << " total vertices "<< std::endl;
        //if (normals)    std::cout << "\tRead " << normals->count   << " total vertex normals " << std::endl;
        if (colors)     std::cout << "\tRead " << colors->count << " total vertex colors " << std::endl;
        //if (texcoords)  std::cout << "\tRead " << texcoords->count << " total vertex texcoords " << std::endl;
        if (faces)      std::cout << "\tRead " << faces->count     << " total faces (triangles) " << std::endl;
        //if (tripstrip)  std::cout << "\tRead " << (tripstrip->buffer.size_bytes() / tinyply::PropertyTable[tripstrip->t].stride) << " total indicies (tristrip) " << std::endl;

        //Lecture des coordonnées dees sommets du maillage
        const size_t numVerticesBytes = vertices->buffer.size_bytes();
        std::vector<float3> verts(vertices->count);
        std::memcpy(verts.data(), vertices->buffer.get(), numVerticesBytes);

        //création d'un tableau de vertex_descriptor qui va contenir les informations des coordonnées des sommets du maillage
        std::vector<vertex_descriptor> vertices_descriptor;
        //création d'un tableau qui va contenir les indices des sommets par face
        std::vector<std::array<int,3>> face_indices;
        //Création des sommets du maillage 
        for (const auto& vertex: verts)
        {
            // ajout du sommet au maillage de notre Surface Mesh dont les coordonnées seront représentées par un objet de type Point_3
            // et stockage du vertex_descriptor retourné dans le conteneur servant à construire ultérieurement les faces du maillage
            vertices_descriptor.push_back(m_surface_mesh.add_vertex(Point_3(vertex.x, vertex.y, vertex.z)));
        }
        
        
        //Lecture des indices des sommets par face
        const size_t numFacesBytes = faces->buffer.size_bytes();
        std::vector<uint3> lfaces(faces->count);
        std::memcpy(lfaces.data(), faces->buffer.get(), numFacesBytes);
         
        // Création des faces du maillage par rapport aux indices des sommets récupérés
        for (const auto& lface: lfaces)
        {
            // ajout de la face triangulaire aux données du maillage
            face_descriptor face = m_surface_mesh.add_face(vertices_descriptor[lface.x], vertices_descriptor[lface.y], vertices_descriptor[lface.z]);
            // association de la liste des sommets de la face à la face du maillage correspondante
            //m_face_vertices[face] = {vertices_descriptor[lface.x], vertices_descriptor[lface.y], vertices_descriptor[lface.z]};
        }
    }
    catch (const std::exception & e)
    {
        std::cerr << "Caught tinyply exception: " << e.what() << std::endl;
    }
}

void SurfaceMesh::readObjFile(const std::string& filepath)
{
    //Ouverture du fichier .obj
    std::ifstream objFile(filepath);
    if (!objFile) {
        throw std::runtime_error("Le fichier .obj n'a pas pu être ouvert. Vérifiez le chemin vers votre fichier .obj.");
    }

    //création d'un tableau de vertex_descriptor qui va contenir les informations des coordonnées des sommets du maillage
    std::vector<vertex_descriptor> vertices_descriptor;
    //création d'un tableau qui va contenir les indices des sommets par face
    std::vector<std::array<int,3>> face_indices;

    // Lecture du fichier ligne par ligne
    std::string line;
    while (std::getline(objFile, line)) {
        // création d'un buffer pour y stocker la ligne lue
        std::istringstream buffer(line);
        // création d'un variable pour y stocker le premier caractère de la ligne et nous renseigner quant aux données lues
        // v = sommet et f = face. Les autres données seront ignorées
        std::string id;

        // Lecture du caractère et stockage dans la variable id
        buffer >> id;

        //S'il s'agit d'un sommet
        if (id == "v") {
            // Lire les coordonnées d'un sommet
            // création de trois variables pour y stocker les coordonnées des sommets
            float x, y, z;
            // lecture des coordonnées du sommet
            buffer >> x >> y >> z;
            // ajout du sommet au maillage de notre Surface Mesh dont les coordonnées seront représentées par un objet de type Point_3
            // et stockage du vertex_descriptor retourné dans le conteneur servant à construire ultérieurement les faces du maillage
            vertices_descriptor.push_back(m_surface_mesh.add_vertex(Point_3(x, y, z)));
        }
        // Sinon s'il s'agit d'une face
        else if (id == "f") {
            // Lire les indices d'une face
            // création d'un tableau pour stocker les indices des sommets d'une face
            std::array<int,3> indices;
            // Lecture des indices des sommets des faces
            // Création d'une variable pour y stocker les données liées à un sommet. Il se peut que dans certains fichiers .obj, les données des sommets des faces soient 
            // de cette forme : v/vt/vn (seule la première donnée nous intéresse ici)
            std::string data;
            // Tant qu'il reste des indices à lire
            int i = 0;
            while (buffer >> data) {
                // lecture de l'indice et ajout de ce dernier dans le tableau des indices de la face courante en enlevant 1 à sa valeur car
                // les indices des sommets commencent à 1 dans le fichier obj
                try{
                    indices[i] = std::stoi(data.substr(0, data.find("/"))) - 1;
                }catch(const std::exception& e){
                    std::cout << "Erreur lors de la lecture des indices des faces : " << e.what() << std::endl;
                }
                i++;
            }
            //stockage des indices de cette face dans le tableau face_indices
            face_indices.push_back(indices);
        }
    }

    // Fermeture du fichier .obj
    objFile.close();

    // Création des faces du maillage par rapport aux indices des sommets récupérés
    for (int i = 0; i < face_indices.size(); i++) {
        // récupération du tableau d'indices
        const std::array<int,3>& indices = face_indices[i];
        // ajout de la face triangulaire aux données du maillage
        face_descriptor face = m_surface_mesh.add_face(vertices_descriptor[indices[0]], vertices_descriptor[indices[1]], vertices_descriptor[indices[2]]);
        // association de la liste des sommets de la face à la face du maillage correspondante
        //m_face_vertices[face] = {vertices_descriptor[indices[0]], vertices_descriptor[indices[1]], vertices_descriptor[indices[2]]};
    }
}

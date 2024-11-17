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

SurfaceMesh::SurfaceMesh(const std::string& filepath, float decimation_factor) : m_decimation_factor(decimation_factor), m_min_valency(0), m_max_valency(0) {
    //Récupération de l'extension du fichier à lire
    std::string extension = filepath.substr(filepath.find_last_of('.') + 1);
    //Si le fichier a pour extension .obj
    if(extension == "obj"){
        readObjFile(filepath);
    }
    //sinon si le fichier a pour extension .ply
    else if(extension == "ply"){
        readPlyFile(filepath, true);
    }
    //sinon nous levons une exception
    else{
        throw std::runtime_error("Le fichier lu doit avoir pour extension .obj ou .ply...");
    }
}

void SurfaceMesh::displaySurfaceMeshInfos() {
    // Affichage du nombre de sommets et de faces du maillage
    // Première solution en utilisant les fonctions number_of_vertices et number_of_faces
    std::cout << "Nombre de sommets de de faces du maillage :" << std::endl;
    std::cout << "Nombres de sommets : " << m_surface_mesh.number_of_vertices() << std::endl;
    std::cout << "Nombres de faces : " << m_surface_mesh.number_of_faces() << std::endl;
    std::cout << std::endl;

    // Deuxième solution en récupérant la taille des tableaux contenant les sommets (fonction vertices() de Surface_mesh)
    // et les faces (fonction faces() de Surface_mesh)
    // std::cout << "Nombre de sommets de de faces en utilisant les tailles des tableaux de sommets et de faces :" << std::endl;
    // std::cout << "Nombres de sommets : " << m_surface_mesh.vertices().size() << std::endl;
    // std::cout << "Nombres de faces : " << m_surface_mesh.faces().size() << std::endl;
}

void SurfaceMesh::computeVerticesValency() {
    // Calcul de la valence de chaque sommet du maillage. 
    m_vertex_valency.clear();
    //La valence pour un sommet donné correspond au nombre
    // de sommets qui sont directement voisins de ce sommet. Une autre façon de définir la valence d'un sommet va être de calculer le
    // nombre d'arêtes possédant ce sommet comme membre. C'est la stratégie utilisée ici.
    // Nous commençons par parcourir les arêtes du maillage
    for (edge_descriptor edge : m_surface_mesh.edges()) {
        // récupération d'une demi-arête associée à cette arête
        halfedge_descriptor he = m_surface_mesh.halfedge(edge);
        // récupération des deux sommets associés à cette demi-arête
        vertex_descriptor v1 = m_surface_mesh.source(he);
        vertex_descriptor v2 = m_surface_mesh.target(he);

        // Incrémentation de la valence de ces deux sommets dans la propriété du maillage sur les valences
        m_vertex_valency[v1]++;
        m_vertex_valency[v2]++;
    }
}

void SurfaceMesh::displayValencyInfos() {
    //Affichage des valences des sommets du maillage
    for (vertex_descriptor v : m_surface_mesh.vertices()) {
        std::cout << "Valence de " << v << " : " << m_vertex_valency[v] << std::endl;
    }
}

void SurfaceMesh::exportVerticesValencyAsCSV(const std::string csvFileName) {
    // Nous allons nous servir de la propriété du maillage m_vertex_valency qui associe à chaque sommet du maillage sa valence associée.
    // Comme l'objectif est de pouvoir exporter les données de sorte à pouvoir réaliser un histogramme, nous allons préparer ces dernières
    // de sorte à pouvoir afficher un histogramme représentant le nombre de sommets en fonction de leur valence
    // Création d'une map permettant de stocker ces nouvelles données (la clé représente la valence et la valeur le nombre de sommets ayant cette valence)
    std::map<int, int> valency_data;

    // Si les valeurs minimum et maximum des valences des sommets n'ont pas encore été déterminées
    if (!m_min_valency && !m_max_valency) {
        //Nous commençons par rechercher la plus grande et la plus petite valeur de valence parmi toutes celles obtenues
        int min_valency = 100;
        int max_valency = 0;

        for(vertex_descriptor v : m_surface_mesh.vertices()) {
            min_valency = std::min(m_vertex_valency[v], min_valency);
            max_valency = std::max(m_vertex_valency[v], max_valency);
        }

        // Affectation des min et max obtenus aux attributs de la classe
        m_min_valency = min_valency;
        m_max_valency = max_valency;
    }
    
    //Initialisation des données de valency_data
    for(int i = m_min_valency; i <= m_max_valency; i++) {
        valency_data[i] = 0;
    }

    // Nous parcourons la liste des sommets du maillage
    for (vertex_descriptor v : m_surface_mesh.vertices()) {
        // récupération de la valence associée à ce sommet dans la table de propriété
        int valency = m_vertex_valency[v];
        // incrémentation du nombre de sommets avec cette valence dans la map
        valency_data[valency]++;
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
        for (const auto& [key, value] : valency_data) {
            csvOutputFile << key << "," << value << "\n";
        }

        // Fermeture du fichier CSV
        csvOutputFile.close();
        std::cout << "Exportation des données de valence réussie." << std::endl;
    }
}

void SurfaceMesh::computeDihedralAngles() {
    m_dihedral_angles.clear();
    // Nous allons avoir besoin pour le calcul des angles dièdres entre faces adjacentes des coordonnées des sommets du maillage (contenue dans la structure de données
    // interne de la Surface Mesh de CGAL) ainsi que de la liste des sommets associés à une face (table de propriété m_face_vertices de la classe)
    // Récupération de la table de propriété des coordonnées des sommets du maillage
    Surface_mesh::Property_map<vertex_descriptor, Point_3> vertices_coordinates = m_surface_mesh.property_map<vertex_descriptor, Point_3>("v:point").first;

    // Les valeurs des angles dièdres (calculés ici en degré), seront stockées dans la propriété m_dihedral_angles de la classe

    // Remplissage de la propriété m_face_normal de la classe associant à chaque face du maillage le vecteur normal (de type Vector_3) correspondant
    for (face_descriptor face : m_surface_mesh.faces()) {
        // Récupération de la liste des sommets associés à cette face
        std::vector<vertex_descriptor> vertices = m_face_vertices[face];

        // Calcul de deux vecteurs de cette face
        Vector_3 v1 = vertices_coordinates[vertices[1]] - vertices_coordinates[vertices[0]];
        Vector_3 v2 = vertices_coordinates[vertices[2]] - vertices_coordinates[vertices[0]];

        // Calcul du vecteur normal à cette face
        Vector_3 normal = CGAL::cross_product(v1, v2);

        // Stockage de la normale à cette face dans la table de propriété associée
        m_face_normal[face] = normal;
    }

    // Détermination des faces adjacentes à une face donnée qui seront stockées dans la table de propriété m_adjacent_faces
    m_adjacent_faces.clear();
    // La stratégie ici est de parcourir l'ensemble des demi-arêtes d'une face et de définir les faces adjacentes en prenant les demi-arêtes opposées
    // à ces dernières (fonction de CGAL faces_aroud_face)
    // Parcours des faces du maillage 
    for (face_descriptor current_face : m_surface_mesh.faces()) {
        // Parcours des demi-arêtes de cette face
        for (face_descriptor face : faces_around_face(m_surface_mesh.halfedge(current_face), m_surface_mesh)) {
            if (face > current_face) {
                m_adjacent_faces[current_face].push_back(face);
            }
        }
    }

    // Calcul des angles dièdres entre faces adjacentes (le résultat sera stocké dans la table de propriété m_dihedral_angles de la classe)
    // A chaque face, sera associé la liste des angles dièdres des faces (avec un indice plus élevé) qui lui sont adjacentes

    // Remplissage de la table de propriété
    for(face_descriptor face : m_surface_mesh.faces()) {
        // Récupération de la normale de la face courante
        Vector_3 current_normal = m_face_normal[face];
        // Calcul de la norme de ce vecteur
        double current_norm = CGAL::sqrt(current_normal.squared_length());
        // Récupération des faces adjacentes à celle-ci
        std::vector<face_descriptor> faces = m_adjacent_faces[face];

        for(face_descriptor adjacent_face : faces) {
            // Récupération de la normale de cette face
            Vector_3 normal = m_face_normal[adjacent_face];
            // Calcul de la norme de ce vecteur
            double norm = CGAL::sqrt(normal.squared_length());
            // Calcul de l'angle dièdre entre cette face et la face courante en degrés (compris entre 0 et 180°)
            double dihedral_angle = std::acos((current_normal * normal) / (current_norm * norm)) * 180 / CGAL_PI;
            // Ajout de l'angle dièdre à la table de propriété
            m_dihedral_angles[face].push_back(dihedral_angle);
        } 
    }
}

void SurfaceMesh::displayDihedralAnglesInfos() {
    // Affichage des angles dièdres des faces adjacentes
    for (face_descriptor face : m_surface_mesh.faces()) {
        std::cout << "Faces adjacentes à " << face << " : ";
        std::vector<face_descriptor>::iterator adjacent_face = m_adjacent_faces[face].begin();
        std::vector<double>::iterator dihedral_angle = m_dihedral_angles[face].begin();
        for (; adjacent_face != m_adjacent_faces[face].end() && dihedral_angle != m_dihedral_angles[face].end(); ++adjacent_face, ++dihedral_angle) {
            std::cout << *adjacent_face << " d'angle dièdre : " << *dihedral_angle << "°, ";
        }
        std::cout << std::endl;
    }
}

void SurfaceMesh::exportDihedralAnglesAsCSV(const std::string csvFileName) {
    // Nous allons nous servir pour créer le fichier CSV de la table de propriété de la classe m_dihedral_angles contenant l'ensemble des angles dièdres entre faces adjacentes

    // Comme l'objectif est de pouvoir exporter les données de sorte à pouvoir réaliser un histogramme, nous allons préparer ces dernières
    // de sorte à pouvoir afficher un histogramme représentant le nombre d'occurences ayant une certaine valeur d'angle
    // De plus, afin de limiter la taille de l'histogramme, nous allons arrondir les valeurs d'angles obtenues à l'entier le plus proche
    // Création d'une map permettant de stocker ces nouvelles données (la clé représente un intervalle de valeurs d'angle dièdre et la valeur le nombre d'occurrences trouvées dans cet intervalle)
    std::map<int, int> dihedral_angles_data;
    // Création d'un tableau contenant les descriptions d'intervalles et qui seront utilisés pour créer le fichier csv
    std::vector<std::string> intervals = {"[0;5[","[5;10[","[10;15[","[15;20[","[20;25[","[25;30[","[30;35[","[35;40[","[40;45[","[45;50[","[50;55[","[55;60[","[60;65[","[65;70[","[70;75[","[75;80[","[80;85[","[85;90[",
    "[90;95[","[95;100[","[100;105[","[105;110[","[110;115[","[115;120[","[120;125[","[125;130[","[125;130[","[130;135[","[135;140[","[140;145[","[145;150[","[150;155[","[155;160[","[160;165[","[165;170[","[170;175[","[175;180]"};

    //Initialisation des valeurs pour les données des angles dièdres
    for(int i = 0; i < intervals.size(); i++){
        dihedral_angles_data[i] = 0;
    }

    // Nous parcourons la liste des faces du maillage
    for (face_descriptor face : m_surface_mesh.faces()) {
        // récupération de la liste des angles dièdres associés à cette face
        std::vector<double> angles = m_dihedral_angles[face];
        // Parcours de la liste de ces angles
        for(double angle : angles) {
            // arrondissement de la valeur de l'angle dièdre à l'entier le plus proche
            int rounded_angle = (int)std::round(angle);
            // incrémentation du nombre d'occurrences avec cette valeur d'angle dans l'intervalle correspondant
            if(rounded_angle < 5)
                dihedral_angles_data[0]++;
            else if (rounded_angle < 10)
                dihedral_angles_data[1]++;
            else if (rounded_angle < 15)
                dihedral_angles_data[2]++;
            else if (rounded_angle < 20)
                dihedral_angles_data[3]++;
            else if (rounded_angle < 25)
                dihedral_angles_data[4]++;
            else if (rounded_angle < 30)
                dihedral_angles_data[5]++;
            else if (rounded_angle < 35)
                dihedral_angles_data[6]++;
            else if (rounded_angle < 40)
                dihedral_angles_data[7]++;
            else if (rounded_angle < 45)
                dihedral_angles_data[8]++;
            else if (rounded_angle < 50)
                dihedral_angles_data[9]++;
            else if (rounded_angle < 55)
                dihedral_angles_data[10]++;
            else if (rounded_angle < 60)
                dihedral_angles_data[11]++;
            else if (rounded_angle < 65)
                dihedral_angles_data[12]++;
            else if (rounded_angle < 70)
                dihedral_angles_data[13]++;
            else if (rounded_angle < 75)
                dihedral_angles_data[14]++;
            else if (rounded_angle < 80)
                dihedral_angles_data[15]++;
            else if (rounded_angle < 85)
                dihedral_angles_data[16]++;
            else if (rounded_angle < 90)
                dihedral_angles_data[17]++;
            else if (rounded_angle < 95)
                dihedral_angles_data[18]++;
            else if (rounded_angle < 100)
                dihedral_angles_data[19]++;
            else if (rounded_angle < 105)
                dihedral_angles_data[20]++;
            else if (rounded_angle < 110)
                dihedral_angles_data[21]++;
            else if (rounded_angle < 115)
                dihedral_angles_data[22]++;
            else if (rounded_angle < 120)
                dihedral_angles_data[23]++;
            else if (rounded_angle < 125)
                dihedral_angles_data[24]++;
            else if (rounded_angle < 130)
                dihedral_angles_data[25]++;
            else if (rounded_angle < 135)
                dihedral_angles_data[26]++;
            else if (rounded_angle < 140)
                dihedral_angles_data[27]++;
            else if (rounded_angle < 145)
                dihedral_angles_data[28]++;
            else if (rounded_angle < 150)
                dihedral_angles_data[29]++;
            else if (rounded_angle < 155)
                dihedral_angles_data[30]++;
            else if (rounded_angle < 160)
                dihedral_angles_data[31]++;
            else if (rounded_angle < 165)
                dihedral_angles_data[32]++;
            else if (rounded_angle < 170)
                dihedral_angles_data[33]++;
            else if (rounded_angle < 175)
                dihedral_angles_data[34]++;
            else
                dihedral_angles_data[35]++;
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
        std::map<int, int>::iterator it_values = dihedral_angles_data.begin();
        for(; it_keys != intervals.end() && it_values != dihedral_angles_data.end(); ++it_keys, ++it_values) {
            csvOutputFile << *it_keys << "," << (*it_values).second << "\n";
        }

        // Fermeture du fichier CSV
        csvOutputFile.close();
        std::cout << "Exportation des données des angles dièdres réussie." << std::endl;
    }
}

void SurfaceMesh::computeAreaOfFaces() {
    m_face_area.clear();
    // Nous allons avoir besoin pour le calcul des aires des faces des coordonnées des sommets du maillage (contenue dans la structure de données
    // interne de la Surface Mesh de CGAL) ainsi que de la liste des sommets associés à une face (table de propriété m_face_vertices de la classe)
    // Récupération de la table de propriété des coordonnées des sommets du maillage
    Surface_mesh::Property_map<vertex_descriptor, Point_3> vertices_coordinates = m_surface_mesh.property_map<vertex_descriptor, Point_3>("v:point").first;

    // Calcul de l'aire des faces (le résultat sera stocké dans la table de propriété m_face_area)
    // Parcours de la liste des faces du maillage
    for (face_descriptor face : m_surface_mesh.faces()) {
        // Récupération de la liste des sommets de cette face
        std::vector<vertex_descriptor> vertices = m_face_vertices[face];
        // Si la face est triangulaire
        if (vertices.size() == 3) {
            // Récupération des coordonnées des trois sommets de la face
            Point_3 v1 = vertices_coordinates[vertices[0]];
            Point_3 v2 = vertices_coordinates[vertices[1]];
            Point_3 v3 = vertices_coordinates[vertices[2]];

            // Calcul de l'aire du triangle en utilisant la formule suivante
            //                |     | x1 y1 z1 ||
            // Aire = 1 / 2 * | det | x2 y2 z2 ||
            //                |     | x3 y3 z3 ||

            double area = 0.5 * CGAL::abs(v1.x()*v2.y()*v3.z() - v1.x()*v3.y()*v2.z() - v2.x()*v1.y()*v3.z() + v2.x()*v3.y()*v1.z() + v3.x()*v1.y()*v2.z() - v3.x()*v2.y()*v1.z());

            // Stockage de l'aire de la face résultante dans la table de propriété
            m_face_area[face] = area;
        }
        // Sinon elle est quadrangulaire
        else {
            // Récupération des coordonnées des quatre sommets de la face
            Point_3 v1 = vertices_coordinates[vertices[0]];
            Point_3 v2 = vertices_coordinates[vertices[1]];
            Point_3 v3 = vertices_coordinates[vertices[2]];
            Point_3 v4 = vertices_coordinates[vertices[3]];

            // Calcul de l'aire de la face en la subdivisant en deux faces triangulaires, puis calcul de l'aire des deux triangles
            // L'aire de la face quadrangulaire correspondra à la somme de ces deux aires
            // Première aire en prenant les sommets v1, v2 et v3
            double area1 = 0.5 * CGAL::abs(v1.x()*v2.y()*v3.z() - v1.x()*v3.y()*v2.z() - v2.x()*v1.y()*v3.z() + v2.x()*v3.y()*v1.z() + v3.x()*v1.y()*v2.z() - v3.x()*v2.y()*v1.z());
            // Seconde aire en prenant les sommets v3, v4 et v1
            double area2 = 0.5 * CGAL::abs(v3.x()*v4.y()*v1.z() - v3.x()*v1.y()*v4.z() - v4.x()*v3.y()*v1.z() + v4.x()*v1.y()*v3.z() + v1.x()*v3.y()*v4.z() - v1.x()*v4.y()*v3.z());

            // Stockage de l'aire de la face résultante dans la table de propriété
            m_face_area[face] = area1 + area2;
        }
    }
}

void SurfaceMesh::displayFaceAreaInfos() {
    // Affichage de l'aire des faces du maillage
    for(face_descriptor face : m_surface_mesh.faces()) {
        std::cout << "Face " << face << " : aire = " << m_face_area[face] << " m²" << std::endl;
    }
}

void SurfaceMesh::computeGaussianCurvature() {
    m_vertex_gaussian_curvature.clear();
    // La technique utilisée ici est une approche en approximant la courbure gaussienne de chaque sommet du maillage en utilisant 
    // les normales et les aires des faces
    // Comme nous allons utiliser l'aire des faces dans ce calcul et que le résultat de la courbure y est directement lié,
    // nous devons donc normaliser les courbures gaussiennes résultantes afin de pouvoir généraliser le code couleur de la carte de courbure gaussienne
    // à des maillages de tailles diverses

    // Calcul des approximations de courbure gaussienne (le résultat sera stocké dans la table de propriété m_vertex_gaussian_curvature)
    // Parcours de l'ensemble des sommets du maillage 
    for (vertex_descriptor vertex : m_surface_mesh.vertices()) {
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
        halfedge_descriptor he_target = m_surface_mesh.halfedge(vertex);
        // Parcours de l'ensemble des demi-arêtes autour de ce sommet
        for (halfedge_descriptor he : m_surface_mesh.halfedges_around_target(he_target)) {
            // Si l'arête contenant cette demi-arête n'est pas une arête de bord du maillage
            if (!m_surface_mesh.is_border(m_surface_mesh.edge(he))) {
                // Récupération des deux faces ayant cette arête en commun
                face_descriptor f1 = m_surface_mesh.face(he);
                face_descriptor f2 = m_surface_mesh.face(m_surface_mesh.opposite(he));

                // Ajout de l'aire de la deuxième face à la somme des aires
                area_sum += m_face_area[f1];

                // Récupération des normales à ces deux faces
                Vector_3 n1 = m_face_normal[f1];
                Vector_3 n2 = m_face_normal[f2];
                // Calcul de la norme de ces deux vecteurs
                double n1_norm = CGAL::sqrt(n1.squared_length());
                double n2_norm = CGAL::sqrt(n2.squared_length());
                // Calcul de l'angle entre les deux normales en radian
                double angle = std::acos((n1 * n2) / (n1_norm * n2_norm));
                // Ajout du résultat à la somme des angles obtenus
                angle_sum += angle;
            }
            else {
                face_descriptor face = m_surface_mesh.face(m_surface_mesh.opposite(he));
                area_sum += m_face_area[face];
            }
        }
        //std::cout << vertex << " somme des angles : " << angle_sum << ", somme des aires : " << area_sum << std::endl;
        // Calcul de l'approximation de la courbure gaussienne pour ce sommet
        double gaussian_curvature = (2 * CGAL_PI - angle_sum) / area_sum;
        // et ajout du résultat à la table de propriété
        if(std::isnan(gaussian_curvature)) {
            gaussian_curvature = 0.0;
        }
        m_vertex_gaussian_curvature[vertex] = gaussian_curvature;
    }

    // Création d'une variable contenant la somme des courbures gaussiennes des sommets
    double sum = 0.0;

    for (vertex_descriptor v : m_surface_mesh.vertices()) {
        sum += m_vertex_gaussian_curvature[v];
    }

    double average_area = sum / m_surface_mesh.number_of_faces();

    // Normalisation des valeurs 
    for (vertex_descriptor v : m_surface_mesh.vertices()) {
        m_vertex_gaussian_curvature[v] /= average_area;
    }

    // Affichage des résultats
    /*for (vertex_descriptor vertex : m_surface_mesh.vertices()) {
        std::cout << "Sommet " << vertex << ", courbure gaussienne: " << m_vertex_gaussian_curvature[vertex] << std::endl;
    }*/
}

void SurfaceMesh::exportGaussianCurvatureAsOBJ(const std::string objFileName, bool surface_mesh_indices) {
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
            double gaussian_curvature = m_vertex_gaussian_curvature[v];
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
        if (surface_mesh_indices) {
            // Remplissage du fichier obj avec les indices des sommets des faces du maillage
            for (face_descriptor f : m_surface_mesh.faces()) {
                // Récupération des sommets composants cette face
                std::vector<vertex_descriptor> vertices = m_face_vertices[f];
                // Et écriture de leur indice dans le fichier
                objOutputFile << "f ";
                for (vertex_descriptor v : vertices) {
                    objOutputFile << v.idx() + 1 << " ";
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
            m_face_vertices[face] = {vertices_descriptor[lface.x], vertices_descriptor[lface.y], vertices_descriptor[lface.z]};
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
        m_face_vertices[face] = {vertices_descriptor[indices[0]], vertices_descriptor[indices[1]], vertices_descriptor[indices[2]]};
    }
}

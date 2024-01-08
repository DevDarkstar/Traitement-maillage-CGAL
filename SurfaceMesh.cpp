#include "SurfaceMesh.hpp"

SurfaceMesh::SurfaceMesh(const std::string objFilePath) {
    //Ouverture du fichier .obj
    std::ifstream objFile(objFilePath);
    if (!objFile) {
        throw std::runtime_error("Le fichier .obj n'a pas pu être ouvert. Vérifiez le chemin vers votre fichier .obj.");
    }

    //création d'un tableau de vector_descriptor qui va contenir les informations des coordonnées des sommets du maillage
    std::vector<vertex_descriptor> vertices_descriptor;
    //création d'un tableau qui va contenir les indices des sommets par face
    std::vector<std::vector<int>> face_indices;

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
            vertex_descriptor vertex = m_surface_mesh.add_vertex(Point_3(x, y, z));
            // et ajout du sommet dans le tableau vertices_descriptor qui servira à construire les faces du maillage
            vertices_descriptor.push_back(vertex);
        }
        // Sinon s'il s'agit d'une face
        else if (id == "f") {
            // Lire les indices d'une face
            // création d'un tableau pour stocker les indices des sommets d'une face
            std::vector<int> indices;
            // Lecture des indices des sommets des faces
            // Création d'une variable pour y stocker les données liées à un sommet. Il se peut que dans certains fichiers .obj, les données des sommets des faces soient 
            // de cette forme : v/vt/vn (seule la première donnée nous intéresse ici)
            std::string data;
            // Tant qu'il reste des indices à lire
            while (buffer >> data) {
                // extraction de l'indice d'un sommet d'une face
                int index = std::stoi(data.substr(0, data.find("/")));
                // et ajout dans le tableau des indices d'une face
                indices.push_back(index);
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
        std::vector<int> indices = face_indices[i];
        // récupération de la taille du tableau
        std::size_t indices_size = indices.size();
        //s'il s'agit d'une face triangulaire
        if (indices_size == 3) {
            // ajout de la face triangulaire aux données du maillage
            // attention à toujours retirer 1 aux indices des sommets car ils commencent à 1 dans le fichier .obj mais à 0 dans vertices_descriptor
            face_descriptor face = m_surface_mesh.add_face(vertices_descriptor[indices[0] - 1], vertices_descriptor[indices[1] - 1], vertices_descriptor[indices[2] - 1]);
            // association de la liste des sommets de la face à la face du maillage correspondante
            m_face_vertices[face] = {vertices_descriptor[indices[0] - 1], vertices_descriptor[indices[1] - 1], vertices_descriptor[indices[2] - 1]};
        }
        // Sinon s'il s'agit d'une face quadrangulaire
        else if (indices_size == 4) {
            // ajout de la face quadrangulaire aux données du maillage
            face_descriptor face = m_surface_mesh.add_face(vertices_descriptor[indices[0] - 1], vertices_descriptor[indices[1] - 1], vertices_descriptor[indices[2] - 1], vertices_descriptor[indices[3] - 1]);
            // association de la liste des sommets de la face à la face du maillage correspondante
            m_face_vertices[face] = {vertices_descriptor[indices[0] - 1], vertices_descriptor[indices[1] - 1], vertices_descriptor[indices[2] - 1], vertices_descriptor[indices[3] - 1]};
        }
        // Sinon il s'agit d'un n-gon, c'est-à-dire une face formée de n > 4 sommets. Dans ce cas, nous ne pouvons pas créer le maillage
        else {
            throw std::runtime_error("Le maillage ne peut être composé que de faces triangulaires ou quadrangulaires.");
        }
    }
}

void SurfaceMesh::displaySurfaceMeshInfos() {
    // Affichage du nombre de sommets et de faces du maillage
    // Première solution en utilisant les fonctions number_of_vertices et number_of_faces
    std::cout << "Nombre de sommets de de faces en utilisant les fonctions number_of_vertices et number_of_faces :" << std::endl;
    std::cout << "Nombres de sommets : " << m_surface_mesh.number_of_vertices() << std::endl;
    std::cout << "Nombres de faces : " << m_surface_mesh.number_of_faces() << std::endl;
    std::cout << std::endl;

    // Deuxième solution en récupérant la taille des tableaux contenant les sommets (fonction vertices() de Surface_mesh)
    // et les faces (fonction faces() de Surface_mesh)
    std::cout << "Nombre de sommets de de faces en utilisant les tailles des tableaux de sommets et de faces :" << std::endl;
    std::cout << "Nombres de sommets : " << m_surface_mesh.vertices().size() << std::endl;
    std::cout << "Nombres de faces : " << m_surface_mesh.faces().size() << std::endl;
}

void SurfaceMesh::computeVerticesValency() {
    // Calcul de la valence de chaque sommet du maillage. La valence pour un sommet donné correspond au nombre
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
        std::cout << "Exportation des données de valence dans le fichier CSV réussie." << std::endl;
    }
}

void SurfaceMesh::computeDihedralAngles() {
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
            // Calcul de l'angle dièdre entre cette face et la face courante en degrés
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
    // Création d'une map permettant de stocker ces nouvelles données (la clé représente la valeur de l'angle dièdre et la valeur le nombre d'occurrences trouvées ayant cet angle)
    std::map<int, int> dihedral_angles_data;

    // Nous parcourons la liste des faces du maillage
    for (face_descriptor face : m_surface_mesh.faces()) {
        // récupération de la liste des angles dièdres associés à cette face
        std::vector<double> angles = m_dihedral_angles[face];
        // Parcours de la liste de ces angles
        for(double angle : angles) {
            // arrondissement de la valeur de l'angle dièdre à l'entier le plus proche
            int rounded_angle = (int)std::round(angle);
            // incrémentation du nombre d'occurrences avec cette valeur d'angle
            dihedral_angles_data[rounded_angle]++;
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
        csvOutputFile << "Angle dièdre,Nombre d'occurrences\n";
        // Remplissage du fichier avec les données des angles dièdres
        for (const auto& [key, value] : dihedral_angles_data) {
            csvOutputFile << key << "," << value << "\n";
        }

        // Fermeture du fichier CSV
        csvOutputFile.close();
        std::cout << "Exportation des données de valence dans le fichier CSV réussie." << std::endl;
    }
}

void SurfaceMesh::computeAreaOfFaces() {
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
    // La technique utilisée ici est une approche en approximant la courbure gaussienne de chaque sommet du maillage en utilisant 
    // les normales et les aires des faces

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
                double angle = std::acos((CGAL::abs(n1 * n2)) / (n1_norm * n2_norm));
                // Ajout du résultat à la somme des angles obtenus
                angle_sum += angle;
            }
            else {
                face_descriptor face = m_surface_mesh.face(m_surface_mesh.opposite(he));
                area_sum += m_face_area[face];
            }
        }

        // Calcul de l'approximation de la courbure gaussienne pour ce sommet
        double gaussian_curvature = (2 * CGAL_PI - angle_sum) / area_sum;
        // et ajout du résultat à la table de propriété
        m_vertex_gaussian_curvature[vertex] = gaussian_curvature;
    }

    // Affichage des résultats
    /*for (vertex_descriptor vertex : m_surface_mesh.vertices()) {
        std::cout << "Sommet " << vertex << ", courbure gaussienne: " << m_vertex_gaussian_curvature[vertex] << std::endl;
    }*/
}

void SurfaceMesh::exportGaussianCurvatureAsOBJ(const std::string objFileName) {
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
        objOutputFile << "#OBJ file created by Richard Leestmans\n#https://github.com/DevDarkstar/Traitement-maillage-CGAL\no Mesh\n";
        // Remplissage du fichier avec les coordonnées des sommets du maillage ainsi que les couleurs déterminées selon la courbure gaussienne
        for (vertex_descriptor v : m_surface_mesh.vertices()) {
            // Récupération des coordonnées du sommet
            Point_3 coordinates = vertices_coordinates[v];
            // écriture des coordonnées dans le fichier OBJ
            objOutputFile << "v " << coordinates.x() << " " << coordinates.y() << " " << coordinates.z() << " ";

            // Récupération de la courbure gaussienne de ce sommet
            double gaussian_curvature = m_vertex_gaussian_curvature[v];
            // et stockage de la couleur associée à ce sommet en angle de la courbure
            // Si la valeur est supérieur strictement à 5000, le sommet sera vert
            if (gaussian_curvature > 5000) {
                objOutputFile << 1 << " " << 0 << " " << 1 << "\n";
            }
            else if (gaussian_curvature > 2500) {
                objOutputFile << 1 << " " << 0 << " " << 0 << "\n";
            }
            else if (gaussian_curvature > 1000) {
                objOutputFile << 1 << " " << 0.5 << " " << 0 << "\n";
            }
            else if (gaussian_curvature > 500) {
                objOutputFile << 1 << " " << 1 << " " << 0 << "\n";
            }
            else if (gaussian_curvature > 250) {
                objOutputFile << 0 << " " << 1 << " " << 0 << "\n";
            }
            else if (gaussian_curvature > 100) {
                objOutputFile << 0 << " " << 1 << " " << 1 << "\n";
            }
            else {
                objOutputFile << 0 << " " << 0 << " " << 1 << "\n";
            }
        }

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

        // Fermeture du fichier OBJ
        objOutputFile.close();
        std::cout << "Création du fichier OBJ relatif aux courbures gaussiennes réussie" << std::endl;
    }
}

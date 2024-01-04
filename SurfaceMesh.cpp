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

    // Création d'une table de propriété permettant d'associer la liste des sommets appartenant à la face à la face du maillage elle-même
    std::pair<Surface_mesh::Property_map<face_descriptor, std::vector<vertex_descriptor>>, bool> property = m_surface_mesh.add_property_map<face_descriptor, std::vector<vertex_descriptor>>("f:vertices");
    // Création d'une variable dans laquelle la table de propriété sera stockée
    Surface_mesh::Property_map<face_descriptor, std::vector<vertex_descriptor>> face_vertices_property;
    // Vérification si la table de propriété s'est créée correctement
    if (!property.second) {
        throw std::runtime_error("La table de propriété de la liste des sommets relatifs à une face ne s'est pas créée correctement.");
    } else {
        face_vertices_property = property.first;
    }

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
            // association de la liste des sommets de la face à la face du maillage
            face_vertices_property[face] = {vertices_descriptor[indices[0] - 1], vertices_descriptor[indices[1] - 1], vertices_descriptor[indices[2] - 1]};
        }
        // Sinon s'il s'agit d'une face quadrangulaire
        else if (indices_size == 4) {
            // ajout de la face quadrangulaire aux données du maillage
            face_descriptor face = m_surface_mesh.add_face(vertices_descriptor[indices[0] - 1], vertices_descriptor[indices[1] - 1], vertices_descriptor[indices[2] - 1], vertices_descriptor[indices[3] - 1]);
            // association de la liste des sommets de la face à la face du maillage
            face_vertices_property[face] = {vertices_descriptor[indices[0] - 1], vertices_descriptor[indices[1] - 1], vertices_descriptor[indices[2] - 1], vertices_descriptor[indices[3] - 1]};
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

void SurfaceMesh::calculateVerticesValency() {
    // Création d'une table de propriété associant à un sommet du maillage une valence
    std::pair<Surface_mesh::Property_map<vertex_descriptor, int>, bool> property = m_surface_mesh.add_property_map<vertex_descriptor, int>("v:valency");
    // Création d'une variable dans laquelle la table de propriété sera stockée
    Surface_mesh::Property_map<vertex_descriptor, int> vertices_valency;

    // Vérification si la table de propriété s'est créée correctement
    if (!property.second) {
        throw std::runtime_error("La table de propriété de la valence des sommets ne s'est pas créée correctement.");
    } else {
        vertices_valency = property.first;
    }

    // Calcul de la valence de chaque sommet du maillage. La valence pour un sommet donné correspond au nombre
    // de sommets qui sont directement voisins de ce sommet. Une autre façon de définir la valence d'un sommet va être de calculer le
    // nombre d'arêtes possédant ce sommet comme membre. C'est la stratégie utilisée ici.
    // Nous commençons par parcourir les arêtes du maillage
    for (edge_descriptor edge : m_surface_mesh.edges()) {
        // récupération de la demi-arête associée à cette arête
        halfedge_descriptor he = m_surface_mesh.halfedge(edge);
        // récupération des deux sommets associés à cette demi-arête
        vertex_descriptor v1 = m_surface_mesh.source(he);
        vertex_descriptor v2 = m_surface_mesh.target(he);

        // Incrémentation de la valence de ces deux sommets dans la table de propriété
        vertices_valency[v1]++;
        vertices_valency[v2]++;
    }

    //Affichage des valences des sommets du maillage
    for (vertex_descriptor v : m_surface_mesh.vertices()) {
        std::cout << "Valence de " << v << " : " << vertices_valency[v] << std::endl;
    }
}

void SurfaceMesh::exportVerticesValencyAsCSV(const std::string csvFileName) {
    // Récupération de la table de propriété contenant les valences des sommets du maillage
    Surface_mesh::Property_map<vertex_descriptor, int> vertices_valency = m_surface_mesh.property_map<vertex_descriptor, int>("v:valency").first;

    // Comme l'objectif est de pouvoir exporter les données de sorte à pouvoir réaliser un histogramme, nous allons préparer ces dernières
    // de sorte à pouvoir afficher un histogramme représentant le nombre de sommets en fonction de leur valence
    // Création d'une map permettant de stocker ces nouvelles données (la clé représente la valence et la valeur le nombre de sommets ayant cette valance)
    std::map<int, int> valency_data;

    // Nous parcourons la liste des sommets du maillage
    for (vertex_descriptor v : m_surface_mesh.vertices()) {
        // récupération de la valence associée à ce sommet dans la table de propriété
        int valency = vertices_valency[v];
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

void SurfaceMesh::calculateDihedralAngles() {
    // Récupération de la table de propriété des coordonnées des sommets du maillage
    Surface_mesh::Property_map<vertex_descriptor, Point_3> vertices_coordinates = m_surface_mesh.property_map<vertex_descriptor, Point_3>("v:point").first;

    // Récupération de la table de propriété de la liste des sommets associés à chaque face du maillage
    Surface_mesh::Property_map<face_descriptor, std::vector<vertex_descriptor>> face_vertices_property = m_surface_mesh.property_map<face_descriptor, std::vector<vertex_descriptor>>("f:vertices").first; 

    // Création d'une table de propriété qui va associer à chaque face du maillage sa normale
    std::pair<Surface_mesh::Property_map<face_descriptor, Vector_3>, bool> normal_property = m_surface_mesh.add_property_map<face_descriptor, Vector_3>("f:normal");

    Surface_mesh::Property_map<face_descriptor, Vector_3> face_normal;

    // Vérification si la table de propriété s'est créée correctement
    if (!normal_property.second) {
        throw std::runtime_error("La table de propriété des normales aux faces ne s'est pas créée correctement.");
    } else {
        face_normal = normal_property.first;
    }

    // Remplissage du tableau des normales aux faces
    for (face_descriptor face : m_surface_mesh.faces()) {
        // Récupération de la liste des sommets associés à cette face
        std::vector<vertex_descriptor> vertices = face_vertices_property[face];

        // Calcul de deux vecteurs de cette face
        Vector_3 v1 = vertices_coordinates[vertices[1]] - vertices_coordinates[vertices[0]];
        Vector_3 v2 = vertices_coordinates[vertices[2]] - vertices_coordinates[vertices[0]];

        // Calcul du vecteur normal à cette face
        Vector_3 normal = CGAL::cross_product(v1, v2);

        // Stockage de la normale à cette face dans la table de propriété associée
        face_normal[face] = normal;
    }

    // Détermination des faces adjacentes à une face donnée
    // Création d'une table de propriété qui va associer à chaque face du maillage les faces qui lui sont adjacentes et ayant un indice supérieur à cette dernière
    std::pair<Surface_mesh::Property_map<face_descriptor, std::vector<face_descriptor>>, bool> adjacent_property = m_surface_mesh.add_property_map<face_descriptor, std::vector<face_descriptor>>("f:adjacent");

    Surface_mesh::Property_map<face_descriptor, std::vector<face_descriptor>> adjacent_faces;

    // Vérification si la table de propriété s'est créée correctement
    if (!adjacent_property.second) {
        throw std::runtime_error("La table de propriété des faces adjacentes ne s'est pas créée correctement.");
    } else {
        adjacent_faces = adjacent_property.first;
    }

    // La stratégie ici est de définir que deux faces sont adjacentes si elles possèdent deux sommets en commun
    // Parcours des faces du maillage 
    for (face_descriptor face : m_surface_mesh.faces()) {
        // Récupération des sommets de cette face
        std::vector<vertex_descriptor> current_face_vertices = face_vertices_property[face];
        // Création d'un itérateur sur la liste des faces du maillage à partir de cette face
        auto it = std::find(m_surface_mesh.faces().begin(), m_surface_mesh.faces().end(), face);
        // Vérification s'il ne s'agit pas de la dernière face du maillage
        if (it != m_surface_mesh.faces().end()) {
            // Incrémentation de l'itérateur afin d'exclure la face courante
            std::advance(it, 1);
            // Parcours de la liste des faces juusq'à la fin
            while(it != m_surface_mesh.faces().end()) {
                // Récupération des sommets de cette face
                std::vector<vertex_descriptor> vertices = face_vertices_property[*it];
                // Création d'un compteur de sommets similaires
                int same_vertices_count = 0;
                // Parcours des tableaux de sommets de la face courante et de cette face afin de
                // déterminer si elles sont adjacentes
                for (vertex_descriptor v1 : current_face_vertices) {
                    for (vertex_descriptor v2 : vertices) {
                        if (v1 == v2) {
                            same_vertices_count++;
                        }
                    }
                }

                // Si le nombre de sommets en commun est supérieur ou égal à deux, les deux faces sont adjacentes
                // et nous ajoutons cette face à la liste des faces adjacentes à la face courante
                if (same_vertices_count >= 2) {
                    adjacent_faces[face].push_back(*it);
                }
                // incrémentation de l'itérateur
                std::advance(it, 1);
            }

        }
    }

    // Création d'une table de propriété qui va associer à chaque face du maillage les angles dièdres des faces qui lui sont adjacentes
    std::pair<Surface_mesh::Property_map<face_descriptor, std::vector<double>>, bool> dihedral_property = m_surface_mesh.add_property_map<face_descriptor, std::vector<double>>("f:dihedral_angles");

    Surface_mesh::Property_map<face_descriptor, std::vector<double>> dihedral_angles;

    // Vérification si la table de propriété s'est créée correctement
    if (!dihedral_property.second) {
        throw std::runtime_error("La table de propriété des angles dièdres ne s'est pas créée correctement.");
    } else {
        dihedral_angles = dihedral_property.first;
    }

    // Remplissage de la table de propriété
    for(face_descriptor face : m_surface_mesh.faces()) {
        // Récupération de la normale de la face courante
        Vector_3 current_normal = face_normal[face];
        // Calcul de la norme de ce vecteur
        double current_norm = CGAL::sqrt(current_normal.squared_length());
        // Récupération des faces adjacentes à celle-ci
        std::vector<face_descriptor> faces = adjacent_faces[face];

        for(face_descriptor adjacent_face : faces) {
            // Récupération de la normale de cette face
            Vector_3 normal = face_normal[adjacent_face];
            // Calcul de la norme de ce vecteur
            double norm = CGAL::sqrt(normal.squared_length());
            // Calcul de l'angle dièdre entre cette face et la face courante en degrés
            double dihedral_angle = std::acos((current_normal * normal) / (current_norm * norm)) * 180 / CGAL_PI;
            // Ajout de l'angle dièdre à la table de propriété
            dihedral_angles[face].push_back(dihedral_angle);
        } 
    }

    // Affichage des angles dièdres des faces adjacentes
    for (face_descriptor face : m_surface_mesh.faces()) {
        std::cout << "Faces adjacentes à " << face << " : ";
        std::vector<face_descriptor>::iterator adjacent_face = adjacent_faces[face].begin();
        std::vector<double>::iterator dihedral_angle = dihedral_angles[face].begin();
        for (; adjacent_face != adjacent_faces[face].end() && dihedral_angle != dihedral_angles[face].end(); ++adjacent_face, ++dihedral_angle) {
            std::cout << *adjacent_face << " d'angle dièdre : " << *dihedral_angle << "°, ";
        }
        std::cout << std::endl;
    }
}

void SurfaceMesh::exportDihedralAnglesAsCSV(const std::string csvFileName) {
    // Récupération de la table de propriété contenant les angles dièdres entre les faces adjacentes du maillage
    Surface_mesh::Property_map<face_descriptor, std::vector<double>> dihedral_angles = m_surface_mesh.property_map<face_descriptor, std::vector<double>>("f:dihedral_angles").first;

    // Comme l'objectif est de pouvoir exporter les données de sorte à pouvoir réaliser un histogramme, nous allons préparer ces dernières
    // de sorte à pouvoir afficher un histogramme représentant le nombre d'occurences ayant une certaine valeur d'angle
    // De plus, afin de limiter la taille de l'histogramme, nous allons arrondir les valeurs d'angles obtenues à l'entier le plus proche
    // Création d'une map permettant de stocker ces nouvelles données (la clé représente la valeur de l'angle dièdre et la valeur le nombre d'occurrences trouvées ayant cet angle)
    std::map<int, int> dihedral_angles_data;

    // Nous parcourons la liste des faces du maillage
    for (face_descriptor face : m_surface_mesh.faces()) {
        // récupération de la liste des angles dièdres associés à cette face
        std::vector<double> angles = dihedral_angles[face];
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

void SurfaceMesh::calculateAreaOfFaces() {
    // Récupération de la table de propriété des coordonnées des sommets du maillage
    Surface_mesh::Property_map<vertex_descriptor, Point_3> vertices_coordinates = m_surface_mesh.property_map<vertex_descriptor, Point_3>("v:point").first;

    // Récupération de la table de propriété de la liste des sommets associés à chaque face du maillage
    Surface_mesh::Property_map<face_descriptor, std::vector<vertex_descriptor>> face_vertices_property = m_surface_mesh.property_map<face_descriptor, std::vector<vertex_descriptor>>("f:vertices").first;

    // Création d'une table de propriété qui va associer à chaque face du maillage son aire
    std::pair<Surface_mesh::Property_map<face_descriptor, double>, bool> area_property = m_surface_mesh.add_property_map<face_descriptor, double>("f:area");

    Surface_mesh::Property_map<face_descriptor, double> face_area;

    // Vérification si la table de propriété s'est créée correctement
    if (!area_property.second) {
        throw std::runtime_error("La table de propriété des aires des faces ne s'est pas créée correctement.");
    } else {
        face_area = area_property.first;
    }

    // Parcours de la liste des faces du maillage
    for (face_descriptor face : m_surface_mesh.faces()) {
        // Récupération de la liste des sommets de cette face
        std::vector<vertex_descriptor> vertices = face_vertices_property[face];
        // Si la face est triangulaire
        if (vertices.size() == 3) {
            // Récupération des coordonnées des trois sommets de la face
            Point_3 v1 =  vertices_coordinates[vertices[0]];
            Point_3 v2 =  vertices_coordinates[vertices[1]];
            Point_3 v3 =  vertices_coordinates[vertices[2]];

            // Calcul de l'aire du triangle en utilisant la formule suivante
            //                |     | x1 y1 z1 ||
            // Aire = 1 / 2 * | det | x2 y2 z2 ||
            //                |     | x3 y3 z3 ||

            double area = 0.5 * CGAL::abs(v1.x()*v2.y()*v3.z() - v1.x()*v3.y()*v2.z() - v2.x()*v1.y()*v3.z() + v2.x()*v3.y()*v1.z() + v3.x()*v1.y()*v2.z() - v3.x()*v2.y()*v1.z());

            // Stockage de l'aire de la face résultante dans la table de propriété
            face_area[face] = area;
        }
        // Sinon elle est quadrangulaire
        else {
            // Récupération des coordonnées des quatre sommets de la face
            Point_3 v1 =  vertices_coordinates[vertices[0]];
            Point_3 v2 =  vertices_coordinates[vertices[1]];
            Point_3 v3 =  vertices_coordinates[vertices[2]];
            Point_3 v4 =  vertices_coordinates[vertices[3]];

            // Calcul de l'aire de la face en la subdivisant en deux faces triangulaires, puis calcul de l'aire des deux triangles
            // L'aire de la face quadrangulaire correspondra à la somme de ces deux aires
            // Première aire en prenant les sommets v1, v2 et v3
            double area1 = 0.5 * CGAL::abs(v1.x()*v2.y()*v3.z() - v1.x()*v3.y()*v2.z() - v2.x()*v1.y()*v3.z() + v2.x()*v3.y()*v1.z() + v3.x()*v1.y()*v2.z() - v3.x()*v2.y()*v1.z());
            // Seconde aire en prenant les sommets v3, v4 et v1
            double area2 = 0.5 * CGAL::abs(v3.x()*v4.y()*v1.z() - v3.x()*v1.y()*v4.z() - v4.x()*v3.y()*v1.z() + v4.x()*v1.y()*v3.z() + v1.x()*v3.y()*v4.z() - v1.x()*v4.y()*v3.z());

            // Stockage de l'aire de la face résultante dans la table de propriété
            face_area[face] = area1 + area2;
        }
    }

    // Affichage de l'aire des faces du maillage
    for(face_descriptor face : m_surface_mesh.faces()) {
        std::cout << "Face " << face << " : aire = " << face_area[face] << " m²" << std::endl;
    }
}

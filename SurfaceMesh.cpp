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

    // Création d'une table de propriété permettant d'associer les indices des sommets appartenant à la face à la face du maillage elle-même
    std::pair<Surface_mesh::Property_map<face_descriptor, std::vector<int>>, bool> property = m_surface_mesh.add_property_map<face_descriptor, std::vector<int>>("f:indices");
    // Création d'une variable dans laquelle la table de propriété sera stockée
    Surface_mesh::Property_map<face_descriptor, std::vector<int>> face_indices_property;
    // Vérification si la table de propriété s'est créée correctement
    if (!property.second) {
        throw std::runtime_error("La table de propriété des indices des sommets relatifs à une face ne s'est pas créée correctment.");
    } else {
        face_indices_property = property.first;
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
            // association des indices des sommets de la face à la face du maillage
            face_indices_property[face] = indices;
        }
        // Sinon s'il s'agit d'une face quadrangulaire
        else if (indices_size == 4) {
            // ajout de la face quadrangulaire aux données du maillage
            face_descriptor face = m_surface_mesh.add_face(vertices_descriptor[indices[0] - 1], vertices_descriptor[indices[1] - 1], vertices_descriptor[indices[2] - 1], vertices_descriptor[indices[3] - 1]);
            // association des indices des sommets de la face à la face du maillage
            face_indices_property[face] = indices;
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

void SurfaceMesh::generateVerticesValency() {
    // Création d'une table de propriété associant à un sommet du maillage une valence
    std::pair<Surface_mesh::Property_map<vertex_descriptor, int>, bool> property = m_surface_mesh.add_property_map<vertex_descriptor, int>("v:valency");
    // Création d'une variable dans laquelle la table de propriété sera stockée
    Surface_mesh::Property_map<vertex_descriptor, int> vertices_valency;

    // Vérification si la table de propriété s'est créée correctement
    if (!property.second) {
        throw std::runtime_error("La table de propriété de la valence des sommets ne s'est pas créée correctment.");
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
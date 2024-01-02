#include "SurfaceMesh.hpp"

SurfaceMesh::SurfaceMesh(const std::string objFilePath){
    //Ouverture du fichier .obj
    std::ifstream objFile(objFilePath);
    if (!objFile) {
        throw std::runtime_error("Le fichier .obj n'a pas pu etre ouvert. Verifiez que le fichier .obj se situe bien dans le dossier 'data' de CGAL.");
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
            float x;
            float y;
            float z;
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
            m_surface_mesh.add_face(vertices_descriptor[indices[0] - 1], vertices_descriptor[indices[1] - 1], vertices_descriptor[indices[2] - 1]);
        }
        // Sinon s'il s'agit d'une face quadrangulaire
        else if (indices_size == 4) {
            std::cout << "quads" << std::endl;
            // ajout de la face quadrangulaire aux données du maillage
            m_surface_mesh.add_face(vertices_descriptor[indices[0] - 1], vertices_descriptor[indices[1] - 1], vertices_descriptor[indices[2] - 1], vertices_descriptor[indices[3] - 1]);
        }
        // Sinon il s'agit d'un n-gon, c'est-à-dire une face formée de n > 4 sommets. Dans ce cas, nous ne pouvons pas créer le maillage
        else {
            throw std::runtime_error("Le maillage ne peut etre composé que de faces triangulaires ou quadrangulaires.");
        }
    }

    std::cout << "Nombres de sommets : " << num_vertices(m_surface_mesh) << std::endl;
    std::cout << "Nombres d'arêtes : " << num_edges(m_surface_mesh) << std::endl;
    std::cout << "Nombres de faces : " << num_faces(m_surface_mesh) << std::endl;
    //std::cout << "Nombres de faces : " << m_surface_mesh.faces().size() << std::endl;
    std::cout << std::endl;
}
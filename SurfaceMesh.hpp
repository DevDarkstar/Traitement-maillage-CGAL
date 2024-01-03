#ifndef SURFACEMESH_HPP
#define SURFACEMESH_HPP

#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include <stdexcept>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

//Simplification des noms des classes CGAL
typedef CGAL::Simple_cartesian<double>          Kernel;
typedef Kernel::Point_3                         Point_3;
typedef CGAL::Surface_mesh<Point_3>             Surface_mesh;
typedef Surface_mesh::Vertex_index              vertex_descriptor;
typedef Surface_mesh::Face_index                face_descriptor;
typedef Surface_mesh::Edge_index                edge_descriptor;
typedef Surface_mesh::Halfedge_index            halfedge_descriptor;

class SurfaceMesh{
    public:
    explicit SurfaceMesh(const std::string objFilePath); // Constructeur de la classe permettant de construire un objet de type Surface_mesh
    void displaySurfaceMeshInfos(); // Affichage du nombre de sommets et de faces de l'attribut de la classe m_surface_mesh
    void generateVerticesValency(); // Calcul de la valence des sommets du maillage

    private:
    Surface_mesh m_surface_mesh;
};

#endif
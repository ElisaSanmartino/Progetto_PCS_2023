#ifndef __EMPTY_H
#define __EMPTY_H

#include <iostream>
#include "Eigen/Eigen"
#include <cmath>
#include <unordered_map>

using namespace std;
using namespace Eigen;

namespace ProjectLibrary
{
  /*class Empty
  {
    public:
      void Show() const { std::cout<< "Hello world;"<< std::endl; }
  };*/

  constexpr double max_tolerance(const double& x, const double& y)
  {
    return x > y ? x : y;
  }

  class Point
  {
    public:

      double x;
      double y;

      static constexpr double geometricTol = 1.0e-12;
      static constexpr double geometricTol_Squared = max_tolerance(Point::geometricTol * Point::geometricTol,
                                                                   numeric_limits<double>::epsilon());

      Point() = default;
      Point(const Point&p):
          x(p.x), y(p.y)
      {}
      Point(const double &x,const double &y):
          x(x),y(y){}

      bool circCircoscritta(Triangle&tr); //true interno, false esterno, det matrice
      Triangle puntoInternoAlTriang(Triangle&tr);//restituisce il triangolo a cui q è interno
  };

  struct Segment
  {
    public:
      Point inizio;
      Point fine;
      double lunghezza;

      Segment() = default;
      Segment(Point&inizio,Point&fine):
          inizio(inizio),fine(fine){}
  };

  struct Triangle
  {
    public:
      array<Point, 3> verticesTriangle; //vett di id vertici del triangolo

      Triangle() = default;
      Triangle(array<Point,3>&verticesTriangle): verticesTriangle(verticesTriangle){}//nel costruttore controllare che i vertici siano in senso antiorario e sistemarli nel caso non lo fossero
  };

  class Mesh
  {
    public:
      vector<Point> pointsInMesh;
      vector<Triangle> trianglesInMesh;
      vector<Point> pointsSulConvexHull;
      unordered_map<Segment, vector<Triangle>> adiacenza; //Segment:(tr1, tr2)

      Mesh(vector<Point>&pointsInMesh, vector<Point> pointsSulConvexHull):
          pointsInMesh(pointsInMesh), pointsSulConvexHull(pointsSulConvexHull){}

      void aggiungiPuntiConvexHull(vector<Point> &pointsSulConvexHull);
      void aggiungiPuntoInterno(Triangle&tr,vector<Point>&pointsInMesh);//crea i 3 triangolini in tr
  };

  inline bool operator==(const Point& p1, const Point& p2)
  {
    return (normSquared(p1.x - p2.x, p1.y - p2.y) <=
            Point::geometricTol * Point::geometricTol *
            max(normSquared(p1.x, p1.y), normSquared(p2.x, p2.y)));
  }

  inline bool operator==(const Triangle& t1, const Triangle& t2)
  {
    return (t1.verticesTriangle[0]==t2.verticesTriangle[0]
            && t1.verticesTriangle[1]==t2.verticesTriangle[1]
            && t1.verticesTriangle[2]==t2.verticesTriangle[2]);
  }

  bool ipotesiDelaunay(Segment&seg);//alfa = acos(( b^2 + c2 - a2)/2bc). se è rispettata aggiorna adiacenza.
  void flip(Triangle&t1, Triangle&t2, Segment &seg); //fa il flip. toglie e mette da trianglesInMesh

}




#endif // __EMPTY_H

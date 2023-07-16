#include "empty_class.hpp"
using namespace std;

//convex hull dà i punti in senso antiorario, il primo è quello con la x minore
//convex hull non ha punti allineati, solo quelli esterni, quelli che sarebbero allineati sono considerati interni

namespace ProjectLibrary
{
///convex hull

///mesh
    /*convex hull facendo attenzione che i punti potrebbero essere allineati
     * poi si aggiungono prima i punti del convex hull così poi tutti gli altri
     * sono interni alla mesh
     * */

void Mesh::aggiungiPuntiConvexHull(vector<Point> &pointsSulConvexHull)
{/*

     * prendo i primi tre punti del convex hull per avere i vertici del triangolo iniziale
     * si prende il punto successivo e si unisce al precedente e al primo vertice del triangolo iniziale
     * rimangono sempre solo due nuovi segmenti
     * si crea quindi un solo nuovo triangolo
     * ipotesiDelaunay su tutte le coppie di triangoli che hanno un lato in comune
     * eventuale flip
     * aggiornare triangleInMesh,
     * adiacenza
    */

    unsigned int i=3;
    Point primoVertice = pointsSulConvexHull[0];
    array<Point> vertPrimoTr = (pointsSulConvexHull[0],pointsSulConvexHull[1],pointsSulConvexHull[2]);
    Triangle trPrec(vertPrimoTr);
    while (i<pointsSulConvexHull.size())
    {
        Point q = pointsSulConvexHull[i];
        // non prendiamo i vertici del primo triangolo
        Segment(pointsSulConvexHull[i-1],q);
        Segment(primoVertice,q);
        array<Point,3> vertTrNuovo = (primoVertice,q,pointsSulConvexHull[i-1]);
        Triangle trNuovo(vertTrNuovo);
        Segment latoInComune(pointsSulConvexHull[i-1],primoVertice);
        //aggiornare adiacenza
        adiacenza[latoInComune] = (trPrec,trNuovo);

        bool ipDel = ipotesiDelaunay(latoInComune);//lato in comune è quello tra pointsSulConvexHull[i-1] e primoVertice, dice se è risp
        if (ipDel==true)
        {
            trianglesInMesh.push_back(trNuovo);
            trPrec = trNuovo;
        }
        else
        {
            flip(trNuovo,trPrec,latoInComune);
            unsigned int i=0;
                trovato = false;
                while (i<2 && trovato==false)
                {
                    if (i==0)
                    {
                       if (trNuovo.verticesTriangle[i]==primoVertice || trNuovo.verticesTriangle[i]==q)//primo punto è uno dei due che serve
                       {
                           if (trNuovo.verticesTriangle[i+1]==primoVertice || trNuovo.verticesTriangle[i+1]==q)//guarda il secondo punto se è l'altro che serve
                            {
                               trPrec = trNuovo; //abbiamo il tr cercato
                               trovato = true;
                           }
                           else
                            {
                               if (trNuovo.verticesTriangle[i+2]==primoVertice || trNuovo.verticesTriangle[i+2]==q)//guardi il terzo punto se è l'altro che serve
                               {
                                   trPrec = trNuovo; //abbiam trovato li tr
                                   trovato = true;
                               }

                               else
                               {
                                   trPrec = trPrec; // il tr è l'altro
                                   trovato = true;
                               }
                           }

                       }
                    }
                    else
                    {
                        if (trNuovo.verticesTriangle[i]==primoVertice || trNuovo.verticesTriangle[i]==q) //i = 1
                        {
                            if (trNuovo.verticesTriangle[i+1]==primoVertice || trNuovo.verticesTriangle[i+1]==q)
                             {
                                trPrec = trNuovo;
                                trovato = true;
                            }
                            else
                            {
                                trPrec = trPrec;
                                trovato = true;
                            }
                        }
                        else
                        {
                            trPrec = trPrec;
                            trovato = true;
                        }

                    }
                    i++;
                }

        }
        trianglesInMesh.push_back(trNuovo);
    }
}

void Mesh::aggiungiPuntoInterno(Triangle &tr, vector<Point> &pointsInMesh)
{
    n = pointsInMesh.size();
    for (unsigned int i=0;i<n;i++)
    {
        pointsInMesh[i] = q;
        Segment lato1(tr.verticesTriangle[0],tr.verticesTriangle[1]);
        Segment lato2(tr.verticesTriangle[1],tr.verticesTriangle[2]);
        Segment lato3(tr.verticesTriangle[2],tr.verticesTriangle[0]);
        array<Point,3> verticit1 = (q,tr.verticesTriangle[0],tr.verticesTriangle[1]);
        array<Point,3> verticit2 = (q,tr.verticesTriangle[1],tr.verticesTriangle[2]);
        array<Point,3> verticit3 = (q,tr.verticesTriangle[2],tr.verticesTriangle[0]);
        Triangle t1(verticit1);
        Triangle t2(verticit2);
        Triangle t3(verticit3);
        trianglesInMesh.push_back((t1));
        trianglesInMesh.push_back((t2));
        trianglesInMesh.push_back((t3));
        //delaunay solo con quelli fuori da tr
    }
}

///segment
Segment::Segment(Point&inizio,Point&fine)
{
    lunghezza = sqrt(pow(fine.y-inizio.y,2) + pow(fine.x-inizio.x,2));
}



///Triangle
Triangle::Triangle(array<Point, 3> &verticesTriangle) //ordina i vertici in senso antiorario se non lo sono
{
    //senso antiorario area pos

    double area;
    for (unsigned int i=1; i<4; i++)
    {
        if (i<3)
            area =+ verticesTriangle[i].x*verticesTriangle[i+1].y - verticesTriangle[i+1].x*verticesTriangle[i].y;
        else
            area = 0.5 * abs(area + verticesTriangle[i].x*verticesTriangle[1].y - verticesTriangle[1].x*verticesTriangle[i].y);
    }
    if (area<0)
    {
        Point temp = verticesTriangle[0];
        verticesTriangle[0] = verticesTriangle[2];
        verticesTriangle[2] = temp;
    }
}

///Point
bool Point::circCircoscritta(Triangle&tr)
{
    tr.verticesTriangle[0] = a;
    tr.verticesTriangle[1] = b;
    tr.verticesTriangle[2] = c;
    double det = (a.x-x)*((b.y-y)*((c.x-x)*(c.x-x)+(c.y-y)*(c.y-y))-(c.y-y)*((b.x-x)*(b.x-x)+(b.y-y)*(b.y-y)))
            -(a.y-y)*((b.x-x)*((c.x-x)*(c.x-x)+(c.y-y)*(c.y-y))-(c.x-x)*((a.x-x)*(a.x-x)+(a.y-y)*(a.y-y)))
            +((a.x-x)*(a.x-x)+(a.y-y)*(a.y-y))*((b.x-x)*(c.y-y)-(c.x-x)*(b.y-y));
    if (det>0) //cade dentro la circ
        return true;
}

Triangle Point::puntoInternoAlTriang(Triangle&tr)
{
    tr.verticesTriangle[0] = a;
    tr.verticesTriangle[1] = b;
    tr.verticesTriangle[2] = c;
    double cross_prod_ab = ((b.x-a.x)*(y-a.y)-(b.y-a.y)*(x-a.x));
    double cross_prod_bc = ((c.x-b.x)*(y-b.y)-(c.y-b.y)*(x-b.x));
    double cross_prod_ca = ((a.x-c.x)*(y-c.y)-(a.y-c.y)*(x-c.x));
    if (cross_prod_ab>=0 && cross_prod_bc>=0 && cross_prod_ca>=0)
        return tr; //è interno a tr
    if (cross_prod_ab<0)
    {
        //prendiamo il tr adiac ad ab
        Segment ab(a,b);
        if (adiacenza[ab][0]==tr)
            //return this->puntoInternoAlTriang(adiacenza[ab][1]);
        else
            //return this->puntoInternoAlTriang(adiacenza[ab][0]);
    }
    if (cross_prod_bc<0)
    {
        //prendiamo il tr adiac ad bc
        Segment bc(b,c);
        if (adiacenza[bc][0]==tr)
            //return this->puntoInternoAlTriang(adiacenza[bc][1]);
        else
            //return this->puntoInternoAlTriang(adiacenza[bc][0]);
    }
    if (cross_prod_ca<0)
    {
        //prendiamo il tr adiac ad ca
        Segment ca(c,a);
        if (adiacenza[ca][0]==tr)
            //return this->puntoInternoAlTriang(adiacenza[ca][1]);
        else
            //return this->puntoInternoAlTriang(adiacenza[ca][0]);
    }

}



///
void flip(Triangle&t1,Triangle&t2,Segment&seg)//seg è il lato in comune
{
    Point estr1, estr2;
    for (unsigned int i=0;i<3;i++)
    {
        if (t1.verticesTriangle[i]!=seg.inizio && t1.verticesTriangle[i]!=seg.fine)
            estr1 = t1.verticesTriangle[i]; //estremo nuovo lato
    }
    for (unsigned int i=0;i<3;i++)
    {
        if (t2.verticesTriangle[i]!=seg.inizio && t2.verticesTriangle[i]!=seg.fine)
            estr2 = t2.verticesTriangle[i];
    }

    Segment nuovoLato(estr1,estr2); //ad
    Segment lato1(seg.inizio,estr1); //ca
    Segment lato2(seg.inizio,estr2); //cd
    array<Point,3> vert_tr1 = (estr1,estr2,seg.inizio);
    array<Point,3> vert_tr2 = (estr1,estr2,seg.fine);
    Triangle t1_nuovo(vert_tr1);
    Triangle t2_nuovo(vert_tr2);
    vector<Triangle> nuoviTr;
    nuoviTr.reserve(2);
    nuoviTr = (t1_nuovo,t2_nuovo);
    Segment lato3(seg.fine,estr1); //ab
    Segment lato4(seg.fine,estr2); //bd

    //aggiornare adiacenza
    adiacenza.erase(seg);
    adiacenza[nuovoLato] = nuoviTr;
    if (adiacenza.contains(lato1))
    {
        for (unsigned int i=0;i<2;i++)
        {
            if (adiacenza[lato1][i] == t1 or adiacenza[lato1][i] == t2)
                adiacenza[lato1][i] = t1_nuovo;
        }
    }
    if (adiacenza.contains(lato2))
    {
        for (unsigned int i=0;i<2;i++)
        {
            if (adiacenza[lato2][i] == t1 or adiacenza[lato2][i] == t2)
                adiacenza[lato2][i] = t1_nuovo;
        }
    }
    if (adiacenza.contains(lato3))
    {
        for (unsigned int i=0;i<2;i++)
        {
            if (adiacenza[lato3][i] == t1 or adiacenza[lato3][i] == t2)
                adiacenza[lato3][i] = t2_nuovo;
        }
    }
    if (adiacenza.contains(lato4))
    {
        for (unsigned int i=0;i<2;i++)
        {
            if (adiacenza[lato4][i] == t1 or adiacenza[lato4][i] == t2)
                adiacenza[lato4][i] = t2_nuovo;
        }
    }
    t1 = t1_nuovo;
    t2 = t2_nuovo;
    //chiamata ricorsiva del flip tra gli adiacenti
    /*chiamo ipotesiDelaunay sull'altro adiacente di lato1, se serve si fa il flip che ricorsivamente richiama dalaunay flip...
     * e questo anche per lato2, 3, 4. tanto se non ci sono più delaunay non lo fa
     * */
}

bool ipotesiDelaunay(Segment&seg)
{
    /*calcola angolo in A, angolo in D, somma degli angoli
     * se somma<180 a posto
     * se somma >= 180 flip: cancelli bc e fai ad
     * */
    if (adiacenza.contains(seg))
    {
        Triangle t1 = (adiacenza[seg])[0];
        Triangle t2 = (adiacenza[seg])[1];
        vector<Segment> edgesTrianglet1;
        edgesTrianglet1.reserve(2);
        for (unsigned int i=0;i<3;i++)
        {
            if (i<2)
                edgesTrianglet1.push_back(Segment(t1.verticesTriangle[i],t1.verticesTriangle[i+1]));
            else
                edgesTrianglet1.push_back(Segment(t1.verticesTriangle[i],t1.verticesTriangle[0]));
        }
        vector<Segment,2> edgesTrianglet2;
        edgesTrianglet2.reserve(2);
        for (unsigned int i=0;i<3;i++)
        {
            if (i<2)
                edgesTrianglet2.push_back(Segment(t2.verticesTriangle[i],t2.verticesTriangle[i+1]));
            else
                edgesTrianglet2.push_back(Segment(t2.verticesTriangle[i],t2.verticesTriangle[0]));
        }
        unsigned int k = 0;
        vector<Segment> b_e_c1;
        vector<Segment> b_e_c2;
        for (unsigned int i = 0;i<3;i++)
        {
            if (t1.edgesTriangle[i]!=seg)
            {
                b_e_c1[k] = t1.edgesTriangle[i];
                k++;
            }
            if (t2.edgesTriangle[i]!=seg)
            {
                b_e_c2[k] = t2.edgesTriangle[i];
                k++;
            }
        }
        double alfa1 = acos(b_e_c1[0].lunghezza*b_e_c1[0].lunghezza + b_e_c1[1].lunghezza*b_e_c1[1].lunghezza - seg.lunghezza*seg.lunghezza);
        double alfa2 = acos(b_e_c2[0].lunghezza*b_e_c2[0].lunghezza + b_e_c2[1].lunghezza*b_e_c2[1].lunghezza - seg.lunghezza*seg.lunghezza);
        double somma = alfa1+alfa2;
        if (somma>=180)
            return false;
        else
            return true;
    }
    else
        return true;
}

Mesh::Mesh(vector<Point>&pointsInMesh, vector<Point>&pointsSulConvexHull)
{

    aggiungiPuntiConvexHull(pointsSulConvexHull);
    unsigned int j = 0;
    for (unsigned int i=0;i<pointsInMesh().size();i++)
    {
        while (trovato==false && j<trianglesInMesh.size())
        {
            Triangle triangolo(trianglesInMesh[j]);
            if (pointsInMesh[i].circCircoscritta(triangolo)==true)
            {
                Triangle tr = pointsInMesh[i].puntoInternoAlTriang(triangolo);
                aggiungiPuntoInterno(tr,pointsInMesh);
                trovato = true;
            }
            else
                j++;
        }

    }

}
}

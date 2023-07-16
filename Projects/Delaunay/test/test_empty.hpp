#ifndef __TEST_EMPTY_H
#define __TEST_EMPTY_H

#include <gtest/gtest.h>

#include "empty_class.hpp"

using namespace testing;
using namespace ProjectLibrary;
/*TEST(TestEmpty, TestEmpty)
{
  ProjectLibrary::Empty empty;
  ASSERT_NO_THROW(empty.Show());
}*/

TEST(TestPoint,TestCircDentro)
{
    Point p1(2,1);
    Point p2(4,3);
    Point p3(3,5);
    Point p(2.2,3.8);
    array<Point,3> lati1 = (p1,p2,p3);
    Triangle t1(lati1);
    bool circ = p.circCircoscritta(t1);

    EXPECT_EQ(circ,true);
}

TEST(TestPoint,TestCircFuori)
{
    Point p1(2,1);
    Point p2(4,3);
    Point p3(3,5);
    Point p(5,6);
    array<Point,3> lati1 = (p1,p2,p3);
    Triangle t1(lati1);
    bool circ = p.circCircoscritta(t1);

    EXPECT_EQ(circ,false);
}


TEST(TestIpotesiDel,TestPos)
{
    Point p1(2,1);
    Point p2(4,3);
    Point p3(3,5);
    array<Point,3> lati1 = (p1,p2,p3);
    Triangle t1(lati1);
    Point p4(5,4);
    array<Point,3> lati2 = (p2,p4,p3);
    Triangle t2(lati2);
    Segment latoComune(p2,p3);
    bool ip = ipotesiDelaunay(t1,t2,latoComune);
    bool giusto = true;

    EXPECT_EQ(ip,giusto);
}

TEST(TestIpotesiDel,TestNeg)
{
    Point p1(2,1);
    Point p2(5,0.5);
    Point p3(1,2);
    Point p4(3,2.5);
    array<Point,3> lati1 = (p1,p2,p3);
    array<Point,3> lati2 = (p2,p4,p3);
    Triangle t1(lati1);
    Triangle t2(lati2);
    Segment latoComune(p2,p3);
    bool ip = ipotesiDelaunay(t1,t2,latoComune);
    bool giusto = false;

    EXPECT_EQ(ip,giusto);
}

TEST(TestFlip,Testf)
{
    Point p1(2,1);
    Point p2(5,0.5);
    Point p3(1,2);
    Point p4(3,2.5);
    array<Point,3> lati1 = (p1,p2,p3);
    array<Point,3> lati2 = (p2,p4,p3);
    Triangle t1(lati1);
    Triangle t2(lati2);
    Segment latoComune(p2,p3);
    flip(t1,t2,latoComune);
    array<Point> lati_n1 = (p1,p4,p3);
    array<Point> lati_n2 = (p1,p2,p3);
    Triangle t1_nuovo(lati_n1);
    Triangle t2_nuovo(lati_n2);
    bool uguali = false;
    if ((t1_nuovo==t1 && t2_nuovo==t2) || (t1_nuovo==t2 && t2_nuovo==t1))
        uguali = true;

    EXPECT_TRUE(uguali == true);
}


#endif // __TEST_EMPTY_H

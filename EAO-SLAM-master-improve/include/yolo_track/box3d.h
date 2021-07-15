#ifndef BOX3D_H
#define BOX3D_H
#include <algorithm>
#include <string>
#include <vector>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include <fstream>
#include <vector>
#include <string>
#include <iostream>
#include <time.h>
#include "math.h"
#include "stdlib.h"
using namespace std;
using namespace cv;

//#define show_all_3dbox 1

#define DEEP_SORT 1
//#define SORT 1


/* camera setting */
extern float box3d_fx,box3d_fy,box3d_cx,box3d_cy,box3d_DepthMapFactor;

extern float w_scale,l_scale,h_scale;  //xsize , ysize , zsize

/* box2d  setting */
/*
corner order:
    left-up, right-up, left-down, right-down
*/
extern vector<int>  box2d_class;
extern vector<Point2f> box2d_corner;  //camera image coordinate

/* box3d  setting */
/*
corner order:
the box define is:
    4-------------5
   /|            /|
  / |           / |
 7--|----------6  |
 |  0----------|--1
 | /           | /
 |/            |/
 3-------------2
*/
extern vector<int>  box3d_class;
extern vector<Point3f> box3d_corner;   //real world coordinate current frame
extern vector<vector<Point3f>> box3d_corner_all;   //real world coordinate all frames
extern vector<vector<int>> box3d_class_all;

extern Mat box3d_RGB,box3d_depth;

extern vector<Mat> box3d_RT;
extern Mat box3d_current_RT;

extern vector<Point3f> box3d_p1,box3d_p2,box3d_p3,box3d_p4,box3d_p5,box3d_p6,box3d_p7,box3d_p8;


void write_box3dcorner_obj(vector<int>box3d_class, vector<Point3f>box3d_corner, string filename);

#endif  BOX3D_H
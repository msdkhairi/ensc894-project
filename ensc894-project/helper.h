#pragma once

#include "ensc-488.h"

class Transformation {
private:
    double matrix[4][4];

public:
    Transformation();
    Transformation(char type, char axis, double value);

    void rotateX(double theta);
    void rotateY(double theta);
    void rotateZ(double theta);
    void translate(double tx, double ty, double tz);
    void multiply(double mat[4][4]);
    void display();
};
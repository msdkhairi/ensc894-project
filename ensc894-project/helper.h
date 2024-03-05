#pragma once

#include <vector>
#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <cmath>

#include "ensc-488.h"


typedef double ROBSIMDouble;

//template <typename T>
//class Matrix {
//public:
//    // Constructors
//    Matrix();
//    Matrix(int rows, int cols);
//
//    // Element access
//    T& operator()(int row, int col);
//    const T& operator()(int row, int col) const;
//
//    // Getters for dimensions
//    int getRows() const;
//    int getCols() const;
//
//    // Matrix operations
//    Matrix<T> operator+(const Matrix<T>& other) const;
//    Matrix<T> operator-(const Matrix<T>& other) const;
//    Matrix<T> operator*(const Matrix<T>& other) const;
//    Matrix<T> transpose() const;
//
//    // Other methods
//    void setToIdentity();
//    Matrix<ROBSIMDouble> getSub3x3Matrix() const;
//    void display() const;
//
//private:
//    int rows_;
//    int cols_;
//    std::vector<T> data_;
//};

template <typename T>
class Matrix {
public:
    // Default constructor (creates a 1x1 matrix)
    Matrix() : rows_(1), cols_(1), data_(1) {}

    // Constructor with specified dimensions
    Matrix(int rows, int cols) : rows_(rows), cols_(cols), data_(rows* cols) {}

    // Access element at given indices (row, column)
    T& operator()(int row, int col) {
        if (row < 0 || row >= rows_ || col < 0 || col >= cols_) {
            throw std::out_of_range("Index out of bounds");
        }
        return data_[row * cols_ + col];
    }

    const T& operator()(int row, int col) const {
        if (row < 0 || row >= rows_ || col < 0 || col >= cols_) {
            throw std::out_of_range("Index out of bounds");
        }
        return data_[row * cols_ + col];
    }

    // Get the number of rows
    int getRows() const { return rows_; }

    // Get the number of columns
    int getCols() const { return cols_; }

    // Add two matrices (check dimensions first)
    Matrix<T> operator+(const Matrix<T>& other) const {
        if (rows_ != other.rows_ || cols_ != other.cols_) {
            throw std::invalid_argument("Matrices must have the same dimensions for addition");
        }
        Matrix<T> result(rows_, cols_);
        for (int i = 0; i < rows_; ++i) {
            for (int j = 0; j < cols_; ++j) {
                result(i, j) = (*this)(i, j) + other(i, j);
            }
        }
        return result;
    }

    // Subtract two matrices (check dimensions first)
    Matrix<T> operator-(const Matrix<T>& other) const {
        if (rows_ != other.rows_ || cols_ != other.cols_) {
            throw std::invalid_argument("Matrices must have the same dimensions for subtraction");
        }
        Matrix<T> result(rows_, cols_);
        for (int i = 0; i < rows_; ++i) {
            for (int j = 0; j < cols_; ++j) {
                result(i, j) = (*this)(i, j) - other(i, j);
            }
        }
        return result;
    }

    // Multiply two matrices (check compatibility of dimensions first)
    Matrix<T> operator*(const Matrix<T>& other) const {
        if (cols_ != other.rows_) {
            throw std::invalid_argument("Incompatible matrix dimensions for multiplication");
        }
        Matrix<T> result(rows_, other.cols_);
        for (int i = 0; i < rows_; ++i) {
            for (int j = 0; j < other.cols_; ++j) {
                result(i, j) = 0;
                for (int k = 0; k < cols_; ++k) {
                    result(i, j) += (*this)(i, k) * other(k, j);
                }
            }
        }
        return result;
    }

    // Calculate the transpose (works for any matrix)
    Matrix<T> transpose() const {
        Matrix<T> result(cols_, rows_);
        for (int i = 0; i < rows_; ++i) {
            for (int j = 0; j < cols_; ++j) {
                result(j, i) = (*this)(i, j);
            }
        }
        return result;
    }

    // Set the matrix to the identity matrix
    void setToIdentity() {
        if (rows_ != cols_) {
            throw std::invalid_argument("Matrix must be square to be set to identity");
        }
        for (int i = 0; i < rows_; ++i) {
            for (int j = 0; j < cols_; ++j) {
                (*this)(i, j) = (i == j) ? T(1) : T(0);
            }
        }
    }

    Matrix<T> getSub3x3Matrix() const {
        if (rows_ < 3 || cols_ < 3) {
            throw std::invalid_argument("Matrix must be at least 3x3 to get submatrix");
        }
        Matrix<T> submatrix(3, 3);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                submatrix(i, j) = (*this)(i, j);  // Access elements using matrix_
            }
        }
        return submatrix;
    }

    // Display the matrix in a formatted way
    void display() const {
        for (int i = 0; i < rows_; ++i) {
            for (int j = 0; j < cols_; ++j) {
                std::cout << std::setw(7) << std::setprecision(3) << std::fixed << (*this)(i, j);  // Use setw(5) for fixed width of 5 characters
            }
            std::cout << std::endl;
        }
    }

private:
    // Number of rows
    int rows_;

    // Number of columns
    int cols_;

    // Data storage (using a single vector)
    std::vector<T> data_;
};

//// Forward declaration for circular dependency
//class Translation;

class Rotation {
public:
    // Default constructor
    Rotation();

    // Constructor with 3x3 matrix
    Rotation(const Matrix<ROBSIMDouble>& matrix);

    // Constructor with axis and angle
    Rotation(char axis, ROBSIMDouble angle);

    // Access element at given 2D index (row, column)
    ROBSIMDouble& operator()(int row, int col);
    const ROBSIMDouble& operator()(int row, int col) const;

    // Get underlying rotation matrix
    Matrix<ROBSIMDouble> getMatrix() const;

    // Get homogeneous transformation matrix
    Matrix<ROBSIMDouble> getHomogenMatrix() const;

    // Print the rotation matrix
    void display() const;

private:
    Matrix<ROBSIMDouble> matrix_;
    ROBSIMDouble theta_; // Rotation angle (used for constructor with axis)
};

class Translation {
public:
    // Constructor specifying axis and distance
    Translation(char axis, ROBSIMDouble dist);

    // Constructor specifying individual components
    Translation(ROBSIMDouble x = 0.0, ROBSIMDouble y = 0.0, ROBSIMDouble z = 0.0) : x_(x), y_(y), z_(z) {}

    // Getters for x, y, z components
    ROBSIMDouble getX() const { return x_; }
    ROBSIMDouble getY() const { return y_; }
    ROBSIMDouble getZ() const { return z_; }

    // Setters for x, y, z components
    void setX(ROBSIMDouble x) { x_ = x; }
    void setY(ROBSIMDouble y) { y_ = y; }
    void setZ(ROBSIMDouble z) { z_ = z; }

    // Calculate magnitude of the translation vector
    ROBSIMDouble getDistance() const;

    // Get underlying translation matrix (3x1)
    Matrix<ROBSIMDouble> getMatrix() const;

    // Get homogeneous transformation matrix
    Matrix<ROBSIMDouble> getHomogenMatrix() const;

private:
    ROBSIMDouble x_, y_, z_;
};


class Transformation {
public:
    // Default constructor
    Transformation() : rotation_(), translation_() {}

    // Constructor with rotation and translation
    Transformation(const Rotation& rotation, const Translation& translation) : rotation_(rotation), translation_(translation) {}

    // Constructor with a 4x4 matrix (validation included)
    Transformation(const Matrix<ROBSIMDouble>& matrix);

    // Set the rotation
    void setRotation(const Rotation& rotation);

    // Get the rotation matrix
    Rotation getRotation() const;

    // Set the translation
    void setTranslation(const Translation& translation);

    // Get the translation vector
    Translation getTranslation() const;

    // Create a combined 4x4 homogeneous transformation matrix
    Matrix<ROBSIMDouble> getMatrix() const;

    // Multiply the current transformation with another transformation
    Transformation operator*(const Transformation& other) const;

    // Calculate the inverse transformation
    Transformation inverse() const;

    // Display the transformation matrix
    void display() const;

private:
    Rotation rotation_;
    Translation translation_;
    Matrix<ROBSIMDouble> matrix_;
};

class DHParameter {
public:
    // Default constructor
    DHParameter() : alpha(0), a(0), d(0), theta(0) {}

    // Constructor with individual parameters
    DHParameter(ROBSIMDouble alpha, ROBSIMDouble a, ROBSIMDouble d, ROBSIMDouble theta) : alpha(alpha), a(a), d(d), theta(theta) {}

    // Copy constructor
    DHParameter(const DHParameter& dh) : alpha(dh.alpha), a(dh.a), d(dh.d), theta(dh.theta) {}

    // Constructor from a vector
    DHParameter(std::vector<ROBSIMDouble> dh) : alpha(dh[0]), a(dh[1]), d(dh[2]), theta(dh[3]) {}

    // Accessor functions for individual parameters
    ROBSIMDouble getAlpha() const { return alpha; }
    ROBSIMDouble getA() const { return a; }
    ROBSIMDouble getD() const { return d; }
    ROBSIMDouble getTheta() const { return theta; }

    // Get all parameters as a vector
    std::vector<ROBSIMDouble> getParams();

private:
    ROBSIMDouble alpha, a, d, theta;
};


Transformation getTransformationFromDH(DHParameter dh);

Transformation getTransformationFromDHParams(ROBSIMDouble alpha, ROBSIMDouble a, ROBSIMDouble d, ROBSIMDouble theta);

// ROBSIM

namespace ROBSIM {

    //   struct vec4 {
    //       ROBSIMDouble x, y, z, phi;
       //};

       //struct frame {
       //    Transformation trnsfrm;
       //};

    class vec4 {
    public:
        ROBSIMDouble x, y, z, phi;
        vec4(ROBSIMDouble x, ROBSIMDouble y, ROBSIMDouble z, ROBSIMDouble phi) : x(x), y(y), z(z), phi(phi) {}
        vec4() : x(0), y(0), z(0), phi(0) {}
        vec4(const vec4& v) : x(v.x), y(v.y), z(v.z), phi(v.phi) {}
        vec4(const vec4* v) : x(v->x), y(v->y), z(v->z), phi(v->phi) {}
        vec4(std::vector<ROBSIMDouble> v);
        vec4(Matrix<ROBSIMDouble> m);

        void display() {
            std::cout << "x: " << x << " y: " << y << " z: " << z << " phi: " << RAD2DEG(phi) << std::endl;
        }
    };

    class frame {
    public:
        Transformation matrix;
    };

    class q_vec {
    public:
        ROBSIMDouble theta1, theta2, d3, theta4;

        q_vec(ROBSIMDouble theta1, ROBSIMDouble theta2, ROBSIMDouble d3, ROBSIMDouble theta4) : theta1(theta1), theta2(theta2), d3(d3), theta4(theta4) {}
        q_vec() : theta1(0), theta2(0), d3(0), theta4(0) {}
        q_vec(const q_vec& q) : theta1(q.theta1), theta2(q.theta2), d3(q.d3), theta4(q.theta4) {}
        q_vec(const q_vec* q) : theta1(q->theta1), theta2(q->theta2), d3(q->d3), theta4(q->theta4) {}
        q_vec(ROBSIMDouble* v) : theta1(v[0]), theta2(v[1]), d3(v[2]), theta4(v[3]) {}
        q_vec(std::vector<ROBSIMDouble> v);
        q_vec(Matrix<ROBSIMDouble> m);

        // getters 
        std::vector<ROBSIMDouble> toVector() const;
        ROBSIMDouble* toArray() const;
    };

    std::vector<DHParameter> getDHParamsFromQVec(q_vec& q);
    std::vector<Transformation> getTransformationFromDHParamsVector(std::vector<DHParameter> dh_vec);

    void UTOI(vec4& v, frame& f);
    void ITOU(frame& f, vec4& v);

    void TMULT(frame& brela, frame& crelb, frame& crela);
    void TINVERT(frame& brela, frame& arelb);

    class ROBOT {
    public:
        q_vec q_;
        frame station, base, wrelb, trels, brels, trelw;

        void setQ(q_vec& q);
        q_vec getQ() { return q_; }
        std::vector<DHParameter> getDHParamsFromQVec(q_vec& q);
    };

    void KIN(q_vec& q, frame& wrelb);

};
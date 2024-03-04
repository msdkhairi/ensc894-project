#include "helper.h"

#include <iostream>
#include <cmath>
#include <vector>
#include <iomanip>

#include "ensc-488.h"

typedef double RoboticsDouble;


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

    // Calculate the determinant (only for square matrices)
    T determinant() const {
        if (rows_ != cols_) {
            throw std::invalid_argument("Determinant calculation only valid for square matrices");
        }
        if (rows_ == 1) {
            return (*this)(0, 0);
        }
        else if (rows_ == 2) {
            return (*this)(0, 0) * (*this)(1, 1) - (*this)(0, 1) * (*this)(1, 0);
        }
        else {
            T det = 0;
            for (int i = 0; i < cols_; ++i) {
                // Create a submatrix excluding the first row and i-th column
                Matrix<T> submatrix(rows_ - 1, cols_ - 1);
                int sub_row = 0;
                for (int j = 0; j < rows_; ++j) {
                    if (j == 0) {
                        continue; // Skip the first row
                    }
                    int sub_col = 0;
                    for (int k = 0; k < cols_; ++k) {
                        if (k == i) {
                            continue; // Skip the i-th column
                        }
                        submatrix(sub_row, sub_col) = (*this)(j, k);
                        ++sub_col;
                    }
                    ++sub_row;
                }
                // Add cofactor * determinant of the submatrix
                det += pow(-1, i) * (*this)(0, i) * submatrix.determinant();
            }
            return det;
        }
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

    Matrix<RoboticsDouble> getSub3x3Matrix() const {
        if (rows_ < 3 || cols_ < 3) {
			throw std::invalid_argument("Matrix must be at least 3x3 to get submatrix");
		}
        Matrix<RoboticsDouble> submatrix(3, 3);
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


class Rotation {
public:
    // default constructor
    Rotation(): matrix_(3, 3) {
        theta_ = 0.0;
        matrix_.setToIdentity();
    }

    // constructor with 3x3 matrix
    Rotation(const Matrix<RoboticsDouble>& matrix) : matrix_(matrix) {
        if (matrix.getRows() != 3 || matrix.getCols() != 3) {
			throw std::invalid_argument("Matrix must be 3x3 for rotation matrix");
		}
	}

    // constructor with axis and angle
    Rotation(char axis, RoboticsDouble angle) : matrix_(3, 3), theta_(angle) {
        if (axis != 'x' && axis != 'y' && axis != 'z') {
            throw std::invalid_argument("Invalid axis specified. Only 'x', 'y', or 'z' allowed.");
        }

        RoboticsDouble theta = DEG2RAD(angle);

        RoboticsDouble cosTheta = std::cos(theta);
        RoboticsDouble sinTheta = std::sin(theta);

        matrix_.setToIdentity();

        switch (axis) {
        case 'x':
            matrix_(1, 1) = cosTheta;
            matrix_(1, 2) = -sinTheta;
            matrix_(2, 1) = sinTheta;
            matrix_(2, 2) = cosTheta;
            break;
        case 'y':
            matrix_(0, 0) = cosTheta;
            matrix_(0, 2) = sinTheta;
            matrix_(2, 0) = -sinTheta;
            matrix_(2, 2) = cosTheta;
            break;
        case 'z':
            matrix_(0, 0) = cosTheta;
            matrix_(0, 1) = -sinTheta;
            matrix_(1, 0) = sinTheta;
            matrix_(1, 1) = cosTheta;
            break;
        }
    }

    // Access element at given 2D index (row, column)
    RoboticsDouble& operator()(int row, int col) {
        return matrix_(row, col);
    }

    const RoboticsDouble& operator()(int row, int col) const {
        return matrix_(row, col);
    }

    Matrix<RoboticsDouble> getMatrix() const {
		return matrix_;
	}

    Matrix<RoboticsDouble> getHomogenMatrix() const {
        Matrix<RoboticsDouble> homogenMatrix(4, 4);
		homogenMatrix.setToIdentity();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
				homogenMatrix(i, j) = matrix_(i, j);  // Access elements using matrix_
			}
		}
		return homogenMatrix;
	}

    // Print the matrix
    void display() const {
        matrix_.display();
    }

private:
    Matrix<RoboticsDouble> matrix_; 
    RoboticsDouble theta_;     // Rotation angle (used for constructor with axis)  
};


class Translation {
public:
    // Constructors
    // Translation() : x_(0.0), y_(0.0), z_(0.0) {}
    Translation(char axis, RoboticsDouble dist) : x_(0.0), y_(0.0), z_(0.0) {
        switch (axis) {
		case 'x':
			x_ = dist;
			break;
		case 'y':
			y_ = dist;
			break;
		case 'z':
			z_ = dist;
			break;
		}
	}
    Translation(RoboticsDouble x = 0.0, RoboticsDouble y = 0.0, RoboticsDouble z = 0.0) : x_(x), y_(y), z_(z) {}

    // Getters and setters for x, y, z components
    RoboticsDouble getX() const { return x_; }
    RoboticsDouble getY() const { return y_; }
    RoboticsDouble getZ() const { return z_; }

    void setX(RoboticsDouble x) { x_ = x; }
    // Similar getter/setter methods for y and z

    // Calculate the magnitude of the translation vector
    RoboticsDouble getDistance() const {
        return sqrt(x_ * x_ + y_ * y_ + z_ * z_);
    }

    Matrix<RoboticsDouble> getMatrix() const {
        Matrix<RoboticsDouble> translation(3, 1);
        translation(0, 0) = x_;
        translation(1, 0) = y_;
        translation(2, 0) = z_;
        return translation;
    }

    Matrix<RoboticsDouble> getHomogenMatrix() const {
        Matrix<RoboticsDouble> translation(4, 4);
        translation.setToIdentity();
        translation(0, 3) = x_;
        translation(1, 3) = y_;
        translation(2, 3) = z_;
        return translation;
    }

private:
    RoboticsDouble x_, y_, z_;
};

class Transformation {
public:
    Transformation() : rotation_(), translation_() {}
    // Set the rotation
    void setRotation(const Rotation& Rotation) {
        rotation_ = Rotation;
    }

    // Get the rotation matrix
    Rotation getRotation() const {
		return rotation_;
	}

    // Set the translation
    void setTranslation(const Translation& translation) {
        translation_ = translation;
    }
    
    // Get the translation vector
    Translation getTranslation() const {
        return translation_;
    }

    // Create a combined 4x4 homogeneous transformation matrix
    Matrix<RoboticsDouble> getMatrix() const {
        Matrix<RoboticsDouble> matrix(4, 4);
        matrix.setToIdentity();

        // Copy rotation from rotation_
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                matrix(i, j) = rotation_(i, j);
            }
        }

        // Set translation components in the 4th column
        matrix(0, 3) = translation_.getX();
        matrix(1, 3) = translation_.getY();
        matrix(2, 3) = translation_.getZ();

        return matrix;
    }

    // Multiply the current transformation with another transformation
    Transformation operator*(const Transformation& other) const {
        // Get the 4x4 matrices of both transformations
        Matrix<RoboticsDouble> matrix1 = getMatrix();
        Matrix<RoboticsDouble> matrix2 = other.getMatrix();

        // Perform matrix multiplication using the Matrix class
        Matrix<RoboticsDouble> resultMatrix = matrix1 * matrix2;

        // Create a new Transformation object with the combined result
        Transformation result;

        // Extract rotation from the top-left 3x3 submatrix
        Matrix<RoboticsDouble> rotationSubmatrix = resultMatrix.getSub3x3Matrix();
        Rotation combinedRotation(rotationSubmatrix);
        result.setRotation(combinedRotation);

        // Extract translation from the last column
        double tx = resultMatrix(0, 3);
        double ty = resultMatrix(1, 3);
        double tz = resultMatrix(2, 3);
        Translation combinedTranslation(tx, ty, tz);
        result.setTranslation(combinedTranslation);

        return result;
    }

    // Display the transformation matrix
    void display() const {
		getMatrix().display();
	}

private:
    Rotation rotation_;
    Translation translation_;
    Matrix<RoboticsDouble> matrix_;
};

class DHParameter {
public:
    DHParameter() : alpha(0), a(0), d(0), theta(0) {}
    DHParameter(RoboticsDouble alpha, RoboticsDouble a, RoboticsDouble d, RoboticsDouble theta) : alpha(alpha), a(a), d(d), theta(theta) {}
    DHParameter(const DHParameter& dh) : alpha(dh.alpha), a(dh.a), d(dh.d), theta(dh.theta) {}
    DHParameter(std::vector<RoboticsDouble> dh) : alpha(dh[0]), a(dh[1]), d(dh[2]), theta(dh[3]) {}

	RoboticsDouble alpha, a, d, theta;

    std::vector<RoboticsDouble> getParams() {
		std::vector<RoboticsDouble> result;
		result.push_back(alpha);
		result.push_back(a);
		result.push_back(d);
		result.push_back(theta);
		return result;
	}
};

Transformation getTransformationFromDH(DHParameter dh) {
    Transformation result, trans_x, rot_x, trans_z, rot_z;
    trans_x.setTranslation(Translation(dh.a, 0, 0));
    rot_x.setRotation(Rotation('x', dh.alpha));
    trans_z.setTranslation(Translation(0, 0, dh.d));
    rot_z.setRotation(Rotation('z', dh.theta));

    // Calculate the transformation matrix using the DH parameters rot_x * trans_x * rot_z * trans_z
    result = rot_x * trans_x * rot_z * trans_z;

    return result;
}

Transformation getTransformationFromDH(RoboticsDouble alpha, RoboticsDouble a, RoboticsDouble d, RoboticsDouble theta) {
	DHParameter dh(alpha, a, d, theta);
	return getTransformationFromDH(dh);
}


int main() {
    // std::cout << "Hello World!\n";
    Rotation rot('z', 0);
    // rot.display();

    // Transformation t_01 = getTransformationFromDH(60, 3, 5, 90);
    Transformation t_01 = getTransformationFromDH(45, 4, 7, 90);
    Transformation t_12 = getTransformationFromDH(-30, -8, 2, -90);
    Transformation t_23 = getTransformationFromDH(180.0, 12.0, -5.0, 30.0);
    Transformation t_34 = getTransformationFromDH(22.5, 1.0, -1.0, 45.0);
    Transformation t_45 = getTransformationFromDH(60.0, 3.0, 1.34, -22.5);

    // Transformation t_04 = t_34 * t_23 * t_12 * t_01;
    Transformation t_04 = t_01 * t_12 * t_23 * t_34;

    Transformation t_05 = t_01 * t_12 * t_23 * t_34 * t_45;

    std::cout << "t_01:\n";
    t_01.getMatrix().display();
    std::cout << "t_12:\n";
    t_12.getMatrix().display();
    std::cout << "t_23:\n";
    t_23.getMatrix().display();
    std::cout << "t_34:\n";
    t_34.getMatrix().display();
    std::cout << "t_45:\n";
    t_45.getMatrix().display();
    std::cout << "t_04:\n";
    t_04.getMatrix().display();
    std::cout << "t_05:\n";
    t_05.getMatrix().display();

    // std::cout << submatrix.size() << std::endl;
}
    
// transformation matrix for homogeneous coordinates for rotation about the x, y, or z axis by an angle theta in a class
// with a method to rotate a point about the x, y, or z axis by an angle theta

class Matrix4x4 {
    //private:

public:
    double matrix[4][4];
    // Constructor
    Matrix4x4() {
        // Initialize the matrix to zeros
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                if (i == j)
                    matrix[i][j] = 1.0;
                else
                    matrix[i][j] = 0.0;
            }
        }
    }

    // destructor
    ~Matrix4x4() {
    }

    void setMatrix(double mat[4][4]) {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                matrix[i][j] = mat[i][j];
            }
        }
    }

    void setMatrixToIdentity() {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                if (i == j)
                    matrix[i][j] = 1.0;
                else
                    matrix[i][j] = 0.0;
            }
        }
    }

    void setMatrixToZero() {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                matrix[i][j] = 0.0;
            }
        }
    }

    // Function to set the value at a specific position
    void set(int row, int col, double value) {
        matrix[row][col] = value;
    }

    // Function to get the value at a specific position
    double get(int row, int col) const {
        return matrix[row][col];
    }

    // Function to add two matrices
    Matrix4x4 add(const Matrix4x4& other) const {
        Matrix4x4 result;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                result.matrix[i][j] = matrix[i][j] + other.matrix[i][j];
            }
        }
        return result;
    }

    // Function to multiply two matrices
    Matrix4x4 multiply(const Matrix4x4& other) const {
        Matrix4x4 result;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                double sum = 0.0;
                for (int k = 0; k < 4; ++k) {
                    sum += matrix[i][k] * other.matrix[k][j];
                }
                result.matrix[i][j] = sum;
            }
        }
        return result;
    }

    // Function to calculate the inverse of the matrix
    Matrix4x4 inverse() const {
        Matrix4x4 result;

        // Calculate the determinant of the matrix
        double det = determinant();

        // Check if the determinant is non-zero
        if (std::abs(det) < 1e-10) {
            std::cerr << "Matrix is singular, cannot find inverse." << std::endl;
            return result; // Return a zero matrix
        }

        // Calculate the inverse using the adjugate matrix
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                result.matrix[i][j] = cofactor(j, i) / det; // Transpose of cofactor matrix
            }
        }

        return result;
    }

    // Function to calculate the determinant of the matrix
    double determinant() const {
        double det = 0.0;
        // Use cofactor expansion along the first row
        for (int i = 0; i < 4; ++i) {
            det += matrix[0][i] * cofactor(0, i);
        }
        return det;
    }

    // Function to calculate the cofactor of a specific element
    double cofactor(int row, int col) const {
        // Create a 3x3 submatrix excluding the given row and column
        double submatrix[3][3];
        int sub_i = 0, sub_j = 0;
        for (int i = 0; i < 4; ++i) {
            if (i == row) continue;
            for (int j = 0; j < 4; ++j) {
                if (j == col) continue;
                submatrix[sub_i][sub_j++] = matrix[i][j];
            }
            sub_i++;
            sub_j = 0;
        }

        // Calculate the determinant of the submatrix
        double det = submatrix[0][0] * (submatrix[1][1] * submatrix[2][2] - submatrix[1][2] * submatrix[2][1]) -
            submatrix[0][1] * (submatrix[1][0] * submatrix[2][2] - submatrix[1][2] * submatrix[2][0]) +
            submatrix[0][2] * (submatrix[1][0] * submatrix[2][1] - submatrix[1][1] * submatrix[2][0]);

        // Return the cofactor
        return (row + col) % 2 == 0 ? det : -det;
    }

    // Function to display the matrix
    void display() const {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                std::cout << matrix[i][j] << " ";
            }
            std::cout << std::endl;
        }
    }
};


class RotationMatrix : public Matrix4x4 {
public:
    char axis;
    double angle;

    // Constructor
    RotationMatrix(char axis, double angle) : axis(axis), angle(angle) {
        setMatrixToIdentity();
        calculateRotationMatrix();
    }

private:
    // Function to calculate the rotation matrix based on axis and angle
    void calculateRotationMatrix() {
        double radians = DEG2RAD(angle); // Convert angle to radians

        switch (axis) {
        case 'x':
        case 'X':
            matrix[1][1] = cos(radians);
            matrix[1][2] = -sin(radians);
            matrix[2][1] = sin(radians);
            matrix[2][2] = cos(radians);
            break;
        case 'y':
        case 'Y':
            matrix[0][0] = cos(radians);
            matrix[0][2] = sin(radians);
            matrix[2][0] = -sin(radians);
            matrix[2][2] = cos(radians);
            break;
        case 'z':
        case 'Z':
            matrix[0][0] = cos(radians);
            matrix[0][1] = -sin(radians);
            matrix[1][0] = sin(radians);
            matrix[1][1] = cos(radians);
            break;
        default:
            std::cerr << "Invalid axis specified!" << std::endl;
        }
    }
};


class TranslationMatrix : public Matrix4x4 {
public:
    double distanceX, distanceY, distanceZ;
    char axis;

    // Constructor taking individual distances along x, y, z axes
    TranslationMatrix(double dx, double dy, double dz) : distanceX(dx), distanceY(dy), distanceZ(dz), axis('f') {
        setMatrixToIdentity();
        calculateTranslationMatrix();
    }

    // Constructor taking a single distance and axis
    TranslationMatrix(char axis, double distance) : distanceX(0.0), distanceY(0.0), distanceZ(0.0), axis(axis) {
        switch (axis) {
        case 'x':
        case 'X':
            distanceX = distance;
            break;
        case 'y':
        case 'Y':
            distanceY = distance;
            break;
        case 'z':
        case 'Z':
            distanceZ = distance;
            break;
        default:
            std::cerr << "Invalid axis specified!" << std::endl;
        }
        calculateTranslationMatrix();
    }
private:
    // Function to calculate the translation matrix based on distances along x, y, z axes
    void calculateTranslationMatrix() {
        matrix[0][3] = distanceX;
        matrix[1][3] = distanceY;
        matrix[2][3] = distanceZ;
    }
};




//class Transformation {
//private:
//    Matrix4x4 T;
//
//public:
//    Transformation() {
//        // Initialize the matrix to the identity matrix
//    }
//
//    // Constructor with initialization using axis and angle or translation parameters
//    Transformation(char type, char axis, double value) {
//        // Initialize the matrix to the identity matrix
//        for (int i = 0; i < 4; ++i) {
//            for (int j = 0; j < 4; ++j) {
//                if (i == j)
//                    T.matrix[i][j] = 1.0;
//                else
//                    T.matrix[i][j] = 0.0;
//            }
//        }
//
//        // Apply rotation or translation based on the given type, axis, and value
//        if (type == 'r') {
//            if (axis == 'x')
//                rotateX(value);
//            else if (axis == 'y')
//                rotateY(value);
//            else if (axis == 'z')
//                rotateZ(value);
//            else
//                std::cerr << "Invalid axis. Please use 'x', 'y', or 'z'." << std::endl;
//        }
//        else if (type == 't') {
//            if (axis == 'x')
//                translate(value, 0.0, 0.0);
//            else if (axis == 'y')
//                translate(0.0, value, 0.0);
//            else if (axis == 'z')
//                translate(0.0, 0.0, value);
//            else
//                std::cerr << "Invalid axis. Please use 'x', 'y', or 'z'." << std::endl;
//        }
//        else {
//            std::cerr << "Invalid transformation type. Please use 'r' for rotation or 't' for translation." << std::endl;
//        }
//    }
//
//    // Function to perform rotation about the x axis
//    void rotateX(double theta) {
//        double radians = DEG2RAD(theta);
//        double cosTheta = cos(radians);
//        double sinTheta = sin(radians);
//
//        double rotationMatrix[4][4] = {
//            {1.0, 0.0, 0.0, 0.0},
//            {0.0, cosTheta, -sinTheta, 0.0},
//            {0.0, sinTheta, cosTheta, 0.0},
//            {0.0, 0.0, 0.0, 1.0}
//        };
//
//        // Multiply the current matrix by the rotation matrix
//        multiply(rotationMatrix);
//    }
//
//    // Function to perform rotation about the y axis
//    void rotateY(double theta) {
//        double radians = DEG2RAD(theta);
//        double cosTheta = cos(radians);
//        double sinTheta = sin(radians);
//
//        double rotationMatrix[4][4] = {
//            {cosTheta, 0.0, sinTheta, 0.0},
//            {0.0, 1.0, 0.0, 0.0},
//            {-sinTheta, 0.0, cosTheta, 0.0},
//            {0.0, 0.0, 0.0, 1.0}
//        };
//
//        // Multiply the current matrix by the rotation matrix
//        multiply(rotationMatrix);
//    }
//
//    // Function to perform rotation about the z axis
//    void rotateZ(double theta) {
//        double radians = DEG2RAD(theta);
//        double cosTheta = cos(radians);
//        double sinTheta = sin(radians);
//
//        double rotationMatrix[4][4] = {
//            {cosTheta, -sinTheta, 0.0, 0.0},
//            {sinTheta, cosTheta, 0.0, 0.0},
//            {0.0, 0.0, 1.0, 0.0},
//            {0.0, 0.0, 0.0, 1.0}
//        };
//
//        // Multiply the current matrix by the rotation matrix
//        multiply(rotationMatrix);
//    }
//
//    // Function to perform translation along x, y, or z axis
//    void translate(double tx, double ty, double tz) {
//        double translationMatrix[4][4] = {
//            {1.0, 0.0, 0.0, tx},
//            {0.0, 1.0, 0.0, ty},
//            {0.0, 0.0, 1.0, tz},
//            {0.0, 0.0, 0.0, 1.0}
//        };
//
//        // Multiply the current matrix by the translation matrix
//        multiply(translationMatrix);
//    }
//
//    // Function to multiply the current matrix by another matrix
//    void multiply(double mat[4][4]) {
//        double result[4][4];
//
//        for (int i = 0; i < 4; ++i) {
//            for (int j = 0; j < 4; ++j) {
//                result[i][j] = 0;
//                for (int k = 0; k < 4; ++k) {
//                    result[i][j] += matrix[i][k] * mat[k][j];
//                }
//            }
//        }
//
//        // Update the current matrix with the result
//        for (int i = 0; i < 4; ++i) {
//            for (int j = 0; j < 4; ++j) {
//                matrix[i][j] = result[i][j];
//            }
//        }
//    }
//
//    // Function to display the transformation matrix
//    void display() {
//        std::cout << "Transformation Matrix:" << std::endl;
//        for (int i = 0; i < 4; ++i) {
//            for (int j = 0; j < 4; ++j) {
//                std::cout << matrix[i][j] << " ";
//            }
//            std::cout << std::endl;
//        }
//    }
//};

Matrix4x4 getTransformationFromDH(double alpha, double a, double d, double theta) {
    Matrix4x4 transformation, transformation2;

    RotationMatrix Rx('x', alpha);
    RotationMatrix Rz('z', theta);
    TranslationMatrix Tx(a, 0.0, 0.0);
    TranslationMatrix Tz(0.0, 0.0, d);

    transformation = Tz.multiply(Rz.multiply(Tx.multiply(Rx)));
    transformation2 = Rx.multiply(Tx.multiply(Rz.multiply(Tz)));

    return transformation;
}


Transformation getTransformationFromDH(RoboticsDouble alpha, RoboticsDouble a, RoboticsDouble d, RoboticsDouble theta) {
    Transformation result, trans_x, rot_x, trans_z, rot_z;
    trans_x.setTranslation(Translation(a, 0, 0));
    rot_x.setRotation(Rotation('x', alpha));
    trans_z.setTranslation(Translation(0, 0, d));
    rot_z.setRotation(Rotation('z', theta));

    // Calculate the transformation matrix using the DH parameters rot_x * trans_x * rot_z * trans_z
    result = rot_x * trans_x * rot_z * trans_z;

    return result;
}
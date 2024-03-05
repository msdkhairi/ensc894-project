#include "helper.h"

static const ROBSIMDouble L0 = 405;
static const ROBSIMDouble L1 = 70;
static const ROBSIMDouble L2 = 195;
static const ROBSIMDouble L3 = 142;
static const ROBSIMDouble L4 = 410 + 130;
static const ROBSIMDouble L5 = 10;

Rotation::Rotation() : matrix_(3, 3) {
	theta_ = 0.0;
	matrix_.setToIdentity();
}

Rotation::Rotation(const Matrix<ROBSIMDouble>& matrix) : matrix_(matrix) {
    if (matrix.getRows() != 3 || matrix.getCols() != 3) {
		throw std::invalid_argument("Matrix must be 3x3 for rotation matrix");
	}
	theta_ = atan2(matrix(1, 0), matrix(0, 0));
}

Rotation::Rotation(char axis, ROBSIMDouble angle) : matrix_(3, 3), theta_(angle) {
    if (axis != 'x' && axis != 'y' && axis != 'z') {
		throw std::invalid_argument("Invalid axis specified. Only 'x', 'y', or 'z' allowed.");
	}

	ROBSIMDouble theta = DEG2RAD(angle);

	ROBSIMDouble cosTheta = std::cos(theta);
	ROBSIMDouble sinTheta = std::sin(theta);

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

ROBSIMDouble& Rotation::operator()(int row, int col) {
	return matrix_(row, col);
}

const ROBSIMDouble& Rotation::operator()(int row, int col) const {
	return matrix_(row, col);
}

Matrix<ROBSIMDouble> Rotation::getMatrix() const {
	return matrix_;
}

Matrix<ROBSIMDouble> Rotation::getHomogenMatrix() const {
	Matrix<ROBSIMDouble> homogenMatrix(4, 4);
	homogenMatrix.setToIdentity();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
			homogenMatrix(i, j) = matrix_(i, j);  // Access elements using matrix_
		}
	}
	return homogenMatrix;
}

void Rotation::display() const {
	matrix_.display();
}


// Translation class

Translation::Translation(char axis, ROBSIMDouble dist) : x_(0.0), y_(0.0), z_(0.0) {
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

ROBSIMDouble Translation::getDistance() const {
	return sqrt(x_ * x_ + y_ * y_ + z_ * z_);
}

Matrix<ROBSIMDouble> Translation::getMatrix() const {
	Matrix<ROBSIMDouble> translation(3, 1);
	translation(0, 0) = x_;
	translation(1, 0) = y_;
	translation(2, 0) = z_;
	return translation;
}

Matrix<ROBSIMDouble> Translation::getHomogenMatrix() const {
	Matrix<ROBSIMDouble> translation(4, 4);
	translation.setToIdentity();
	translation(0, 3) = x_;
	translation(1, 3) = y_;
	translation(2, 3) = z_;
	return translation;
}

// Transformation class

Transformation::Transformation(const Matrix<ROBSIMDouble>& matrix) : matrix_(matrix) {
    if (matrix.getRows() != 4 || matrix.getCols() != 4) {
		throw std::invalid_argument("Matrix must be 4x4 for transformation matrix");
	}
	// Extract rotation from the top-left 3x3 submatrix
	Matrix<ROBSIMDouble> rotationSubmatrix = matrix.getSub3x3Matrix();
	rotation_ = Rotation(rotationSubmatrix);

	// Extract translation from the last column
	double tx = matrix(0, 3);
	double ty = matrix(1, 3);
	double tz = matrix(2, 3);
	translation_ = Translation(tx, ty, tz);
}

void Transformation::setRotation(const Rotation& Rotation) {
	rotation_ = Rotation;
}

Rotation Transformation::getRotation() const {
	return rotation_;
}

void Transformation::setTranslation(const Translation& translation) {
	translation_ = translation;
}

Translation Transformation::getTranslation() const {
	return translation_;
}

Matrix<ROBSIMDouble> Transformation::getMatrix() const {
	Matrix<ROBSIMDouble> matrix(4, 4);
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

Transformation Transformation::operator*(const Transformation& other) const {
	// Get the 4x4 matrices of both transformations
	Matrix<ROBSIMDouble> matrix1 = getMatrix();
	Matrix<ROBSIMDouble> matrix2 = other.getMatrix();

	// Perform matrix multiplication using the Matrix class
	Matrix<ROBSIMDouble> resultMatrix = matrix1 * matrix2;

	// Create a new Transformation object with the combined result
	Transformation result(resultMatrix);
	return result;
}

Transformation Transformation::inverse() const {
	// Get the 4x4 matrix of the current transformation
	Matrix<ROBSIMDouble> matrix = getMatrix();

	// Calculate the inverse of the rotation matrix
	Matrix<ROBSIMDouble> rotationInverse = rotation_.getMatrix().transpose();

	// Calculate the inverse of the translation matrix
	Matrix<ROBSIMDouble> translationInverse = translation_.getMatrix();
	for (int i = 0; i < 3; ++i) {
		translationInverse(i, 0) *= -1;
	}
	translationInverse = rotationInverse * translationInverse;

	// Create a new Transformation object with the combined result
	Transformation result;
	result.setRotation(Rotation(rotationInverse));
	result.setTranslation(Translation(translationInverse(0, 0), translationInverse(1, 0), translationInverse(2, 0)));
	return result;
}

void Transformation::display() const {
	// Display the transformation matrix
	getMatrix().display();
}

// DHParameter class

std::vector<ROBSIMDouble> DHParameter::getParams() {
	std::vector<ROBSIMDouble> result;
	result.push_back(alpha);
	result.push_back(a);
	result.push_back(d);
	result.push_back(theta);
	return result;
}



Transformation getTransformationFromDH(DHParameter dh) {
    Transformation result, trans_x, rot_x, trans_z, rot_z;
    trans_x.setTranslation(Translation(dh.getA(), 0, 0));
    rot_x.setRotation(Rotation('x', dh.getAlpha()));
    trans_z.setTranslation(Translation(0, 0, dh.getD()));
    rot_z.setRotation(Rotation('z', dh.getTheta()));

    // Calculate the transformation matrix using the DH parameters rot_x * trans_x * rot_z * trans_z
    result = rot_x * trans_x * rot_z * trans_z;

    return result;
}

Transformation getTransformationFromDHParams(ROBSIMDouble alpha, ROBSIMDouble a, ROBSIMDouble d, ROBSIMDouble theta) {
	DHParameter dh(alpha, a, d, theta);
	return getTransformationFromDH(dh);
}


ROBSIM::vec4::vec4(std::vector<double> v) {
	x = v[0];
	y = v[1];
	z = v[2];
	phi = v[3];
}

ROBSIM::vec4::vec4(Matrix<ROBSIMDouble> m) {
	// check if the matrix is 4x1
	if (m.getRows() != 4 || m.getCols() != 1) {
		throw std::invalid_argument("Matrix must be 4x1 for vec4");
	}
	x = m(0, 0);
	y = m(1, 0);
	z = m(2, 0);
	phi = m(3, 0);
}


// q_vec class
ROBSIM::q_vec::q_vec(std::vector<ROBSIMDouble> v) {
	// check if the vector is 4x1
	if (v.size() != 4) {
		throw std::invalid_argument("Vector must be 4x1 for q_vec");
	}
	theta1 = v[0];
	theta2 = v[1];
	d3 = v[2];
	theta4 = v[3];
}

ROBSIM::q_vec::q_vec(Matrix<ROBSIMDouble> m) {
	// check if the matrix is 4x1
	if (m.getRows() != 4 || m.getCols() != 1) {
		throw std::invalid_argument("Matrix must be 4x1 for q_vec");
	}
	theta1 = m(0, 0);
	theta2 = m(1, 0);
	d3 = m(2, 0);
	theta4 = m(3, 0);
}

std::vector<ROBSIMDouble> ROBSIM::q_vec::toVector() const {
	std::vector<ROBSIMDouble> v = { theta1, theta2, d3, theta4 };
	return v;
}

ROBSIMDouble* ROBSIM::q_vec::toArray() const {
	ROBSIMDouble* v = new ROBSIMDouble[4];
	v[0] = theta1;
	v[1] = theta2;
	v[2] = d3;
	v[3] = theta4;
	return v;
}


std::vector<DHParameter> ROBSIM::getDHParamsFromQVec(q_vec& q) {
	std::vector<DHParameter> dhParams;
	dhParams.push_back(DHParameter(0, 0, L0+L1, q.theta1));
	dhParams.push_back(DHParameter(0, L2, 0, q.theta2));
	dhParams.push_back(DHParameter(180, L3, L4+q.d3, 0));
	dhParams.push_back(DHParameter(0, 0, 0, q.theta4));
	dhParams.push_back(DHParameter(0, 0, L5, 0));
	return dhParams;
}

std::vector<Transformation> ROBSIM::getTransformationFromDHParamsVector(std::vector<DHParameter> dh_vec) {
	std::vector<Transformation> result;
	for (unsigned i = 0; i < dh_vec.size(); i++) {
		result.push_back(getTransformationFromDH(dh_vec[i]));
	}
	return result;
}

void ROBSIM::UTOI(vec4& v, frame& f) {
	Rotation rotation('z', v.phi);
	Translation translation(v.x, v.y, v.z);
	Transformation transformation(rotation, translation);
	f.matrix = transformation;
}

void ROBSIM::ITOU(frame& f, vec4& v) {
	Transformation transformation = f.matrix;
	Rotation rotation = transformation.getRotation();
	Translation translation = transformation.getTranslation();
	v.x = translation.getX();
	v.y = translation.getY();
	v.z = translation.getZ();
	v.phi = atan2(rotation(1, 0), rotation(0, 0));
}

void ROBSIM::TMULT(frame& brela, frame& crelb, frame& crela) {
	crela.matrix = brela.matrix * crelb.matrix;
}

void ROBSIM::TINVERT(frame& brela, frame& arelb) {
	arelb.matrix = brela.matrix.inverse();
}

void ROBSIM::ROBOT::setQ(q_vec& q) {
	// check constraints
	// theta1 is in -150 to 150
	if (q.theta1 < -150 || q.theta1 > 150) {
		throw std::invalid_argument("theta1 must be in range -150 to 150");
	}
	// theta2 is in -100 to 100
	if (q.theta2 < -100 || q.theta2 > 100) {
		throw std::invalid_argument("theta2 must be in range -100 to 100");
	}
	// d3 is between -200 and -100
	if (q.d3 < -200 || q.d3 > -100) {
		throw std::invalid_argument("d3 must be in range -200 to -100");
	}
	// theta4 is in -160 to 160
	if (q.theta4 < -160 || q.theta4 > 160) {
		throw std::invalid_argument("theta4 must be in range -160 to 160");
	}
	q_ = q;
}


void ROBSIM::KIN(q_vec& q, frame& wrelb) {
	
	auto dhParams = getDHParamsFromQVec(q);
	auto transformations = getTransformationFromDHParamsVector(dhParams);

	// mulitply the transformations in vector to get the final transformation using for loop
	Transformation result = transformations[0];
	for (unsigned i = 1; i < transformations.size(); i++) {
		result = result * transformations[i];
	}
	wrelb.matrix = result;
}



//int main() {
//    // std::cout << "Hello World!\n";
//    Rotation rot('z', 0);
//    // rot.display();
//
//    // Transformation t_01 = getTransformationFromDH(60, 3, 5, 90);
//    Transformation t_01 = getTransformationFromDH(45, 4, 7, 90);
//    Transformation t_12 = getTransformationFromDH(-30, -8, 2, -90);
//    Transformation t_23 = getTransformationFromDH(180.0, 12.0, -5.0, 30.0);
//    Transformation t_34 = getTransformationFromDH(22.5, 1.0, -1.0, 45.0);
//    Transformation t_45 = getTransformationFromDH(60.0, 3.0, 1.34, -22.5);
//
//    // Transformation t_04 = t_34 * t_23 * t_12 * t_01;
//    Transformation t_04 = t_01 * t_12 * t_23 * t_34;
//
//    Transformation t_05 = t_01 * t_12 * t_23 * t_34 * t_45;
//
//    std::cout << "t_01:\n";
//    t_01.getMatrix().display();
//    std::cout << "t_12:\n";
//    t_12.getMatrix().display();
//    std::cout << "t_23:\n";
//    t_23.getMatrix().display();
//    std::cout << "t_34:\n";
//    t_34.getMatrix().display();
//    std::cout << "t_45:\n";
//    t_45.getMatrix().display();
//    std::cout << "t_04:\n";
//    t_04.getMatrix().display();
//    std::cout << "t_05:\n";
//    t_05.getMatrix().display();
//
//    // std::cout << submatrix.size() << std::endl;
//}
//    
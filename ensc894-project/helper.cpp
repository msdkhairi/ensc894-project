#include "helper.h"

static const ROBSIMDouble L0 = 405;
static const ROBSIMDouble L1 = 70;
static const ROBSIMDouble L2 = 195;
static const ROBSIMDouble L3 = 142;
static const ROBSIMDouble L4 = 410 + 130;
static const ROBSIMDouble L5 = 10;
static const ROBSIMDouble THETA5 = 0;
//static const bool TOOL_FRAME_UP = false;

template<typename T>
Matrix<T>::Matrix(int rows, int cols) : rows_(rows), cols_(cols), data_(rows* cols) {}

template<typename T>
T& Matrix<T>::operator()(int row, int col) {
	if (row < 0 || row >= rows_ || col < 0 || col >= cols_) {
		throw std::out_of_range("index out of bounds");
	}
	return data_[row * cols_ + col];
}

template<typename T>
const T& Matrix<T>::operator()(int row, int col) const {
	if (row < 0 || row >= rows_ || col < 0 || col >= cols_) {
		throw std::out_of_range("Index out of bounds");
	}
	return data_[row * cols_ + col];
}

template<typename T>
int Matrix<T>::getRows() const {
	return rows_;
}

template<typename T>
int Matrix<T>::getCols() const {
	return cols_;
}

template<typename T>
Matrix<T> Matrix<T>::operator+(const Matrix<T>& other) const {
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
template<typename T>
Matrix<T> Matrix<T>::operator-(const Matrix<T>& other) const {
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
template<typename T>
Matrix<T> Matrix<T>::operator*(const Matrix<T>& other) const {
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
template<typename T>
Matrix<T> Matrix<T>::transpose() const {
	Matrix<T> result(cols_, rows_);
	for (int i = 0; i < rows_; ++i) {
		for (int j = 0; j < cols_; ++j) {
			result(j, i) = (*this)(i, j);
		}
	}
	return result;
}

// Set the matrix to the identity matrix
template<typename T>
void Matrix<T>::setToIdentity() {
	if (rows_ != cols_) {
		throw std::invalid_argument("Matrix must be square to be set to identity");
	}
	for (int i = 0; i < rows_; ++i) {
		for (int j = 0; j < cols_; ++j) {
			(*this)(i, j) = (i == j) ? T(1) : T(0);
		}
	}
}

template<typename T>
Matrix<T> Matrix<T>::getSub3x3Matrix() const {
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
template<typename T>
void Matrix<T>::display() const {
	for (int i = 0; i < rows_; ++i) {
		for (int j = 0; j < cols_; ++j) {
			std::cout << std::setw(9) << std::setprecision(3) << std::fixed << (*this)(i, j);
		}
		std::cout << std::endl;
	}
}




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
    if (axis != 'x' && axis != 'y' && axis != 'z' && axis != 't') {
		throw std::invalid_argument("Invalid axis specified. Only 'x', 'y', 'z', 't' allowed.");
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
	case 't':
		matrix_(0, 0) = cosTheta;
		matrix_(0, 1) = sinTheta;
		matrix_(1, 0) = sinTheta;
		matrix_(1, 1) = -cosTheta;
		matrix_(2, 2) = -1;
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

Rotation Rotation::operator*(const Rotation& other) const {
	Matrix<ROBSIMDouble> resultMatrix = matrix_ * other.getMatrix();
	return Rotation(resultMatrix);
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
	//Transformation screw_x = rot_x * trans_x;
	//Transformation screw_z = rot_z * trans_z;
	//result = screw_x * screw_z;
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

void ROBSIM::q_vec::toJOINT(JOINT& result) const {
	result[0] = theta1;
	result[1] = theta2;
	result[2] = d3;
	result[3] = theta4;
}

ROBSIMDouble ROBSIM::getDistance(ROBSIMDouble theta_g, ROBSIMDouble theta_s) {
	ROBSIMDouble dist1 = abs(theta_g - theta_s);
	ROBSIMDouble dist2 = -sign(theta_g - theta_s) * 360 + (theta_g - theta_s);
	return abs(dist1) < abs(dist2) ? dist1 : dist2;
}

ROBSIMDouble ROBSIM::q_vec::getDistance(q_vec& q, bool theta1_any) {
	ROBSIMDouble distance = 0;
	std::vector<ROBSIMDouble> v1 = toVector();
	std::vector<ROBSIMDouble> v2 = q.toVector();
	for (unsigned i = 0; i < v1.size(); i++) {
		if (i == 0 && theta1_any) continue;  // skip theta1 if any theta1 is valid
		//if (i == 2) continue;  // skip d3
		distance += pow(ROBSIM::getDistance(v1[i], v2[i]), 2) * q_dist_weights_[i];
	}
	return sqrt(distance);
}

ROBSIMDouble ROBSIM::getDistance(q_vec& q1, q_vec& q2, bool theta1_any) {
	return q1.getDistance(q2);
}

void ROBSIM::q_vec::display() {
	std::cout << "theta1: " << theta1 << " theta2: " << theta2 << " d3: " << d3 << " theta4: " << theta4 << std::endl << std::flush;
}


std::vector<DHParameter> ROBSIM::getDHParamsFromQVec(q_vec& q) {
	std::vector<DHParameter> dhParams;
	dhParams.push_back(DHParameter(0, 0, 0, 0));
	dhParams.push_back(DHParameter(0, 0, L0+L1, q.theta1));
	dhParams.push_back(DHParameter(0, L2, 0, q.theta2));
	dhParams.push_back(DHParameter(180, L3, L4+q.d3, 0));
	dhParams.push_back(DHParameter(0, 0, 0, q.theta4));
	dhParams.push_back(DHParameter(0, 0, L5/2, THETA5));
	return dhParams;
}

std::vector<Transformation> ROBSIM::getTransformationFromDHParamsVector(std::vector<DHParameter> dh_vec) {
	std::vector<Transformation> result;
	for (unsigned i = 0; i < dh_vec.size(); i++) {
		result.push_back(getTransformationFromDH(dh_vec[i]));
	}
	return result;
}

ROBSIMDouble ROBSIM::sign(ROBSIMDouble x) {
	return (x > 0) - (x < 0);
}

bool ROBSIM::ROBOT::check_q_limits_(ROBSIMDouble param, ROBSIMDouble min, ROBSIMDouble max) {
	return (param >= min && param <= max);
}

bool ROBSIM::ROBOT::check_q_limits_(ROBSIMDouble param, ParameterLimits paramlimits, ROBSIMDouble tolerance) {
	return (param >= paramlimits.min - tolerance && param <= paramlimits.max + tolerance);
	//return (param >= paramlimits.min && param <= paramlimits.max);
}

bool ROBSIM::ROBOT::check_q_limits(q_vec& q, bool verbose) {
	ROBSIMDouble tolerance = 0.0001;
	if (!check_q_limits_(q.theta1, q_limits_.at("theta1"), tolerance)) {
		if (verbose) std::cerr << "theta1 out of limits: " << q.theta1 << " not in [" << q_limits_.at("theta1").min << ", " << q_limits_.at("theta1").max << "]" << std::endl;
		return false;
	}
	if (!check_q_limits_(q.theta2, q_limits_.at("theta2"), tolerance)) {
		if (verbose) std::cerr << "theta2 out of limits: " << q.theta2 << " not in [" << q_limits_.at("theta2").min << ", " << q_limits_.at("theta2").max << "]" << std::endl;
		return false;
	}
	if (!check_q_limits_(q.d3, q_limits_.at("d3"), tolerance)) {
		if (verbose) std::cerr << "d3 out of limits: " << q.d3 << " not in [" << q_limits_.at("d3").min << ", " << q_limits_.at("d3").max << "]" << std::endl;
		return false;
	}
	if (!check_q_limits_(q.theta4, q_limits_.at("theta4"), tolerance)) {
		if (verbose) std::cerr << "theta4 out of limits: " << q.theta4 << " not in [" << q_limits_.at("theta4").min << ", " << q_limits_.at("theta4").max << "]" << std::endl;
		return false;
	}
	adjust_q_within_tolerance_(q, tolerance);
	return true;
}

bool ROBSIM::ROBOT::check_q_vel_limits(q_vec& q_vel, bool verbose) {
	ROBSIMDouble tolerance = 0.0001;
	if (!check_q_limits_(q_vel.theta1, q_vel_limits_.at("theta1"), tolerance)) {
		if (verbose) std::cerr << "theta1 velocity out of limits: " << q_vel.theta1 << " not in [" << q_vel_limits_.at("theta1").min << ", " << q_vel_limits_.at("theta1").max << "]" << std::endl;
		return false;
	}
	if (!check_q_limits_(q_vel.theta2, q_vel_limits_.at("theta2"), tolerance)) {
		if (verbose) std::cerr << "theta2 velocity out of limits: " << q_vel.theta2 << " not in [" << q_vel_limits_.at("theta2").min << ", " << q_vel_limits_.at("theta2").max << "]" << std::endl;
		return false;
	}
	if (!check_q_limits_(q_vel.d3, q_vel_limits_.at("d3"), tolerance)) {
		if (verbose) std::cerr << "d3 velocity out of limits: " << q_vel.d3 << " not in [" << q_vel_limits_.at("d3").min << ", " << q_vel_limits_.at("d3").max << "]" << std::endl;
		return false;
	}
	if (!check_q_limits_(q_vel.theta4, q_vel_limits_.at("theta4"), tolerance)) {
		if (verbose) std::cerr << "theta4 velocity out of limits: " << q_vel.theta4 << " not in [" << q_vel_limits_.at("theta4").min << ", " << q_vel_limits_.at("theta4").max << "]" << std::endl;
		return false;
	}
	adjust_q_within_tolerance_(q_vel, tolerance);
	return true;
}

bool ROBSIM::ROBOT::check_q_acc_limits(q_vec& q_acc, bool verbose) {
	ROBSIMDouble tolerance = 0.0001;
	if (!check_q_limits_(q_acc.theta1, q_acc_limits_.at("theta1"), tolerance)) {
		if (verbose) std::cerr << "theta1 acceleration out of limits: " << q_acc.theta1 << " not in [" << q_acc_limits_.at("theta1").min << ", " << q_acc_limits_.at("theta1").max << "]" << std::endl;
		return false;
	}
	if (!check_q_limits_(q_acc.theta2, q_acc_limits_.at("theta2"), tolerance)) {
		if (verbose) std::cerr << "theta2 acceleration out of limits: " << q_acc.theta2 << " not in [" << q_acc_limits_.at("theta2").min << ", " << q_acc_limits_.at("theta2").max << "]" << std::endl;
		return false;
	}
	if (!check_q_limits_(q_acc.d3, q_acc_limits_.at("d3"), tolerance)) {
		if (verbose) std::cerr << "d3 acceleration out of limits: " << q_acc.d3 << " not in [" << q_acc_limits_.at("d3").min << ", " << q_acc_limits_.at("d3").max << "]" << std::endl;
		return false;
	}
	if (!check_q_limits_(q_acc.theta4, q_acc_limits_.at("theta4"), tolerance)) {
		if (verbose) std::cerr << "theta4 acceleration out of limits: " << q_acc.theta4 << " not in [" << q_acc_limits_.at("theta4").min << ", " << q_acc_limits_.at("theta4").max << "]" << std::endl;
		return false;
	}
	adjust_q_within_tolerance_(q_acc, tolerance);
	return true;
}

void ROBSIM::ROBOT::adjust_q_within_tolerance_(q_vec& q, ROBSIMDouble tolerance) {
	ROBSIMDouble tolerance_ = 0.000001;
	if (q.theta1 <= q_limits_.at("theta1").min && q.theta1 >= q_limits_.at("theta1").min - tolerance) {
		q.theta1 = q_limits_.at("theta1").min + tolerance_;
	}

	if (q.theta1 >= q_limits_.at("theta1").max && q.theta1 <= q_limits_.at("theta1").max + tolerance) {
		q.theta1 = q_limits_.at("theta1").max - tolerance_;
	}
	
	if (q.theta2 <= q_limits_.at("theta2").min && q.theta2 >= q_limits_.at("theta2").min - tolerance) {
		q.theta2 = q_limits_.at("theta2").min + tolerance_;	
	}
	if (q.theta2 >= q_limits_.at("theta2").max && q.theta2 <= q_limits_.at("theta2").max + tolerance) {
		q.theta2 = q_limits_.at("theta2").max - tolerance_;
	}
	if (q.d3 <= q_limits_.at("d3").min && q.d3 >= q_limits_.at("d3").min - tolerance) {
		q.d3 = q_limits_.at("d3").min + tolerance_;
	}
	if (q.d3 >= q_limits_.at("d3").max && q.d3 <= q_limits_.at("d3").max + tolerance) {
		q.d3 = q_limits_.at("d3").max - tolerance_;
	}
	if (q.theta4 <= q_limits_.at("theta4").min && q.theta4 >= q_limits_.at("theta4").min - tolerance) {
		q.theta4 = q_limits_.at("theta4").min + tolerance_;
	}
	if (q.theta4 >= q_limits_.at("theta4").max && q.theta4 <= q_limits_.at("theta4").max + tolerance) {
		q.theta4 = q_limits_.at("theta4").max - tolerance_;
	}
}


bool ROBSIM::ROBOT::setQ(q_vec& q) {
	// check constraints
	if (!check_q_limits(q, true)) {
		//throw std::invalid_argument("Invalid joint q vector");
		robot_q_valid_ = false;
		return false;
	}
	q_ = q;

	// update the dh_params_ vector
	dh_params_ = getDHParamsFromQVec(q);

	// updata dh_params_dict_
	// base = 0, wrist = 4, tool = 5
	dh_params_dict_ = {
		{"brels", dh_params_[0]},
		{"1relb", dh_params_[1]},
		{"2rel1", dh_params_[2]},
		{"3rel2", dh_params_[3]},
		{"4rel3", dh_params_[4]},
		{"trelw", dh_params_[5]}
	};

	dh_params_set_= true;
	update_transformations_();
	robot_q_valid_ = true;
	return true;
}

ROBSIM::q_vec ROBSIM::ROBOT::current_config() {
	JOINT current_config;
	GetConfiguration(current_config);
	ROBSIM::q_vec q1(current_config);
	return q1;
}

void ROBSIM::ROBOT::update_transformations_() {
	// update the frames
	if (!dh_params_set_) {
		throw std::invalid_argument("DH parameters not set");
	}
	
	//transformations_ = getTransformationFromDHParamsVector(dh_params_);
	// base = 0, wrist = 4, tool = 5
	tr_brels_ = getTransformationFromDH(dh_params_dict_["brels"]);
	tr_1relb_ = getTransformationFromDH(dh_params_dict_["1relb"]);
	tr_2rel1_ = getTransformationFromDH(dh_params_dict_["2rel1"]);
	tr_3rel2_ = getTransformationFromDH(dh_params_dict_["3rel2"]);
	tr_4rel3_ = getTransformationFromDH(dh_params_dict_["4rel3"]);
	tr_trelw_ = getTransformationFromDH(dh_params_dict_["trelw"]);


	brels_.setMatrix(tr_brels_);
	trelw_.setMatrix(tr_trelw_);

	wrelb_.setMatrix(tr_1relb_ * tr_2rel1_ * tr_3rel2_ * tr_4rel3_);

	//trels.setMatrix(brels.getMatrix() * wrelb.getMatrix() * trelw.getMatrix());
	update_frames_();
	
}

void ROBSIM::ROBOT::update_frames_() {
	// intermediate frame trelb
	frame trelb;
	TMULT(wrelb_, trelw_, trelb);
	TMULT(brels_, trelb, trels_);
}

void ROBSIM::UTOI(vec4& v, frame& f) {
	Rotation rotation('t', v.phi);
	//Rotation rot_x('x', 180);
	
	Translation translation(v.x, v.y, v.z);
	Transformation transformation(rotation, translation);
	// TODO: check if it is needed to multiply the rotation by rot_x
	//Transformation transformation(rotation*rot_x, translation);
	
	f.setMatrix(transformation);
}

void ROBSIM::ITOU(frame& f, vec4& v) {
	Transformation transformation = f.getMatrix();
	Rotation rotation = transformation.getRotation();
	Translation translation = transformation.getTranslation();
	//printf("rotation in ITOU \n");
	//rotation.display();
	v.x = translation.getX();
	v.y = translation.getY();
	v.z = translation.getZ();

	// TODO: check if this is correct
	//Rotation rot_x('x', 180);
	//rotation = rotation * rot_x;

	v.phi = RAD2DEG(atan2(rotation(1, 0), rotation(0, 0)));
}

void ROBSIM::TMULT(frame& brela, frame& crelb, frame& crela) {
	crela.setMatrix(brela.getMatrix() * crelb.getMatrix());
}

void ROBSIM::TINVERT(frame& brela, frame& arelb) {
	arelb.setMatrix(brela.getMatrix().inverse());
	//arelb.matrix = brela.matrix.inverse();
}

// Kinematics functions
void ROBSIM::ROBOT::KIN(q_vec& q, frame& wrelb) {
	// update the q
	if (!setQ(q)) {
		printf("Invalid joint q vector\n");
		return;
	}
	wrelb = get_wrelb();
}

ROBSIM::vec4 ROBSIM::ROBOT::WHERE(q_vec& q, frame& trels) {
	// update the q
	vec4 v;
	if (!setQ(q)) {
		printf("Invalid joint q vector\n");
		return v;
	}
	trels = get_trels();
	ITOU(trels, v);
	return v;
}

// Inverse Kinematics functions
void ROBSIM::ROBOT::INVKIN(frame& wrelb, q_vec& current, q_vec& near, q_vec& far, bool& sol) {

	double tolerance = 0.00001;

	if (!setQ(current)) {
		printf("Invalid joint q vector\n");
		sol = false;
		return;
	}

	bool check_q_limit_verbose = false;

	//trelb = wrelb * trelw;
	frame current_frame = wrelb;

	// get x, y, z from current frame
	/*Translation pos = current_frame.getMatrix().getTranslation();
	ROBSIMDouble x = pos.getX();
	ROBSIMDouble y = pos.getY();
	ROBSIMDouble z = pos.getZ();*/

	// get x, y, z from current frame
	vec4 current_vec;
	ITOU(wrelb, current_vec);
	ROBSIMDouble x = current_vec.x;
	ROBSIMDouble y = current_vec.y;
	ROBSIMDouble z = current_vec.z;
	ROBSIMDouble phi = current_vec.phi;

	// check if the pos z is reachable
	if (z < L0 + L1 - L4 - tolerance || z > L0 + L1 + L4 + tolerance) {
		sol = false;
		std::cerr << "No solution for inverse kinematics; Unreachable z" << std::endl;
		return;
	}

	ROBSIMDouble c2 = (x * x + y * y - L2 * L2 - L3 * L3) / (2 * L2 * L3);
	// check if the position is reachable
	if (c2 > 1 + tolerance || c2 < -1 - tolerance) {
		sol = false;
		std::cerr << "No solution for inverse kinematics; Unreachable x or y" << std::endl;
		return;
	}
	ROBSIMDouble s2_near, s2_far;
	ROBSIMDouble under_sqrt = 1 - c2 * c2;
	if (under_sqrt < 0 || std::abs(under_sqrt) < tolerance) {
		s2_near = s2_far = 0;
	}
	else {
		s2_near = sqrt(1 - c2 * c2);
		s2_far = -s2_near;
	}
	
	// two solutions for theta2
	ROBSIMDouble theta2_near = RAD2DEG(atan2(s2_near, c2));
	ROBSIMDouble theta2_far = RAD2DEG(atan2(s2_far, c2));

	// cramers rule for theta1 using s2_near
	ROBSIMDouble a = L2 + L3 * c2;
	ROBSIMDouble b_near = L3 * s2_near;

	bool theta1_any = false;
	// check if the position is reachable
	if (a == 0 && b_near == 0) {
		if (x == 0 && y == 0) {
			std::cout << "theta1 can be any angle" << std::endl;
			bool theta1_any = true;
		}
		else {
			sol = false;
			std::cerr << "No solution for inverse kinematics x,y = 0" << std::endl;
			return;
		}
	}

	// this theta1_any deosn't seem to be possible based on the robot geometry
	// still we include it for completeness
	ROBSIMDouble theta1_near = current.theta1;
	ROBSIMDouble theta1_far = current.theta1;
	if (!theta1_any) {
		ROBSIMDouble c1_near = x * a + y * b_near;
		ROBSIMDouble s1_near = y * a - x * b_near;

		theta1_near = RAD2DEG(atan2(s1_near, c1_near));

		// cramers rule for theta1 using s2_far
		ROBSIMDouble b_far = L3 * s2_far;

		ROBSIMDouble c1_far = (x * a + y * b_far);
		ROBSIMDouble s1_far = (y * a - x * b_far);

		theta1_far = RAD2DEG(atan2(s1_far, c1_far));
	}

	// solution for d3
	ROBSIMDouble d3 = L0 + L1 - L4 - z;

	// solution for theta4
	// first get phi from the rotation matrix in the current frame

	ROBSIMDouble theta4_near = theta1_near + theta2_near - phi;
	ROBSIMDouble theta4_far = theta1_far + theta2_far - phi;

	near.setQ(theta1_near, theta2_near, d3, theta4_near);
	far.setQ(theta1_far, theta2_far, d3, theta4_far);
	
	// check if both solutions are within the limits
	if (check_q_limits(near, check_q_limit_verbose) && check_q_limits(far, check_q_limit_verbose)) {
		if (current.getDistance(far, theta1_any) < current.getDistance(near, theta1_any)) {
			// switch near and far
			q_vec temp = near;
			near = far;
			far = temp;
		}
		sol = true;
		if (near.getDistance(far, theta1_any) < tolerance) {
			std::cout << "One Solution found for INVKIN, ";// << std::endl;
			std::cout << "Solution: ";
			near.display();
			return;
		}
		else {
			std::cout << "Two Solutions found for INVKIN, ";// << std::endl;
			std::cout << "Nearest solution: ";
			near.display();
			//std::cout << "Farthest solution: ";
			//far.display();
		}
		return;
	}
	else if (check_q_limits(far, check_q_limit_verbose)){
		sol = true;
		near = far;
		std::cout << "One Solution found for INVKIN, ";// << std::endl;
		std::cout << "Solution: ";
		near.display();
		return;
	}
	else if (check_q_limits(near, check_q_limit_verbose)) {
		sol = true;
		std::cout << "One Solution found for INVKIN, ";// << std::endl;
		std::cout << "Solution: ";
		near.display();
		return;
	}
	else {
		sol = false;
		std::cerr << "No solution for inverse kinematics" << std::endl;
		return;
	}
}

void ROBSIM::ROBOT::SOLVE(frame& trels, q_vec& current, q_vec& near, q_vec& far, bool& sol) {
	frame wrelb, wrelt, srelb, wrels;
	frame trelw = get_trelw();
	frame brels = get_brels();

	TINVERT(trelw, wrelt);
	TINVERT(brels, srelb);

	// wrelb = srelb * trels * wrelt
	// wrels = trels * wrelt
	TMULT(trels, wrelt, wrels);
	TMULT(srelb, wrels, wrelb);

	INVKIN(wrelb, current, near, far, sol);
}

void ROBSIM::ROBOT::SOLVE(vec4& pose, q_vec& current, q_vec& near, q_vec& far, bool& sol) {
	frame trels;
	UTOI(pose, trels);
	SOLVE(trels, current, near, far, sol);
}

bool ROBSIM::ROBOT::SOLVE(vec4& p0, vec4& p1, vec4& p2, vec4& p3, vec4& pf, q_vec& q0, q_vec& q1, q_vec& q2, q_vec& q3, q_vec& qf) {
	bool sol;
	q_vec q_far;
	SOLVE(p1, q0, q1, q_far, sol);
	if (!sol) {
		std::cerr << "No solution for p1" << std::endl;
		sol = false;
	}
	SOLVE(p2, q1, q2, q_far, sol);
	if (!sol) {
		std::cerr << "No solution for p2" << std::endl;
		sol = false;
	}
	SOLVE(p3, q2, q3, q_far, sol);
	if (!sol) {
		std::cerr << "No solution for p3" << std::endl;
		sol = false;
	}
	SOLVE(pf, q3, qf, q_far, sol);
	if (!sol) {
		std::cerr << "No solution for pf" << std::endl;
		sol = false;
	}
	return sol;
}

ROBSIM::cubic_poly ROBSIM::get_cubic_poly(ROBSIMDouble y0, ROBSIMDouble yf, ROBSIMDouble v0, ROBSIMDouble vf, ROBSIMDouble t0, ROBSIMDouble tf) {
	//ROBSIMDouble h = tf - t0;
	//ROBSIMDouble a = y0;
	//ROBSIMDouble b = v0 * h;
	//ROBSIMDouble c = 3 * (yf - y0) - 2 * v0 * h - vf * h;
	//ROBSIMDouble d = -2 * (yf - y0) + v0 * h + vf * h;
	//return ROBSIM::cubic_poly(a, b, c, d);

	ROBSIMDouble h = tf - t0;
	ROBSIMDouble a = y0;
	ROBSIMDouble b = v0;
	ROBSIMDouble c = 3 * (yf - y0) / (h * h) - 2 * v0 / h - vf / h;
	ROBSIMDouble d = -2 * (yf - y0) / (h * h * h) + (v0 + vf) / (h * h);
	return ROBSIM::cubic_poly(a, b, c, d);
}

ROBSIM::cubic_poly ROBSIM::get_cubic_poly(ROBSIMDouble y0, ROBSIMDouble yf, ROBSIMDouble v0, ROBSIMDouble vf, ROBSIMDouble h) {
	return get_cubic_poly(y0, yf, v0, vf, 0, h);
}

ROBSIM::q_cubic_poly ROBSIM::get_q_cubic_poly(q_vec& q0, q_vec& qf, q_vec& v0, q_vec& vf, ROBSIMDouble t0, ROBSIMDouble tf) {
	
	ROBSIM::cubic_poly q_poly_theta1 = get_cubic_poly(q0.theta1, qf.theta1, v0.theta1, vf.theta1, t0, tf);
	ROBSIM::cubic_poly q_poly_theta2 = get_cubic_poly(q0.theta2, qf.theta2, v0.theta2, vf.theta2, t0, tf);
	ROBSIM::cubic_poly q_poly_d3 = get_cubic_poly(q0.d3, qf.d3, v0.d3, vf.d3, t0, tf);
	ROBSIM::cubic_poly q_poly_theta4 = get_cubic_poly(q0.theta4, qf.theta4, v0.theta4, vf.theta4, t0, tf);

	q_cubic_poly result(q_poly_theta1, q_poly_theta2, q_poly_d3, q_poly_theta4);
	return result;
}

ROBSIMDouble ROBSIM::get_q_velocity(ROBSIMDouble q0, ROBSIMDouble q_mid, ROBSIMDouble qf, ROBSIMDouble t0, ROBSIMDouble t_mid, ROBSIMDouble tf) {
	ROBSIMDouble v0 = (q_mid - q0) / (t_mid - t0);
	ROBSIMDouble vf = (qf - q_mid) / (tf - t_mid);
	return (v0 + vf) / 2;
}


std::vector<ROBSIMDouble> ROBSIM::get_q_velocities(ROBSIMDouble q0, ROBSIMDouble q1, ROBSIMDouble q2, ROBSIMDouble q3, ROBSIMDouble qf,
	ROBSIMDouble t0, ROBSIMDouble t1, ROBSIMDouble t2, ROBSIMDouble t3, ROBSIMDouble tf) {
	ROBSIMDouble v0 = 0;
	ROBSIMDouble v1 = get_q_velocity(q0, q1, q2, t0, t1, t2);
	ROBSIMDouble v2 = get_q_velocity(q1, q2, q3, t1, t2, t3);
	ROBSIMDouble v3 = get_q_velocity(q2, q3, qf, t2, t3, tf);
	ROBSIMDouble vf = 0;
	return { v0, v1, v2, v3, vf };
}

void ROBSIM::get_q_velocities(q_vec& q0, q_vec& q1, q_vec& q2, q_vec& q3, q_vec& qf,
	ROBSIMDouble t0, ROBSIMDouble t1, ROBSIMDouble t2, ROBSIMDouble t3, ROBSIMDouble tf, 
	q_vec& v0, q_vec& v1, q_vec& v2, q_vec& v3, q_vec& vf){
	
	std::vector<ROBSIMDouble> theta1_vels = get_q_velocities(q0.theta1, q1.theta1, q2.theta1, q3.theta1, qf.theta1, t0, t1, t2, t3, tf);
	std::vector<ROBSIMDouble> theta2_vels = get_q_velocities(q0.theta2, q1.theta2, q2.theta2, q3.theta2, qf.theta2, t0, t1, t2, t3, tf);
	std::vector<ROBSIMDouble> d3_vels = get_q_velocities(q0.d3, q1.d3, q2.d3, q3.d3, qf.d3, t0, t1, t2, t3, tf);
	std::vector<ROBSIMDouble> theta4_vels = get_q_velocities(q0.theta4, q1.theta4, q2.theta4, q3.theta4, qf.theta4, t0, t1, t2, t3, tf);

	v0.setQ(theta1_vels[0], theta2_vels[0], d3_vels[0], theta4_vels[0]);
	v1.setQ(theta1_vels[1], theta2_vels[1], d3_vels[1], theta4_vels[1]);
	v2.setQ(theta1_vels[2], theta2_vels[2], d3_vels[2], theta4_vels[2]);
	v3.setQ(theta1_vels[3], theta2_vels[3], d3_vels[3], theta4_vels[3]);
	vf.setQ(theta1_vels[4], theta2_vels[4], d3_vels[4], theta4_vels[4]);	
}

void ROBSIM::calculateTimeSegments(ROBSIMDouble t0, ROBSIMDouble tf, ROBSIMDouble& t1, ROBSIMDouble& t2, ROBSIMDouble& t3) {
	ROBSIMDouble interval = (tf - t0) / 4.0;
	t1 = t0 + interval;
	t2 = t1 + interval;
	t3 = t2 + interval;
}

bool ROBSIM::ROBOT::check_trajectory_q_limits(q_vec& q0, q_vec& q1, q_vec& q2, q_vec& q3, q_vec& qf, bool verbose) {
	if (!check_q_limits(q0, verbose)) {
		std::cout << "q0 is not within the limits" << std::endl;
		return false;
	}

	if (!check_q_limits(q1, verbose)) {
		std::cout << "q1 is not within the limits" << std::endl;
		return false;
	}

	if (!check_q_limits(q2, verbose)) {
		std::cout << "q2 is not within the limits" << std::endl;
		return false;
	}

	if (!check_q_limits(q3, verbose)) {
		std::cout << "q3 is not within the limits" << std::endl;
		return false;
	}

	if (!check_q_limits(qf, verbose)) {
		std::cout << "qf is not within the limits" << std::endl;
		return false;
	}
	return true;
}

std::vector<ROBSIM::q_cubic_poly> ROBSIM::ROBOT::trajectory_planner(q_vec& q0, q_vec& q1, q_vec& q2, q_vec& q3, q_vec& qf, ROBSIMDouble t0, ROBSIMDouble tf) {

	// check if the q vectors are valid
	if (!check_trajectory_q_limits(q0, q1, q2, q3, qf, true)) {
		std::cout << "Invalid joint q vectors in trajectory" << std::endl;
	}

	ROBSIMDouble t1, t2, t3;
	calculateTimeSegments(t0, tf, t1, t2, t3);

	q_vec v0, v1, v2, v3, vf;
	get_q_velocities(q0, q1, q2, q3, qf, t0, t1, t2, t3, tf, v0, v1, v2, v3, vf);

	q_cubic_poly q_poly_01 = get_q_cubic_poly(q0, q1, v0, v1, t0, t1);
	q_cubic_poly q_poly_12 = get_q_cubic_poly(q1, q2, v1, v2, t1, t2);
	q_cubic_poly q_poly_23 = get_q_cubic_poly(q2, q3, v2, v3, t2, t3);
	q_cubic_poly q_poly_3f = get_q_cubic_poly(q3, qf, v3, vf, t3, tf);

	return { q_poly_01, q_poly_12, q_poly_23, q_poly_3f };
}

ROBSIM::pos_vel_acc ROBSIM::ROBOT::sample_q_cubic_poly(q_cubic_poly q_poly, ROBSIMDouble t, ROBSIMDouble t0, ROBSIMDouble tf) {
	//auto time = (t - t0) / (tf - t0);
	auto time = (t - t0);
	q_vec pos = q_poly.get_position(time);
	q_vec vel = q_poly.get_velocity(time);
	q_vec acc = q_poly.get_acceleration(time);
	return pos_vel_acc(t, pos, vel, acc);
}

void ROBSIM::ROBOT::create_trajectory_planner_logging_files() {
	// Filenames
	std::string planned_positions_filename = "planned_positions.csv";
	std::string executed_positions_filename = "executed_positions.csv";
	std::string planned_trajectory_filename = "planned_trajectory.csv";
	std::string executed_trajectory_filename = "executed_trajectory.csv";
	std::string velocities_filename = "velocities.csv";
	std::string accelerations_filename = "accelerations.csv";

	// Open files for writing and clear existing content
	std::ofstream planned_positions_file(planned_positions_filename, std::ios::out);
	std::ofstream executed_positions_file(executed_positions_filename, std::ios::out);
	std::ofstream planned_trajectory_file(planned_trajectory_filename, std::ios::out);
	std::ofstream executed_trajectory_file(executed_trajectory_filename, std::ios::out);
	std::ofstream velocities_file(velocities_filename, std::ios::out);
	std::ofstream accelerations_file(accelerations_filename, std::ios::out);

	// Write headers
	planned_positions_file << "time,pos_x,pos_y,pos_z,pos_phi\n";
	executed_positions_file << "time,pos_x,pos_y,pos_z,pos_phi\n";
	planned_trajectory_file << "time,pos_theta1,pos_theta2,pos_d3,pos_theta4\n";
	executed_trajectory_file << "time,pos_theta1,pos_theta2,pos_d3,pos_theta4\n";
	velocities_file << "time,vel_theta1,vel_theta2,vel_d3,vel_theta4\n";
	accelerations_file << "time,acc_theta1,acc_theta2,acc_d3,acc_theta4\n";

	planned_positions_file.close();
	executed_positions_file.close();
	planned_trajectory_file.close();
	executed_trajectory_file.close();
	velocities_file.close();
	accelerations_file.close();

}


void ROBSIM::ROBOT::dump_to_csv(const std::vector<ROBSIM::pos_vel_acc>& data) {
	std::ofstream planned_trajectory("planned_trajectory.csv");
	std::ofstream velocities("velocities.csv");
	std::ofstream accelerations("accelerations.csv");

	// Write headers
	planned_trajectory << "time,pos_theta1,pos_theta2,pos_d3,pos_theta4\n";
	velocities << "time,vel_theta1,vel_theta2,vel_d3,vel_theta4\n";
	accelerations << "time,acc_theta1,acc_theta2,acc_d3,acc_theta4\n";

	// Write data
	for (const auto& entry : data) {
		planned_trajectory << entry.time << ',' << entry.pos.theta1 << ',' << entry.pos.theta2 << ',' << entry.pos.d3 << ',' << entry.pos.theta4 << '\n';
		velocities << entry.time << ',' << entry.vel.theta1 << ',' << entry.vel.theta2 << ',' << entry.vel.d3 << ',' << entry.vel.theta4 << '\n';
		accelerations << entry.time << ',' << entry.acc.theta1 << ',' << entry.acc.theta2 << ',' << entry.acc.d3 << ',' << entry.acc.theta4 << '\n';
	}

	planned_trajectory.close();
	velocities.close();
	accelerations.close();
}

void ROBSIM::ROBOT::dump_to_csv(const ROBSIM::pos_vel_acc& data) {
	// Filenames
	std::string trajectory_filename = "planned_trajectory.csv";
	std::string velocities_filename = "velocities.csv";
	std::string accelerations_filename = "accelerations.csv";

	

	// Open files for writing
	std::ofstream trajectory(trajectory_filename, std::ios::out | std::ios::app);
	std::ofstream velocities(velocities_filename, std::ios::out | std::ios::app);
	std::ofstream accelerations(accelerations_filename, std::ios::out | std::ios::app);

	// Write one row of data
	trajectory << data.time << ',' << data.pos.theta1 << ',' << data.pos.theta2 << ',' << data.pos.d3 << ',' << data.pos.theta4 << '\n';
	velocities << data.time << ',' << data.vel.theta1 << ',' << data.vel.theta2 << ',' << data.vel.d3 << ',' << data.vel.theta4 << '\n';
	accelerations << data.time << ',' << data.acc.theta1 << ',' << data.acc.theta2 << ',' << data.acc.d3 << ',' << data.acc.theta4 << '\n';

	// Close the files
	trajectory.close();
	velocities.close();
	accelerations.close();
}

void ROBSIM::ROBOT::dump_to_csv(const std::vector<std::tuple<ROBSIM::vec4, ROBSIMDouble>>& data) {
	std::ofstream positions("positions.csv");

	// Write headers
	positions << "time,pos_x,pos_y,pos_z,pos_phi\n";

	// Write data
	for (const auto& entry : data) {
		positions << std::get<1>(entry) << ',' << std::get<0>(entry).x << ',' << std::get<0>(entry).y << ',' << std::get<0>(entry).z << ',' << std::get<0>(entry).phi << '\n';
	}

	positions.close();
}


void ROBSIM::ROBOT::dump_to_csv(std::tuple<ROBSIM::vec4, ROBSIMDouble>& data) {
	// Filenames
	std::string positions_filename = "positions.csv";

	// Open files for writing
	std::ofstream positions(positions_filename, std::ios::out | std::ios::app);

	// Write one row of data
	positions << std::get<1>(data) << ',' << std::get<0>(data).x << ',' << std::get<0>(data).y << ',' << std::get<0>(data).z << ',' << std::get<0>(data).phi << '\n';

	// Close the files
	positions.close();
}

void ROBSIM::ROBOT::dump_to_csv(ROBSIM::vec4 data, ROBSIMDouble time) {
	// Filenames
	std::string positions_filename = "positions.csv";

	// Open files for writing
	std::ofstream positions(positions_filename, std::ios::out | std::ios::app);

	// Write one row of data
	positions << time << ',' << data.x << ',' << data.y << ',' << data.z << ',' << data.phi << '\n';

	// Close the files
	positions.close();
}

void ROBSIM::ROBOT::dump_to_csv(ROBSIM::vec4 data, ROBSIMDouble time, std::string filename) {
	// Open files for writing
	std::ofstream positions(filename, std::ios::out | std::ios::app);

	// Write one row of data
	positions << time << ',' << data.x << ',' << data.y << ',' << data.z << ',' << data.phi << '\n';

	// Close the files
	positions.close();
}

void ROBSIM::ROBOT::dump_to_csv(const ROBSIM::q_vec& data, ROBSIMDouble time) {
	// Filenames
	std::string planned_trajectory_filename = "executed_trajectory.csv";

	// Open files for writing
	std::ofstream planned_trajectory(planned_trajectory_filename, std::ios::out | std::ios::app);

	// Write one row of data
	planned_trajectory << time << ',' << data.theta1 << ',' << data.theta2 << ',' << data.d3 << ',' << data.theta4 << '\n';

	// Close the files
	planned_trajectory.close();
}


std::vector<ROBSIM::pos_vel_acc> ROBSIM::ROBOT::create_trajectory_planner2(
	q_vec& q0, q_vec& q1, q_vec& q2, q_vec& q3, q_vec& qf, ROBSIMDouble t0, ROBSIMDouble tf, ROBSIMDouble num_samples, bool moverobot) {

	create_trajectory_planner_logging_files();

	ROBSIMDouble t1, t2, t3;
	//tf = tf - t0;
	//t0 = 0;
	t0 = (double)clock() / CLOCKS_PER_SEC;
	tf = t0 + tf;

	calculateTimeSegments(t0, tf, t1, t2, t3);

	std::vector<q_cubic_poly> q_polys = trajectory_planner(q0, q1, q2, q3, qf, t0, tf);
	ROBSIMDouble interval = (tf - t0) / num_samples;

	std::vector<pos_vel_acc> trajectory;
	std::vector<std::tuple<vec4, ROBSIMDouble>> positions;

	ROBSIMDouble time;
	pos_vel_acc sample;
	vec4 pos;
	frame dummy_frame;
	ROBOT dummy_robot;

	auto next_sample_time = std::chrono::high_resolution_clock::now();

	auto robot_limits_violated = false;

	for (ROBSIMDouble t = t0; t <= tf; t += interval) {
		std::this_thread::sleep_until(next_sample_time);
		if (t <= t1) {
			sample = sample_q_cubic_poly(q_polys[0], t, t0, t1);
			pos = WHERE(sample.pos, dummy_frame);

			dump_to_csv(sample);
			dump_to_csv(pos, t);

			//trajectory.push_back(sample);
			//positions.push_back(std::make_tuple(pos, t));
		}
		else if (t <= t2) {
			sample = sample_q_cubic_poly(q_polys[1], t, t1, t2);
			pos = WHERE(sample.pos, dummy_frame);

			dump_to_csv(sample);
			dump_to_csv(pos, t);

			//trajectory.push_back(sample);
			//positions.push_back(std::make_tuple(pos, t));
		}
		else if (t <= t3) {
			sample = sample_q_cubic_poly(q_polys[2], t, t2, t3);
			pos = WHERE(sample.pos, dummy_frame);

			dump_to_csv(sample);
			dump_to_csv(pos, t);

			//trajectory.push_back(sample);
			//positions.push_back(std::make_tuple(pos, t));
		}
		else {
			sample = sample_q_cubic_poly(q_polys[3], t, t3, tf);
			pos = WHERE(sample.pos, dummy_frame);

			dump_to_csv(sample);
			dump_to_csv(pos, t);

			//trajectory.push_back(sample);
			//positions.push_back(std::make_tuple(pos, t));

		}
		// Move the robot
		if (moverobot && !robot_limits_violated) {
			if (!move_conf_vel_acc(sample.pos, sample.vel, sample.acc)) {
				robot_limits_violated = true;
			}
		}
		// Update the next sample time
		next_sample_time += std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(
			std::chrono::duration<ROBSIMDouble>(interval));
	}
	//dump_to_csv(trajectory);
	//dump_to_csv(positions);
	q_vec zero_vel, zero_acc;
	zero_vel.setQ(0, 0, 0, 0);
	zero_acc.setQ(0, 0, 0, 0);
	move_conf_vel_acc(qf, zero_vel, zero_acc);
	return trajectory;
}

std::vector<ROBSIM::pos_vel_acc> ROBSIM::ROBOT::create_trajectory_planner(
	q_vec& q0, q_vec& q1, q_vec& q2, q_vec& q3, q_vec& qf, ROBSIMDouble t0, ROBSIMDouble tf, ROBSIMDouble num_samples, bool moverobot) {

	create_trajectory_planner_logging_files();
	auto robot_limits_violated = false;

	ROBSIMDouble time;
	pos_vel_acc sample;
	vec4 pos;
	frame dummy_frame;
	ROBOT dummy_robot;

	t0 = (double)clock() / CLOCKS_PER_SEC;
	auto total_time = tf;
	tf = t0 + tf;

	std::vector<q_cubic_poly> q_polys = trajectory_planner(q0, q1, q2, q3, qf, t0, tf);

	ROBSIMDouble t1, t2, t3;
	calculateTimeSegments(t0, tf, t1, t2, t3);

	print_trajectory_debugging(t0, t1, t2, t3, tf, q0, q1, q2, q3, qf, q_polys, 0);

	ROBSIMDouble sample_t_size = (tf - t0) / num_samples;
	ROBSIMDouble delta_t = 0.0;
	ROBSIMDouble prev_t = t0;
	ROBSIMDouble now_t = t0;

	while (now_t <= tf) {
		now_t = (double)clock() / CLOCKS_PER_SEC;
		delta_t = now_t - prev_t;
		if (delta_t >= sample_t_size) {
			prev_t = now_t;
			if (now_t <= t1) {
				sample = sample_q_cubic_poly(q_polys[0], now_t, t0, t1);
				pos = WHERE(sample.pos, dummy_frame);

				dump_to_csv(sample);
				dump_to_csv(pos, now_t, "planned_positions.csv");
			}
			else if (now_t <= t2) {
				sample = sample_q_cubic_poly(q_polys[1], now_t, t1, t2);
				pos = WHERE(sample.pos, dummy_frame);

				dump_to_csv(sample);
				dump_to_csv(pos, now_t, "planned_positions.csv");

			}
			else if (now_t <= t3) {
				sample = sample_q_cubic_poly(q_polys[2], now_t, t2, t3);
				pos = WHERE(sample.pos, dummy_frame);

				dump_to_csv(sample);
				dump_to_csv(pos, now_t, "planned_positions.csv");
			}
			else {
				sample = sample_q_cubic_poly(q_polys[3], now_t, t3, tf);
				pos = WHERE(sample.pos, dummy_frame);

				dump_to_csv(sample);
				dump_to_csv(pos, now_t, "planned_positions.csv");
			}

			// Move the robot
			if (moverobot && !robot_limits_violated) {
				if (move_conf_vel_acc(sample.pos, sample.vel, sample.acc)) {
					auto current_robot_config = current_config();
					dump_to_csv(current_robot_config, now_t);

					auto actual_pos = dummy_robot.WHERE(current_robot_config, dummy_frame);
					dump_to_csv(actual_pos, now_t, "executed_positions.csv");
				}
				else {
					robot_limits_violated = true;
				}
			}
		}
	}

	
	std::vector<pos_vel_acc> trajectory;
	//dump_to_csv(trajectory);
	q_vec zero_vel, zero_acc;
	zero_vel.setQ(0, 0, 0, 0);
	zero_acc.setQ(0, 0, 0, 0);
	move_conf_vel_acc(qf, zero_vel, zero_acc);
	return trajectory;
}

bool ROBSIM::ROBOT::move_conf_vel_acc(q_vec& qconf, q_vec& qvel, q_vec& qacc) {
	
	bool valid = true;
	bool success = false;
	double interval = 1;
	if (!check_q_limits(qconf, true)) {
		std::cout << "Invalid joint q vector; Robot Not Moving" << std::endl;
		valid = false;
	}
	if (!check_q_vel_limits(qvel, true)) {
		std::cout << "Invalid joint q velocity vector; Robot Not Moving" << std::endl;
		valid = false;
	}
	if (!check_q_acc_limits(qacc, true)) {
		std::cout << "Invalid joint q acceleration vector; Robot Not Moving" << std::endl;
		valid = false;
	}

	qvel = qvel * interval;
	qacc = qacc * interval * interval;
	
	if (valid) {
		JOINT conf, vel, acc;
		qconf.toJOINT(conf);
		qvel.toJOINT(vel);
		qacc.toJOINT(acc);
		//std::cout << "conf: " << conf[0] << " " << conf[1] << " " << conf[2] << " " << conf[3] << std::endl;
		//std::cout << "vel: " << vel[0] << " " << vel[1] << " " << vel[2] << " " << vel[3] << std::endl;
		//std::cout << "acc: " << acc[0] << " " << acc[1] << " " << acc[2] << " " << acc[3] << std::endl;
		if (MoveWithConfVelAcc(conf, vel, acc)) {
			success = true;
		}
		//MoveToConfiguration(conf);
	}
	return success;
}

void ROBSIM::ROBOT::print_trajectory_debugging(ROBSIMDouble t0, ROBSIMDouble t1, ROBSIMDouble t2, ROBSIMDouble t3, ROBSIMDouble tf,
	q_vec& q0, q_vec& q1, q_vec& q2, q_vec& q3, q_vec& qf, std::vector<q_cubic_poly>& q_polys, int verbose)
	{
	if (verbose == 0) {
		return;
	}
	
	if (verbose >= 1) {
		std::cout << "q0: ";
		q0.display();
		std::cout << "q1: ";
		q1.display();
		std::cout << "q2: ";
		q2.display();
		std::cout << "q3: ";
		q3.display();
		std::cout << "qf: ";
		qf.display();
	}
	if (verbose >= 2) {
		std::cout << "t0: " << t0 << std::endl;
		std::cout << "t1: " << t1 << std::endl;
		std::cout << "t2: " << t2 << std::endl;
		std::cout << "t3: " << t3 << std::endl;
		std::cout << "tf: " << tf << std::endl;
	}
	if (verbose >= 3) {
		std::cout << "q_polys size: " << q_polys.size() << std::endl;
		std::cout << "q_poly_0: " << std::endl;
		q_polys[0].display();
		std::cout << "q_poly_1: " << std::endl;
		q_polys[1].display();
		std::cout << "q_poly_2: " << std::endl;
		q_polys[2].display();
		std::cout << "q_poly_3: " << std::endl;
		q_polys[3].display();
	}
}
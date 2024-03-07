#pragma once

#include <vector>
#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <unordered_map>

#include "ensc-488.h"


typedef double ROBSIMDouble;

//ROBSIMDouble atan2_DEG(ROBSIMDouble y, ROBSIMDouble x) { return RAD2DEG(atan2(y, x));}

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

	// operator overloading for multiplication
	Rotation operator*(const Rotation& other) const;

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

	ROBSIMDouble sign(ROBSIMDouble x);

	struct ParameterLimits {
		ROBSIMDouble min;
		ROBSIMDouble max;
	};
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
			std::cout << "x: " << x << " y: " << y << " z: " << z << " phi: " << phi << std::endl;
		}
	};

	class frame {
	public:
		Transformation matrix, base;
		frame(Transformation matrix) : matrix(matrix) {}
		frame() : matrix() {}
		frame(const frame& f) : matrix(f.matrix) {}
		frame(const frame* f) : matrix(f->matrix) {}

		void setBase(Transformation base) { this->base = base; }
		void setMatrix(Transformation transformation) { matrix = transformation; }
		Transformation getBase() { return base; }
		Transformation getMatrix() { return matrix; }
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

		// setters
		void setQ(ROBSIMDouble theta1, ROBSIMDouble theta2, ROBSIMDouble d3, ROBSIMDouble theta4) { this->theta1 = theta1; this->theta2 = theta2; this->d3 = d3; this->theta4 = theta4; }
		void setQ(q_vec& q) { theta1 = q.theta1; theta2 = q.theta2; d3 = q.d3; theta4 = q.theta4; }
		void setQ(ROBSIMDouble* v) { theta1 = v[0]; theta2 = v[1]; d3 = v[2]; theta4 = v[3]; }
		void setQ(std::vector<ROBSIMDouble> v) { theta1 = v.at(0); theta2 = v.at(1); d3 = v.at(2); theta4 = v.at(3); }

		// getters 
		std::vector<ROBSIMDouble> toVector() const;
		ROBSIMDouble* toArray() const;
		void toJOINT(JOINT& result) const;

		ROBSIMDouble getDistance(q_vec& q, bool theta1_any = false);
		std::vector<ROBSIMDouble> get_q_dist_weights() { return q_dist_weights_; }

		void display();

	private:
		/*struct QDistWeights {
			ROBSIMDouble theta1 = 1, theta2 = 1, d3 = 1, theta4 = 1;
		} q_dist_weights;*/
		std::vector<ROBSIMDouble> q_dist_weights_ = { 1, 1, 1, 1 };
	};

	ROBSIMDouble getDistance(q_vec& q1, q_vec& q2, bool theta1_any = false);
	ROBSIMDouble getDistance(ROBSIMDouble theta_g, ROBSIMDouble theta_s);

	std::vector<DHParameter> getDHParamsFromQVec(q_vec& q);
	std::vector<Transformation> getTransformationFromDHParamsVector(std::vector<DHParameter> dh_vec);

	void UTOI(vec4& v, frame& f);
	void ITOU(frame& f, vec4& v);

	void TMULT(frame& brela, frame& crelb, frame& crela);
	void TINVERT(frame& brela, frame& arelb);

	class ROBOT {
	public:
		frame wrelb_, trels_;
		ROBOT() { }

		bool check_q_limits(q_vec& q, bool verbose = false);
		//std::vector<bool> check_q_limits(const q_vec& q);
		bool setQ(q_vec& q);
		q_vec getQ() { return q_; }

		q_vec current_config();

		void setDHParams(std::vector<DHParameter> dh_params) { dh_params_ = dh_params; }
		std::vector<DHParameter> getDHParams() { return dh_params_; }

		//void init_robot_();

		void update_transformations_();
		void update_frames_();

		void set_brels(frame& f) { brels_ = f; update_frames_(); }
		void set_trelw(frame& f) { trelw_ = f; update_frames_(); }
		//void set_wrelb(frame& f) { wrelb_ = f; update_frames_(); } // this is derived from dh_params
		//void set_trels(frame& f) { trels = f; } // this is dervied from other frames

		frame get_brels() { return brels_; }
		frame get_trelw() { return trelw_; }
		frame get_wrelb() { return wrelb_; }
		frame get_trels() { return trels_; }

		void KIN(q_vec& q, frame& wrelb);
		vec4 WHERE(q_vec& q, frame& trels);

		void INVKIN(frame& wrelb, q_vec& current, q_vec& near, q_vec& far, bool& sol);
		void SOLVE(frame& trels, q_vec& current, q_vec& near, q_vec& far, bool& sol);
		void SOLVE(vec4& pose, q_vec& current, q_vec& near, q_vec& far, bool& sol);

	private:
		q_vec q_;
		std::vector<DHParameter> dh_params_;
		std::unordered_map<std::string, DHParameter> dh_params_dict_;
		bool dh_params_set_ = false;
		Transformation tr_brels_, tr_1relb_, tr_2rel1_, tr_3rel2_, tr_4rel3_, tr_trelw_;
		std::vector<Transformation> transformations_;
		frame brels_, trelw_;//, wrelb_, trels_;

		bool check_q_limits_(ROBSIMDouble param, ROBSIMDouble min, ROBSIMDouble max);
		bool check_q_limits_(ROBSIMDouble param, ParameterLimits paramlimits, ROBSIMDouble tolerance);
		void adjust_q_within_tolerance_(q_vec& q, ROBSIMDouble tolerance);

		std::unordered_map<std::string, ParameterLimits> q_limits_ = {
												{"theta1", {-150, 150}},
												{"theta2", {-100, 100}},
												{"d3", {-200, -100}},
												{"theta4", {-160, 160}}
		};
	};
};
#pragma once

#include <vector>
#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <unordered_map>
#include <fstream>
#include <chrono>
#include <thread>
#include <ctime>

#include "ensc-488.h"


typedef double ROBSIMDouble;

//ROBSIMDouble atan2_DEG(ROBSIMDouble y, ROBSIMDouble x) { return RAD2DEG(atan2(y, x));}

template <typename T>
class Matrix {
public:
    // Constructors
	Matrix() : rows_(1), cols_(1), data_(1) {}
    Matrix(int rows, int cols);

    // Element access
    T& operator()(int row, int col);
    const T& operator()(int row, int col) const;

    // Getters for dimensions
    int getRows() const;
    int getCols() const;

    // Matrix operations
    Matrix<T> operator+(const Matrix<T>& other) const;
    Matrix<T> operator-(const Matrix<T>& other) const;
    Matrix<T> operator*(const Matrix<T>& other) const;
    Matrix<T> transpose() const;

    // Other methods
    void setToIdentity();
    Matrix<T> getSub3x3Matrix() const;
    void display() const;

private:
    int rows_;
    int cols_;
    std::vector<T> data_;
};

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

		q_vec operator*(const ROBSIMDouble& scalar) const {
			return q_vec(theta1 * scalar, theta2 * scalar, d3 * scalar, theta4 * scalar);
		}

		void display();

	private:
		/*struct QDistWeights {
			ROBSIMDouble theta1 = 1, theta2 = 1, d3 = 1, theta4 = 1;
		} q_dist_weights;*/
		std::vector<ROBSIMDouble> q_dist_weights_ = { 1, 1, 1, 1 };
	};

	class cubic_poly {
		std::vector<ROBSIMDouble> coeffs;
		public:
			cubic_poly() : coeffs({0, 0, 0, 0}) {}
			cubic_poly(std::vector<ROBSIMDouble> coeffs) : coeffs(coeffs) {}
			cubic_poly(ROBSIMDouble a0, ROBSIMDouble a1, ROBSIMDouble a2, ROBSIMDouble a3) : coeffs({a0, a1, a2, a3}) {}
			ROBSIMDouble operator()(ROBSIMDouble t) {
				return coeffs[0] + coeffs[1] * t + coeffs[2] * t * t + coeffs[3] * t * t * t;
			}
			ROBSIMDouble operator[](int i) {
				return coeffs[i];
			}
			ROBSIMDouble get_position(ROBSIMDouble t) {
				return coeffs[0] + coeffs[1] * t + coeffs[2] * t * t + coeffs[3] * t * t * t;
			}
			ROBSIMDouble get_velocity(ROBSIMDouble t) {
				return coeffs[1] + 2 * coeffs[2] * t + 3 * coeffs[3] * t * t;
			}
			ROBSIMDouble get_acceleration(ROBSIMDouble t) {
				return 2 * coeffs[2] + 6 * coeffs[3] * t;
			}
			void display() {
				std::cout << "a0: " << coeffs[0] << " a1: " << coeffs[1] << " a2: " << coeffs[2] << " a3: " << coeffs[3] << std::endl;
			}
	};

	struct q_cubic_poly{
		cubic_poly theta1, theta2, d3, theta4;
		q_cubic_poly() {}
		q_cubic_poly(cubic_poly theta1, cubic_poly theta2, cubic_poly d3, cubic_poly theta4) : theta1(theta1), theta2(theta2), d3(d3), theta4(theta4) {}
		q_vec operator()(ROBSIMDouble t) {
			return q_vec(theta1(t), theta2(t), d3(t), theta4(t));
		}
		q_vec get_position(ROBSIMDouble t) {
			return q_vec(theta1.get_position(t), theta2.get_position(t), d3.get_position(t), theta4.get_position(t));
		}
		q_vec get_velocity(ROBSIMDouble t) {
			return q_vec(theta1.get_velocity(t), theta2.get_velocity(t), d3.get_velocity(t), theta4.get_velocity(t));
		}
		q_vec get_acceleration(ROBSIMDouble t) {
			return q_vec(theta1.get_acceleration(t), theta2.get_acceleration(t), d3.get_acceleration(t), theta4.get_acceleration(t));
		}
		void display() {
			std::cout << "Theta1: "; theta1.display();
			std::cout << "Theta2: "; theta2.display();
			std::cout << "D3: "; d3.display();
			std::cout << "Theta4: "; theta4.display();
		}
	};

	struct pos_vel_acc {
		q_vec pos, vel, acc;
		ROBSIMDouble time;
		pos_vel_acc() {}
		pos_vel_acc(ROBSIMDouble time, q_vec pos, q_vec vel, q_vec acc) : time(time), pos(pos), vel(vel), acc(acc) {}
	};
	

	ROBSIMDouble getDistance(q_vec& q1, q_vec& q2, bool theta1_any = false);
	ROBSIMDouble getDistance(ROBSIMDouble theta_g, ROBSIMDouble theta_s);

	std::vector<DHParameter> getDHParamsFromQVec(q_vec& q);
	std::vector<Transformation> getTransformationFromDHParamsVector(std::vector<DHParameter> dh_vec);

	void UTOI(vec4& v, frame& f);
	void ITOU(frame& f, vec4& v);

	void TMULT(frame& brela, frame& crelb, frame& crela);
	void TINVERT(frame& brela, frame& arelb);

	cubic_poly get_cubic_poly(ROBSIMDouble y0, ROBSIMDouble yf, ROBSIMDouble v0, ROBSIMDouble vf, ROBSIMDouble t0, ROBSIMDouble tf);
	cubic_poly get_cubic_poly(ROBSIMDouble y0, ROBSIMDouble yf, ROBSIMDouble v0, ROBSIMDouble vf, ROBSIMDouble h);
	q_cubic_poly get_q_cubic_poly(q_vec& q0, q_vec& qf, q_vec& v0, q_vec& vf, ROBSIMDouble t0, ROBSIMDouble tf);
	

	ROBSIMDouble get_q_velocity(ROBSIMDouble q0, ROBSIMDouble q_mid, ROBSIMDouble qf, ROBSIMDouble t0, ROBSIMDouble t_mid, ROBSIMDouble tf);
	//std::vector<ROBSIMDouble> get_q_velocities(ROBSIMDouble q0, std::vector<ROBSIMDouble> intermediate, ROBSIMDouble qf);
	std::vector<ROBSIMDouble> get_q_velocities(ROBSIMDouble q0, ROBSIMDouble q1, ROBSIMDouble q2, ROBSIMDouble q3, ROBSIMDouble qf,
											   ROBSIMDouble t0, ROBSIMDouble t1, ROBSIMDouble t2, ROBSIMDouble t3, ROBSIMDouble tf);

	void get_q_velocities(q_vec& q0, q_vec& q1, q_vec& q2, q_vec& q3, q_vec& qf, 
					ROBSIMDouble t0, ROBSIMDouble t1, ROBSIMDouble t2, ROBSIMDouble t3, ROBSIMDouble tf,
					q_vec& v0, q_vec& v1, q_vec& v2, q_vec& v3, q_vec& vf);

	void calculateTimeSegments(ROBSIMDouble t0, ROBSIMDouble tf, ROBSIMDouble& t1, ROBSIMDouble& t2, ROBSIMDouble& t3);
	

	class ROBOT {
	public:
		frame wrelb_, trels_;
		ROBOT() {}

		bool check_q_limits(q_vec& q, bool verbose = false);
		//std::vector<bool> check_q_limits(const q_vec& q);
		bool setQ(q_vec& q);
		q_vec getQ() { return q_; }

		q_vec current_config();

		void setDHParams(std::vector<DHParameter> dh_params) { dh_params_ = dh_params; }
		std::vector<DHParameter> getDHParams() { return dh_params_; }

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
		
		bool is_robot_q_valid() { return robot_q_valid_; }	

		void KIN(q_vec& q, frame& wrelb);
		vec4 WHERE(q_vec& q, frame& trels);

		void INVKIN(frame& wrelb, q_vec& current, q_vec& near, q_vec& far, bool& sol);
		void SOLVE(frame& trels, q_vec& current, q_vec& near, q_vec& far, bool& sol);
		void SOLVE(vec4& pose, q_vec& current, q_vec& near, q_vec& far, bool& sol);

		bool SOLVE(vec4& p0, vec4& p1, vec4& p2, vec4& p3, vec4& pf, q_vec& q0, q_vec& q1, q_vec& q2, q_vec& q3, q_vec& qf);

		bool check_trajectory_q_limits(q_vec& q0, q_vec& q1, q_vec& q2, q_vec& q3, q_vec& qf, bool verbose=true);
		std::vector<q_cubic_poly> trajectory_planner(q_vec& q0, q_vec& q1, q_vec& q2, q_vec& q3, q_vec& qf, ROBSIMDouble t0, ROBSIMDouble tf);

		pos_vel_acc sample_q_cubic_poly(q_cubic_poly q_poly, ROBSIMDouble t, ROBSIMDouble t0, ROBSIMDouble tf);
		std::vector<pos_vel_acc> create_trajectory_planner(q_vec& q0, q_vec& q1, q_vec& q2, q_vec& q3, q_vec& qf,
			ROBSIMDouble t0, ROBSIMDouble tf, ROBSIMDouble num_samples, bool moverobot=false);
		std::vector<pos_vel_acc> create_trajectory_planner2(q_vec& q0, q_vec& q1, q_vec& q2, q_vec& q3, q_vec& qf,
			ROBSIMDouble t0, ROBSIMDouble tf, ROBSIMDouble num_samples, bool moverobot=false);
		
		void dump_to_csv(const std::vector<pos_vel_acc>& data);
		void dump_to_csv(const ROBSIM::pos_vel_acc& data);
		
		void dump_to_csv(const std::vector<std::tuple<ROBSIM::vec4, ROBSIMDouble>>& data);
		void dump_to_csv(std::tuple<ROBSIM::vec4, ROBSIMDouble>& data);
		void dump_to_csv(ROBSIM::vec4 data, ROBSIMDouble time);
		void dump_to_csv(const ROBSIM::q_vec& data, ROBSIMDouble time);
		void dump_to_csv(ROBSIM::vec4 data, ROBSIMDouble time, std::string filename);

		bool check_q_vel_limits(q_vec& q_vel, bool verbose = false);
		bool check_q_acc_limits(q_vec& q_acc, bool verbose = false);

		void create_trajectory_planner_logging_files();

		bool move_conf_vel_acc(q_vec& conf, q_vec& vel, q_vec& acc);

		void print_trajectory_debugging(ROBSIMDouble t0, ROBSIMDouble t1, ROBSIMDouble t2, ROBSIMDouble t3, ROBSIMDouble tf,
			q_vec& q0, q_vec& q1, q_vec& q2, q_vec& q3, q_vec& qf, std::vector<q_cubic_poly>& q_polys, int verbose=0);

	private:
		bool robot_q_valid_ = false;
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
		std::unordered_map<std::string, ParameterLimits> q_vel_limits_ = {
												{"theta1", {-150, 150}},
												{"theta2", {-150, 150}},
												{"d3", {-50, 50}},
												{"theta4", {-150, 150}}
		};
		std::unordered_map<std::string, ParameterLimits> q_acc_limits_ = {
												{"theta1", {-600, 600}},
												{"theta2", {-600, 600}},
												{"d3", {-200, 200}},
												{"theta4", {-600, 600}}
		};
	};
};

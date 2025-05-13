#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <cmath>

using namespace Eigen;

#define PI 3.14159265358979

bool isSO3(const Matrix3d& R);
bool isSE3(const Matrix4d& T);

void fillSparseBlock(
  SparseMatrix<double>& M, int startrow, int startcol,
  const MatrixXd& blockMat);
const MatrixXd getSparseBlock(
  SparseMatrix<double>& M, int startrow, int startcol, int nRow, int nCol);

const Matrix3d skew3(const Vector3d& v);
const Vector3d unSkew3(const Matrix3d& vHat);

const Matrix4d skew6(const VectorXd& S);
const VectorXd unSkew6(const Matrix4d& sHat);

const Matrix4d invSE3(const Matrix4d& T);
const MatrixXd AdSE3(const Matrix4d& T);

const MatrixXd ad6(const VectorXd& V);

const MatrixXd diag66(const Matrix3d& M1, const Matrix3d& M2);

const Vector3d clip(
  const Vector3d& v, const Vector3d& uBound, const Vector3d& lBound);
const double clip(double val, double uBound, double lBound);

/* Matrix log & unskewing*/
const Vector3d matLogSO3(const Matrix3d& R); // for SO(3)
const VectorXd matLogSE3(const Matrix4d& T); // for SE(3)

/* Function 'G' for se3 exponential calculation */
const Matrix3d expG(double theta, const Vector3d& om);

/* Derivative of function G */
const Matrix3d expGprime(double theta, const Vector3d& om);

/* Function 'F' for jacobian calculation */
const Vector3d jacF(
  double theta, const Vector3d& om, const Vector3d& v, const Vector3d& p);

/* Derivative of function F */
const Vector3d jacFprime(
  double theta, const Vector3d& om, const Vector3d& v, const Vector3d& p);
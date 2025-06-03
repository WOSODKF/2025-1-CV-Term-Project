#include "utils/math.hpp"

using namespace Eigen;

bool isSO3(const Matrix3d& R) {
  const double tolerance = 1e-10;
  Matrix3d test = R.transpose() * R - Matrix3d::Identity();
  bool det = (abs(R.determinant() - 1) < tolerance);
  bool inv = false;

  if (test.isZero(tolerance))
    inv = true;

  return det && inv;
}

bool isSE3(const Matrix4d& T) {
  const double tolerance = 1e-10;

  bool SO3 = isSO3(T.block<3, 3>(0, 0));
  bool lastRow =
    (T.block<1, 4>(3, 0) - Vector4d {0, 0, 0, 1}.transpose()).isZero(tolerance);

  if (!SO3)
    std::cout << "NOT SO3" << std::endl;
  if (!lastRow)
    std::cout << "lastRow violation" << std::endl;

  return SO3 && lastRow;
}

// Fill sparse matrix with block matrix from start row and col
void fillSparseBlock(
  SparseMatrix<double>& M, int startrow, int startcol,
  const MatrixXd& blockMat) {
  // block matrix size validness check
  bool isRowValid = blockMat.rows() <= M.rows() - startrow;
  bool isColValid = blockMat.cols() <= M.cols() - startcol;

  if (isRowValid && isColValid) {
    // fill block
    for (int i = 0; i < blockMat.rows(); i++) {
      for (int j = 0; j < blockMat.cols(); j++) {
        // M.insert(i + startrow, j + startcol) = blockMat(i, j);   // method
        // 'insert' cannot change existing values    ->  use 'coeffRef' instead
        M.coeffRef(i + startrow, j + startcol) = blockMat(i, j);
      }
    }
  } else {
    std::cout << "Invalid block assignment in fillSparseBlock: " << std::endl
         << blockMat << std::endl
         << "to " << startrow << ", " << startcol << "of " << blockMat.rows()
         << " x " << blockMat.cols() << std::endl;
    exit(-1);
  }
}

// Get nRow x nCol submatrix from sparse matrix
const MatrixXd getSparseBlock(
  SparseMatrix<double>& M, int startrow, int startcol, int nRow, int nCol) {
  // block matrix size validness check
  bool isRowValid = nRow <= M.rows() - startrow;
  bool isColValid = nCol <= M.cols() - startcol;

  if (isRowValid && isColValid) {
    // access block
    MatrixXd blockMat(nRow, nCol);
    blockMat.setZero();

    for (int i = 0; i < nRow; i++) {
      for (int j = 0; j < nCol; j++) {
        blockMat(i, j) = M.coeff(startrow + i, startcol + j);
      }
    }

    return blockMat;
  } else {
    std::cout << "Invalid block access in getSparseBlock: " << std::endl
         << "to " << startrow << ", " << startcol << "of " << nRow << " x "
         << nCol << std::endl;
    exit(-1);
  }
}

const Matrix3d skew3(const Vector3d& v) {
  Matrix3d vHat = Matrix3d::Zero();
  vHat << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;

  return vHat;
}

const Vector3d unSkew3(const Matrix3d& vHat) {
  Vector3d v = {0, 0, 0};

  const double tolerance = 1e-10;
  // bool isHat = abs(vHat(0, 0)) < tolerance && abs(vHat(1, 1)) < tolerance &&
  // abs(vHat(2, 2)) < tolerance && vHat(0, 1) + vHat(1, 0) < tolerance &&
  // vHat(0, 2) + vHat(2, 0) < tolerance && vHat(1, 2) + vHat(2, 1) < tolerance;
  bool isHat = (vHat + vHat.transpose()).isZero(tolerance);
  if (!isHat) {
    std::cout << "Wrong matrix input to unSkew3 operation: " << std::endl << vHat << std::endl;

    // return v;
    exit(-1);
  } else {
    v(0) = vHat(2, 1);
    v(1) = vHat(0, 2);
    v(2) = vHat(1, 0);

    // std::cout << "Input matrix to vee operation: " << std::endl
    //      << vHat << std::endl
    //      << "Return: " << std::endl
    //      << v.transpose() << std::endl;

    return v;
  }
}

const Matrix4d skew6(const VectorXd& S) {
  if (S.size() != 6) {
    std::cout << "Wrong vector input to skew6 operation: " << std::endl
         << S.transpose() << std::endl;
    exit(-1);
  }

  Matrix4d sHat;
  sHat.setZero();

  Vector3d om = S.block<3, 1>(0, 0);
  Vector3d v = S.block<3, 1>(3, 0);

  sHat.block<3, 3>(0, 0) = skew3(om);
  sHat.block<3, 1>(0, 3) = v;

  return sHat;
}

const VectorXd unSkew6(const Matrix4d& sHat) {
  const double tolerance = 1e-10;
  if (!sHat.block<1, 4>(3, 0).isZero(tolerance)) {
    std::cout << "Wrong matrix input to unSkew6 operation: " << std::endl << sHat << std::endl;
    exit(-1);

    // cf: omHat of S will be checked in unSkew3 function
  }

  VectorXd S(6);
  S.setZero();

  S.block<3, 1>(0, 0) = unSkew3(sHat.block<3, 3>(0, 0));
  S.block<3, 1>(3, 0) = sHat.block<3, 1>(0, 3);

  return S;
}

const Matrix4d invSE3(const Matrix4d& T) {
  if (!isSE3(T)) {
    std::cout << "Input matrix to invSE3 is not SE(3): " << std::endl << T << std::endl;
    exit(-1);
  }
  Matrix3d R = T.block<3, 3>(0, 0);
  Vector3d p = T.block<3, 1>(0, 3);

  Matrix4d invT;
  invT.setIdentity();

  invT.block<3, 3>(0, 0) = R.transpose();
  invT.block<3, 1>(0, 3) = -R.transpose() * p;

  return invT;
}

const MatrixXd AdSE3(const Matrix4d& T) {
  if (!isSE3(T)) {
    std::cout << "Input matrix to AdSE3 is not SE(3): " << std::endl << T << std::endl;
    exit(-1);
  }
  Matrix3d R = T.block<3, 3>(0, 0);
  Vector3d p = T.block<3, 1>(0, 3);

  MatrixXd AdT(6, 6);
  AdT.setZero();

  AdT.block<3, 3>(0, 0) = R;
  AdT.block<3, 3>(3, 0) = skew3(p) * R;
  AdT.block<3, 3>(3, 3) = R;

  return AdT;
}

const MatrixXd ad6(const VectorXd& V) {
  if (V.size() != 6) {
    std::cout << "Wrong input to ad6: " << std::endl << V << std::endl;
    exit(-1);
  }

  MatrixXd adV(6, 6);
  adV.setZero();

  adV.topLeftCorner(3, 3) = skew3(V.topLeftCorner(3, 1));
  adV.bottomLeftCorner(3, 3) = skew3(V.bottomLeftCorner(3, 1));
  adV.bottomRightCorner(3, 3) = skew3(V.topLeftCorner(3, 1));

  return adV;
}

const MatrixXd diag66(const Matrix3d& M1, const Matrix3d& M2) {
  MatrixXd M(6, 6);
  M.setZero();

  M.block<3, 3>(0, 0) = M1;
  M.block<3, 3>(3, 3) = M2;

  return M;
}

const Vector3d clip(
  const Vector3d& v, const Vector3d& uBound, const Vector3d& lBound) {
  Vector3d vClip;

  for (int i = 0; i < 3; i++) {
    vClip(i) = std::max(lBound(i), std::min(uBound(i), v(i)));
  }

  return vClip;
}
const double clip(double val, double uBound, double lBound){
  return std::max(std::min(val, uBound), lBound);
}

const Vector3d matLogSO3(const Matrix3d& R) {
  // cf) SO(3) checking -> update later
  const double tolerance = 1e-6;
  double theta = acos((R.trace() - 1) / 2);
  double coeff = abs(theta) > tolerance ? (theta / (2 * sin(theta))) : 0.5;
  Matrix3d omSkew = R - R.transpose();
  Vector3d om = unSkew3(omSkew);

  // std::cout << "trace: " << M.trace() << std::endl;
  // std::cout << "theta: " << theta << std::endl;
  // std::cout << "coeff: " << coeff << std::endl;

  // exception for PI rot
  if (abs(theta - MY_PI) < tolerance) {
    // std::cout << "special case: theta = PI" << std::endl;
    Matrix3d omHatSq = 0.5 * (R - Matrix3d::Identity());
    Vector3d omPi = {
      sqrt(0.5 * (omHatSq(0, 0) - omHatSq(1, 1) - omHatSq(2, 2))),
      sqrt(0.5 * (-omHatSq(0, 0) + omHatSq(1, 1) - omHatSq(2, 2))),
      sqrt(0.5 * (-omHatSq(0, 0) - omHatSq(1, 1) + omHatSq(2, 2)))};

    int sgn2 = 2 * ((omHatSq(0, 1) > 0) - 0.5);
    int sgn3 = 2 * ((omHatSq(0, 2) > 0) - 0.5);
    // std::cout << "signs: " << sgn2 << " " << sgn3 << std::endl;
    // std::cout << "values: " << omPi.transpose() << std::endl;
    omPi(1) = sgn2 * omPi(1);
    omPi(2) = sgn3 * omPi(2);
    return omPi;
  } else
    return coeff * om;
}

const VectorXd matLogSE3(const Matrix4d& T){
  const double tolerance = 1e-6;

  Matrix3d R = T.block<3, 3>(0, 0);
  Vector3d t = T.block<3, 1>(0, 3);

  Vector3d omega = matLogSO3(R);
  double theta = omega.norm();

  Matrix3d omegaHat = skew3(omega);  // user-defined or use unSkew3 inverse
  Matrix3d V_inv;

  if (theta < tolerance) {
    V_inv = Matrix3d::Identity() - 0.5 * omegaHat +
      (1.0 / 12.0) * omegaHat * omegaHat;
  } else {
    double sin_theta = sin(theta);
    double cos_theta = cos(theta);
    double theta2 = theta * theta;
    V_inv = Matrix3d::Identity() - 0.5 * omegaHat +
      (1.0 / (theta2)) *
        (1.0 - (sin_theta / theta) / (2.0 * (1.0 - cos_theta / theta))) *
        omegaHat * omegaHat;
  }

  Vector3d v = V_inv * t;

  VectorXd xi;
  xi.resize(6);
  xi.head<3>() = omega;
  xi.tail<3>() = v;
  return xi;
}

const Matrix3d expG(double theta, const Vector3d& om) {
  return theta * Matrix3d::Identity() + (1 - cos(theta)) * skew3(om) +
    (theta - sin(theta)) * skew3(om) * skew3(om);
}

const Matrix3d expGprime(double theta, const Vector3d& om) {
  return Matrix3d::Identity() + sin(theta) * skew3(om) +
    (1 - cos(theta)) * skew3(om) * skew3(om);
}

const Vector3d jacF(
  double theta, const Vector3d& om, const Vector3d& v, const Vector3d& p) {
  Matrix3d exponent = theta * skew3(om);
  return exponent.exp() * p + expG(theta, om) * v;
}

const Vector3d jacFprime(
  double theta, const Vector3d& om, const Vector3d& v, const Vector3d& p) {
  Matrix3d exponent = theta * skew3(om);
  return skew3(om) * exponent.exp() * p + expGprime(theta, om) * v;
}
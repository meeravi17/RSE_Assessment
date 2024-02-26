#ifndef MATRIX_UTILS_H
#define MATRIX_UTILS_H

#include <ros/ros.h>
#include <Eigen/Dense>

class MatrixUtils {
private:
  int rotation_row = 3;
  int rotation_column = 3;
  int translation_row = 1;
  int translation_column = 3;

public:
  double **initializeDummyRotationMatrix() {
    double **rotation = new double *[rotation_row];
    for (int i = 0; i < rotation_row; i++) {
      rotation[i] = new double[rotation_column];
      for (int j = 0; j < rotation_column; j++) {
        if (i == j) {
          rotation[i][j] = 1;
        } else {
          rotation[i][j] = 0;
        }
      }
    }
    return rotation;
  }

  double **initializeDummyTranslationMatrix() {
    double **translation = new double *[translation_row];
    for (int i = 0; i < translation_row; i++) {
      translation[i] = new double[translation_column];
      for (int j = 0; j < translation_column; j++) {
        translation[i][j] = 0;
      }
    }
    return translation;
  }

  double **getRotationFromEigen(Eigen::Matrix4d mat) {
    double **rotation = new double *[rotation_row];
    for (int i = 0; i < rotation_row; i++) {
      rotation[i] = new double[rotation_column];
      for (int j = 0; j < rotation_column; j++) {
        rotation[i][j] = mat(i, j);
      }
    }
    return rotation;
  }

  double **getTranslationFromEigen(Eigen::Matrix4d mat) {
    double **translation = new double *[translation_row];
    for (int i = 0; i < translation_row; i++) {
      translation[i] = new double[translation_column];
      for (int j = 0; j < translation_column; j++) {
        translation[i][j] = mat(j, translation_column);
      }
    }
    return translation;
  }

  Eigen::Matrix4d transformEigenToMatrix(double **rotation_matrix, double **translation_matrix) {
    Eigen::Matrix4d mat;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        mat(i, j) = rotation_matrix[i][j];
      }
    }

    for (int i = 0; i < 3; i++) {
      mat(i, 3) = translation_matrix[0][i];
    }
    return mat;
  }

  double **MatrixAddition(double **a, double **b) {
    double **addition = new double *[translation_row];

    for (int i = 0; i < translation_row; i++) {
      addition[i] = new double[translation_column];
      for (int j = 0; j < translation_column; j++) {
        addition[i][j] = a[i][j] + b[i][j];
      }
    }
    return addition;
  }

  double **MatrixMultiplication(double **a, double **b) {
    double temp_product = 0;
    double **product = new double *[rotation_row];

    // perform matrix multiplication
    for (int i = 0; i < rotation_row; i++) {
      product[i] = new double[rotation_column];
      for (int j = 0; j < rotation_column; j++) {
        for (int k = 0; k < 3; k++) {
          temp_product += a[i][k] * b[k][j];
        }
        product[i][j] = temp_product;
        temp_product = 0;
      }
    }
    return product;
  }

  int getMinimumValueIndex(std::vector<double> vec) {
    int loc = 0;
    double temp = vec[0];
    for (int i = 0; i < vec.size(); i++) {
      if (vec[i] < temp) {
        temp = vec[i];
        loc = i;
      }
    }
    return loc;
  }

  Eigen::Matrix4d addTransformation(std::vector<Eigen::Matrix4d> mat) {
    double **rotation_result, **translation_result, **stored_rotation,
        **stored_translation;
    // initialize dummy rotation matrix and translation matrix
    // so that during the first iteration, the result would be the same as the
    // stored value
    rotation_result = initializeDummyRotationMatrix();
    translation_result = initializeDummyTranslationMatrix();
    for (int i = 0; i < mat.size(); i++) {
      stored_rotation = getRotationFromEigen(mat[i]);
      stored_translation = getTranslationFromEigen(mat[i]);
      rotation_result = MatrixMultiplication(rotation_result, stored_rotation);
      translation_result =
          MatrixAddition(translation_result, stored_translation);
    }
    return transformEigenToMatrix(rotation_result, translation_result);
  }
};

#endif
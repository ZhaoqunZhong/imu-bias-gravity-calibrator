//
// Created by zhongzhaoqun on 2021/8/6.
//

#ifndef VIO_ACC_BIAS_CALIBRATOR_H
#define VIO_ACC_BIAS_CALIBRATOR_H
#include <ceres/ceres.h>

class AccMeasureFactor : public ceres::SizedCostFunction<1, 3, 1> {
public:
    AccMeasureFactor(Eigen::Vector3d & acc_measure) : acc(acc_measure) {};
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
        Eigen::Vector3d ba(parameters[0][0], parameters[0][1], parameters[0][2]);
        double g_norm = parameters[1][0];

        double minus_ba_norm = sqrt((acc - ba).transpose() * (acc - ba));
        *residuals = minus_ba_norm - g_norm;
        if (jacobians) {
            if (jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jacobian_ba(jacobians[0]);
                jacobian_ba = (ba.transpose() - acc.transpose()) / minus_ba_norm;
            }
            if (jacobians[1]) {
                *jacobians[1] = -1;
            }
        }
        return true;
    }

    Eigen::Vector3d acc;
};

class AccMeasureFactorSquared : public ceres::SizedCostFunction<1, 3, 1> {
public:
    AccMeasureFactorSquared(Eigen::Vector3d & acc_measure) : acc(acc_measure) {};
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
        Eigen::Vector3d ba(parameters[0][0], parameters[0][1], parameters[0][2]);
        double g_norm = parameters[1][0];

        double minus_ba_norm = (acc - ba).transpose() * (acc - ba);
        *residuals = minus_ba_norm - g_norm * g_norm;
        if (jacobians) {
            if (jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jacobian_ba(jacobians[0]);
                jacobian_ba = 2*(ba.transpose() - acc.transpose());
            }
            if (jacobians[1]) {
                *jacobians[1] = -2*g_norm;
            }
        }
        return true;
    }

    Eigen::Vector3d acc;
};


#endif //VIO_ACC_BIAS_CALIBRATOR_H

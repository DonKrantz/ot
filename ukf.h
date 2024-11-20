#pragma once
/*************************************************************************************************************
 * Class for Discrete Unscented Kalman Filter
 *
 *
 * See https://github.com/pronenewbits for more!
 ************************************************************************************************************/
#ifndef UKF_H
#define UKF_H

#include "konfig.h"
#include "matrix.h"

#if ((2*SS_X_LEN + 1) > MATRIX_MAXIMUM_SIZE)
#error("The MATRIX_MAXIMUM_SIZE is too small for sigma points (at least need (2*SS_X_LEN + 1))!");
#endif

class UKF
{
public:
    UKF(const Matrix& XInit, const Matrix& PInit, const Matrix& Rv, const Matrix& Rn_sensor1, const Matrix& Rn_sensor2, const Matrix& Rn_combined,
        bool (*bNonlinearUpdateX)(Matrix&, const Matrix&, const Matrix&),
        bool (*bNonlinearUpdateY_sensor1)(Matrix&, const Matrix&, const Matrix&),
        bool (*bNonlinearUpdateY_sensor2)(Matrix&, const Matrix&, const Matrix&),
        bool (*bNonlinearUpdateY_combined)(Matrix&, const Matrix&, const Matrix&));
    void vReset(const Matrix& XInit, const Matrix& PInit, const Matrix& Rv, const Matrix& Rn_sensor1, const Matrix& Rn_sensor2);
    bool bUpdate(const Matrix& Y, const Matrix& U, bool sensor1Available, bool sensor2Available);

    const Matrix GetX()   const { return X_Est; }
    const Matrix GetY()   const { return Y_Est; }
    const Matrix GetP()   const { return P; }
    const Matrix GetErr() const { return Err; }

protected:
    bool bCalculateSigmaPoint(void);
    bool bUnscentedTransform(Matrix& Out, Matrix& OutSigma, Matrix& P, Matrix& DSig,
        bool (*_vFuncNonLinear)(Matrix& xOut, const Matrix& xInp, const Matrix& U),
        const Matrix& InpSigma, const Matrix& InpVector,
        const Matrix& _CovNoise);

private:
    bool (*bNonlinearUpdateX) (Matrix& X_dot, const Matrix& X, const Matrix& U);
    bool (*bNonlinearUpdateY_sensor1) (Matrix& Y_Est, const Matrix& X, const Matrix& U);
    bool (*bNonlinearUpdateY_sensor2) (Matrix& Y_Est, const Matrix& X, const Matrix& U);
    bool (*bNonlinearUpdateY_combined) (Matrix& Y_Est, const Matrix& X, const Matrix& U);

    Matrix X_Est{ SS_X_LEN, 1 };
    Matrix X_Sigma{ SS_X_LEN, (2 * SS_X_LEN + 1) };

    Matrix Y_Est{ SS_Z_LEN, 1 };
    Matrix Y_Sigma_sensor1{ SS_Z_LEN, (2 * SS_X_LEN + 1) };
    Matrix Y_Sigma_sensor2{ SS_Z_LEN, (2 * SS_X_LEN + 1) };

    Matrix P{ SS_X_LEN, SS_X_LEN };
    Matrix P_Chol{ SS_X_LEN, SS_X_LEN };

    Matrix DX{ SS_X_LEN, (2 * SS_X_LEN + 1) };
    Matrix DY{ SS_Z_LEN, (2 * SS_X_LEN + 1) };

    Matrix Py{ SS_Z_LEN, SS_Z_LEN };
    Matrix Pxy{ SS_X_LEN, SS_Z_LEN };

    Matrix Wm{ 1, (2 * SS_X_LEN + 1) };
    Matrix Wc{ 1, (2 * SS_X_LEN + 1) };

    Matrix Rv{ SS_X_LEN, SS_X_LEN };
    Matrix Rn_sensor1{ SS_Z_LEN, SS_Z_LEN };
    Matrix Rn_sensor2{ SS_Z_LEN, SS_Z_LEN };
    Matrix Rn_combined{ SS_Z_LEN * 2, SS_Z_LEN * 2 };

    Matrix Err{ SS_Z_LEN, 1 };
    Matrix Gain{ SS_X_LEN, SS_Z_LEN };
    float_prec Gamma;
};

#endif // UKF_H

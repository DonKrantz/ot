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
    UKF(const Matrix& XInit, const Matrix& PInit, const Matrix& Rv, const Matrix& Rn_rovl, const Matrix& Rn_dvl, const Matrix& Rn_combined,
        bool (*bNonlinearUpdateX)(Matrix&, const Matrix&, const Matrix&),
        bool (*bNonlinearUpdateY_rovl)(Matrix&, const Matrix&, const Matrix&),
        bool (*bNonlinearUpdateY_dvl)(Matrix&, const Matrix&, const Matrix&),
        bool (*bNonlinearUpdateY_combined)(Matrix&, const Matrix&, const Matrix&));
    void vReset(const Matrix& XInit, const Matrix& PInit, const Matrix& Rv, const Matrix& Rn_rovl, const Matrix& Rn_dvl);
    bool bUpdate(const Matrix& Y, const Matrix& U, bool rovl_available, bool dvl_available);

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
    bool (*bNonlinearUpdateY_rovl) (Matrix& Y_Est, const Matrix& X, const Matrix& U);
    bool (*bNonlinearUpdateY_dvl) (Matrix& Y_Est, const Matrix& X, const Matrix& U);
    bool (*bNonlinearUpdateY_combined) (Matrix& Y_Est, const Matrix& X, const Matrix& U);

    Matrix X_Est{ SS_X_LEN, 1 };
    Matrix X_Sigma{ SS_X_LEN, (2 * SS_X_LEN + 1) };

    Matrix Y_Est{ SS_Z_LEN_COMBINED, 1 };
    //Matrix Y_Sigma_rovl{ SS_Z_LEN, (2 * SS_X_LEN + 1) };
    //Matrix Y_Sigma_dvl{ SS_Z_LEN, (2 * SS_X_LEN + 1) };

    Matrix P{ SS_X_LEN, SS_X_LEN };
    Matrix P_Chol{ SS_X_LEN, SS_X_LEN };

    Matrix DX{ SS_X_LEN, (2 * SS_X_LEN + 1) };
    Matrix DY{ SS_Z_LEN_COMBINED, (2 * SS_X_LEN + 1) };

    Matrix Py{ SS_Z_LEN_COMBINED, SS_Z_LEN_COMBINED };
    Matrix Pxy{ SS_X_LEN, SS_Z_LEN_COMBINED };

    Matrix Wm{ 1, (2 * SS_X_LEN + 1) };
    Matrix Wc{ 1, (2 * SS_X_LEN + 1) };

    Matrix Rv{ SS_X_LEN, SS_X_LEN };
    Matrix Rn_rovl{ SS_Z_LEN_ROVL, SS_Z_LEN_ROVL};
    Matrix Rn_dvl{ SS_Z_LEN_DVL, SS_Z_LEN_DVL };
    Matrix Rn_combined{ SS_Z_LEN_COMBINED, SS_Z_LEN_COMBINED };

    Matrix Err{ SS_Z_LEN_COMBINED, 1 };
    Matrix Gain{ SS_X_LEN, SS_Z_LEN_COMBINED };
    float_prec Gamma;
};

#endif // UKF_H

/*************************************************************************************************************
 *  Template project for Discrete Unscented Kalman Filter library
 *
 * See https://github.com/pronenewbits for more!
 ************************************************************************************************************/
#include "konfig.h"
#include "matrix.h"
#include "ukf.h"
#include "utilities.h"
#include "system_state.h"
#include <cmath>
#include "OmniUkf.h"
#include "configuration.h"


#include <random>



 /* ============================================ UKF Variables/function declaration ============================================ */
 /* Just example; in konfig.h:
  *  SS_X_LEN = 2
  *  SS_Z_LEN = 1
  *  SS_U_LEN = 1
  */
  /* UKF initialization constant -------------------------------------------------------------------------------------- */
#define P_INIT              (10.0f)
#define Rv_INIT_POS         (0.001f) // variance in state position
#define Rv_INIT_VEL         (0.005f) // variance in state velocity

#define Rn_INIT_ROVL_BEAR   (30.f)       // bearing/elevation variance
#define Rn_INIT_ROVL_SR     (1.f) //Slant range variance
#define Rn_INIT_DVL         (0.01f)
/* P(k=0) variable -------------------------------------------------------------------------------------------------- */
float_prec UKF_PINIT_data[SS_X_LEN * SS_X_LEN] = {  P_INIT, 0,      0,      0,      0,      0,
                                                    0,      P_INIT, 0,      0,      0,      0,
                                                    0,      0,      P_INIT, 0,      0,      0,
                                                    0,      0,      0,      P_INIT, 0,      0,
                                                    0,      0,      0,      0,      P_INIT, 0,
                                                    0,      0,      0,      0,      0,      P_INIT };
Matrix UKF_PINIT(SS_X_LEN, SS_X_LEN, UKF_PINIT_data);
/* Rv constant ------------------------------------------------------------------------------------------------------ */
float_prec UKF_RVINIT_data[SS_X_LEN * SS_X_LEN] = { Rv_INIT_POS, 0,      0,      0,      0,      0,
                                                    0,      Rv_INIT_VEL, 0,      0,      0,      0,
                                                    0,      0,      Rv_INIT_POS, 0,      0,      0,
                                                    0,      0,      0,      Rv_INIT_VEL, 0,      0,
                                                    0,      0,      0,      0,      Rv_INIT_POS, 0,
                                                    0,      0,      0,      0,      0,      Rv_INIT_VEL };
Matrix UKF_RvINIT(SS_X_LEN, SS_X_LEN, UKF_RVINIT_data);
/* Rn constant ------------------------------------------------------------------------------------------------------ */
float_prec UKF_RNINIT_data_ROVL[SS_Z_LEN * SS_Z_LEN] = { Rn_INIT_ROVL_SR,   0,                  0,                  
                                                         0,                 Rn_INIT_ROVL_BEAR,  0,                  
                                                         0,                 0,                  Rn_INIT_ROVL_BEAR};

float_prec UKF_RNINIT_data_DVL[SS_Z_LEN * SS_Z_LEN] = { Rn_INIT_DVL,   0,           0,
                                                        0,             Rn_INIT_DVL, 0,
                                                        0,             0,           Rn_INIT_DVL };

float_prec UKF_RNINIT_data_COMBINED[SS_Z_LEN * SS_Z_LEN * 4] = { Rn_INIT_ROVL_SR, 0,        0,           0,      0,      0,
                                                                 0,      Rn_INIT_ROVL_BEAR, 0,           0,      0,      0,
                                                                 0,      0,      Rn_INIT_ROVL_BEAR,      0,      0,      0,
                                                                 0,      0,      0,         Rn_INIT_DVL, 0,      0,
                                                                 0,      0,      0,         0,           Rn_INIT_DVL,    0,
                                                                 0,      0,      0,         0,           0,      Rn_INIT_DVL };

Matrix UKF_RnINIT_ROVL(SS_Z_LEN, SS_Z_LEN, UKF_RNINIT_data_ROVL);
Matrix UKF_RnINIT_DVL(SS_Z_LEN, SS_Z_LEN, UKF_RNINIT_data_DVL);
Matrix UKF_RnINIT_COMBINED(SS_Z_LEN * 2, SS_Z_LEN * 2, UKF_RNINIT_data_COMBINED);


/* Nonlinear & linearization function ------------------------------------------------------------------------------- */
bool Main_bUpdateNonlinearX(Matrix& X_Next, const Matrix& X, const Matrix& U);
bool Main_bUpdateNonlinearY_ROVL(Matrix& Y, const Matrix& X, const Matrix& U);
bool Main_bUpdateNonlinearY_DVL(Matrix& Y, const Matrix& X, const Matrix& U);
bool Main_bUpdateNonlinearY_COMBINED(Matrix& Y, const Matrix& X, const Matrix& U);
/* UKF variables ---------------------------------------------------------------------------------------------------- */
Matrix X(SS_X_LEN, 1);
Matrix Y_ROVL(SS_Z_LEN, 1);
Matrix Y_DVL(SS_Z_LEN, 1);
Matrix Y_COMBINED(SS_Z_LEN * 2, 1);

Matrix U(SS_U_LEN, 1);
/* UKF system declaration ------------------------------------------------------------------------------------------- */
UKF UKF_OMNI(X, UKF_PINIT, UKF_RvINIT, UKF_RnINIT_ROVL, UKF_RnINIT_DVL, UKF_RnINIT_COMBINED, Main_bUpdateNonlinearX, Main_bUpdateNonlinearY_ROVL, Main_bUpdateNonlinearY_DVL, Main_bUpdateNonlinearY_COMBINED);



/* ========================================= Auxiliary variables/function declaration ========================================= */
TIMING timerLed, timerUKF;
double computeTime;
char bufferTxSer[100];

bool run_ukf = false;

void run()
{
    setup();
    run_ukf = true;
}

void stop()
{
    run_ukf = false;
}

void setup() {
    /* serial to display data */
    //Serial.begin(115200);
    //while (!Serial) {}

    X.vSetToZero();
    X[0][0] = 0.5f;
    X[1][0] = 0.f;
    X[2][0] = -1.3;
    X[3][0] = 0.f;
    X[4][0] = 4.2f;
    X[5][0] = 0.f;
    UKF_OMNI.vReset(X, UKF_PINIT, UKF_RvINIT, UKF_RnINIT_ROVL, UKF_RnINIT_DVL);
}

std::random_device rd{};
std::mt19937 generator{ rd() };

std::normal_distribution<float> distribution{ 0, 10 };


void loop() {
if (run_ukf && elapsed(timerUKF) >= SS_DT) {
        timerUKF = Clock().now();



        /* ================== Read the sensor data / simulate the system here ================== */
        /* ... */

        //0: slant, 1: bearing, 2: elevation


        //static float velX = 0;

        //Y[0][0] = 1 + velX + distribution(generator);

        ////// Add error
        //Y[1][0] = 45 + distribution(generator);
        ///*if (Y[1][0] > 360)
        //    Y[1][0] = Y[1][0] - 360;*/
        //Y[2][0] = -15;

        //velX += 1.f;

        //Get ROVL data
        Y_ROVL[0][0] = rovl_usrth.slant_range;
        Y_ROVL[1][0] = rovl_usrth.apparent_bearing_math;
        Y_ROVL[2][0] = rovl_usrth.apparent_elevation;

        //Get DVL velocity in world frame
        //Change uS to S
        float delta_time = t650_dvpdx.delta_time_uS / 1000000.f;
        // TODO: Figure out frame issue here. Negative is just a band aid
        float x_vel = t650_dvpdx.position_delta_x / delta_time;
        float y_vel = t650_dvpdx.position_delta_y / delta_time;
        float z_vel = t650_dvpdx.position_delta_z / delta_time;


        vec3 rel_vel(x_vel, y_vel, z_vel);

        Quaternion mav_orientation = Quaternion(mav_roll, mav_pitch, mav_yaw, ANGLE_TYPE::RADIANS);
        vec3 world_vel = mav_orientation.Rotate(rel_vel);

        // World_vel is in NED. Need to rotate to ENU
        float tempY = world_vel.y;
        world_vel.y = world_vel.x;
        world_vel.x = tempY;
        world_vel.z = -world_vel.z;

        Y_DVL[0][0] = world_vel.x;
        Y_DVL[1][0] = world_vel.y;
        Y_DVL[2][0] = world_vel.z;

        //TODO: DELETE JUST TESTING
       /* Quaternion offset_test(0, 0, 30);

        world_vel = offset_test.Rotate(world_vel);*/
        //END DELETE

        Y_DVL[0][0] = world_vel.x;
        Y_DVL[1][0] = world_vel.y;
        Y_DVL[2][0] = world_vel.z;


        /*static float vel = 0;
        Y_ROVL[0][0] = 10;
        Y_ROVL[1][0] = 90 + distribution(generator);
        Y_ROVL[2][0] = -15;
        vel++;*/

        /*Y_DVL[0][0] = 0.1;
        Y_DVL[1][0] = -1;
        Y_DVL[2][0] = 0;*/

        Y_COMBINED[0][0] = Y_ROVL[0][0];
        Y_COMBINED[1][0] = Y_ROVL[1][0];
        Y_COMBINED[2][0] = Y_ROVL[2][0];
        Y_COMBINED[3][0] = Y_DVL[0][0];
        Y_COMBINED[4][0] = Y_DVL[1][0];
        Y_COMBINED[5][0] = Y_DVL[2][0];

        // Don't use rovl data until we get gnss orientation. Time difference is usually ~10 ms but can get up to 100.
        // Could try integrating angular rates, but feels unnecessary and error prone. Could request orientation based on delay
        bool rovl_data = rovl_usrth.slant_range != 0 && usrth_timestamp < gnss_timestamp;
        bool dvl_data = !std::isnan(Y_DVL[0][0]) && Y_DVL[0][0] != 0;

        /* ------------------ Read the sensor data / simulate the system here ------------------ */

        Matrix Y_DATA = rovl_data && dvl_data ? Y_COMBINED : rovl_data ? Y_ROVL : Y_DVL;
        /* ============================= Update the Kalman Filter ============================== */
        computeTime = elapsed(timerUKF);
        if (!UKF_OMNI.bUpdate(Y_DATA, U, rovl_data, dvl_data)) {
            setup();
            printf("Whoop\n");
        }
        computeTime = (elapsed(timerUKF) - computeTime);
        /* ----------------------------- Update the Kalman Filter ------------------------------ */


        /* =========================== Print to serial (for plotting) ========================== */
#if (1)
    /* Print: Computation time, X_Est[0] */
        Matrix x = UKF_OMNI.GetX();
        //printf("%.3f %.3f %.3f %.3f %.3f %.3f\n", x[0][0], x[1][0], x[2][0], x[3][0], x[4][0], x[5][0]);
        omnifusion.sendUKF(x[0][0], x[2][0], x[4][0], mav_orientation);

        /*vec3 estVel(x[1][0], x[3][0], x[5][0]);

        printf("DIFF: %f\n", estVel.length() - world_vel.length());*/

        if (rovl_data)
        {
            //printf("Difference in time: %f \n", usrth_timestamp - gnss_timestamp);
            omnifusion.sendRovlRawToMap(Y_ROVL[1][0], Y_ROVL[2][0], Y_ROVL[0][0], true);
            rovl_usrth.slant_range = 0;
        }
        
        /*snprintf(bufferTxSer, sizeof(bufferTxSer) - 1, "%.3f %.3f %.3f ", ((float)computeTime), x[0][0], x[1][0]);
        printf(bufferTxSer);
        printf("\n");*/
#endif
        

        //printf("ELAPSED: %f\n", elapsed(timerUKF));
        /* --------------------------- Print to serial (for plotting) -------------------------- */
    }
}


bool Main_bUpdateNonlinearX(Matrix& X_Next, const Matrix& X, const Matrix& U)
{
    /* Insert the nonlinear update transformation here
     *          x(k+1) = f[x(k), u(k)]
     */
    //JUST ROVL RN. Just adding velocity estimate * dt
    X_Next[0][0] = X[0][0] + X[1][0] * SS_DT;
    X_Next[1][0] = X[1][0];
    X_Next[2][0] = X[2][0] + X[3][0] * SS_DT;
    X_Next[3][0] = X[3][0];
    X_Next[4][0] = X[4][0] + X[5][0] * SS_DT;
    X_Next[5][0] = X[5][0];

    return true;
}

bool Main_bUpdateNonlinearY_ROVL(Matrix& Y, const Matrix& X, const Matrix& U)
{
    /* Insert the nonlinear measurement transformation here
     *          y(k)   = h[x(k), u(k)]
     */
    float_prec x = X[0][0];
    float_prec y = X[2][0];
    float_prec z = X[4][0];

    //State represetns offset from buoy in world frame
    vec3 rov_offset_world(x, y, z);

    Quaternion omnitrack_rotation = gnss_orientation;

    omnitrack_rotation.Invert();

    Quaternion rovl_yaw_offset(0, 0, -90);
    rovl_yaw_offset.Invert();

    //rotate into omni frame then into ROVL frame
    vec3 rov_offset_omni = omnitrack_rotation.Rotate(rov_offset_world);
    vec3 rov_offset_rovl = rovl_yaw_offset.Rotate(rov_offset_omni);

    x = rov_offset_rovl.x;
    y = rov_offset_rovl.y;
    z = rov_offset_rovl.z;


    Y[0][0] = sqrt(x*x + y*y + z*z);

    //Sign issue. Range of atan and asin: [-90, 90]
    float_prec bearing = atan2(y, x) * 180 / fPI;
    if (bearing < 0)
    {
        bearing += 360;
    }

    Y[1][0] = bearing;

    float_prec elevation = asin(z / Y[0][0]) * 180 / fPI;

    /*printf("ELEVATION %.3f BEARING %.3f\n", elevation, bearing);*/
    Y[2][0] = elevation;

    /*Y[0][0] = x;

    Y[1][0] = y;
    Y[2][0] = z;*/

    return true;
}

bool Main_bUpdateNonlinearY_DVL(Matrix& Y, const Matrix& X, const Matrix& U)
{
    Y[0][0] = X[1][0];
    Y[1][0] = X[3][0];
    Y[2][0] = X[5][0];

    return true;
}

bool Main_bUpdateNonlinearY_COMBINED(Matrix& Y, const Matrix& X, const Matrix& U)
{
    Main_bUpdateNonlinearY_ROVL(Y, X, U);
    
    Y[3][0] = X[1][0];
    Y[4][0] = X[3][0];
    Y[5][0] = X[5][0];

    return true;
}




void SPEW_THE_ERROR(char const* str)
{
#if (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_PC)
    cout << (str) << endl;
#elif (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_EMBEDDED_ARDUINO)
    Serial.println(str);
#else
    /* Silent function */
#endif
    while (1);
}

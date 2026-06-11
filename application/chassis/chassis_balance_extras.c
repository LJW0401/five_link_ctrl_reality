#include "chassis_balance_extras.h"

#if (CHASSIS_TYPE == CHASSIS_BALANCE)
#include "math.h"

#define K_TABLE_N 30

// L0 网格点 (m)，由 lqr_config.json 的 L0_values 导出
static const float L0_TABLE[K_TABLE_N] = {
    0.1f, 0.11034483f, 0.12068966f, 0.13103448f, 0.14137931f,
    0.15172414f, 0.16206897f, 0.17241379f, 0.18275862f, 0.19310345f,
    0.20344828f, 0.2137931f, 0.22413793f, 0.23448276f, 0.24482759f,
    0.25517241f, 0.26551724f, 0.27586207f, 0.2862069f, 0.29655172f,
    0.30689655f, 0.31724138f, 0.32758621f, 0.33793103f, 0.34827586f,
    0.35862069f, 0.36896552f, 0.37931034f, 0.38965517f, 0.4f,
};

// 各网格点对应的 LQR K 矩阵 (2x6)，由 lqr_config.json 的 K_table 导出
static const float K_TABLE[K_TABLE_N][2][6] = {
    {
        {56.354931f, 7.5454802f, 2.1898984f, 0.91527431f, -32.090422f, -5.1705952f},
        {378.33131f, 49.043909f, -26.534636f, -21.524289f, -124.04604f, -29.366074f},
    },
    {
        {75.304401f, 10.388394f, -1.1975206f, -1.8562337f, -35.974059f, -5.9248663f},
        {290.80947f, 39.15182f, -27.772628f, -22.671177f, -49.61501f, -17.761308f},
    },
    {
        {80.753495f, 11.518372f, -3.4227477f, -3.7134336f, -32.989866f, -5.5297303f},
        {208.01323f, 29.00991f, -23.784364f, -19.571466f, 3.3473214f, -9.2346997f},
    },
    {
        {80.078716f, 11.802696f, -4.668532f, -4.7877541f, -28.382429f, -4.8262715f},
        {148.94212f, 21.472115f, -19.081304f, -15.843853f, 34.964733f, -4.0324411f},
    },
    {
        {77.503481f, 11.785475f, -5.349805f, -5.406457f, -24.115866f, -4.1490073f},
        {109.45724f, 16.283965f, -15.086143f, -12.65212f, 53.324528f, -0.95850144f},
    },
    {
        {74.655598f, 11.692357f, -5.7313419f, -5.7808592f, -20.599245f, -3.5773838f},
        {82.934914f, 12.714592f, -11.959699f, -10.141199f, 64.240469f, 0.89652623f},
    },
    {
        {72.058244f, 11.604089f, -5.9527606f, -6.0231946f, -17.784791f, -3.1113133f},
        {64.655908f, 10.202881f, -9.5547279f, -8.2016191f, 70.977035f, 2.0563942f},
    },
    {
        {69.836145f, 11.545703f, -6.0858655f, -6.1915507f, -15.535425f, -2.7327342f},
        {51.678259f, 8.386122f, -7.6970662f, -6.6980874f, 75.295354f, 2.8086053f},
    },
    {
        {67.981589f, 11.521969f, -6.1684164f, -6.3166752f, -13.722488f, -2.4230336f},
        {42.200876f, 7.0367927f, -6.2460204f, -5.5199683f, 78.161429f, 3.3130685f},
    },
    {
        {66.448976f, 11.530742f, -6.2209841f, -6.4154439f, -12.244276f, -2.1669446f},
        {35.103536f, 6.01068f, -5.097758f, -4.5850795f, 80.122939f, 3.6615409f},
    },
    {
        {65.187316f, 11.567913f, -6.255196f, -6.4974807f, -11.024258f, -1.9527274f},
        {29.671015f, 5.214229f, -4.1773737f, -3.8338679f, 81.501857f, 3.9085499f},
    },
    {
        {64.150507f, 11.629212f, -6.2778563f, -6.5684786f, -10.005508f, -1.7715139f},
        {25.433562f, 4.5851204f, -3.4308034f, -3.223198f, 82.494181f, 4.0876181f},
    },
    {
        {63.299746f, 11.710797f, -6.2930723f, -6.6319206f, -9.1455481f, -1.6166072f},
        {22.074096f, 4.0807434f, -2.8186548f, -2.7215447f, 83.223102f, 4.2200089f},
    },
    {
        {62.603267f, 11.809405f, -6.3033932f, -6.6900049f, -8.4124046f, -1.4829134f},
        {19.373103f, 3.6712403f, -2.3118556f, -2.3055784f, 83.768311f, 4.3195957f},
    },
    {
        {62.035325f, 11.922322f, -6.3104397f, -6.7441583f, -7.7817321f, -1.3665192f},
        {17.175066f, 3.3352161f, -1.8886525f, -1.9577984f, 84.182706f, 4.3956614f},
    },
    {
        {61.575089f, 12.047312f, -6.3152645f, -6.7953319f, -7.2347721f, -1.2643837f},
        {15.367498f, 3.0570241f, -1.532546f, -1.6648984f, 84.502223f, 4.4545591f},
    },
    {
        {61.205659f, 12.182542f, -6.3185639f, -6.8441753f, -6.7568986f, -1.1741177f},
        {13.867495f, 2.8250099f, -1.2308579f, -1.4166299f, 84.751778f, 4.5007255f},
    },
    {
        {60.913256f, 12.326499f, -6.3208063f, -6.8911426f, -6.3365774f, -1.0938229f},
        {12.612913f, 2.6303467f, -0.97372722f, -1.2050036f, 84.948977f, 4.5373155f},
    },
    {
        {60.686584f, 12.477936f, -6.3223113f, -6.9365586f, -5.9646143f, -1.0219755f},
        {11.556458f, 2.4662488f, -0.75339711f, -1.023721f, 85.106467f, 4.5666097f},
    },
    {
        {60.516323f, 12.635818f, -6.3232991f, -6.9806606f, -5.6336068f, -0.95734f},
        {10.661635f, 2.3274272f, -0.56370211f, -0.8677645f, 85.233471f, 4.5902808f},
    },
    {
        {60.394736f, 12.799282f, -6.3239238f, -7.0236255f, -5.3375391f, -0.89890588f},
        {9.8999298f, 2.2097088f, -0.3996942f, -0.73309852f, 85.336811f, 4.6095724f},
    },
    {
        {60.315358f, 12.967605f, -6.3242935f, -7.0655883f, -5.0714793f, -0.84583917f},
        {9.2488034f, 2.1097632f, -0.25736712f, -0.6164485f, 85.421595f, 4.6254204f},
    },
    {
        {60.272755f, 13.140178f, -6.3244849f, -7.1066538f, -4.8313519f, -0.79744632f},
        {8.690258f, 2.0249048f, -0.13345042f, -0.51513557f, 85.491693f, 4.6385367f},
    },
    {
        {60.262334f, 13.316483f, -6.3245528f, -7.1469051f, -4.6137638f, -0.75314626f},
        {8.2097817f, 1.9529478f, -0.025254112f, -0.42695156f, 85.550067f, 4.6494689f},
    },
    {
        {60.280188f, 13.496083f, -6.3245363f, -7.1864093f, -4.4158698f, -0.71244882f},
        {7.7955712f, 1.8920969f, 0.069449901f, -0.35006356f, 85.599007f, 4.6586415f},
    },
    {
        {60.322982f, 13.678602f, -6.3244634f, -7.2252217f, -4.2352693f, -0.67493783f},
        {7.4379478f, 1.8408665f, 0.15251915f, -0.28294025f, 85.640301f, 4.666387f},
    },
    {
        {60.387849f, 13.863717f, -6.3243543f, -7.2633885f, -4.0699241f, -0.64025788f},
        {7.128916f, 1.7980173f, 0.22551159f, -0.22429457f, 85.675354f, 4.6729678f},
    },
    {
        {60.472317f, 14.05115f, -6.3242235f, -7.3009489f, -3.9180938f, -0.60810374f},
        {6.8618247f, 1.7625088f, 0.28974123f, -0.17303866f, 85.705281f, 4.6785922f},
    },
    {
        {60.574243f, 14.24066f, -6.3240812f, -7.3379367f, -3.778285f, -0.57821189f},
        {6.6311055f, 1.7334615f, 0.34632217f, -0.12824825f, 85.73097f, 4.6834271f},
    },
    {
        {60.691761f, 14.432038f, -6.3239348f, -7.3743816f, -3.6492094f, -0.55035373f},
        {6.4320686f, 1.7101278f, 0.39620362f, -0.089134195f, 85.753139f, 4.6876067f},
    },
};

/**
 * @brief 获取K矩阵
 * @param[in]  l 腿长
 * @param[out] k K矩阵
 * @note 在 L0_TABLE 网格上对 K_TABLE 做线性插值，超出范围时钳位到端点
 */
void GetK(float l, float k[2][6], bool is_take_off)
{
    // 定位 l 所在区间 [L0_TABLE[lo], L0_TABLE[hi]]
    int hi = 1;
    while (hi < K_TABLE_N - 1 && l > L0_TABLE[hi]) {
        hi++;
    }
    int lo = hi - 1;

    // 插值系数，钳位到 [0,1] 以处理超出网格范围的腿长
    float t = (l - L0_TABLE[lo]) / (L0_TABLE[hi] - L0_TABLE[lo]);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 6; j++) {
            k[i][j] = K_TABLE[lo][i][j] * (1.0f - t) + K_TABLE[hi][i][j] * t;
        }
    }

    if (is_take_off) {
        k[0][0] = 0;
        k[0][1] = 0;
        k[0][2] = 0;
        k[0][3] = 0;
        k[0][4] = 0;
        k[0][5] = 0;
        // k[1][0] = 0;
        // k[1][1] = 0;
        k[1][2] = 0;
        k[1][3] = 0;
        k[1][4] = 0;
        k[1][5] = 0;
    }
}

/**
 * @brief 通过关节phi1和phi4的值获取L0和Phi0
 * @param[in]  phi1
 * @param[in]  phi4
 * @param[out] L0_Phi0 L0和Phi0
 */
void GetL0AndPhi0(float phi1, float phi4, float L0_Phi0[2])
{
    float YD, YB, XD, XB, lBD, A0, B0, C0, phi2, XC, YC;
    float L0, Phi0;
    YD = LEG_L4 * sin(phi4);
    YB = LEG_L1 * sin(phi1);
    XD = LEG_L5 + LEG_L4 * cos(phi4);
    XB = LEG_L1 * cos(phi1);
    lBD = sqrt((XD - XB) * (XD - XB) + (YD - YB) * (YD - YB));
    A0 = 2 * LEG_L2 * (XD - XB);
    B0 = 2 * LEG_L2 * (YD - YB);
    C0 = LEG_L2 * LEG_L2 + lBD * lBD - LEG_L3 * LEG_L3;
    phi2 = 2 * atan2((B0 + sqrt(A0 * A0 + B0 * B0 - C0 * C0)), (A0 + C0));
    XC = LEG_L1 * cos(phi1) + LEG_L2 * cos(phi2);
    YC = LEG_L1 * sin(phi1) + LEG_L2 * sin(phi2);
    L0 = sqrt((XC - LEG_L5 / 2) * (XC - LEG_L5 / 2) + YC * YC);
    Phi0 = atan2(YC, (XC - LEG_L5 / 2));

    L0_Phi0[0] = L0;
    L0_Phi0[1] = Phi0;
}

/**
 * @brief 获取dL0和dPhi0
 * @param[in]  J 雅可比矩阵
 * @param[in]  d_phi1 
 * @param[in]  d_phi4 
 */
void GetdL0AnddPhi0(float J[2][2], float d_phi1, float d_phi4, float dL0_dPhi0[2])
{
    // clang-format off
    float d_l0   = J[0][0] * d_phi1 + J[0][1] * d_phi4;
    float d_phi0 = J[1][0] * d_phi1 + J[1][1] * d_phi4;
    // clang-format on
    dL0_dPhi0[0] = d_l0;
    dL0_dPhi0[1] = d_phi0;
}

/**
 * @brief 获取腿部摆杆的等效力
 * @param[in]  J 雅可比矩阵
 * @param[in]  T1 
 * @param[in]  T2 
 * @param[out] F 0-F0 1-Tp
 */
void GetLegForce(float J[2][2], float T1, float T2, float F[2])
{
    float det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
    // clang-format off
    float inv_J[4] = {J[1][1] / det, -J[0][1] / det, 
                     -J[1][0] / det,  J[0][0] / det};
    // clang-format on
    //F = (inv_J.') * T
    float F0 = inv_J[0] * T1 + inv_J[2] * T2;
    float Tp = inv_J[1] * T1 + inv_J[3] * T2;

    F[0] = F0;
    F[1] = Tp;
}

/**
 * @brief 计算雅可比矩阵
 * @param phi1 
 * @param phi4 
 * @param J 
 */
void CalcJacobian(float phi1, float phi4, float J[2][2])
{
    float YD, YB, XD, XB, lBD, A0, B0, C0, XC, YC;
    float phi2, phi3;
    float L0, phi0;
    float j11, j12, j21, j22;

    YD = LEG_L4 * sin(phi4);
    YB = LEG_L1 * sin(phi1);
    XD = LEG_L5 + LEG_L4 * cos(phi4);
    XB = LEG_L1 * cos(phi1);
    lBD = sqrt((XD - XB) * (XD - XB) + (YD - YB) * (YD - YB));
    A0 = 2 * LEG_L2 * (XD - XB);
    B0 = 2 * LEG_L2 * (YD - YB);
    C0 = LEG_L2 * LEG_L2 + lBD * lBD - LEG_L3 * LEG_L3;
    phi2 = 2 * atan2((B0 + sqrt(A0 * A0 + B0 * B0 - C0 * C0)), A0 + C0);
    phi3 = atan2(YB - YD + LEG_L2 * sin(phi2), XB - XD + LEG_L2 * cos(phi2));
    XC = LEG_L1 * cos(phi1) + LEG_L2 * cos(phi2);
    YC = LEG_L1 * sin(phi1) + LEG_L2 * sin(phi2);
    L0 = sqrt((XC - LEG_L5 / 2) * (XC - LEG_L5 / 2) + YC * YC);
    phi0 = atan2(YC, XC - LEG_L5 / 2);

    j11 = (LEG_L1 * sin(phi0 - phi3) * sin(phi1 - phi2)) / sin(phi3 - phi2);
    j12 = (LEG_L4 * sin(phi0 - phi2) * sin(phi3 - phi4)) / sin(phi3 - phi2);
    j21 = (LEG_L1 * cos(phi0 - phi3) * sin(phi1 - phi2)) / (L0 * sin(phi3 - phi2));
    j22 = (LEG_L4 * cos(phi0 - phi2) * sin(phi3 - phi4)) / (L0 * sin(phi3 - phi2));

    J[0][0] = j11;
    J[0][1] = j12;
    J[1][0] = j21;
    J[1][1] = j22;
}

/**
 * @brief 计算VMC
 * @param[in]  F0 沿杆方向的力
 * @param[in]  Tp 髋关节力矩
 * @param[in]  J 雅可比矩阵
 * @param[out] T 2个关节的输出力矩
 */
void CalcVmc(float F0, float Tp, float J[2][2], float T[2])
{
    // clang-format off
    float JT[2][2] = {{J[0][0],J[1][0]}, // 转置矩阵
                      {J[0][1],J[1][1]}};
    float F[2] = {F0, Tp};
    // clang-format on
    float T1 = JT[0][0] * F[0] + JT[0][1] * F[1];
    float T2 = JT[1][0] * F[0] + JT[1][1] * F[1];

    T[0] = T1;
    T[1] = T2;
}

/**
 * @brief 通过L0和Phi0的值计算关节phi1和phi4
 * @param[in]  phi0
 * @param[in]  l0
 * @param[out] phi1_phi4 phi1和phi4
 * @note 用于位置控制时求逆解
 */
void CalcPhi1AndPhi4(float phi0, float l0, float phi1_phi4[2])
{
    float L5_2_pow;
    float Lca2, Lce2;
    float cos_phi11, cos_phi12, cos_phi41, cos_phi42;
    float phi11, phi12, phi41, phi42;
    float phi1, phi4;

    L5_2_pow = (LEG_L5 / 2) * (LEG_L5 / 2);  //(LEG_L5 / 2)^2
    Lca2 = l0 * l0 + L5_2_pow + l0 * LEG_L5 * cos(phi0);
    Lce2 = l0 * l0 + L5_2_pow - l0 * LEG_L5 * cos(phi0);

    cos_phi11 = (L5_2_pow + Lca2 - l0 * l0) / (LEG_L5 * sqrt(Lca2));
    cos_phi12 = (LEG_L1 * LEG_L1 + Lca2 - LEG_L2 * LEG_L2) / (2 * LEG_L1 * sqrt(Lca2));
    cos_phi41 = (L5_2_pow + Lce2 - l0 * l0) / (LEG_L5 * sqrt(Lce2));
    cos_phi42 = (LEG_L4 * LEG_L4 + Lce2 - LEG_L3 * LEG_L3) / (2 * LEG_L5 * sqrt(Lce2));

    phi11 = acos(cos_phi11);
    phi12 = acos(cos_phi12);

    phi41 = acos(cos_phi41);
    phi42 = acos(cos_phi42);

    phi1 = phi11 + phi12;
    phi4 = M_PI - (phi41 + phi42);

    phi1_phi4[0] = phi1;
    phi1_phi4[1] = phi4;
}

/**
 * @brief 通过当前底盘姿态和目标roll角计算两腿长度期望差值
 * @param[in]  Ld0 (m)当前左右腿长度差值(L0l - L0r)
 * @param[in]  theta0 (rad)当前底盘roll角
 * @param[in]  theta1 (rad)目标roll角
 * @return 两腿长度期望差值(m)(L1l - L1r)
 */
inline float CalcLegLengthDiff(float Ld0, float theta0, float theta1)
{
    return WHEEL_BASE * tanf(theta1) -
           cosf(theta0) / cosf(theta1) * (WHEEL_BASE * tanf(theta0) - Ld0);
}

/**
 * @brief 双腿腿长协调控制，维持腿长目标在范围内，同时尽可能达到两腿目标差值
 * @param[in]  Ll_ref   (m)左腿长度指针
 * @param[in]  Lr_ref   (m)右腿长度指针
 * @param[in]  diff (m)腿长差值
 * @param[in]  add  (m)腿长差值补偿量
 */
void CoordinateLegLength(float * Ll_ref, float * Lr_ref, float diff, float add)
{
    *Ll_ref = *Ll_ref + diff * 0.5f + add;
    *Lr_ref = *Lr_ref - diff * 0.5f - add;

    // float delta = *Ll_ref - *Lr_ref;
    // if (delta > MAX_LEG_LENGTH - MIN_LEG_LENGTH) {
    //     *Ll_ref = MAX_LEG_LENGTH;
    //     *Lr_ref = MIN_LEG_LENGTH;
    //     return;
    // } else if (delta < MIN_LEG_LENGTH - MAX_LEG_LENGTH) {
    //     *Ll_ref = MIN_LEG_LENGTH;
    //     *Lr_ref = MAX_LEG_LENGTH;
    //     return;
    // }

    //先判断短腿范围，再判断长腿范围
    float * short_leg = *Ll_ref < *Lr_ref ? Ll_ref : Lr_ref;
    float * long_leg = *Ll_ref < *Lr_ref ? Lr_ref : Ll_ref;
    float move = 0;
    move = MIN_LEG_LENGTH - *short_leg;
    if (move > 0) {
        *short_leg += move;
        *long_leg += move;
    }
    if (*long_leg > MAX_LEG_LENGTH) {
        *long_leg = MAX_LEG_LENGTH;
    }
}

#endif /* CHASSIS_BALANCE */

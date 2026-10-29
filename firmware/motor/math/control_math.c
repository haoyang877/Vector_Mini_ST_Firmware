#include "control_math.h"
#include <math.h>

/* 四分之一周期 256 等分；表值为 sin(i * pi / 512)，端点为 0 和 1。 */
static const float s_sin_quarter_tab[257] = {
    0.0f,           0.006135884649f, 0.01227153829f, 0.01840672991f, 0.02454122852f, 0.03067480318f,
    0.03680722294f, 0.04293825693f,  0.04906767433f, 0.05519524435f, 0.0613207363f,  0.06744391956f,
    0.0735645636f,  0.07968243797f,  0.08579731234f, 0.0919089565f,  0.09801714033f, 0.1041216339f,
    0.1102222073f,  0.1163186309f,   0.1224106752f,  0.1284981108f,  0.1345807085f,  0.1406582393f,
    0.1467304745f,  0.1527971853f,   0.1588581433f,  0.1649131205f,  0.1709618888f,  0.1770042204f,
    0.183039888f,   0.1890686641f,   0.195090322f,   0.2011046348f,  0.2071113762f,  0.2131103199f,
    0.2191012402f,  0.2250839114f,   0.2310581083f,  0.237023606f,   0.2429801799f,  0.2489276057f,
    0.2548656596f,  0.2607941179f,   0.2667127575f,  0.2726213554f,  0.2785196894f,  0.2844075372f,
    0.2902846773f,  0.2961508882f,   0.3020059493f,  0.30784964f,    0.3136817404f,  0.3195020308f,
    0.3253102922f,  0.3311063058f,   0.3368898534f,  0.3426607173f,  0.3484186802f,  0.3541635254f,
    0.3598950365f,  0.3656129978f,   0.371317194f,   0.3770074102f,  0.3826834324f,  0.3883450467f,
    0.3939920401f,  0.3996241998f,   0.405241314f,   0.4108431711f,  0.4164295601f,  0.4220002708f,
    0.4275550934f,  0.4330938189f,   0.4386162385f,  0.4441221446f,  0.4496113297f,  0.4550835871f,
    0.460538711f,   0.4659764958f,   0.4713967368f,  0.4767992301f,  0.4821837721f,  0.4875501601f,
    0.4928981922f,  0.498227667f,    0.5035383837f,  0.5088301425f,  0.5141027442f,  0.5193559902f,
    0.5245896827f,  0.5298036247f,   0.5349976199f,  0.5401714727f,  0.5453249884f,  0.5504579729f,
    0.555570233f,   0.5606615762f,   0.5657318108f,  0.5707807459f,  0.5758081914f,  0.5808139581f,
    0.5857978575f,  0.5907597019f,   0.5956993045f,  0.6006164794f,  0.6055110414f,  0.6103828063f,
    0.6152315906f,  0.6200572118f,   0.6248594881f,  0.6296382389f,  0.6343932842f,  0.6391244449f,
    0.6438315429f,  0.648514401f,    0.653172843f,   0.6578066933f,  0.6624157776f,  0.6669999223f,
    0.6715589548f,  0.6760927036f,   0.6806009978f,  0.6850836678f,  0.6895405447f,  0.6939714609f,
    0.6983762494f,  0.7027547445f,   0.7071067812f,  0.7114321957f,  0.7157308253f,  0.720002508f,
    0.724247083f,   0.7284643904f,   0.7326542717f,  0.7368165689f,  0.7409511254f,  0.7450577854f,
    0.7491363945f,  0.753186799f,    0.7572088465f,  0.7612023855f,  0.7651672656f,  0.7691033376f,
    0.7730104534f,  0.7768884657f,   0.7807372286f,  0.7845565972f,  0.7883464276f,  0.7921065773f,
    0.7958369046f,  0.7995372691f,   0.8032075315f,  0.8068475535f,  0.8104571983f,  0.8140363297f,
    0.8175848132f,  0.821102515f,    0.8245893028f,  0.8280450453f,  0.8314696123f,  0.834862875f,
    0.8382247056f,  0.8415549774f,   0.8448535652f,  0.8481203448f,  0.8513551931f,  0.8545579884f,
    0.85772861f,    0.8608669386f,   0.8639728561f,  0.8670462455f,  0.8700869911f,  0.8730949784f,
    0.8760700942f,  0.8790122264f,   0.8819212643f,  0.8847970984f,  0.8876396204f,  0.8904487232f,
    0.8932243012f,  0.8959662498f,   0.8986744657f,  0.901348847f,   0.9039892931f,  0.9065957045f,
    0.9091679831f,  0.911706032f,    0.9142097557f,  0.9166790599f,  0.9191138517f,  0.9215140393f,
    0.9238795325f,  0.9262102421f,   0.9285060805f,  0.9307669611f,  0.9329927988f,  0.9351835099f,
    0.9373390119f,  0.9394592236f,   0.9415440652f,  0.9435934582f,  0.9456073254f,  0.947585591f,
    0.9495281806f,  0.951435021f,    0.9533060404f,  0.9551411683f,  0.9569403357f,  0.9587034749f,
    0.9604305194f,  0.9621214043f,   0.9637760658f,  0.9653944417f,  0.966976471f,   0.9685220943f,
    0.9700312532f,  0.971503891f,    0.9729399522f,  0.9743393828f,  0.97570213f,    0.9770281427f,
    0.9783173707f,  0.9795697657f,   0.9807852804f,  0.9819638691f,  0.9831054874f,  0.9842100924f,
    0.9852776424f,  0.9863080972f,   0.9873014182f,  0.9882575677f,  0.98917651f,    0.9900582103f,
    0.9909026354f,  0.9917097537f,   0.9924795346f,  0.9932119492f,  0.99390697f,    0.9945645707f,
    0.9951847267f,  0.9957674145f,   0.9963126122f,  0.9968202993f,  0.9972904567f,  0.9977230666f,
    0.9981181129f,  0.9984755806f,   0.9987954562f,  0.9990777278f,  0.9993223846f,  0.9995294175f,
    0.9996988187f,  0.9998305818f,   0.9999247018f,  0.9999811753f,  1.0f,
};

//#pragma arm section code = "CCMRAMCODE"

/**
    * @brief  Constrain function
    * @param  amt: value to be constrained
    * @param  low: lower limit
    * @param  high: upper limit
    * @retval constrained value
 **/
float Math_Constrain(float amt, float low, float high)
{
    return ((amt < low) ? (low) : ((amt > high) ? high : amt));
}

/**
    * @brief  Normalize angle to 0-2PI
    * @param  angle: raw angle
    * @retval normalized angle
 **/
float Math_NormalizeAngle(float angle)
{
    float a;
    int a_int;

    a_int = (int)(angle * ONE_BY_2PI);
    a = angle - a_int * _2PI;

    return a >= 0 ? a : (a + _2PI);
}

/**
    * @brief  Transform float to uint32 without precision loss
    * @param  float_val: float value to be transformed
    * @retval uint32 value
 **/
uint32_t Math_FloatToBits(float float_val)
{
    uint32_t *integer_bits;
    integer_bits = (uint32_t *)(&float_val);
    return *integer_bits;
}

/**
    * @brief  Transform uint32_t to float without precision loss
    * @param  int_val: uint32 value to be transformed
    * @retval float value
 **/
float Math_BitsToFloat(uint32_t int_val)
{
    float *float_value;
    float_value = (float *)(&int_val);
    return *float_value;
}

/**
    * @brief  IIR Butterworth low-pass filter
              2 order
    * @param  input: value to be filtered
    * @param  *Math_FilterButterworth: Math_FilterButterworth struct pointer
    * @retval filtered value
 **/
float Math_FilterButterworth(float input, IirButterworthState *Math_FilterButterworth)
{
    float output;

    Math_FilterButterworth->state_n =
        Math_FilterButterworth->gain0 * input -
        Math_FilterButterworth->a1 * Math_FilterButterworth->state_n_1 -
        Math_FilterButterworth->a2 * Math_FilterButterworth->state_n_2;

    output = Math_FilterButterworth->gain1 *
             (Math_FilterButterworth->b0 * Math_FilterButterworth->state_n +
              Math_FilterButterworth->b1 * Math_FilterButterworth->state_n_1 +
              Math_FilterButterworth->b2 * Math_FilterButterworth->state_n_2);

    Math_FilterButterworth->state_n_2 = Math_FilterButterworth->state_n_1;
    Math_FilterButterworth->state_n_1 = Math_FilterButterworth->state_n;

    return output;
}

/**
    * @brief  Fast absolute value
    * @param  x: input value
    * @retval absolute value
 **/
float Math_Abs(float x)
{
    return x >= 0 ? x : -x;
}

/**
    * @brief  Fast square calculation
    * @param  x: input value
    * @retval square value
 **/
float Math_Square(float x)
{
    return x * x;
}

/**
    * @brief  Fast maximum of two values
    * @param  x: first input value
    * @param  y: second input value
    * @retval maximum value
 **/
float Math_Max(float x, float y)
{
    return (((x) > (y)) ? (x) : (y));
}

/**
    * @brief  Fast maximum of three values
    * @param  x: first input value
    * @param  y: second input value
    * @param  z: third input value
    * @retval maximum value
 **/
float fast_max_3(float x, float y, float z)
{
    float max_xy = (x > y) ? x : y;

    return (max_xy > z) ? max_xy : z;
}

/**
    * @brief  Fast minimum of two values
    * @param  x: first input value
    * @param  y: second input value
    * @retval minimum value
 **/
float Math_Min(float x, float y)
{
    return (((x) < (y)) ? (x) : (y));
}

/**
    * @brief  Fast minimum of three values
    * @param  x: first input value
    * @param  y: second input value
    * @param  z: third input value
    * @retval minimum value
 **/
float fast_min_3(float x, float y, float z)
{
    float min_xy = (x < y) ? x : y;

    return (min_xy < z) ? min_xy : z;
}

/* 用象限对称读取单位圆节点，再旋转不足一个表间隔的小角；保持亚分度步长。 */
void Math_SinCos(float theta_rad, float *sin_out, float *cos_out)
{
    if (!isfinite(theta_rad))
    {
        *sin_out = NAN;
        *cos_out = NAN;
        return;
    }
    if (theta_rad == 0.0f)
    {
        *sin_out = theta_rad;
        *cos_out = 1.0f;
        return;
    }
    float angle_rad = fabsf(theta_rad);
    if (angle_rad >= _2PI)
    {
        /* 跨一圈使用与fmodf逐位等价的减法，多圈保留原归一化。 */
        angle_rad = angle_rad < 2.0f * _2PI ? angle_rad - _2PI : fmodf(angle_rad, _2PI);
    }
    float position = angle_rad * (1024.0f / _2PI);
    unsigned index = (unsigned)position;
    unsigned offset = index & 255U;
    float rising = s_sin_quarter_tab[offset];
    float falling = s_sin_quarter_tab[256U - offset];
    float sin0;
    float cos0;
    switch ((index >> 8U) & 3U)
    {
    case 0U:
        sin0 = rising;
        cos0 = falling;
        break;
    case 1U:
        sin0 = falling;
        cos0 = -rising;
        break;
    case 2U:
        sin0 = -rising;
        cos0 = -falling;
        break;
    default:
        sin0 = -falling;
        cos0 = rising;
        break;
    }
    /* delta属于[0,pi/512)，三阶sin/四阶cos截断误差分别小于7.3e-14/7.5e-17。
     * 使用旋转保持单位圆；不能用线性弦插值，后者会使交接积分器反复旋转时衰减。 */
    float delta = (position - (float)index) * (_2PI / 1024.0f);
    float squared = delta * delta;
    float sin_delta = delta * (1.0f - squared * (1.0f / 6.0f));
    float cos_delta = 1.0f - squared * (0.5f - squared * (1.0f / 24.0f));
    float sine = sin0 * cos_delta + cos0 * sin_delta;
    *cos_out = cos0 * cos_delta - sin0 * sin_delta;
    *sin_out = theta_rad < 0.0f ? -sine : sine;
}

/**
    * @brief  Fast atan2 approximation
    * @param  y: y input value
    * @param  x: x input value
    * @retval angle value
 **/
float Math_Atan2(float y, float x)
{
    float abs_y = Math_Abs(y) + 1e-20f; // kludge to prevent 0/0 condition
    float angle;

    if (x >= 0)
    {
        float r = (x - abs_y) / (x + abs_y);
        float rsq = r * r;
        angle = ((0.1963f * rsq) - 0.9817f) * r + (_PI / 4.0f);
    }
    else
    {
        float r = (x + abs_y) / (abs_y - x);
        float rsq = r * r;
        angle = ((0.1963f * rsq) - 0.9817f) * r + (3.0f * _PI / 4.0f);
    }

    if (y < 0)
    {
        return (-angle);
    }
    else
    {
        return (angle);
    }
}

/**
    * @brief  Fast square root approximation
    * @param  val: input value
    * @retval square root value
 **/
float Math_Sqrt(float val)
{
    if (val < 1.19209290e-7f)
    {
        return 0.0f;
    }

    /* Preserve the legacy small-input cutoff. The target compiler emits
     * single-precision hardware sqrt, avoiding two Newton-step divisions
     * and the old float/long aliasing (which also assumed a 32-bit long). */
    return sqrtf(val);
}

/**
    * @brief  Fast integer power calculation
    * @param  x: base value
    * @param  y: exponent value
    * @retval power value
 **/
float Math_Power(float x, int y)
{
    float result = 1.0f;
    for (int i = 0; i < y; i++)
    {
        result *= x;
    }
    return result;
}

/**
    * @brief  Hard sign function
    * @param  x: input value
    * @retval sign value
 **/
float Math_Sign(float x)
{
    if (x < 0)
    {
        return -1;
    }
    else
    {
        return 1;
    }
}

/**
    * @brief  Copy sign from one value to another
    * @param  x: magnitude value
    * @param  y: sign source value
    * @retval signed value
 **/
float Math_CopySign(float x, float y)
{
    if (y >= 0)
    {
        return x;
    }
    else
    {
        return -x;
    }
}

//#pragma arm section

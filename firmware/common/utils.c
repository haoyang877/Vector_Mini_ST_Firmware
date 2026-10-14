#include "utils.h"
#include <math.h>

/* 保留原 2π/1023 采样网格，后半周期由 1023-index 镜像恢复。 */
static const float sin_half_tab[512] = {
    0.0f, 0.006141883f, 0.012283533f, 0.018424721f, 0.024565213f, 0.030704779f, 0.036843186f, 0.042980204f,
    0.0491156f, 0.055249144f, 0.061380603f, 0.067509747f, 0.073636344f, 0.079760163f, 0.085880974f, 0.091998545f,
    0.098112645f, 0.104223045f, 0.110329512f, 0.116431818f, 0.122529732f, 0.128623023f, 0.134711462f, 0.14079482f,
    0.146872866f, 0.152945372f, 0.159012108f, 0.165072846f, 0.171127357f, 0.177175412f, 0.183216784f, 0.189251244f,
    0.195278565f, 0.20129852f, 0.207310881f, 0.213315422f, 0.219311915f, 0.225300136f, 0.231279858f, 0.237250854f,
    0.243212902f, 0.249165774f, 0.255109247f, 0.261043097f, 0.266967099f, 0.27288103f, 0.278784668f, 0.284677789f,
    0.290560171f, 0.296431592f, 0.302291831f, 0.308140666f, 0.313977878f, 0.319803245f, 0.325616548f, 0.331417568f,
    0.337206086f, 0.342981884f, 0.348744743f, 0.354494446f, 0.360230777f, 0.365953519f, 0.371662456f, 0.377357372f,
    0.383038054f, 0.388704286f, 0.394355855f, 0.399992548f, 0.405614152f, 0.411220454f, 0.416811245f, 0.422386312f,
    0.427945445f, 0.433488435f, 0.439015072f, 0.444525148f, 0.450018455f, 0.455494787f, 0.460953935f, 0.466395695f,
    0.471819862f, 0.477226229f, 0.482614594f, 0.487984754f, 0.493336505f, 0.498669646f, 0.503983976f, 0.509279294f,
    0.5145554f, 0.519812096f, 0.525049182f, 0.530266463f, 0.53546374f, 0.540640817f, 0.5457975f, 0.550933594f,
    0.556048905f, 0.56114324f, 0.566216407f, 0.571268215f, 0.576298473f, 0.581306991f, 0.58629358f, 0.591258052f,
    0.59620022f, 0.601119898f, 0.6060169f, 0.610891041f, 0.615742137f, 0.620570006f, 0.625374464f, 0.630155332f,
    0.634912428f, 0.639645574f, 0.64435459f, 0.649039299f, 0.653699524f, 0.65833509f, 0.662945821f, 0.667531544f,
    0.672092085f, 0.676627273f, 0.681136937f, 0.685620906f, 0.690079011f, 0.694511085f, 0.698916959f, 0.703296468f,
    0.707649447f, 0.711975731f, 0.716275157f, 0.720547563f, 0.724792787f, 0.72901067f, 0.733201053f, 0.737363777f,
    0.741498685f, 0.745605622f, 0.749684433f, 0.753734962f, 0.757757059f, 0.761750571f, 0.765715347f, 0.769651238f,
    0.773558095f, 0.777435772f, 0.781284121f, 0.785102998f, 0.788892258f, 0.792651758f, 0.796381358f, 0.800080916f,
    0.803750292f, 0.807389348f, 0.810997947f, 0.814575952f, 0.818123229f, 0.821639644f, 0.825125065f, 0.828579359f,
    0.832002396f, 0.835394048f, 0.838754186f, 0.842082684f, 0.845379416f, 0.848644257f, 0.851877086f, 0.855077778f,
    0.858246215f, 0.861382275f, 0.864485842f, 0.867556798f, 0.870595027f, 0.873600414f, 0.876572847f, 0.879512212f,
    0.882418399f, 0.885291299f, 0.888130803f, 0.890936804f, 0.893709196f, 0.896447875f, 0.899152737f, 0.90182368f,
    0.904460604f, 0.907063408f, 0.909631995f, 0.912166268f, 0.914666132f, 0.917131491f, 0.919562254f, 0.921958327f,
    0.924319622f, 0.926646048f, 0.928937519f, 0.931193947f, 0.933415247f, 0.935601337f, 0.937752132f, 0.939867553f,
    0.941947519f, 0.943991951f, 0.946000774f, 0.94797391f, 0.949911286f, 0.951812829f, 0.953678466f, 0.955508127f,
    0.957301744f, 0.959059248f, 0.960780574f, 0.962465656f, 0.96411443f, 0.965726836f, 0.967302811f, 0.968842297f,
    0.970345235f, 0.971811568f, 0.973241242f, 0.974634202f, 0.975990396f, 0.977309773f, 0.978592282f, 0.979837876f,
    0.981046508f, 0.982218131f, 0.983352702f, 0.984450178f, 0.985510518f, 0.986533681f, 0.987519629f, 0.988468324f,
    0.989379732f, 0.990253817f, 0.991090547f, 0.991889889f, 0.992651815f, 0.993376295f, 0.994063301f, 0.994712809f,
    0.995324792f, 0.995899229f, 0.996436098f, 0.996935378f, 0.997397051f, 0.997821098f, 0.998207505f, 0.998556256f,
    0.998867339f, 0.999140742f, 0.999376453f, 0.999574465f, 0.999734771f, 0.999857363f, 0.999942237f, 0.99998939f,
    0.999998821f, 0.999970529f, 0.999904515f, 0.999800781f, 0.999659332f, 0.999480172f, 0.999263309f, 0.999008751f,
    0.998716507f, 0.998386589f, 0.998019008f, 0.997613779f, 0.997170916f, 0.996690438f, 0.996172361f, 0.995616706f,
    0.995023492f, 0.994392744f, 0.993724484f, 0.993018737f, 0.992275531f, 0.991494893f, 0.990676853f, 0.989821442f,
    0.988928691f, 0.987998635f, 0.987031309f, 0.986026749f, 0.984984993f, 0.98390608f, 0.982790051f, 0.981636948f,
    0.980446815f, 0.979219697f, 0.977955639f, 0.97665469f, 0.975316898f, 0.973942315f, 0.972530991f, 0.971082981f,
    0.969598338f, 0.968077119f, 0.966519381f, 0.964925183f, 0.963294585f, 0.961627649f, 0.959924437f, 0.958185014f,
    0.956409445f, 0.954597798f, 0.95275014f, 0.950866541f, 0.948947073f, 0.946991808f, 0.945000819f, 0.942974182f,
    0.940911972f, 0.938814269f, 0.936681151f, 0.934512699f, 0.932308993f, 0.930070118f, 0.927796158f, 0.925487199f,
    0.923143328f, 0.920764632f, 0.918351203f, 0.91590313f, 0.913420507f, 0.910903427f, 0.908351985f, 0.905766277f,
    0.903146401f, 0.900492455f, 0.89780454f, 0.895082756f, 0.892327208f, 0.889537998f, 0.886715233f, 0.883859017f,
    0.88096946f, 0.87804667f, 0.875090757f, 0.872101833f, 0.86908001f, 0.866025404f, 0.862938128f, 0.859818299f,
    0.856666036f, 0.853481456f, 0.850264681f, 0.847015831f, 0.843735029f, 0.840422398f, 0.837078064f, 0.833702153f,
    0.830294793f, 0.826856111f, 0.823386237f, 0.819885303f, 0.81635344f, 0.812790782f, 0.809197463f, 0.805573618f,
    0.801919385f, 0.798234901f, 0.794520305f, 0.790775737f, 0.787001339f, 0.783197252f, 0.779363621f, 0.77550059f,
    0.771608305f, 0.767686912f, 0.76373656f, 0.759757397f, 0.755749574f, 0.751713242f, 0.747648553f, 0.74355566f,
    0.739434718f, 0.735285882f, 0.731109309f, 0.726905156f, 0.722673583f, 0.718414747f, 0.714128811f, 0.709815936f,
    0.705476284f, 0.70111002f, 0.696717307f, 0.692298313f, 0.687853202f, 0.683382144f, 0.678885306f, 0.674362859f,
    0.669814973f, 0.665241819f, 0.66064357f, 0.6560204f, 0.651372483f, 0.646699994f, 0.642003109f, 0.637282006f,
    0.632536863f, 0.627767858f, 0.622975173f, 0.618158986f, 0.613319481f, 0.60845684f, 0.603571245f, 0.598662882f,
    0.593731936f, 0.588778592f, 0.583803038f, 0.578805461f, 0.57378605f, 0.568744993f, 0.563682482f, 0.558598707f,
    0.55349386f, 0.548368133f, 0.54322172f, 0.538054816f, 0.532867614f, 0.527660311f, 0.522433103f, 0.517186187f,
    0.511919761f, 0.506634024f, 0.501329175f, 0.496005415f, 0.490662943f, 0.485301963f, 0.479922675f, 0.474525283f,
    0.46910999f, 0.463677002f, 0.458226522f, 0.452758756f, 0.447273911f, 0.441772193f, 0.43625381f, 0.430718971f,
    0.425167883f, 0.419600757f, 0.414017802f, 0.408419229f, 0.402805249f, 0.397176074f, 0.391531917f, 0.385872989f,
    0.380199506f, 0.37451168f, 0.368809726f, 0.36309386f, 0.357364297f, 0.351621253f, 0.345864944f, 0.340095589f,
    0.334313404f, 0.328518607f, 0.322711418f, 0.316892056f, 0.311060739f, 0.305217688f, 0.299363123f, 0.293497265f,
    0.287620336f, 0.281732557f, 0.27583415f, 0.269925337f, 0.264006343f, 0.258077389f, 0.2521387f, 0.246190499f,
    0.240233011f, 0.234266461f, 0.228291073f, 0.222307074f, 0.216314689f, 0.210314143f, 0.204305664f, 0.198289478f,
    0.192265812f, 0.186234892f, 0.180196948f, 0.174152206f, 0.168100894f, 0.162043241f, 0.155979476f, 0.149909826f,
    0.143834521f, 0.137753791f, 0.131667863f, 0.125576969f, 0.119481338f, 0.1133812f, 0.107276784f, 0.101168322f,
    0.095056043f, 0.088940179f, 0.082820959f, 0.076698615f, 0.070573378f, 0.064445479f, 0.058315148f, 0.052182618f,
    0.046048119f, 0.039911883f, 0.033774142f, 0.027635126f, 0.021495068f, 0.015354199f, 0.009212751f, 0.003070956f,
};

//#pragma arm section code = "CCMRAMCODE"

/**
	* @brief  Constrain function
    * @param  amt: value to be constrained 
    * @param  low: lower limit
	* @param  high: upper limit
	* @retval constrained value 
 **/
float constrain(float amt, float low, float high)
{
	return ((amt < low) ? (low) : ((amt > high) ? high: amt));
}

/**
	* @brief  Normalize angle to 0-2PI
    * @param  angle: raw angle
	* @retval normalized angle 
 **/
float normalizeAngle(float angle)
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
uint32_t FloatToIntBit(float float_val)
{
	uint32_t *pInt;
	pInt = (uint32_t *)(&float_val);
	return *pInt;
}

/**
	* @brief  Transform uint32_t to float without precision loss
    * @param  int_val: uint32 value to be transformed 
	* @retval float value 
 **/
float IntBitToFloat(uint32_t int_val)
{
	float *pFloat;
	pFloat = (float *)(&int_val);
	return *pFloat;
}

/**
	* @brief  IIR Butterworth low-pass filter
              2 order
    * @param  input: value to be filtered
    * @param  *IIR_Butterworth: IIR_Butterworth struct pointer
	* @retval filtered value
 **/
float IIR_Butterworth(float input,IIR_Butterworth_TypeDef *IIR_Butterworth)
{
	float output;
	
	IIR_Butterworth->state_n = IIR_Butterworth->gain0 * input
								 -IIR_Butterworth->a1 * IIR_Butterworth->state_n_1
								 -IIR_Butterworth->a2 * IIR_Butterworth->state_n_2;
	
	output = IIR_Butterworth->gain1 * (IIR_Butterworth->b0 * IIR_Butterworth->state_n
									  +IIR_Butterworth->b1 * IIR_Butterworth->state_n_1
								      +IIR_Butterworth->b2 * IIR_Butterworth->state_n_2);
	
	IIR_Butterworth->state_n_2 = IIR_Butterworth->state_n_1;
	IIR_Butterworth->state_n_1 = IIR_Butterworth->state_n;
	
	return output;
}

/**
	* @brief  Fast absolute value
	* @param  x: input value
	* @retval absolute value
 **/
float fast_abs(float x)
{
	return x >=0 ? x: -x;
}

/**
	* @brief  Fast square calculation
	* @param  x: input value
	* @retval square value
 **/
float fast_sq(float x)
{
	return x * x;
}

/**
	* @brief  Fast maximum of two values
	* @param  x: first input value
	* @param  y: second input value
	* @retval maximum value
 **/
float fast_max(float x,float y)
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
float fast_min(float x,float y)
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

/**
	* @brief  Fast sine calculation
	* @param  theta: input angle
	* @retval sine value
 **/
float fast_sin(float theta)
{
	uint32_t index;

    theta = normalizeAngle(theta);
    /* 浮点归一化可能舍入到 2π；索引 1024 按周期环回，禁止越界。 */
    index = ((uint32_t)(LUT_MULT * theta)) & 1023U;
    if (index >= 512U)
    {
        /* 显式保留旧表周期末端的正零，避免快速浮点优化生成负零。 */
        if (index == 1023U)
        {
            return 0.0f;
        }
        return -sin_half_tab[1023U - index];
    }
    return sin_half_tab[index];
}

/**
	* @brief  Fast cosine calculation
	* @param  theta: input angle
	* @retval cosine value
 **/
float fast_cos(float theta)
{
	return fast_sin(_PI_2 - theta);
}

/**
	* @brief  Fast atan2 approximation
	* @param  y: y input value
	* @param  x: x input value
	* @retval angle value
 **/
float fast_atan2(float y, float x) 
{
    float abs_y = fast_abs(y) + 1e-20f; // kludge to prevent 0/0 condition
    float angle;

    if (x >= 0) {
        float r = (x - abs_y) / (x + abs_y);
        float rsq = r * r;
        angle = ((0.1963f * rsq) - 0.9817f) * r + (_PI / 4.0f);
    } else {
        float r = (x + abs_y) / (abs_y - x);
        float rsq = r * r;
        angle = ((0.1963f * rsq) - 0.9817f) * r + (3.0f * _PI / 4.0f);
    }

    if (y < 0) {
        return (-angle);
    } else {
        return (angle);
    }
}

/**
	* @brief  Fast square root approximation
	* @param  val: input value
	* @retval square root value
 **/
float fast_sqrt(float val) 
{
	if(val < 1.19209290e-7f)
		return 0.0f;

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
float fast_pow(float x, int y)
{
	float result = 1.0f;
	for(int i = 0; i < y; i++)
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
float sign_hard(float x)
{
	if(x < 0 )
		return -1;
	else
		return 1;
}

/**
	* @brief  Copy sign from one value to another
	* @param  x: magnitude value
	* @param  y: sign source value
	* @retval signed value
 **/
float copy_sign(float x, float y)
{
	if(y >= 0)
		return x;
	else 
		return -x;
}

//#pragma arm section

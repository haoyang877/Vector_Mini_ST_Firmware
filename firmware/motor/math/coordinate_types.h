#ifndef COORDINATE_TYPES_H
#define COORDINATE_TYPES_H

/** @brief 三相静止坐标；三个分量使用调用方约定的同一单位。 */
typedef struct
{
    float a, b, c;
} ABCVect;

/** @brief 两相静止坐标；两个分量使用调用方约定的同一单位。 */
typedef struct
{
    float alpha, beta;
} AlphaBetaVector;

/** @brief 两相旋转坐标；两个分量使用调用方约定的同一单位。 */
typedef struct
{
    float d, q;
} DqVector;

#endif

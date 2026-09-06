#ifndef CORE_SERVICES_CURRENT_CONTROL_CURRENT_CONTROL_MATH_H
#define CORE_SERVICES_CURRENT_CONTROL_CURRENT_CONTROL_MATH_H

void CurrentControl_Clarke(float phase_a, float phase_b, float phase_c,
    float *alpha, float *beta);
void CurrentControl_InverseClarke(float alpha, float beta,
    float *phase_a, float *phase_b, float *phase_c);
void CurrentControl_Park(float alpha, float beta, float sine, float cosine,
    float *direct, float *quadrature);
void CurrentControl_InversePark(float direct, float quadrature,
    float sine, float cosine, float *alpha, float *beta);

#endif

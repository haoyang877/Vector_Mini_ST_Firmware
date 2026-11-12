/* 离线记录生成器；只由 Python 前端在临时目录调用，不包含板端 IO。 */
#include "parameter_record.h"
#include "motor_axis_profile.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv)
{
    if (argc != 9 && argc != 16)
    {
        return 2;
    }
    uint8_t node = (uint8_t)strtoul(argv[1], NULL, 10);
    MotorWindingThermalConfig heat = {0};
    if (argc == 16)
    {
        heat = (MotorWindingThermalConfig){strtof(argv[9], NULL),
                                           strtof(argv[10], NULL),
                                           strtof(argv[11], NULL),
                                           strtof(argv[12], NULL),
                                           strtof(argv[13], NULL),
                                           strtof(argv[14], NULL),
                                           strtof(argv[15], NULL)};
    }
    MotorConfiguration configuration = {0};
    const MotorAxisPreset *p = MotorAxisProfile_ForNode(node);
    if (p == NULL)
    {
        fprintf(stderr, "no motor preset for node 0x%02X\n", (unsigned)node);
        return 2;
    }
    if (argv[8][0] != '\0' && strcmp(argv[8], p->model_name) != 0)
    {
        fprintf(stderr,
                "node 0x%02X (%s) requires %s, received %s\n",
                (unsigned)node,
                MotorJointType_Name(p->joint),
                p->model_name,
                argv[8]);
        return 2;
    }
    float ratio = argv[7][0] == '1' ? p->gear_ratio : 1.0f;
    float maximum_travel = p->output_travel_limit_rad * p->gear_ratio;
    float minimum = strcmp(argv[2], "default") == 0
                        ? (maximum_travel == 0.0f ? 0.0f : -maximum_travel)
                        : strtof(argv[2], NULL) * ratio;
    float maximum =
        strcmp(argv[3], "default") == 0 ? maximum_travel : strtof(argv[3], NULL) * ratio;
    if (!Param_CreateAxis(node,
                          minimum,
                          maximum,
                          strtof(argv[4], NULL) * ratio,
                          argc == 16 ? &heat : NULL,
                          &configuration))
    {
        fputs(argc == 9 && p->electrical->default_thermal == NULL
                  ? "this motor requires seven verified --thermal values\n"
                  : "invalid travel, speed or winding thermal configuration\n",
              stderr);
        return 2;
    }
    configuration.configuration_confirmed = argv[5][0] == '1';
    InterfaceParam record;
    Param_Upload(&record, &configuration);
    Param_Download(&record, sizeof(record), &configuration);
    if (configuration.preset != p || configuration.device.node_id != node ||
        configuration.device.encoder.reverse != 0U || record.encoder_reverse != 0U ||
        configuration.configuration_confirmed != (argv[5][0] == '1'))
    {
        fputs("record failed firmware readback\n", stderr);
        return 2;
    }
    FILE *out = fopen(argv[6], "wb");
    if (out == NULL)
    {
        return 1;
    }
    bool written = fwrite(&record, sizeof(record), 1, out) == 1;
    if (fclose(out) != 0 || !written)
    {
        return 1;
    }
    printf("node 0x%02X  %s  %s  %.0f:1  motor speed limit %.6g rad/s\n",
           (unsigned)node,
           MotorJointType_Name(p->joint),
           p->model_name,
           (double)p->gear_ratio,
           (double)configuration.application.speed_limit_rad_s);
    return 0;
}

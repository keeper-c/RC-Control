#include <main.h>
#define PI 3.14159265f
#define G 9.81f

#define HIGH 1.112f
#define TARGET_ANGLE_DEG -53

float deg2rad(float deg)
{
    return deg * PI / 180.0f;
}

uint8_t solve_angle_speed(float distance, float *out_angle_deg, float *out_speed)
{
    float target_slope = tan(deg2rad(TARGET_ANGLE_DEG)); // -√3 for -60°

    for (float a_deg = 75.0f; a_deg > 48.0f; a_deg -= 0.1f)
    {
        float a_rad = deg2rad(a_deg);
        float cos_a = cosf(a_rad);
        float sin_a = sinf(a_rad);

        // 构造二次方程的解：
        float numerator = G * distance * distance;
        float denominator = 2 * cos_a * cos_a * (distance * tanf(a_rad) - HIGH);

        if (denominator <= 0)
            continue;

        float v_squared = numerator / denominator;
        float v = sqrtf(v_squared);

        // 验证该角度下末速度是否为指定夹角
        float vx = v * cos_a;
        float t = distance / vx;
        float vy = v * sin_a - G * t;

        float actual_slope = vy / vx;
        float diff = fabsf(actual_slope - target_slope);

        if (diff < 0.01f)
        {
            *out_angle_deg = a_deg;
            *out_speed = v;
            return 1;
        }
    }
    return 0; // 没找到
}

void driver_ddd(float vx, float vy, float vw, float speed[4])
{
    speed[0] = vx - vy - vw; // 左上
    speed[1] = vx + vy - vw; // 右上
    speed[2] = vx + vy + vw; // 左下
    speed[3] = vx - vy + vw; // 右下
}

#define num 14 // 数据数量
const float px[num] = {
    0.5f, 1.04f, 1.61f, 1.95f, 2.42f,
    2.66f, 2.92f, 3.02f, 3.4f, 3.52f,
    3.6f, 3.86f, 4.6f, 6.0f};

const uint16_t py[num] = {
    3380, 3370, 3363, 3360, 3358,
    3350, 3345, 3340, 3320, 3305,
    3292, 3277, 3260, 3250};

// 分段函数：给定 x，返回对应的 y
float Forced_fitting(float x)
{
    uint8_t seg = 0;

    if (x <= px[0])
        seg = 0;
    else if (x >= px[num - 1])
        seg = num - 1;
    else
        for (uint8_t i = 0; i < num - 1; ++i)
            if (x >= px[i] && x <= px[i + 1])
            {
                seg = i;
                break;
            }
    float x1 = px[seg], x2 = px[seg + 1];
    float y1 = py[seg], y2 = py[seg + 1];

    return y1 + (y2 - y1) * (x - x1) / (x2 - x1);
}

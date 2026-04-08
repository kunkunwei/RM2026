#include "Simple_IIR3.h"

using namespace Component;

Simple_IIR3::Simple_IIR3(const float b_in[4], const float a_in[4]){
    for (int i = 0; i < 4; ++i) {
        b[i] = b_in[i];
        a[i] = a_in[i];
    }
    lastY = 0;
    normalizeA0();
    reset();
}

void Simple_IIR3::reset(float v)  {
    for (int i = 0; i < 3; ++i) z[i] = v;
}

float Simple_IIR3::processSample(float x)  {

    lastY = b[0] * x + z[0];              // y = b0*x + z0

    // 状态推进 (3 阶 => 3 个状态)
    // z0' = z1 + b1*x - a1*y
    z[0] = z[1]
           + b[1] * x
           - a[1] * lastY;

    // z1' = z2 + b2*x - a2*y
    z[1] = z[2]
           + b[2] * x
           - a[2] * lastY;

    // z2' = b3*x - a3*y
    z[2] = b[3] * x
           - a[3] * lastY;

    return lastY;
}

float Simple_IIR3::getOutput() {
    return lastY;
}

void Simple_IIR3::normalizeA0() {
    float a0 = a[0];
    if (a0 == 0.0f) {
        // 退化为直通
        b[0] = 1.0f; b[1] = b[2] = b[3] = 0.0f;
        a[0] = 1.0f; a[1] = a[2] = a[3] = 0.0f;
        return;
    }
    if (a0 != 1.0f) {
        for (int i = 0; i < 4; ++i) b[i] /= a0;
        for (int i = 1; i < 4; ++i) a[i] /= a0;
        a[0] = 1.0f;
    }
}



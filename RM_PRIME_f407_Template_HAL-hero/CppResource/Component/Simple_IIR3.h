#ifndef SIMPLE_IIR3_H
#define SIMPLE_IIR3_H

#include <cstddef>

namespace Component {

    class Simple_IIR3 {
    public:
        // 系数数量固定: 4 个前向(b0..b3) 与 4 个反馈(a0..a3)
        Simple_IIR3(const float b_in[4], const float a_in[4]);
        void reset(float v = 0.0f);
        float processSample(float x);
        float getOutput();
    private:
        void normalizeA0();
        float b[4]{1.0f,0.0f,0.0f,0.0f};
        float a[4]{1.0f,0.0f,0.0f,0.0f};
        float z[3]{0.0f,0.0f,0.0f};   // 状态寄存器
        float lastY;
    };

}

/*
用法示例:
#include "Simple_IIR3.h"
constexpr float b_lp[4] = {0.098531f, 0.295593f, 0.295593f, 0.098531f};
constexpr float a_lp[4] = {1.0f, -0.577240f, 0.421787f, -0.056297f};
Component::Simple_IIR3 filt;
filt.configure(b_lp, a_lp);
float y = filt.processSample(x);
*/
#endif // SIMPLE_IIR3_H

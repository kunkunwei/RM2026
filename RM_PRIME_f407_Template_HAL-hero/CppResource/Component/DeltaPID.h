#ifndef __DELTAPID_H
#define __DELTAPID_H

#include "main.h"


namespace Component{
    class DeltaPID{
    public:
        DeltaPID(float p_m , float i,float d,float p_t = 0.f,  float max = 10000.f, float min = -10000.f);
        float Run(float target,float measurement);
        void TmpUpdate(float target,float measurement);
        void Reset();

        float ClampOutput(float input);

        float DeltaOutput;
        float Output;

    private:
        float kpTarget;
        float kpMeasurement;
        float ki;
        float kd;

        float maxOut;
        float minOut;

        float lastMeasurement;
        float lastlastMeasurement;
        float lastTarget;

    };
}


#endif
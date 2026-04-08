#include "DeltaPID.h"


namespace Component{

    DeltaPID::DeltaPID(float p_m , float i,float d,float p_t, float max, float min):
    kpTarget(p_t),
    kpMeasurement(p_m),
    ki(i),
    kd(d),
    maxOut(max),
    minOut(min)
    {
        Output = 0;
        Reset();
    }

    float DeltaPID::Run(float target,float measurement){

        DeltaOutput =    kpTarget * (target - lastTarget) \
                        -kpMeasurement * (measurement - lastMeasurement) \
                        +ki * (target - measurement) \
                        -kd * (measurement + lastlastMeasurement - (2.f*lastMeasurement));

        TmpUpdate( target, measurement);

        Output = ClampOutput(Output + DeltaOutput);

        return DeltaOutput;
    }

    void DeltaPID::TmpUpdate(float target,float measurement){
        lastlastMeasurement = lastMeasurement;
        lastMeasurement = measurement;
        lastTarget = target;
    }

    void DeltaPID::Reset(){
        lastlastMeasurement = 0.f;
        lastMeasurement = 0.f;
        lastTarget = 0.f;
        DeltaOutput = 0.f;
    }

    float DeltaPID::ClampOutput(float input) {
        return input > maxOut ? maxOut : (input < minOut ? minOut : input);
    }
}

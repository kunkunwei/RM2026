#include "ShootTask.h"

namespace ShootFSM{

    bool isFribOpened = false;
    bool isZeroCross = false;
    bool lastb = 0;
    uint32_t FirbCounter[2] = {0,0};

    void FSM_Reset(){
        isFribOpened = false;
        isZeroCross = false;
        lastb = false;
        FirbCounter[0] = 0;
        FirbCounter[1] = 0;
    }

    bool getIsFribOpened(){
        return isFribOpened;
    }

    bool getIsZeroCross(){
        return isZeroCross;
    }

    void ShootFSM_Button_Loop(bool b){
        if(b && !lastb){
            isFribOpened = !isFribOpened;
            isZeroCross = false;
        }

        if(!b){
            isZeroCross = true;
        }

        lastb = b;
    }


    void ShootFSMLoop(short tin){

        if(isFribOpened){
            if(tin < -650){
                FirbCounter[1]++;
                if(FirbCounter[1] >= TOPEN){
                    isFribOpened = false;

                    FirbCounter[0] = 0;
                    FirbCounter[1] = 0;
                }
            } else{
                FirbCounter[1] = 0;
            }

        } else{
            if(tin > 580){
                FirbCounter[0]++;
                if(FirbCounter[0] >= TOPEN){
                    isFribOpened = true;
                    isZeroCross = false;

                    FirbCounter[0] = 0;
                    FirbCounter[1] = 0;
                }
            } else{
                FirbCounter[0] = 0;
            }
        }

        if(isZeroCross == false){
            if(tin > -50 && tin < 50){
                isZeroCross = true;
            }
        }

    }




}

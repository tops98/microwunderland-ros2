#include "servo_controller/Servomotor.hpp"
#include "servo_controller/PiPwmController.hpp"

#include <iostream>

using namespace std;

int main(int argc, char* argv[]){

    shared_ptr<PiPwmController> pwmContr = make_shared<PiPwmController>();
    Servomotor* motor = new Servomotor(pwmContr,26,540,2470,203,50);
    int angle = 0;

    while (1)
    {
        cout<<"Input Rotation angle in degree:"<<endl;
        cin>> angle;
        motor->setAngle(angle);
    }
    return 0;
}
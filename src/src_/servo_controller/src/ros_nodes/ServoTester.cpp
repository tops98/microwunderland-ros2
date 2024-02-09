#include <servo_controller/pwm_controller/Pca9586PwmController.hpp>
#include <servo_controller/pwm_controller/PiPwmController.hpp>
#include <servo_controller/servomotor/Servomotor.hpp>
#include <iostream>
#include <string.h>
#include <memory>

using namespace std;


#define HELP_TEXT \
"\
Command should be used as follows:\n\
servoTester [pwm channel] [min pulse (microsec)] [max pulse (microsec)] [actuationRange (degree)] [freq (hz)]  [pwm controller type: pi| pca] [i2c adress in hex if pca9586 is used]\
"
#define LOG_ERROR(msg) \
    { \
        cout<<(msg)<<endl<<HELP_TEXT<<endl; \
         return 0; \
    }
#define MAX(a,b) (((a)>(b))? (a):(b))
#define MIN(a,b) (((a)<(b))? (a):(b))


void moveServo(shared_ptr<Servomotor> servo, int range){
    int angle = 0;
    string input = "";
    while(1)
    {
        cout<<"Enter the desired angle:"<<endl;
        cin>>input;
        try{
            angle = atoi(input.c_str());
            
            if(angle > range || angle< 0) cout<<"invalid input"<<endl;
            else servo->setAngle(angle);

        }catch( const exception& ex){
            cout<<"invalid input"<<endl;
        }
    }
}


int main(int argc, char* argv[]){
    string pwmControllerType ="";
    int i2cAddress = 0;
    int pwmChannel = 0;

    int minPulse   = 0;
    int maxPulse   = 0;
    int range      = 0;
    int freq       = 0;

    shared_ptr<AbstractPwmController> pwmController;
    shared_ptr<Servomotor> servo;

    
    if(argc < 7) LOG_ERROR("Wrong number of arguments");

    // get pwm channel
    pwmChannel = atoi(argv[1]);
    // get pwm min pulse
    minPulse = atoi(argv[2]);
    // get pwm max pulse
    maxPulse = atoi(argv[3]);
    // get range
    range = atoi(argv[4]);
    // get freq
    freq = atoi(argv[5]);

    // get and create pwm controller
    pwmControllerType = argv[6];
    if(strcmp( pwmControllerType.c_str(), "pi") == 0){

        pwmController = make_shared<PiPwmController>();

    }else if(strcmp( pwmControllerType.c_str(), "pca") == 0){
        
        if(argc < 7) LOG_ERROR("Missing extra argument [i2c address]");
        
        try{
           i2cAddress = stoi(string(argv[7]), 0, 16);
        }catch( const exception& ex){
            LOG_ERROR("invalid i2c address format; address is expected in hex");
        }
        pwmController = make_shared<Pca9586PwmController>(i2cAddress);

    }else LOG_ERROR("unkown controller type");

    // create servo
    servo = make_shared<Servomotor>(pwmController, pwmChannel, minPulse, maxPulse, range, freq);
    
    // run loop
    moveServo(servo,range);

    return 0;
}
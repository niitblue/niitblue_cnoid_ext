/**
   Tank Controller
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/SimpleController>
#include <cnoid/SpotLight>
#include <cnoid/RangeCamera>
#include <cnoid/RangeSensor>
#include <cnoid/Joystick>

using namespace std;
using namespace cnoid;

class TankJoystickController : public SimpleController
{
    SimpleControllerIO* io;
    bool usePseudoContinousTrackMode;
    int flipperActuationMode;
    Link* trackL;
    Link* trackR;
    Link* trackLF;
    Link* trackRF;
    Link* trackLR;
    Link* trackRR;
    Link* flipperJoint[4];
    double qref[4];
    double qprev[4];
    double dt;

    struct DeviceInfo {
        DevicePtr device;
        int buttonId;
        bool prevButtonState;
        bool stateChanged;
        DeviceInfo(Device* device, int buttonId)
            : device(device),
              buttonId(buttonId),
              prevButtonState(false),
              stateChanged(false)
        { }
    };
    vector<DeviceInfo> devices;
    SpotLightPtr spotLight;
    
    Joystick joystick;

public:
    
    virtual bool initialize(SimpleControllerIO* io) override
    {
        this->io = io;
        ostream& os = io->os();
        Body* body = io->body();

        usePseudoContinousTrackMode = true;
        flipperActuationMode = Link::JointEffort;
        for(auto opt : io->options()){
            if(opt == "wheels"){
                usePseudoContinousTrackMode = false;
            }
            if(opt == "position"){
                flipperActuationMode = Link::JointDisplacement;
                os << "The joint-position command mode is used." << endl;
            }
            if(opt == "velocity"){
                flipperActuationMode = Link::JointVelocity;
                os << "The joint-velocity command mode is used." << endl;
            }
        }

        if(usePseudoContinousTrackMode){
            trackL = body->link("TRACK_L");
            trackR = body->link("TRACK_R");
            trackLF = body->link("TRACK_LF");
            trackRF = body->link("TRACK_RF");
            trackLR = body->link("TRACK_LR");
            trackRR = body->link("TRACK_RR");

        } else {
            trackL = body->link("WHEEL_L0");
            trackR = body->link("WHEEL_R0");
        }

        if(!trackL || !trackR){
            os << "The tracks are not found." << endl;
            return false;
        }

        io->enableOutput(trackL, JointVelocity);
        io->enableOutput(trackR, JointVelocity);
        io->enableOutput(trackLF, JointVelocity);
        io->enableOutput(trackRF, JointVelocity);
        io->enableOutput(trackLR, JointVelocity);
        io->enableOutput(trackRR, JointVelocity);

        flipperJoint[0] = body->link("SPACER_LF");
        flipperJoint[1] = body->link("SPACER_RF");
        flipperJoint[2] = body->link("SPACER_LR");
        flipperJoint[3] = body->link("SPACER_RR");
        for(int i=0; i < 4; ++i){
            Link* joint = flipperJoint[i];
            if(!joint){
                os << "Flipper joint " << i << " is not found." << endl;
                return false;
            }
            qref[i] = qprev[i] = joint->q();
            joint->setActuationMode(flipperActuationMode);
            io->enableIO(joint);
        }

        dt = io->timeStep();

        return true;
    }

    virtual bool control() override
    {
        joystick.readCurrentState();
        
        double pos[2];
        for(int i=0; i < 2; ++i){
            pos[i] = joystick.getPosition(
                i==0 ? Joystick::L_STICK_H_AXIS : Joystick::L_STICK_V_AXIS);
            if(fabs(pos[i]) < 0.2){
                pos[i] = 0.0;
            }
        }
        // set the velocity of each tracks
        if(usePseudoContinousTrackMode){
            double k = 1.0;
            trackL->dq_target() = k * (-2.0 * pos[1] + pos[0]);
	    trackLF->dq_target() = trackLR->dq_target() = trackL->dq_target();

            trackR->dq_target() = k * (-2.0 * pos[1] - pos[0]);
	    trackRF->dq_target() = trackRR->dq_target() = trackR->dq_target();
        } else {
            double k = 4.0;
            trackL->dq_target() = k * (-pos[1] + pos[0]);
            trackR->dq_target() = k * (-pos[1] - pos[0]);
        }

        static const double P = 200.0;
        static const double D = 50.0;

        for(int i=0; i < 4; ++i){
            Link* joint = flipperJoint[i];
            double pos = joystick.getPosition(Joystick::R_STICK_V_AXIS);
            if(fabs(pos) < 0.15){
                pos = 0.0;
            }

            if(flipperActuationMode == Link::JointDisplacement){
                if(i==0 || i==1){
                    joint->q_target() = joint->q() + pos * dt * -1.0;
		}
                if(i==2 || i==3){
                    joint->q_target() = joint->q() + pos * dt;
		}

            } else if(flipperActuationMode == Link::JointVelocity){
                joint->dq_target() = pos;

            } else if(flipperActuationMode == Link::JointEffort){
                double q = joint->q();
                double dq = (q - qprev[i]) / dt;
                double dqref = 0.0;
                double deltaq = 0.002 * pos;
                qref[i] += deltaq;
                dqref = deltaq / dt;
                joint->u() = P * (qref[i] - q) + D * (dqref - dq);
                qprev[i] = q;
            }
        }

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(TankJoystickController)

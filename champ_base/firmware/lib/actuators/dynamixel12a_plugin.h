#ifndef _DYNAMIXEL12A_PLUGIN_H_
#define _DYNAMIXEL12A_PLUGIN_H_

#include <DynamixelAX12.h>

namespace DynamixelAX12A
{
    class Plugin
    {
        OneWireMInterface ax12Interface_;
        DynamixelAX12 ax12_;

        int angle_offset_degrees_;
        float angle_offset_radians_;
        
        float min_angle_;
        float max_angle_;
        bool inverted_;
        int inverter_;
        int actuator_driver_id_;
        public:
            unsigned int leg_id;
            Plugin(HardwareSerial &serial_interface, unsigned int actuator_leg_id, unsigned int actuator_driver_id, float min_angle, float max_angle, bool inverted):
            ax12Interface_(serial_interface),
            ax12_(ax12Interface_, actuator_driver_id),
            angle_offset_degrees_(30),
            angle_offset_radians_(0.523599),            
            min_angle_(0),
            max_angle_(0),
            inverted_(false),
            inverter_(1),
            actuator_driver_id_(actuator_driver_id),
            leg_id(0)
            {
                leg_id = actuator_leg_id;
                min_angle_ = min_angle;
                max_angle_ = max_angle;
                inverted_ = inverted;
                if(inverted_)
                    inverter_ *= -1;

                initialize();
            }

            void initialize()
            {
                // OneWireStatus com_status;

                ax12Interface_.begin(1000000, 100);   
                ax12_.init();
                // com_status = ax12_.init();
                // if (com_status != OW_STATUS_OK)
                // {
                //     ax12_.changeActuatorID(1);
                //     ax12_.setActuatorID(actuator_driver_id_);
                // }
                
                ax12_.jointMode();
                ax12_.enableTorque();
            }

            void positionControl(float angle)
            {
                ax12_.goalPositionDegree(toActuatorAngle(angle * inverter_) - angle_offset_degrees_);
            }

            float getJointPosition()
            {
                uint16_t current_angle = 0;
                ax12_.currentPositionDegree(current_angle);
                return toEulerAngle(current_angle * inverter_) - angle_offset_radians_;
            }

            int toActuatorAngle(float angle)
            {
                float actuator_angle = 0;

                if(angle > 0)
                    actuator_angle = mapFloat(angle, 0, PI, 180, 360);

                else if(angle < 0)
                    actuator_angle = mapFloat(angle, -PI, 0, 0, 180);

                else  
                    actuator_angle = 180;
                
                return round(actuator_angle);
            }

            float toEulerAngle(float angle)
            {
                float actuator_angle = 0;

                if(angle > 0)
                    actuator_angle = mapFloat(angle, 180, 360, 0, PI);

                else if(angle < 0)
                    actuator_angle = mapFloat(angle, 0, 180, -PI, 0);

                else  
                    actuator_angle = 180;
                
                return actuator_angle;
            }

            float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
            {
                return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
            }
    };
}

#endif

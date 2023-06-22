#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"


class JoystickNode {

    private:

    ros::Publisher  vel_pub;
    ros::Subscriber joy_subscriber;
    ros::NodeHandle n;
    geometry_msgs::Twist vel_msg;
    
    float   axes[5];
    float  buttons[11];
    bool data_recived=false;
    float linear_speed=0.25,
         angular_speed=0.5;

    public:

    JoystickNode(){
        vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10); 
        joy_subscriber = n.subscribe("joy",10, &JoystickNode::joy_callback, this);
    }

    void joy_callback(const sensor_msgs::Joy  &msg){

        data_recived=true;

        for (int i = 0; i < sizeof(axes)/ sizeof(int); i++)
        {
            axes[i] = msg.axes[i];
        }
        
        for (int j = 0; j < sizeof(buttons)/ sizeof(int); j++)
        {
            buttons[j] = msg.buttons[j];
        }    

    }

    void direction_control(){

       if (data_recived)
        {  

            vel_msg.linear.x  =  axes[1]* linear_speed ;
            vel_msg.angular.z =  axes[0]* angular_speed; 
            
            vel_pub.publish(vel_msg);

        }
    }

    void speed_control(){

       if (data_recived)
        {  
            if (buttons[0]==1)
            {
                 linear_speed +=0.2;
            }

            else if (buttons[1]==1)
            {
                 linear_speed -=0.2;
            }   

            else if (buttons[3]==1)
            {
                 angular_speed +=0.2;
            }    
                      
            else if (buttons[2]==1)
            {
                 angular_speed -=0.2;
            }             
        
        vel_pub.publish(vel_msg);

        }
    }
          
    void print_data(){
        if (data_recived)
        {
            std::cout<< "axes_array: "<<std::endl;
            for (int k = 0; k < sizeof(axes)/ sizeof(int); k++)
            {
                std::cout<<" "<<axes[k]; 
            }

            std::cout<< "\n"<<std::endl;

            std::cout<< "bottons_array: "<<std::endl;

            for (int l = 0; l < sizeof(buttons)/ sizeof(int); l++)
            {
                std::cout<<" "<<buttons[l] ;
            } 
            
            std::cout<< "\n "<<std::endl;
         }
    }

};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "joystick_node");

    JoystickNode joystick;
    ros::Rate loop_rate(10);

    while (ros::ok())
    {

        joystick.speed_control();
        joystick.direction_control();
        
    //*****print data for debuging******
        // joystick.print_data();

        ros::spinOnce();
        loop_rate.sleep();
    }
    
}
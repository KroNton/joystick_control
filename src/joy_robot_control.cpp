#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"


class JoystickNode {

    private:

    ros::Publisher  vel_pub;
    ros::Subscriber joy_subscriber;
    
   
    geometry_msgs::Twist vel_msg;
    
    float   axes[5];
    float  buttons[11];

    bool data_recived=false;

    float linear_speed=0.25,
           angular_speed=0.5;

    std::string cmd_vel_topic="cmd_vel";

    int index1=1;
    int index2=0;
    int index3=0;
    int index4=1;
    int index5=3;
    int index6=2;

    public:

    JoystickNode(){
        ros::NodeHandle n;
        ros::NodeHandle nh("~");
        nh.getParam("cmd_vel_topic", cmd_vel_topic);
        nh.getParam("linear_direction_index", index1);
        nh.getParam("angular_direction_index", index2);
        nh.getParam("linear_speed_increase_index", index3);
        nh.getParam("linear_speed_decrease_index", index4);
        nh.getParam("angular_speed_increase_index", index5);
        nh.getParam("angular_speed_decrease_index", index6);

        vel_pub = n.advertise<geometry_msgs::Twist>(cmd_vel_topic, 10); 
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

            vel_msg.linear.x  =  axes[index1]* linear_speed ;
            vel_msg.angular.z =  axes[index2]* angular_speed; 
            
            vel_pub.publish(vel_msg);

        }
    }

    void speed_control(){

       if (data_recived)
        {  
            if (buttons[index3]==1)
            {
                 linear_speed +=0.2;
            }

            else if (buttons[index4]==1 && linear_speed>0.2)
            {
                 linear_speed -=0.2;
            }   

            else if (buttons[index5]==1)
            {
                 angular_speed +=0.2;
            }    
                      
            else if (buttons[index6]==1&& angular_speed> 0.2)
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
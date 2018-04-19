// Shamelessly based ona CWRU EECS class package's code (sine commander) and heavily modified, courtesy Dr. Wyatt Newman
// Last updated Mar 22 2017 by Trent Ziemer
// Frankensteined Apr 16 2018 by Jerray Dewa for roadPrintz 2 DOF arm

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <math.h>

ros::NodeHandle * nh_ptr;

int g_current_front_angle;
int g_current_rear_angle;
bool g_front_callback_received;
bool g_rear_callback_received;
std::string g_motor_angle_topic_name;

void frontAngleCB(const std_msgs::Int16& frontAngleHolder)
{
    g_current_front_angle = frontAngleHolder.data;
    g_front_callback_received = true;
    ROS_INFO("frontAngleCB called, received angle value of = %d", g_current_front_angle);
}

void rearAngleCB(const std_msgs::Int16& rearAngleHolder)
{
    g_current_rear_angle = rearAngleHolder.data;
    g_rear_callback_received = true;
    ROS_INFO("rearAngleCB called, received angle value of = %d", g_current_rear_angle);
}

// Waits for GLOBALLY DEFINED BOOLEAN to become true for a set number of seconds that are triggered by specific ros topics
// Quick and dirty ROS-based initialization
bool waitForSubs()
{
    ros::Subscriber front_angle_sub = nh_ptr->subscribe(g_motor_angle_topic_name, 1, frontAngleCB);
    ros::Subscriber rear_angle_sub = nh_ptr->subscribe("rear_wobbler/angle", 1, rearAngleCB);

    int count = 0;
    int time_to_wait = 50;
    ros::Rate count_rate(10);

    while(count < time_to_wait)
    {
        if((g_front_callback_received && g_rear_callback_received == true))
        {
            return true;
        }
        ros::spinOnce();
        count_rate.sleep();
        count++;
    }
    return false;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "two_dof_ang_controller"); 
    ros::NodeHandle nh; // Can make "...("~");" if needed.
    nh_ptr = &nh;

    g_motor_angle_topic_name = "front_wobbler/angle"; // CHANGE THIS TO "front_wobbler/angle" for use with final, physical system. Currently configured for an old ROSbag. dynamixel_motor1_ang
    
    ros::Publisher front_motor_pub = nh.advertise<std_msgs::Int16>("front_wobbler/cmd", 1);
    ros::Publisher rear_motor_pub = nh.advertise<std_msgs::Int16>("rear_wobbler/cmd", 1);

    g_front_callback_received = false;
    g_rear_callback_received = false;
    // The lowest and highest angles to go to

    double input_command_1_ang;
    double input_command_2_ang;
    double input_command_1_temp;
    double input_command_2_temp;
    int input_command_1;
    int input_command_2;

    double q1;
    double q2;
    double x;
    double y;

    
    std::cout << "Enter desired x displacement: ";
    std::cin >> x;
    std::cout << "Enter desired y displacement: ";
    std::cin >> y;
    
    if(y<0.0)
    {
        y = y-0.03;
    }

    q2 = acos((x*x + y*y - (0.3775)*(0.3775) - (0.33)*(0.33))/(2*0.3775*0.33)) * (180/M_PI);
    q1 = (atan(y/x) - atan((0.33*sin(q2*(M_PI/180)))/(0.3775+0.33*cos(q2*(M_PI/180))))) * (180/M_PI);

    ROS_INFO("q1: %f; q2: %f", q1,q2);  
    
    /*
    std::cout << "Enter desired angle for inner motor (don't forget about 3:1 gear ratio): ";
    std::cin >> input_command_1_ang;
    std::cout << "Enter desired angle for outer motor: ";
    std::cin >>input_command_2_ang;
    */

    input_command_1_ang = q1;
    input_command_2_ang = q2;
    
    input_command_1_ang = 180.00-q1;
    input_command_2_ang = 180.00-q2;

    //converts angle to dynamixel friendly command: 0-4096 range
    
    input_command_1_temp = 2048+(2048-(input_command_1_ang*11.3777777778))*0.3333333333;
    if(input_command_1_ang == 180)
    {
        input_command_1_temp = 2048;
    }
    input_command_1 = (int)input_command_1_temp;
    ROS_INFO("inner input_command is %d; inner input_command_ang is %f", input_command_1, input_command_1_ang);

    input_command_2_temp = (input_command_2_ang)*11.3777777778;
    input_command_2_temp = 2048+(2048-input_command_2_temp);
    input_command_2 = (int)input_command_2_temp;
    ROS_INFO("outer input_command is %d; outer input_command_ang is %f", input_command_2, input_command_2_ang);


    // THIS IS INITIAL ONE FOR HACKY INITIAL COMMAND SETTING. This is also checked in main program loop below
        // Check if controller parameters are available. If not, choose some defaults
    
    /*
    if(input_command_1<1012)
    {
        input_command_1 = 1012;
        ROS_WARN("WARNING: ENTERED POSITION GOES BEYOND PHYSICAL LIMITS; Setting outer motor position to 1012");
    }
    if(input_command_1>3036)
    {
        input_command_1 = 3036;
        ROS_WARN("WARNING: ENTERED POSITION GOES BEYOND PHYSICAL LIMITS; Setting outer motor position to 3036");
    }
    if(input_command_2<1012)
    {
        input_command_2 = 1012;
        ROS_WARN("WARNING: ENTERED POSITION GOES BEYOND PHYSICAL LIMITS; Setting inner motor position to 1012");
    }
    if(input_command_2>3036)
    {
        input_command_2 = 3036;
        ROS_WARN("WARNING: ENTERED POSITION GOES BEYOND PHYSICAL LIMITS; Setting inner motor position to 3036");
    }*/
    // This value goes "into" the ROS message object.
    short int front_command = g_current_front_angle;
    short int rear_command = g_current_rear_angle;

    //front command
    if(!waitForSubs())
    {
        ROS_WARN("WARNING: COULD NOT FIND WOBBLER ANGLE SUBSCRIPTION! Using a default value");
        front_command = 1000;
        rear_command = 1000;
    }
    else
    {
        if(g_current_front_angle > input_command_1 && g_current_rear_angle > input_command_2)
        {
            front_command = input_command_1;
            rear_command = input_command_2;
        }
        else if (g_current_front_angle > input_command_1 && g_current_rear_angle < input_command_2)
        {
            front_command = input_command_1;
            rear_command = input_command_2;
        }
        else if (g_current_front_angle < input_command_1 && g_current_rear_angle < input_command_2)
        {
            front_command = input_command_1;
            rear_command = input_command_2;
        }
        else if (g_current_front_angle < input_command_1 && g_current_rear_angle > input_command_2)
        {
            front_command = input_command_1;
            rear_command = input_command_2;
        }
        else
        {
            front_command = g_current_front_angle;
        }
    }
    // Let user know what initial value of the angle we are going to command
    ROS_INFO("front motor angle starting command is at %d", front_command);
    ROS_INFO("rear motor angle starting command is at %d", rear_command);

    // Controls whether the wobbler motors will increase or decrease in angle initially.
    bool front_increasing = false;
    bool rear_increasing = false;
    int loop_counter = 0;

    // How much to increase the angle command by each iteration. This controls the wobblers angular speed (rad/s) as a proxy variable (count/iteration)
    int change_ang;

    // ROS message object for the commanded angle
    std_msgs::Int16 int_front_angle; 
    std_msgs::Int16 int_rear_angle;

    // For timing the below main program 'while' loop
    double dt = 0.01;
    ros::Rate naptime(1/dt);

    // do work here in infinite loop (desired for this example)
    while (ros::ok()) 
    {
        
        // Only check params every X loops
        if (loop_counter % 10 == 0)
        {
            // Check if controller parameters are available. If not, choose some defaults
                change_ang = 1;   
        }

        loop_counter++;

        // Cast data to a "short int" so it will be compatible with ROS

        input_command_1 = (short int)input_command_1;
        input_command_2 = (short int)input_command_2;
        change_ang = (short int)change_ang;

        // Change our commanded angle as appropriate
        if(front_increasing == true)
        {
        	front_command = front_command + change_ang;
        }
        else
        {
            front_command = front_command - change_ang;
        }

        if(rear_increasing == true)
        {
            rear_command = rear_command + change_ang;
        }
        else
        {
            rear_command = rear_command - change_ang;
        }

        // If we reach the maximum or minimum angle, either stop or start increasing respectively as necessary
        if(front_command >= input_command_1)
        {
        	front_increasing = false;
        }
        else if(front_command <= input_command_1)
        {
        	front_increasing = true;
        }

        if(rear_command >= input_command_2)
        {
            rear_increasing = false;
        }
        else if(rear_command <= input_command_2)
        {
            rear_increasing = true;
        }
        // Load our data into ROS message
        int_front_angle.data = front_command;
        int_rear_angle.data = rear_command;
        //ROS_INFO("Sending angle command of %d to both motors.", command);

        // Move both motors by the same angle, for convenience and simplicity
        front_motor_pub.publish(int_front_angle); 
        rear_motor_pub.publish(int_rear_angle);
        naptime.sleep(); 
    }
}


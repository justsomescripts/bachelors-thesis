#include <chrono>

#include <rsv_cartesian_interface/rsv_cartesian_interface.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "rsv_cartesian_interface");

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::NodeHandle nh;

    rsv_cartesian_interface::RsvCartesianInterface hardware_interface;
    hardware_interface.init();

    // Set up timers
    ros::Time timestamp;
    ros::Duration period;
    auto stopwatch_last = std::chrono::steady_clock::now();
    auto stopwatch_now = stopwatch_last;

    controller_manager::ControllerManager controller_manager(&hardware_interface, nh);

    hardware_interface.start();
    sleep(2);

    

    
    while (ros::ok())
    {
        //Receive current state from robot
        hardware_interface.read(timestamp, period);

        //Get current time and elapsed time since last read
        timestamp = ros::Time::now();
        stopwatch_now = std::chrono::steady_clock::now();
        period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
        stopwatch_last = stopwatch_now;

        //sleep(1/10);

        //Update the controllers
        controller_manager.update(timestamp, period);

        //Send new setpoint to robot
        hardware_interface.write(timestamp, period);
        
        //mögliche sleep Funktion
        int sleep_time_milli = 200;
        usleep(sleep_time_milli*1000);
    }
    
    spinner.stop();
    ROS_INFO_STREAM_NAMED("rsv_cartesian_interface", "Shutting down.");

    return 0;
    
}
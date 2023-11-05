
#ifndef RSV_CARTESIAN_INTERFACE
#define RSV_CARTESIAN_INTERFACE
#include <vector>
#include <string>

//Fuer TCP IP Verbindung
#include <boost/asio.hpp>

//Wie bindet man diese Header Dateien ein?
#include <ros/console.h>
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>



namespace rsv_cartesian_interface
{
    class RsvCartesianInterface : public hardware_interface::RobotHW
    {
    private:
        //ROS node object
        ros::NodeHandle nh_;

        const unsigned int n_dof_ = 6;
        std::vector<std::string> joint_names_;
        std::vector<double> joint_position_;
        std::vector<double> joint_velocity_;
        std::vector<double> joint_effort_;
        std::vector<double> joint_position_command_;

        const double pi = 3.141592653589;

        // cartesian command -> x, y, z, tz_x, tz_y, tz_z, tx_x, tx_y, tx_z
        const unsigned int n_cart_elemts = 9;
        std::vector<double> cartesian_position_command_;
        std::vector<double> dh_para_theta_;
        std::vector<double> dh_para_d_;
        std::vector<double> dh_para_a_;
        std::vector<double> dh_para_alpha_;
        
        // rows x columns x number of joints
        double T[4][4][6] {0};
        
        // Sonstiges
        double pr_sum1 = 0;
        double pr_sum2 = 0;
        

        //sonstige Werte aus PACTPOS welche in PEXTPOS geschrieben werden
        std::string pactpos_res_command_head_values;
        std::string pactpos_res_command_tail_values;

        //RSV Variablen für eine TCP/IP-Verbindung
        std::string rsv_server_address_;
        std::string rsv_server_port_;
        //buffer length statisch vorgeben?


        //Timing Variablen (für ROS)
        ros::Duration control_period_;
        ros::Duration elapsed_time_;
        double loop_hz_;

        //Interfaces
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::PositionJointInterface position_joint_interface_;

        //RSV socket read/write
        //int rsv_read_state_timeout_ = 5;
        boost::asio::io_service ioservice_;
        //boost::asio::deadline_timer deadline_;
        boost::asio::ip::tcp::endpoint rsv_server_endpoint_;
        boost::asio::ip::tcp::socket rsv_server_socket_;
        //Kuka
        //void rsv_check_read_state_deadline();
        //void rsv_handle_connect(const boost::system::error_code &ec);
        //void rsv_handle_receive(const boost::system::error_code &ec, size_t length, boost::system::error_code* out_ec, size_t* out_length);        
        //Neu
        static void connectHandler(const boost::system::error_code &ec);
        static void readHandler(const boost::system::error_code &ec, size_t amountOfBytes);
        
        bool rsv_check_buffer();
        bool rsv_read_state(std::vector<double> &joint_position);
        bool rsv_write_command(const std::vector<double> &cartesian_position);
        bool forward_kinematic_solver(std::vector<double> &joint_position);

    public:
    
        RsvCartesianInterface();
        ~RsvCartesianInterface();

        void init();
        void start();
        void read(const ros::Time &time, const ros::Duration &period);
        void write(const ros::Time &time, const ros::Duration &period);



    };
}   // namespace rsv_cartesian_interface

#endif //RSV_CARTESIAN_INTERFACE (Terminierung)










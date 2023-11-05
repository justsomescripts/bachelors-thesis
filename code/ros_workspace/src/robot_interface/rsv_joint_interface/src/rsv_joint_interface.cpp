#include <rsv_joint_interface/rsv_joint_interface.h>
#include <angles/angles.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <chrono>
#include <tinyxml.h> 
#include <bitset>

#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>




namespace rsv_joint_interface
{
    //RsvHardwareInterface::RsvHardwareInterface() : joint_position_(n_dof_, 0.0), joint_velocity_(n_dof_, 0.0), joint_position_command_(n_dof_, 0.0), joint_names_(n_dof_) {}

    RsvJointInterface::RsvJointInterface() : joint_position_(n_dof_, 0.0), joint_velocity_(n_dof_, 0.0), joint_effort_(n_dof_, 0.0), joint_position_command_(n_dof_, 0.0), joint_names_(n_dof_), rsv_server_socket_(ioservice_, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), 0))
    {

    }

    RsvJointInterface::~RsvJointInterface() {}


    //---------------------------------------------------------------------------------------------Kuka
    //Ueberwachung ob es zu einer Zeitüberschreitung kommt...
    /*void RsvHardwareInterface::rsv_check_read_state_deadline()
    {
        // Check if deadline has already passed
        if (deadline_.expires_at() <= boost::asio::deadline_timer::traits_type::now())
        {
            rsv_server_socket_.cancel();
            deadline_.expires_at(boost::posix_time::pos_infin);
        }

        // Sleep until deadline exceeded
        deadline_.async_wait(boost::bind(&RsvHardwareInterface::rsv_check_read_state_deadline, this));
    }*/
    //Handling von empfangenen Nachrichten

    //---------------------------------------------------------------------------------------------Kuka
    /*void RsvHardwareInterface::rsv_handle_receive(const boost::system::error_code &ec, size_t length, boost::system::error_code* out_ec, size_t* out_length){
        //Neu
        if (ec)
        {
            std::string msg = "Read failed with msg: '" + ec.message() + "'";
            ROS_ERROR_STREAM(msg);
        }

        //Kuka
        *out_ec = ec;
        *out_length = length;
    }*/

    //---------------------------------------------------------------------------------------------Neu
    void RsvJointInterface::connectHandler(const boost::system::error_code &ec){
        if (ec)
        {
            std::string msg = "Connection failed with errorcode: " + ec.message(); 
            ROS_ERROR_STREAM(msg);
            throw std::runtime_error(msg);
        }
        else{
            ROS_INFO_STREAM_NAMED("rsv_joint_interface", "Connection established!");
        }
        return;
    }
    
    /*void RsvHardwareInterface::readHandler(const boost::system::error_code &ec, size_t amountOfBytes){
        //---------------------------------------------------------------------wird nicht ausgeführt
        if (ec)
        {
            std::string msg = "Starting session and initializing Symbol-Table failed with msg: " + ec.message(); 
            ROS_ERROR_STREAM(msg);
        }
        else{
            ROS_INFO_NAMED("rsv_hw_interface", "Session with RSV-Webserver started and Symbol-Table is initialized");
            //std::cout.write(rsvres_buffer.data(), amountOfBytes);

        }
        ROS_INFO_NAMED("rsv_hw_interface", "Session with RSV-Webserver started and Symbol-Table is initialized");
        return;
    }*/
    //-------------------------------------------------------------------------braucht man die beiden Handler? wahrscheinlich nicht

    void RsvJointInterface::init()
    {
        //Get controller joint names from parameter server
        if (!nh_.getParam("controller_joint_names", joint_names_))
        {
            ROS_ERROR("Cannot find required parameter 'controller_joint_names' on the parameter server.");
            throw std::runtime_error("Cannot find required parameter 'controller_joint_names' on the parameter server.");
        }

        //Get RSV Parameters from parameter server
        const std::string param_addr = "rsv/robot_address";
        const std::string param_port = "rsv/robot_port";
        //const std::string param_socket_timeout = "rsv/socket_timeout";
        //const std::string param_max_cmd_buf_len = "rsv/max_cmd_buf_len";


        if (nh_.getParam(param_addr, rsv_server_address_) && nh_.getParam(param_port, rsv_server_port_))
        {
            ROS_INFO_STREAM_NAMED("rsv_joint_interface", "Configuring RSV joint interface on: "
                                << rsv_server_address_ << ", " << rsv_server_port_);
        }
        else
        {
            std::string msg = "Failed to get RSV address/port from parameter server (looking for '" + param_addr +
                            "', '" + param_port + "')";
            ROS_ERROR_STREAM(msg);
            throw std::runtime_error(msg);
        }
        /*
        if (nh_.getParam(param_socket_timeout, rsv_read_state_timeout_))
        {
            ROS_INFO_STREAM_NAMED("rsv_hw_interface", "Configuring RSV hardware interface socket timeout to "
                                << rsv_read_state_timeout_ << " seconds");
        }
        else
        {
            ROS_INFO_STREAM_NAMED("rsv_hw_interface", "Failed to get RSV socket timeout from parameter server (looking "
                                  "for '" + param_socket_timeout + "'), defaulting to " +
                                  std::to_string(rsv_read_state_timeout_)  + " seconds");
        }*/
        
        //Abfrage der Timeout-Dauer und Bufferlaenge (Für RSV nötig?)...

        //Erstellen der ros_control interfaces fuer joint state und position joint (fuer die einzelnen resourcen)
        for (std::size_t i = 0; i < n_dof_; ++i)
        {
            //-----------------------------------------------------------------------------------------nachfolgend funktioniert noch nicht:
            //Compiler will velocity und effort, beim ausführen kann das handle nicht erstellt werden, weil velocity pointer null ist
            // Joint state interface
            joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));

            // Joint position control interface
            position_joint_interface_.registerHandle(
                hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[i]), &joint_position_command_[i]));
        }

        // Register interfaces
        registerInterface(&joint_state_interface_);
        registerInterface(&position_joint_interface_);

        ROS_INFO_STREAM_NAMED("rsv_joint_interface", "Loaded RSV joint interface");

            //init als erstes Testen
        return;
    }

    void RsvJointInterface::start()
    {


        ROS_INFO_NAMED("rsv_joint_interface", "Starting RSV joint interface...");
        //Verbindung zum RSV-Server aufbauen
        //Start Client
        ROS_INFO_NAMED("rsv_joint_interface", "... connecting to robot's RSV-web-server...");
            //Tutorial 1
        boost::asio::ip::tcp::resolver resolver(ioservice_);
        rsv_server_endpoint_ = *resolver.resolve({boost::asio::ip::tcp::v4(), rsv_server_address_, rsv_server_port_});
        
            //Tutorial 2
        //rsv_server_socket_.connect(tcp::endpoint(boost::asio::ip::address::from_string(rsv_server_address_), 80));

        //Neu
        //Verbindung aufbauen
        //Tutorial 1
        rsv_server_socket_.async_connect(rsv_server_endpoint_, connectHandler);
        
        //rsv_server_socket_.connect(rsv_server_endpoint_, connectHandler);
        
        
        
        //Session mit RSV-Webserver starten
        const std::string startSession = "SYMTABLE_SESSION / \n";
        //std::string startSession("RSVCMD_SESSION / \n");
        //Tutorial 1
        boost::asio::write(rsv_server_socket_, boost::asio::buffer(startSession));

        //Tutorial 2
        //boost::system::error_code ec;
        //boost::asio::write(rsv_server_socket_, boost::asio::buffer(startSession), ec);
        //sleep(5); //5 Sekunden warten

        //Symboltabelle initialisieren
        ROS_INFO_NAMED("rsv_joint_interface", "... initializing symbol table...");
        const std::string initSymbolTable = "<RSVCMD><clientStamp>123</clientStamp><symbolApi><initSymbolTable/></symbolApi></RSVCMD>";

        //Send
               
        boost::asio::write(rsv_server_socket_, boost::asio::buffer(initSymbolTable));
        
        //Receive
        //-----------------------------Tutorial Sync_Client
        //https://www.boost.org/doc/libs/1_63_0/doc/html/boost_asio/example/cpp03/http/client/sync_client.cpp
        boost::asio::streambuf rsvres_buffer;
        boost::asio::read_until(rsv_server_socket_, rsvres_buffer, "</RSVRES>");
        //https://stackoverflow.com/questions/1899750/how-do-i-convert-a-boostasiostreambuf-into-a-stdstring/25507426#25507426?newreg=4659617fa933454aa9391e4da3957d6c
        std::string rsvres_buffer_str( (std::istreambuf_iterator<char>(&rsvres_buffer)), std::istreambuf_iterator<char>() );

        ROS_INFO_STREAM_NAMED("rsv_joint_interface", "RSV responds: " << rsvres_buffer_str);







        //Nach Kuka
        /*boost::array<char, 19> start_session_buf = { SYMTABLE_SESSION /  };
        rsv_server_socket_.send_to(boost::asio::buffer(start_session_buf), rsv_server_endpoint_);  // Aktivierung des RSVCMD-Server Modus
        ROS_INFO_NAMED("rsv_hw_interface", "... initializing robot's symbol table ...");
        boost::array<char, 19> init_symtable_buf = { SYMTABLE_SESSION /  };
        rsv_server_socket_.send_to(boost::asio::buffer(init_symtable_buf), rsv_server_endpoint_);
        // Start persistent actor to check for eki_read_state timeouts (braucht man das?)
        deadline_.expires_at(boost::posix_time::pos_infin);  // do nothing unit a read is invoked (deadline_ = +inf)
        eki_check_read_state_deadline();*/

        //Startwerte abrufen
        /*if (!rsv_read_state(joint_position_))
        {   
            
            std::string msg = "Failed to read from robot RSV server within alloted time of "
                            + std::to_string(rsv_read_state_timeout_) + " seconds.  Make sure eki_hw_interface is running "
                            "on the robot controller and all configurations are correct.";
            ROS_ERROR_STREAM(msg);
            throw std::runtime_error(msg);
            
           //--> Moegliche Fehlermeldungen
        }*/

        //Neu
        //Startwerte auslesen
        rsv_read_state(joint_position_);
        //variable muss gecleart werden, da sonst die neuen Werte nur angehängt werden
        pactpos_res_command_head_values.clear();
        
        //Startwerte in Variablen eintragen
        joint_position_command_ = joint_position_;


        ROS_INFO_NAMED("rsv_hw_interface", "... done. RSV hardware interface started!");
        ROS_INFO_STREAM_NAMED("rsv_hw_interface", "");
        ROS_INFO_STREAM_NAMED("rsv_hw_interface", "");

        return;
    }

    
    //Aktuellen Status auslesen (in diesem Fall sind nur die Positionen wichtig)
    //bool Rv6lHardwareInterface::rv6l_read_state(std::vector<double> &joint_position, std::vector<double> &joint_velocity, std::vector<double> &joint_effort, int &cmd_buff_len)
    bool RsvJointInterface::rsv_read_state(std::vector<double> &joint_position)
    {
        //XML Message
        const std::string read_pactpos_cmd = "<RSVCMD><clientStamp>123</clientStamp><symbolApi><readSymbolValue><name>_PACTPOS</name></readSymbolValue></symbolApi></RSVCMD>";
        boost::asio::streambuf rsvres_pactpos;
        
        //Request PACTPOS
        boost::asio::write(rsv_server_socket_, boost::asio::buffer(read_pactpos_cmd));
        
        //Receive PACTPOS
        //-----------------------------Tutorial Sync_Client
        boost::asio::read_until(rsv_server_socket_, rsvres_pactpos, "</RSVRES>");
        
        std::string rsvres_pactpos_str( (std::istreambuf_iterator<char>(&rsvres_pactpos)), std::istreambuf_iterator<char>() );

        ROS_INFO_STREAM_NAMED("rsv_hw_interface", "RSV responds PACTPOS: " << rsvres_pactpos_str);

        
        std::vector<double> pactpos_joint_position(n_dof_);
        
        //Variables for parsing the values
        std::string single_char;
        std::string double_sign;
        std::string double_value_str_neg = "-0.000000000000000e+000";
        std::string double_value_str_pos = "0.000000000000000e+000";
        //counting the single values
        int double_index = 1;
        
        //go through each character
        for(int i = 0 ; i < rsvres_pactpos_str.length() ; i++){
            single_char = rsvres_pactpos_str.at(i);
            
            //find a double by searching for a point
            if (single_char == ".")
            {
                
                std::string double_index_str = std::to_string(double_index);
                double_sign = rsvres_pactpos_str.at(i-2);
                
                //determine the sign and extract the double value with "for"-loop
                if(double_sign == "-")
                {
                    //negative value
                    for(int u = 0 ; u < 23 ; u++)
                    {
                        double_value_str_neg.at(u) = rsvres_pactpos_str.at(u+i-2);
                    }
                    //ROS_INFO_STREAM_NAMED("rsv_hw_interface", "Negativer double Wert gefunden " << double_value_str_neg);
                    
                    if (double_index >= 10 && double_index <= 15)
                    {
                        //save JOINT value in pactpos_joint_array
                        //parse string into a double
                        std::stringstream double_value_ss(double_value_str_neg);
                        double_value_ss >> pactpos_joint_position[double_index-10];
                    }
                    
                }
                else
                {   
                    //positive value
                    for(int u = 0 ; u < 22 ; u++)
                    {
                        double_value_str_pos.at(u) = rsvres_pactpos_str.at(u+i-1);
                    }
                    //ROS_INFO_STREAM_NAMED("rsv_hw_interface", "Positiver double Wert gefunden " << double_value_str_pos);
                    
                    if (double_index >= 10 && double_index <= 15)
                    {
                        //save joint value in pactpos_joint_array
                        //parse string into a double
                        std::stringstream double_value_ss(double_value_str_pos);
                        double_value_ss >> pactpos_joint_position[double_index-10];
                    }
                } 

                //Command head values auslesen
                //Soll alle Werte zwischen Command head und den Gelenkwerten enthalten
                if (double_index == 9)
                {
                    std::string pactpos_res_command_head = "<RSVRES><clientStamp>123</clientStamp><symbolApi><readSymbolValue><name>_PACTPOS</name><value>";
                    
                    //Bereich um den Wert mit dem Index 9 ausschneiden
                    for (int t = pactpos_res_command_head.length() ; t < i + 21; t++)
                    {
                        //pactpos_res_command_head_char = rsvres_pactpos_str.at(t);
                        pactpos_res_command_head_values += rsvres_pactpos_str.at(t);
                    }
                    ROS_INFO_STREAM_NAMED("rsv_hw_interface", "");
                    ROS_INFO_STREAM_NAMED("rsv_hw_interface", "Head values: " << pactpos_res_command_head_values);
                    ROS_INFO_STREAM_NAMED("rsv_hw_interface", "");
                }
                
                double_index++;
            }
        }

        // Offset Korrektur (nicht mehr nötig)
        //pactpos_joint_position[0] = pactpos_joint_position[0] * (-1);
        //pactpos_joint_position[3] = pactpos_joint_position[3] + 90;

        //Testoutput the joint values and update the joint states
        for (int i = 0; i < n_dof_; i++)
        {
            std::string testoutput_joint_states = std::to_string(pactpos_joint_position[i]);
            ROS_INFO_STREAM_NAMED("rsv_hw_interface", "Testoutput der ausgelesenen Jointstates: " << testoutput_joint_states);
            
            joint_position[i] = angles::from_degrees(pactpos_joint_position[i]); //convert deg to rad
        }

        //clear all strings and buffer for reuse (mögliche Fehlerquelle)
        rsvres_pactpos_str.clear();
        rsvres_pactpos.consume(rsvres_pactpos.size() + 1);
        return true;
    }

    bool RsvJointInterface::rsv_write_command(const std::vector<double> &joint_position_command)
    {
        //Hier wird der Command an die Steuerung gesendet
        //Anfang des Commands
        std::string write_pextpos_cmd = "<RSVCMD><clientStamp>123</clientStamp><symbolApi><writeSymbolValue><name>_PEXTPOS</name><value>";
        boost::asio::streambuf rsvres_pextpos;
        //Ausgelesene Werte einfügen
        write_pextpos_cmd += pactpos_res_command_head_values;
        //variable muss gecleart werden, da sonst die neuen Werte nur angehängt werden
        pactpos_res_command_head_values.clear();


        for (int i = 0; i < n_dof_; i++)
        {   
            //Berücksichtung der Abstände je nach Vorzeichen
            if(joint_position_command[i] < 0)
            {
                write_pextpos_cmd += " ";
            }
            else
            {
                write_pextpos_cmd += "  ";
            }
            write_pextpos_cmd += std::to_string(angles::to_degrees(joint_position_command[i]));
        }
        

        write_pextpos_cmd += "  0.000000000000000e+000  0.000000000000000e+000  0.000000000000000e+000  0.000000000000000e+000  0.000000000000000e+000  0.000000000000000e+000  0.000000000000000e+000  0.000000000000000e+000  0.000000000000000e+000  0.000000000000000e+000  0.000000000000000e+000  0.000000000000000e+000  0.000000000000000e+000  0.000000000000000e+000  0.000000000000000e+000  0.000000000000000e+000  0.000000000000000e+000  0.000000000000000e+000 14  0.000000000000000e+000  0.000000000000000e+000  0.000000000000000e+000  0.000000000000000e+000  0.000000000000000e+000  0.000000000000000e+000  0.000000000000000e+000  0.000000000000000e+000  0.000000000000000e+000  0.000000000000000e+000  0.000000000000000e+000  0.000000000000000e+000 $BASE</value></writeSymbolValue></symbolApi></RSVCMD>";
        ROS_INFO_STREAM_NAMED("rsv_hw_interface", "");
        ROS_INFO_STREAM_NAMED("rsv_hw_interface", "PEXTPOS wird gesendet: " << write_pextpos_cmd);
        ROS_INFO_STREAM_NAMED("rsv_hw_interface", "");
        ROS_INFO_STREAM_NAMED("rsv_hw_interface", "");

                
        //send PEXTPOS command
        boost::asio::write(rsv_server_socket_, boost::asio::buffer(write_pextpos_cmd));
        write_pextpos_cmd.clear();
        boost::asio::read_until(rsv_server_socket_, rsvres_pextpos, "</RSVRES>");

        return true;
    }
    
    bool RsvJointInterface::rsv_check_buffer()
    {
        char iextmode_buffer_overflow;
        
        //Diese Funktion überprüft vor dem Versenden der nächsten Sollvorgabe/PEXTPOS
        //ob der Buffer Aufnahmebereit ist

        //do
        //{
            //XML Message
            const std::string rsvcmd_read_iextmode = "<RSVCMD><clientStamp>123</clientStamp><symbolApi><readSymbolValue><name>_IEXTMODE</name></readSymbolValue></symbolApi></RSVCMD>";
            boost::asio::streambuf rsvres_iextmode;
            TiXmlDocument iextmode_xml;
            std::string iextmode_value_str;
            int iextmode_value_int;
            
            //Request IEXTMODE
            boost::asio::write(rsv_server_socket_, boost::asio::buffer(rsvcmd_read_iextmode));
            
            //Receive IEXTMODE
            //-----------------------------Tutorial Sync_Client
            boost::asio::read_until(rsv_server_socket_, rsvres_iextmode, "</RSVRES>");
            
            std::string rsvres_iextmode_str( (std::istreambuf_iterator<char>(&rsvres_iextmode)), std::istreambuf_iterator<char>() );

            //Parse XML-Document
            //https://stackoverflow.com/questions/2862892/can-tinyxml-load-xml-from-string-instead-of-file
            //https://stackoverflow.com/questions/16810485/cannot-convert-stdstring-to-const-char
            //https://stackoverflow.com/questions/2862892/can-tinyxml-load-xml-from-string-instead-of-file

            iextmode_xml.Parse(rsvres_iextmode_str.c_str(), 0, TIXML_ENCODING_UTF8);
            TiXmlElement *RootElement = iextmode_xml.RootElement();
            if( NULL != RootElement )
            {
                TiXmlElement *SymbolApi = RootElement->FirstChildElement( "symbolApi" );
                if( NULL != SymbolApi)
                {
                    TiXmlElement *ReadSymbolValue = SymbolApi->FirstChildElement( "readSymbolValue" );
                    if ( NULL != ReadSymbolValue )
                    {
                        TiXmlElement *Value = ReadSymbolValue->FirstChildElement( "value" );
                        if ( NULL != Value)
                        {
                            iextmode_value_str = Value->GetText();
                        }
                        
                    }
                    
                }
                
            }


            //Convert from dec to bin
            //String to int
            iextmode_value_int = std::stoi(iextmode_value_str);
            //int to bin string
            std::string iextmode_value_str_bin = std::bitset<32>(iextmode_value_int).to_string();
            //get control bit 2
            std::size_t n_bit_2 = 29;
            iextmode_buffer_overflow = iextmode_value_str_bin.at(n_bit_2);


            ROS_INFO_STREAM_NAMED("rsv_hw_interface", "");
            ROS_INFO_STREAM_NAMED("rsv_hw_interface", "RSV responds IEXTMODE: " << iextmode_value_str_bin);
            ROS_INFO_STREAM_NAMED("rsv_hw_interface", "RSV responds IEXTMODE: " << iextmode_buffer_overflow);
            ROS_INFO_STREAM_NAMED("rsv_hw_interface", "");

        //}while (iextmode_buffer_overflow == '1');

        if (iextmode_buffer_overflow == '1')
        {
            return false;
        }
        else{
            return true;
        }
        
    }


    void RsvJointInterface::read(const ros::Time &time, const ros::Duration &period)
    {
        //Werte auslesen
        rsv_read_state(joint_position_);
    }

    void RsvJointInterface::write(const ros::Time &time, const ros::Duration &period)
    {
        //Buffer überprüfen
        //rsv_check_buffer();
        //Werte als Command senden
        if (rsv_check_buffer()==true)
        {
            rsv_write_command(joint_position_command_);
        }
        
        
    }


}
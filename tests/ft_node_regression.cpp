// Exercise the actual F/T node with a localhost sensor and delayed robot pose.
#define main ft_production_main
#include "../Y2FT_AQ/src/FTGetMain.cpp"
#undef main
#include <cassert>
#include <array>

int main(int argc, char** argv) {
    assert(std::string(std::getenv("ROS_DOMAIN_ID") ? std::getenv("ROS_DOMAIN_ID") : "") == "215");
    rclcpp::init(argc,argv);
    int udp=socket(AF_INET,SOCK_DGRAM,0);assert(udp>=0);
    sockaddr_in addr{};addr.sin_family=AF_INET;addr.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    assert(bind(udp,reinterpret_cast<sockaddr*>(&addr),sizeof(addr))==0);
    socklen_t len=sizeof(addr);getsockname(udp,reinterpret_cast<sockaddr*>(&addr),&len);
    auto sensor=std::make_shared<FTGetMain>("127.0.0.1",ntohs(addr.sin_port),"/audit",2);
    sockaddr_in peer{};len=sizeof(peer);char request[32];
    assert(recvfrom(udp,request,sizeof(request),0,reinterpret_cast<sockaddr*>(&peer),&len)>0);
    auto probe=std::make_shared<rclcpp::Node>("ft_node_regression");
    auto poses=probe->create_publisher<std_msgs::msg::Float64MultiArray>("/audit/currentP",10);
    size_t count=0;geometry_msgs::msg::Wrench latest;
    auto sub=probe->create_subscription<geometry_msgs::msg::WrenchStamped>("/audit/ftdata",10,
        [&](const geometry_msgs::msg::WrenchStamped& m){++count;latest=m.wrench;});
    rclcpp::executors::SingleThreadedExecutor exec;exec.add_node(sensor);exec.add_node(probe);
    auto run=[&](double seconds,double theta,bool publish_pose,bool publish_sensor){
        auto end=std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);
        while(std::chrono::steady_clock::now()<end){
            if(publish_pose){std_msgs::msg::Float64MultiArray p;p.data={0,0,0,theta,0,0};poses->publish(p);}
            if(publish_sensor){
                const float fy=1.6*9.81*std::sin(theta), fz=1.6*9.81*std::cos(theta);
                std::array<float,6> raw={0,fy,fz,float(.149303*fy),0,0};std::array<uint32_t,6> bytes{};
                for(size_t i=0;i<6;++i){std::memcpy(&bytes[i],&raw[i],4);bytes[i]=htonl(bytes[i]);}
                assert(sendto(udp,bytes.data(),sizeof(bytes),0,reinterpret_cast<sockaddr*>(&peer),len)==24);
            }
            exec.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    };
    run(.3,M_PI/2,false,true);assert(count==0); // Wait for robot orientation.
    run(7.,M_PI/2,true,true);assert(count>0);
    assert(std::abs(latest.force.x)+std::abs(latest.force.y)+std::abs(latest.force.z)<.001);
    run(.4,0,true,true); // Change pose after tare: pure gravity must still cancel.
    assert(std::abs(latest.force.x)+std::abs(latest.force.y)+std::abs(latest.force.z)<.001);
    assert(std::abs(latest.torque.x)+std::abs(latest.torque.y)+std::abs(latest.torque.z)<.001);
    run(.1,0,true,false);const auto stopped_count=count;
    run(.2,0,true,false);assert(count==stopped_count);
    close(udp);exec.remove_node(sensor);exec.remove_node(probe);sensor.reset();probe.reset();rclcpp::shutdown();
    std::cout << "PASS: delayed pose, nonidentity tare, gravity force/torque cancellation, no stale publication\n";
}

#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp" 

using namespace std::chrono_literals;
using namespace std;

class BoatMover : public rclcpp::Node {
public:
  BoatMover() : Node("boat_mover_node"), count_(0) {
    // Publisher ke topik Override RC
    publisher_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>("/mavros/rc/override", 10);
    
    // Timer 20Hz (50ms) - Wajib agar Pixhawk tidak Failsafe
    timer_ = this->create_wall_timer(
      50ms, bind(&BoatMover::timer_callback, this));
      
    RCLCPP_INFO(this->get_logger(), "SIAP! Skenario 10 Detik. Yaw & Lateral sudah dibalik (Inverted).");
  }

private:
  void timer_callback() {
    auto msg = mavros_msgs::msg::OverrideRCIn();

    // 1. Reset semua channel ke IGNORE (65535)
    for(int i=0; i<18; i++) msg.channels[i] = 65535; 

    // 2. Netralkan Channel Utama (Wajib 1500)
    msg.channels[0] = 1500; // Pitch
    msg.channels[1] = 1500; // Roll
    msg.channels[2] = 1500; // Throttle
    msg.channels[3] = 1500; // Yaw
    msg.channels[4] = 1500; // Forward
    msg.channels[5] = 1500; // Lateral

    double t = count_ * 0.05; // Hitung waktu berjalan

    // --- SKENARIO PERGERAKAN (DURASI 10 DETIK) ---

    // 0. STANDBY (0 - 3 Detik)
    if (t <= 3.0) {
       if (count_ % 20 == 0) RCLCPP_INFO(this->get_logger(), "Time %.1f: STANDBY...", t);
    } 
    
    // 1. MAJU (3 - 13 Detik) -> Channel 4
    else if (t <= 13.0) {
      msg.channels[4] = 1700; 
      if (count_ % 20 == 0) RCLCPP_INFO(this->get_logger(), "Time %.1f: MAJU...", t);
    }
    
    // 2. YAW KANAN (13 - 23 Detik) -> Channel 3 (PWM 1300 - Inverted)
    else if (t <= 23.0) {
      msg.channels[3] = 1300; 
      if (count_ % 20 == 0) RCLCPP_INFO(this->get_logger(), "Time %.1f: YAW KANAN (Putar)...", t);
    }

    // 3. YAW KIRI (23 - 33 Detik) -> Channel 3 (PWM 1700 - Inverted)
    else if (t <= 33.0) {
      msg.channels[3] = 1700; 
      if (count_ % 20 == 0) RCLCPP_INFO(this->get_logger(), "Time %.1f: YAW KIRI (Putar)...", t);
    }
    
    // 4. MUNCUL / NAIK (33 - 43 Detik) -> Channel 2
    else if (t <= 43.0) {
      msg.channels[2] = 1700; 
      if (count_ % 20 == 0) RCLCPP_INFO(this->get_logger(), "Time %.1f: MUNCUL (Naik)...", t);
    }
    
    // 5. MENYELAM (43 - 53 Detik) -> Channel 2
    else if (t <= 53.0) {
      msg.channels[2] = 1300; 
      if (count_ % 20 == 0) RCLCPP_INFO(this->get_logger(), "Time %.1f: MENYELAM (Turun)...", t);
    }

    // 6. LATERAL KANAN (53 - 63 Detik) -> Channel 5 (PWM 1300 - Inverted)
    else if (t <= 63.0) {
      msg.channels[5] = 1300; 
      if (count_ % 20 == 0) RCLCPP_INFO(this->get_logger(), "Time %.1f: LATERAL KANAN (Geser)...", t);
    }

    // 7. LATERAL KIRI (63 - 73 Detik) -> Channel 5 (PWM 1700 - Inverted)
    else if (t <= 73.0) {
      msg.channels[5] = 1700; 
      if (count_ % 20 == 0) RCLCPP_INFO(this->get_logger(), "Time %.1f: LATERAL KIRI (Geser)...", t);
    }

    // 8. MUNDUR (73 - 83 Detik) -> Channel 4
    else if (t <= 83.0) {
      msg.channels[4] = 1300; 
      if (count_ % 20 == 0) RCLCPP_INFO(this->get_logger(), "Time %.1f: MUNDUR...", t);
    }

    // SELESAI
    else {
      if (count_ % 20 == 0) RCLCPP_INFO(this->get_logger(), "Time %.1f: SELESAI. Stop.", t);
    }

    publisher_->publish(msg);
    count_++;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr publisher_;
  int count_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<BoatMover>());
  rclcpp::shutdown();
  return 0;
}
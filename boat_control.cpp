#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"
#include "mavros_msgs/srv/command_long.hpp" 

using namespace std::chrono_literals;
using namespace std;

class BoatMover : public rclcpp::Node {
public:
  BoatMover() : Node("boat_mover_node"), count_(0) {
    // KITA KEMBALI KE JALUR RESMI: MAVROS RC OVERRIDE
    // Ini satu-satunya jalur yang DIZINKAN Pixhawk untuk memutar motor!
    rc_publisher_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>("/mavros/rc/override", 10);
    
    // Timer 20Hz (50ms) agar Pixhawk tidak Failsafe
    timer_ = this->create_wall_timer(50ms, bind(&BoatMover::timer_callback, this));
      
    RCLCPP_INFO(this->get_logger(), "==========================================================");
    RCLCPP_INFO(this->get_logger(), "🚀 GCS MALANG MBOIS: TEST YAW KANAN DOANG!");
    RCLCPP_INFO(this->get_logger(), "==========================================================");
  }

private:
  void timer_callback() {
    auto msg = mavros_msgs::msg::OverrideRCIn();

    // 1. Matikan channel yang tidak terpakai
    for(int i=0; i<18; i++) msg.channels[i] = 65535; 

    // 2. Netralkan semua sumbu utama (1500 = Diam)
    msg.channels[0] = 1500; // Pitch
    msg.channels[1] = 1500; // Roll
    msg.channels[2] = 1500; // Vertical (Naik/Turun)
    msg.channels[3] = 1500; // Yaw (Putar) -> KITA PAKAI INI
    msg.channels[4] = 1500; // Forward (MAJU/MUNDUR) 
    msg.channels[5] = 1500; // Lateral (Geser Kanan/Kiri)

    double t = count_ * 0.05; // Hitung waktu (detik)

    // --- SKENARIO: MURNI YAW KANAN FULL SPEED ---

    // 0 - 3 Detik: STANDBY
    if (t <= 3.0) {
       if (count_ % 20 == 0) RCLCPP_INFO(this->get_logger(), "[%.1fs] STANDBY...", t);
    } 
    
    // 3 - 13 Detik: MAJU LURUS
    else if (t <= 13.0) {
      msg.channels[4] = 1100; // Perintah: DORONG KAPAL MAJU (NILAI DIBALIK JADI 1100)
      if (count_ % 20 == 0) RCLCPP_INFO(this->get_logger(), "[%.1fs] GAS MAJU! (PWM 1100 pada Channel 4/Forward)", t);
    }
    // 13 - 23 Detik: MUNDUR LURUS 
    else if (t <= 23.0) {
      msg.channels[4] = 1900; // Perintah: DORONG KAPAL MUNDUR (NILAI DIBALIK JADI 1900)
      if (count_ % 20 == 0) RCLCPP_INFO(this->get_logger(), "[%.1fs] GAS MUNDUR! (PWM 1900 pada Channel 4/Forward)", t);
    }
    // 23 - 33 Detik: YAW KIRI FULL SPEED
    else if (t <= 33.0) {
      msg.channels[3] = 1900; // Perintah: PUTAR KIRI FULL SPEED (NILAI DIBALIK JADI 1900)
      if (count_ % 20 == 0) RCLCPP_INFO(this->get_logger(), "[%.1fs] PUTAR KIRI FULL SPEED! (PWM 1900 pada Channel 3/Yaw)", t);
    }
    // 33 - 43 Detik: YAW KANAN FULL SPEED
    else if (t <= 43.0) {
      msg.channels[3] = 1100; // Perintah: PUTAR KANAN FULL SPEED (NILAI DIBALIK JADI 1100)
      if (count_ % 20 == 0) RCLCPP_INFO(this->get_logger(), "[%.1fs] PUTAR KANAN FULL SPEED! (PWM 1100 pada Channel 3/Yaw)", t);
    }
    // 43 - 53 Detik: LATERAL KIRI FULL SPEED
    else if (t <= 53.0) {
      msg.channels[5] = 1900; // Perintah: GESER KAPAL KIRI FULL SPEED (NILAI DIBALIK JADI 1900)
      if (count_ % 20 == 0) RCLCPP_INFO(this->get_logger(), "[%.1fs] GESER KIRI FULL SPEED! (PWM 1900 pada Channel 5/Lateral)", t);
    }
    // 53 - 63 Detik: LATERAL KANAN FULL SPEED
    else if (t <= 63.0) {
      msg.channels[5] = 1100; // Perintah: GESER KAPAL KANAN FULL SPEED (NILAI DIBALIK JADI 1100)
      if (count_ % 20 == 0) RCLCPP_INFO(this->get_logger(), "[%.1fs] GESER KANAN FULL SPEED! (PWM 1100 pada Channel 5/Lateral)", t);
    }
     // 63 - 73 Detik: VERTICAL NAIK FULL SPEED
    else if (t <= 73.0) {
      msg.channels[2] = 1900; // Perintah: DORONG KAPAL NAIK FULL SPEED (NILAI DIBALIK JADI 1100)
      if (count_ % 20 == 0) RCLCPP_INFO(this->get_logger(), "[%.1fs] NAIK FULL SPEED! (PWM 1900 pada Channel 2/Vertical)", t);
    }
     // 73 - 83 Detik: VERTICAL TURUN FULL SPEED
    else if (t <= 83.0) {
      msg.channels[2] = 1100; // Perintah: DORONG KAPAL TURUN FULL SPEED (NILAI DIBALIK JADI 1900)
      if (count_ % 20 == 0) RCLCPP_INFO(this->get_logger(), "[%.1fs] TURUN FULL SPEED! (PWM 1100 pada Channel 2/Vertical)", t);
    }

    
    // > 83 Detik: SELESAI
    else {
      if (count_ % 20 == 0) RCLCPP_INFO(this->get_logger(), "[%.1fs] SELESAI. Semua mesin berhenti.", t);
    }

    // Publish input pergerakan
    rc_publisher_->publish(msg);
    count_++;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_publisher_;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr servo_client_;
  int count_;
  int current_state_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<BoatMover>());
  rclcpp::shutdown();
  return 0;
}

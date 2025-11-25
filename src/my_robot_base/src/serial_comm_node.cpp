#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <serial_driver/serial_driver.hpp>
#include <io_context/io_context.hpp>
#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <limits>

// 使用 drivers::serial_driver 命名空间
namespace drivers
{
namespace serial_driver
{

// -------- 配置宏：这里定义需要发送的字段与顺序（修改这里即可增删/改名/重排） --------
// 发送浮点字段（会依次映射到 TxPayload::chf[0..]）
#ifndef TX_CHF_FIELD_LIST
#define TX_CHF_FIELD_LIST \
    X(angle_target) \
    X(speed_target) \
    X(torque_target)
#endif

// 发送 int8 字段（会依次映射到 TxPayload::chb[0..]）
#ifndef TX_CHB_FIELD_LIST
#define TX_CHB_FIELD_LIST \
    X(mode) \
    X(control_mode)
#endif

// -------- 接收字段同理：这里定义要读取并发布为 key=value 的字段与顺序 --------
// 接收浮点字段（依次从 RxPayload::chf[0..] 取值）
#ifndef RX_CHF_FIELD_LIST
#define RX_CHF_FIELD_LIST \
    X(Angle)   \
    X(Speed)    \
    X(Torque_Motor) \
    X(Torque_Sensor) \
    X(Temperature) \
    X(Resilience)
#endif

// 接收 int16 字段（依次从 RxPayload::chs[0..] 取值）
#ifndef RX_CHS_FIELD_LIST
#define RX_CHS_FIELD_LIST 
    // X(foot_pressure)
#endif

// 接收 int8 字段（依次从 RxPayload::chb[0..] 取值）
#ifndef RX_CHB_FIELD_LIST
#define RX_CHB_FIELD_LIST \
    X(mode_now) \
    X(control_mode_now) \
    X(macro_phase_now)
#endif

class SerialCommNode : public rclcpp::Node
{
public:
    SerialCommNode() : Node("serial_comm_node")
    {
        // 1. 声明并获取参数
        this->declare_parameter<std::string>("port", "/dev/ttyUSB1");
        this->declare_parameter<int>("baudrate", 115200);
        this->declare_parameter<double>("tx_rate_hz", 50.0);
        this->declare_parameter<bool>("publish_raw_feedback", true);
        // 映射参数
        load_channel_mapping_params();
        
        std::string port_name = this->get_parameter("port").as_string();
        auto baud_rate = static_cast<uint32_t>(this->get_parameter("baudrate").as_int());
        double tx_rate_hz = this->get_parameter("tx_rate_hz").as_double();
        publish_raw_feedback_ = this->get_parameter("publish_raw_feedback").as_bool();
        auto flow_control = drivers::serial_driver::FlowControl::NONE;
        auto parity = drivers::serial_driver::Parity::NONE;
        auto stop_bits = drivers::serial_driver::StopBits::ONE;

        // 2. 实例化并配置串口驱动
        RCLCPP_INFO(this->get_logger(), "Connecting to serial port: %s at baudrate: %u", port_name.c_str(), baud_rate);
        
        try {
            // 创建并保存 IoContext，确保其生命周期覆盖整个节点
            m_io_context = std::make_unique<drivers::common::IoContext>();
            // 创建SerialPortConfig对象
            m_port_config = std::make_unique<SerialPortConfig>(baud_rate, flow_control, parity, stop_bits);
            // 创建SerialDriver对象（注意需要传入 IoContext 引用）
            m_driver = std::make_unique<SerialDriver>(*m_io_context);
            // 设置配置并打开串口
            m_driver->init_port(port_name, *m_port_config);
            m_driver->port()->open();
        } catch (const std::exception & e) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open serial port %s: %s", port_name.c_str(), e.what());
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Serial port opened successfully.");

        // 3. 订阅聚合分类话题（mode, macro, phase_idx）
        classification_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "motion_classification", 10,
            [this](const std_msgs::msg::Int32MultiArray::SharedPtr arr){
                // 将数组按 TX_CHB_FIELD_LIST 顺序写入 latest_ 对应字段
                size_t i = 0;
                #define X(name) \
                  if (i < arr->data.size()) { latest_.name = static_cast<decltype(latest_.name)>(arr->data[i]); } ++i;
                TX_CHB_FIELD_LIST
                #undef X
            }
        );

        // 3.1 订阅数值控制量（直接映射到 TX_CHF_FIELD_LIST 中的浮点通道）
        angle_target_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "angle_target", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg){
                latest_.angle_target = msg->data;  // 单位：度（与协议约定一致）
            }
        );
        speed_target_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "speed_target", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg){
                latest_.speed_target = msg->data;  // 单位：度/秒（或与协议一致）
            }
        );
        torque_target_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "torque_target", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg){
                latest_.torque_target = msg->data;  // 单位：牛·米（或与协议一致）
            }
        );

        // 可选：如果上层直接发布模式（更清晰的数值话题）
        mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "mode", 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg){
                latest_.mode = static_cast<uint8_t>(std::clamp(msg->data, 0, 255));
            }
        );
        control_mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "control_mode", 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg){
                latest_.control_mode = static_cast<uint8_t>(std::clamp(msg->data, 0, 255));
            }
        );

        // 4. 创建发布者，用于发布从下位机接收到的数据
        // 话题名: /serial_feedback, 消息类型: std_msgs::msg::String
        feedback_pub_ = this->create_publisher<std_msgs::msg::String>("serial_feedback", 10);
        // 可选：发布原始十六进制数据，便于调试
        if (publish_raw_feedback_) {
            feedback_raw_pub_ = this->create_publisher<std_msgs::msg::String>("serial_feedback_raw", 10);
        }

        // 5. 启动异步接收，收到数据后在回调中发布
        if (m_driver && m_driver->port()) {
            m_driver->port()->async_receive(
                [this](std::vector<uint8_t> & buffer, const size_t & bytes_transferred) {
                    if (bytes_transferred == 0 || buffer.empty()) {
                        return;
                    }
                    // 调试：发布原始 hex 数据
                    if (publish_raw_feedback_ && feedback_raw_pub_) {
                        static const char * hex = "0123456789ABCDEF";
                        std::string dump;
                        dump.reserve(bytes_transferred * 3);
                        for (size_t i = 0; i < bytes_transferred; ++i) {
                            uint8_t b = buffer[i];
                            dump.push_back(hex[b >> 4]);
                            dump.push_back(hex[b & 0x0F]);
                            if (i + 1 < bytes_transferred) dump.push_back(' ');
                        }
                        auto raw_msg = std::make_unique<std_msgs::msg::String>();
                        raw_msg->data = std::move(dump);
                        feedback_raw_pub_->publish(std::move(raw_msg));
                    }
                    // 将数据放入环形缓冲，并尝试解析帧
                    rx_buffer_.insert(rx_buffer_.end(), buffer.begin(), buffer.begin() + bytes_transferred);
                    this->parse_rx_buffer();
                }
            );
        }

        // 5. 定时发送二进制帧（打包最新控制量）
        if (tx_rate_hz > 0.0) {
            using namespace std::chrono_literals;
            auto period = std::chrono::duration<double>(1.0 / tx_rate_hz);
            tx_timer_ = this->create_wall_timer(
                std::chrono::duration_cast<std::chrono::milliseconds>(period),
                std::bind(&SerialCommNode::send_tx_frame, this));
        }
    }

    ~SerialCommNode()
    {
        if (m_driver && m_driver->port() && m_driver->port()->is_open()) {
            m_driver->port()->close();
        }
    }

private:
    // ---------------- 可配置通道映射（通过参数修改索引即可重映射） ----------------
    struct TxChannelMap {
        // 使用宏自动生成每个字段的索引（参数名 tx_chf_<name>_idx / tx_chb_<name>_idx）
        #define X(name) int chf_##name = -1;
        TX_CHF_FIELD_LIST
        #undef X
        #define X(name) int chb_##name = -1;
        TX_CHB_FIELD_LIST
        #undef X
    } tx_map_;

    struct RxChannelMap {
        // 使用宏自动生成接收字段映射（参数名 rx_chf_<name>_idx / rx_chs_<name>_idx）
        #define X(name) int chf_##name = -1;
        RX_CHF_FIELD_LIST
        #undef X
        #define X(name) int chs_##name = -1;
        RX_CHS_FIELD_LIST
        #undef X
        #define X(name) int chb_##name = -1;
        RX_CHB_FIELD_LIST
        #undef X
    } rx_map_;
    // ---------------- 协议与封包定义（固定负载：10个float + 5个int16 + 3个int8） ----------------
    // 帧结构: [0xAA][payload...][0xBB]
    // 端序: 小端（MCU默认小端，Jetson小端）

    #pragma pack(push, 1)
    struct RxPayload {            // STM32 → Jetson
        float  chf[10];           // float channels
        int16_t chs[5];           // int16 channels
        int8_t  chb[3];           // int8 channels
    };

    struct TxPayload {            // Jetson → STM32
        float  chf[10];
        int16_t chs[5];
        int8_t  chb[3];
    };
    #pragma pack(pop)

    static constexpr uint8_t kHeader = 0xAA;
    static constexpr uint8_t kTail   = 0xBB;

    struct LatestControl {
        // 浮点控制量（与 TX_CHF_FIELD_LIST 对应）
        #define X(name) float name{0.0f};
        TX_CHF_FIELD_LIST
        #undef X
        // 分类字节（与 TX_CHB_FIELD_LIST 对应）
        #define X(name) uint8_t name{0};
        TX_CHB_FIELD_LIST
        #undef X
    } latest_;

    // 工具：安全写入带边界检查
    template<typename TArray, typename T>
    void safe_store_float(TArray &arr, int idx, T value, int maxn)
    {
        if (idx >= 0 && idx < maxn) arr[idx] = static_cast<float>(value);
    }
    template<typename TArray, typename T>
    void safe_store_int16(TArray &arr, int idx, T value, int maxn)
    {
        if (idx >= 0 && idx < maxn) arr[idx] = static_cast<int16_t>(value);
    }
    template<typename TArray, typename T>
    void safe_store_int8(TArray &arr, int idx, T value, int maxn)
    {
        if (idx >= 0 && idx < maxn) arr[idx] = static_cast<int8_t>(value);
    }

    // 已移除 ASCII 直通路径；请通过数值话题设置控制量

    // ---------------- 接收解析 ----------------
    void parse_rx_buffer()
    {
        const size_t payload_size = sizeof(RxPayload);
        while (rx_buffer_.size() >= (1 + payload_size + 1)) {
            // 对齐到头
            if (rx_buffer_[0] != kHeader) {
                auto it = std::find(rx_buffer_.begin()+1, rx_buffer_.end(), kHeader);
                if (it == rx_buffer_.end()) {
                    rx_buffer_.clear();
                    return;
                }
                rx_buffer_.erase(rx_buffer_.begin(), it);
                if (rx_buffer_.size() < (1 + payload_size + 1)) return;
            }
            // 期待尾部在固定位置
            if (rx_buffer_[1 + payload_size] != kTail) {
                // 错位，丢弃一个字节并继续
                rx_buffer_.erase(rx_buffer_.begin());
                continue;
            }
            // 复制负载
            RxPayload p{};
            std::memcpy(&p, rx_buffer_.data() + 1, payload_size);
            handle_rx_payload(p);
            // 移除已消费
            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + 1 + payload_size + 1);
        }
    }

    void handle_rx_payload(const RxPayload &p)
    {
        // 以 key=value 输出到 /serial_feedback（字段、顺序、命名由 RX_*_FIELD_LIST 决定）
        std::string out;
        char buf[96];
        bool first = true;
        // 浮点字段
        #define X(name) \
        { \
            float v = (rx_map_.chf_##name >= 0 && rx_map_.chf_##name < 10) ? p.chf[rx_map_.chf_##name] : 0.0f; \
            std::snprintf(buf, sizeof(buf), #name "=%.3f", (double)v); \
            if (!first) out += ", "; else first = false; \
            out += buf; \
        }
        RX_CHF_FIELD_LIST
        #undef X
        // int16 字段
        #define X(name) \
        { \
            int v = (rx_map_.chs_##name >= 0 && rx_map_.chs_##name < 5) ? (int)p.chs[rx_map_.chs_##name] : 0; \
            std::snprintf(buf, sizeof(buf), #name "=%d", v); \
            if (!first) out += ", "; else first = false; \
            out += buf; \
        }
        RX_CHS_FIELD_LIST
        #undef X
        // int8 字段
        #define X(name) \
        { \
            int v = (rx_map_.chb_##name >= 0 && rx_map_.chb_##name < 3) ? (int)p.chb[rx_map_.chb_##name] : 0; \
            std::snprintf(buf, sizeof(buf), #name "=%d", v); \
            if (!first) out += ", "; else first = false; \
            out += buf; \
        }
        RX_CHB_FIELD_LIST
        #undef X

        auto msg = std::make_unique<std_msgs::msg::String>();
        msg->data = std::move(out);
        feedback_pub_->publish(std::move(msg));
    }

    // ---------------- 发送打包 ----------------
    void send_tx_frame()
    {
        if (!(m_driver && m_driver->port())) return;
        TxPayload p{};
        // 清零
        for (int i=0;i<10;++i) p.chf[i] = 0;
        for (int i=0;i<5;++i)  p.chs[i] = 0;
        for (int i=0;i<3;++i)  p.chb[i] = 0;
    // 使用可配置映射写入（由宏统一生成）
    #define X(name) safe_store_float(p.chf, tx_map_.chf_##name, latest_.name, 10);
    TX_CHF_FIELD_LIST
    #undef X
    #define X(name) safe_store_int8  (p.chb, tx_map_.chb_##name, latest_.name, 3);
    TX_CHB_FIELD_LIST
    #undef X

        const size_t payload_size = sizeof(TxPayload);
        std::vector<uint8_t> frame;
        frame.resize(1 + payload_size + 1);
        frame[0] = kHeader;
        std::memcpy(frame.data() + 1, &p, payload_size);
        frame[1 + payload_size] = kTail;

        try {
            m_driver->port()->send(frame);
        } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send TX frame: %s", e.what());
        }
    }

    // 串口驱动和配置
    std::unique_ptr<drivers::common::IoContext> m_io_context;
    std::unique_ptr<drivers::serial_driver::SerialPortConfig> m_port_config;
    std::unique_ptr<drivers::serial_driver::SerialDriver> m_driver;

    // ROS 2 订阅者、发布者和定时器
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr classification_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr control_mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_target_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_target_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr torque_target_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_raw_pub_;
    rclcpp::TimerBase::SharedPtr read_timer_;
    rclcpp::TimerBase::SharedPtr tx_timer_;

    // 状态
    std::vector<uint8_t> rx_buffer_;
    bool publish_raw_feedback_{true};

    // ---------------- 参数读取辅助 ----------------
    void load_channel_mapping_params()
    {
        // Tx 映射：按照列表顺序给出默认索引（可通过参数覆盖）
        {
            int idx = 0;
            #define X(name) tx_map_.chf_##name = this->declare_parameter<int>("tx_chf_" #name "_idx", idx++);
            TX_CHF_FIELD_LIST
            #undef X
        }
        {
            int idx = 0;
            #define X(name) tx_map_.chb_##name = this->declare_parameter<int>("tx_chb_" #name "_idx", idx++);
            TX_CHB_FIELD_LIST
            #undef X
        }

        // Rx 映射：按照列表顺序给出默认索引（可通过参数覆盖）
        {
            int idx = 0;
            #define X(name) rx_map_.chf_##name = this->declare_parameter<int>("rx_chf_" #name "_idx", idx++);
            RX_CHF_FIELD_LIST
            #undef X
        }
        {
            int idx = 0;
            #define X(name) rx_map_.chs_##name = this->declare_parameter<int>("rx_chs_" #name "_idx", idx++);
            RX_CHS_FIELD_LIST
            #undef X
        }
        {
            int idx = 0;
            #define X(name) rx_map_.chb_##name = this->declare_parameter<int>("rx_chb_" #name "_idx", idx++);
            RX_CHB_FIELD_LIST
            #undef X
        }
    }
};

}  // namespace serial_driver
}  // namespace drivers

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<drivers::serial_driver::SerialCommNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
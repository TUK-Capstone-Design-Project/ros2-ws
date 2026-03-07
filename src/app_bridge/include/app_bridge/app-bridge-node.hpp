#pragma once

#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>

#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <nlohmann/json.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace app_bridge
{

namespace beast     = boost::beast;
namespace websocket = beast::websocket;
using boost::asio::ip::tcp;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using json           = nlohmann::json;

// ── 개별 WebSocket 세션 ────────────────────────────────
class WsSession : public std::enable_shared_from_this<WsSession>
{
public:
    using Ptr = std::shared_ptr<WsSession>;

    explicit WsSession(tcp::socket socket);

    /// WebSocket 핸드셰이크 후 읽기 루프 시작
    void run(std::function<void(const std::string &, Ptr)> on_message, std::function<void(Ptr)> on_close);

    void send(const std::string &data);

    std::string remote_addr() const;

private:
    void on_accept(beast::error_code ec);
    void do_read();
    void on_read(beast::error_code ec, std::size_t bytes_transferred);

    websocket::stream<beast::tcp_stream>          ws_;
    beast::flat_buffer                            read_buf_;
    std::string                                   remote_addr_;
    std::function<void(const std::string &, Ptr)> on_message_;
    std::function<void(Ptr)>                      on_close_;
};

// ── 메인 ROS2 노드 ────────────────────────────────────
class AppBridgeNode : public rclcpp::Node
{
public:
    explicit AppBridgeNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~AppBridgeNode() override;

private:
    // WebSocket 서버
    void start_accept();
    void on_client_message(const std::string &text, WsSession::Ptr session);
    void on_client_close(WsSession::Ptr session);
    void broadcast(const json &msg);

    // Nav2 액션
    void send_nav_goal(double x, double y, double theta);
    void on_goal_response(
        rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr goal_handle
    );
    void on_goal_result(
        const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &result
    );

    // 로봇 위치 주기 전송
    void publish_robot_pose();

    // ROS2 구성요소
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    std::shared_ptr<tf2_ros::Buffer>                 tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>      tf_listener_;
    rclcpp::TimerBase::SharedPtr                     pose_timer_;

    // Boost.Asio 구성요소
    boost::asio::io_context io_ctx_;
    tcp::acceptor           acceptor_;
    std::thread             io_thread_;

    // 클라이언트 관리
    std::mutex               clients_mutex_;
    std::set<WsSession::Ptr> clients_;

    // 파라미터
    int    port_;
    double pose_publish_hz_;
};

} // namespace app_bridge

#include "app_bridge/app-bridge-node.hpp"

#include <cmath>
#include <tf2/utils.h>

namespace app_bridge
{

// ════════════════════════════════════════════════════════
//  WsSession 구현
// ════════════════════════════════════════════════════════

WsSession::WsSession(tcp::socket socket) : ws_(std::move(socket))
{
    // 핸드셰이크 전에 주소를 저장 (나중에 소켓이 닫히면 접근 불가)
    try {
        auto ep      = beast::get_lowest_layer(ws_).socket().remote_endpoint();
        remote_addr_ = ep.address().to_string() + ":" + std::to_string(ep.port());
    } catch (...) {
        remote_addr_ = "unknown";
    }
}

void WsSession::run(
    std::function<void(const std::string &, Ptr)> on_message,
    std::function<void(Ptr)>                      on_close
)
{
    on_message_ = std::move(on_message);
    on_close_   = std::move(on_close);

    // 텍스트 모드 (JSON)
    ws_.text(true);

    // 비동기 WebSocket 핸드셰이크 수락
    ws_.async_accept(
        beast::bind_front_handler(&WsSession::on_accept, shared_from_this())
    );
}

void WsSession::on_accept(beast::error_code ec)
{
    if (ec) {
        on_close_(shared_from_this());
        return;
    }
    do_read();
}

void WsSession::send(const std::string &data)
{
    auto self = shared_from_this();
    auto buf  = std::make_shared<std::string>(data);

    // post를 통해 strand에서 쓰기 (동시 쓰기 방지)
    boost::asio::post(
        beast::get_lowest_layer(ws_).socket().get_executor(),
        [this, self, buf]() {
            beast::error_code ec;
            ws_.write(boost::asio::buffer(*buf), ec);
            // 실패 시 다음 read에서 감지
        }
    );
}

std::string WsSession::remote_addr() const
{
    return remote_addr_;
}

void WsSession::do_read()
{
    read_buf_.clear();
    ws_.async_read(
        read_buf_,
        beast::bind_front_handler(&WsSession::on_read, shared_from_this())
    );
}

void WsSession::on_read(beast::error_code ec, std::size_t /*bytes_transferred*/)
{
    if (ec) {
        on_close_(shared_from_this());
        return;
    }

    std::string text = beast::buffers_to_string(read_buf_.data());
    if (!text.empty()) {
        on_message_(text, shared_from_this());
    }

    do_read();
}

// ════════════════════════════════════════════════════════
//  AppBridgeNode 구현
// ════════════════════════════════════════════════════════

AppBridgeNode::AppBridgeNode(const rclcpp::NodeOptions &options) : Node("app_bridge_node", options),
                                                                   acceptor_(io_ctx_)
{
    // 파라미터 선언
    this->declare_parameter("port", 5000);
    this->declare_parameter("pose_publish_hz", 2.0);
    port_            = this->get_parameter("port").as_int();
    pose_publish_hz_ = this->get_parameter("pose_publish_hz").as_double();

    // Nav2 액션 클라이언트
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // TF 리스너
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 로봇 위치 주기적 전송 타이머
    auto period = std::chrono::milliseconds(
        static_cast<int>(1000.0 / pose_publish_hz_)
    );
    pose_timer_ = this->create_wall_timer(
        period, std::bind(&AppBridgeNode::publish_robot_pose, this)
    );

    // TCP Acceptor 설정
    tcp::endpoint ep(tcp::v4(), static_cast<unsigned short>(port_));
    acceptor_.open(ep.protocol());
    acceptor_.set_option(tcp::acceptor::reuse_address(true));
    acceptor_.bind(ep);
    acceptor_.listen(boost::asio::socket_base::max_listen_connections);
    start_accept();

    // io_context를 별도 스레드에서 구동
    io_thread_ = std::thread([this]() { io_ctx_.run(); });

    RCLCPP_INFO(this->get_logger(), "App Bridge started — WebSocket port %d, pose @ %.1f Hz", port_, pose_publish_hz_);
}

AppBridgeNode::~AppBridgeNode()
{
    io_ctx_.stop();
    if (io_thread_.joinable()) {
        io_thread_.join();
    }
}

// ── WebSocket 서버 ─────────────────────────────────────

void AppBridgeNode::start_accept()
{
    acceptor_.async_accept(
        [this](boost::system::error_code ec, tcp::socket socket) {
            if (!ec) {
                auto session = std::make_shared<WsSession>(std::move(socket));
                RCLCPP_INFO(this->get_logger(), "Android connected: %s", session->remote_addr().c_str());

                {
                    std::lock_guard<std::mutex> lock(clients_mutex_);
                    clients_.insert(session);
                }

                session->run(
                    [this](const std::string &text, WsSession::Ptr s) {
                        on_client_message(text, s);
                    },
                    [this](WsSession::Ptr s) {
                        on_client_close(s);
                    }
                );
            }
            start_accept();
        }
    );
}

void AppBridgeNode::on_client_message(
    const std::string &text, WsSession::Ptr /*session*/
)
{
    try {
        auto msg  = json::parse(text);
        auto type = msg.value("type", "");

        if (type == "navigate") {
            double x     = msg.at("x").get<double>();
            double y     = msg.at("y").get<double>();
            double theta = msg.value("theta", 0.0);
            send_nav_goal(x, y, theta);
        } else if (type == "cancel") {
            nav_client_->async_cancel_all_goals();
            RCLCPP_INFO(this->get_logger(), "Navigation cancelled by app");
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown message type: %s", type.c_str());
        }
    } catch (const json::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "JSON parse error: %s", e.what());
    }
}

void AppBridgeNode::on_client_close(WsSession::Ptr session)
{
    RCLCPP_INFO(this->get_logger(), "Android disconnected: %s", session->remote_addr().c_str());
    std::lock_guard<std::mutex> lock(clients_mutex_);
    clients_.erase(session);
}

void AppBridgeNode::broadcast(const json &msg)
{
    std::string                 data = msg.dump();
    std::lock_guard<std::mutex> lock(clients_mutex_);
    for (auto &client : clients_) {
        client->send(data);
    }
}

// ── Nav2 액션 ──────────────────────────────────────────

void AppBridgeNode::send_nav_goal(double x, double y, double theta)
{
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available");
        broadcast({
            {   "type",         "nav_result" },
            { "status", "server_unavailable" }
        });
        return;
    }

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id    = "map";
    goal.pose.header.stamp       = this->now();
    goal.pose.pose.position.x    = x;
    goal.pose.pose.position.y    = y;
    goal.pose.pose.orientation.z = std::sin(theta / 2.0);
    goal.pose.pose.orientation.w = std::cos(theta / 2.0);

    auto send_opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_opts.result_callback =
        [this](const auto &result) { on_goal_result(result); };

    nav_client_->async_send_goal(goal, send_opts);
    RCLCPP_INFO(this->get_logger(), "Nav goal sent: (%.2f, %.2f, %.2f)", x, y, theta);

    broadcast({
        {  "type", "nav_accepted" },
        {     "x",              x },
        {     "y",              y },
        { "theta",          theta }
    });
}

void AppBridgeNode::on_goal_response(
    rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr goal_handle
)
{
    if (!goal_handle) {
        RCLCPP_WARN(this->get_logger(), "Goal rejected by Nav2");
        broadcast({
            {   "type", "nav_result" },
            { "status",   "rejected" }
        });
    }
}

void AppBridgeNode::on_goal_result(
    const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &result
)
{
    std::string status;
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
        status = "succeeded";
        break;
    case rclcpp_action::ResultCode::ABORTED:
        status = "aborted";
        break;
    case rclcpp_action::ResultCode::CANCELED:
        status = "canceled";
        break;
    default:
        status = "unknown";
        break;
    }

    RCLCPP_INFO(this->get_logger(), "Navigation result: %s", status.c_str());
    broadcast({
        {   "type", "nav_result" },
        { "status",       status }
    });
}

// ── 로봇 위치 전송 ────────────────────────────────────

void AppBridgeNode::publish_robot_pose()
{
    {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        if (clients_.empty()) return;
    }

    try {
        auto transform = tf_buffer_->lookupTransform(
            "map", "base_link", tf2::TimePointZero
        );

        auto &t = transform.transform.translation;
        auto &r = transform.transform.rotation;

        double yaw = std::atan2(
            2.0 * (r.w * r.z + r.x * r.y),
            1.0 - 2.0 * (r.y * r.y + r.z * r.z)
        );

        broadcast({
            {  "type", "robot_pose" },
            {     "x",          t.x },
            {     "y",          t.y },
            { "theta",          yaw }
        });
    } catch (const tf2::TransformException &) {
        // TF 아직 준비되지 않음
    }
}

} // namespace app_bridge

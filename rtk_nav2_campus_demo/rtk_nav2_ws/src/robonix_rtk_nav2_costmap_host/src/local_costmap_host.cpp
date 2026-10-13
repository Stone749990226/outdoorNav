// 局部代价地图的宿主节点。
//
// 为什么需要它（docs/rtk-nav2-brief.md §10 裁定 2）：nav2 的局部代价地图
// `local_costmap` 只有作为某个"服务器"的子节点才会被创建——nav2 原生是挂在
// `controller_server` 上。但本阶段**不接管控制**，而 `controller_server` 即便不给它
// goal，也带一条 `cmd_vel` 的发布通路（`publish_zero_velocity` 一类参数一开就会发零速，
// 或有残留指令），与 `route_follower` 抢同一话题的症状是**偶发停车**、极难排查。
// 所以这里自写一个只挂 `Costmap2DROS` 的宿主：**没有任何控制输出**。
//
// 它做的全部事情就是把生命周期回调转发给内部的 costmap，并在激活时与
// lifecycle_manager 建立 bond（nav2 的规矩：bond 由**宿主**建，不由 Costmap2DROS 建）。
//
// 可核查判据（进产物）：运行中 `ros2 topic info -v /cmd_vel` 的发布者**只有 route_follower**。
//
// 节点名/命名空间：`Costmap2DROS("local_costmap", get_namespace(), "local_costmap")`
// —— 与 nav2 自己的 `ControllerServer` 完全一致的三个实参，所以内部 costmap 的
// 全限定名是 `/local_costmap/local_costmap`，参数也就落在 nav2 惯例的那一节
// `local_costmap: local_costmap: ros__parameters:` 里，不需要另立一套参数约定。

#include <fstream>
#include <sstream>
#include <memory>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robonix_rtk_nav2_costmap_host
{

class LocalCostmapHost : public nav2_util::LifecycleNode
{
public:
  explicit LocalCostmapHost(const rclcpp::NodeOptions & options)
  : nav2_util::LifecycleNode("local_costmap_host", "", options)
  {
    // 把内部 costmap 的**实际生效参数**落一份到文件（留痕用；空字符串 = 不落）。
    //
    // 为什么要在**进程内**落、而不是事后 `ros2 param dump`：内部 costmap 节点
    // `/local_costmap/local_costmap` 的参数服务在这套接线下**不应答**
    // （实测：ListParameters 超时；`ros2 param dump` 返回 0 字节）。
    // 而"两张图的实际参数"是 brief §4 要的证据（要能看出"文件改了但运行时没生效"），
    // 不能因为一个应外部查询的工具不通就缺一份。进程内自己读自己，最可靠。
    declare_parameter("param_dump_path", std::string(""));
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "local_costmap", std::string{get_namespace()}, "local_costmap");
    RCLCPP_INFO(
      get_logger(),
      "局部代价地图宿主启动：内部 costmap 节点 = %s（本节点只做生命周期转发，"
      "不发布任何控制指令）", costmap_ros_->get_name());
  }

  nav2_util::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override
  {
    RCLCPP_INFO(get_logger(), "configure：把生命周期转发给 local_costmap");
    return costmap_ros_->on_configure(state);
  }

  nav2_util::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override
  {
    // bond 必须在宿主侧建立，否则 lifecycle_manager 在 bond_timeout（默认 4 s）后
    // 会认为本节点掉线并把它 deactivate——局部图会"起来 4 秒就没了"。
    createBond();
    RCLCPP_INFO(get_logger(), "activate：把生命周期转发给 local_costmap");
    auto rc = costmap_ros_->on_activate(state);
    dumpCostmapParameters();
    return rc;
  }

  nav2_util::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    return costmap_ros_->on_deactivate(state);
  }

  nav2_util::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state) override
  {
    return costmap_ros_->on_cleanup(state);
  }

  nav2_util::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state) override
  {
    return costmap_ros_->on_shutdown(state);
  }

private:
  // 读出那一刻的**墙钟**（UTC，ISO8601），供四字段头的 `produced_at` 用。
  static std::string nowUtcIso8601()
  {
    const std::time_t t = std::time(nullptr);
    std::tm tmv{};
    gmtime_r(&t, &tmv);
    char buf[32] = {0};
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tmv);
    return std::string(buf);
  }

  // 从落盘路径推 run_id：`.../artifacts/rtk-nav2/<RUN_ID>/nav2/<file>.yaml`
  // ⇒ 取文件名上一级的上一级目录名。取不到就返回 "unknown"（不猜）。
  static std::string runIdFromPath(const std::string & path)
  {
    const auto slash1 = path.find_last_of('/');
    if (slash1 == std::string::npos) {return "unknown";}
    const auto slash2 = path.find_last_of('/', slash1 - 1);
    if (slash2 == std::string::npos) {return "unknown";}
    const auto slash3 = path.find_last_of('/', slash2 - 1);
    if (slash3 == std::string::npos) {return "unknown";}
    return path.substr(slash3 + 1, slash2 - slash3 - 1);
  }

  void dumpCostmapParameters()
  {
    const auto path = get_parameter("param_dump_path").as_string();
    if (path.empty()) {
      return;
    }
    std::vector<rclcpp::Parameter> params;
    try {
      const auto listed = costmap_ros_->list_parameters({}, 0);   // 0 = 递归
      params = costmap_ros_->get_parameters(listed.names);
    } catch (const std::exception & exc) {
      RCLCPP_WARN(get_logger(), "参数落盘失败（列/读参数异常）：%s", exc.what());
      return;
    }
    std::ofstream out(path, std::ios::trunc);
    if (!out) {
      RCLCPP_WARN(get_logger(), "参数落盘失败（打不开 %s）", path.c_str());
      return;
    }
    out << "# produced_by: local_costmap_host\n";
    // `produced_at` 必须是"**读出那一刻的墙钟**"，不是文件 mtime：语义不同——
    // 一个是"这个值是什么时候的"，一个是"这个文件什么时候被写的"。
    out << "# produced_at: " << nowUtcIso8601() << "\n";
    out << "# via: in-process\n";
    out << "# run_id: " << runIdFromPath(path) << "\n";
    out << "# 说明：本文件在 on_activate 时由本进程**直连服务**读出（外部\n";
    out << "# `ros2 param dump /local_costmap/local_costmap` 在本接线下超时/返回空，\n";
    out << "#  那条 CLI 通路属 `via: cli-daemon`，是另一份文件、不覆盖本文件）\n";
    out << "/local_costmap/local_costmap:\n  ros__parameters:\n";
    for (const auto & p : params) {
      out << "    " << p.get_name() << ": ";
      switch (p.get_type()) {
        case rclcpp::ParameterType::PARAMETER_BOOL:
          out << (p.as_bool() ? "true" : "false"); break;
        case rclcpp::ParameterType::PARAMETER_INTEGER:
          out << p.as_int(); break;
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
          out << p.as_double(); break;
        case rclcpp::ParameterType::PARAMETER_STRING:
          out << "'" << p.as_string() << "'"; break;
        default: {
            std::ostringstream oss;
            oss << p.value_to_string();
            out << oss.str();
          }
      }
      out << "\n";
    }
    out << "# 共 " << params.size() << " 个参数\n";
    out.close();
    RCLCPP_INFO(get_logger(), "已把 %zu 个 costmap 参数落盘到 %s",
                params.size(), path.c_str());
  }

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
};

}  // namespace robonix_rtk_nav2_costmap_host

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robonix_rtk_nav2_costmap_host::LocalCostmapHost>(
    rclcpp::NodeOptions());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

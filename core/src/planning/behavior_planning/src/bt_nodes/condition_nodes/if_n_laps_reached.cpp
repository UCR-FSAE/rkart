#include "behavior_planning/bt_nodes/condition_nodes/if_n_laps_reached.hpp"
#include "rclcpp/logging.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "behaviortree_cpp_v3/basic_types.h"

#include "behavior_planning/common/utils.hpp"

namespace roar
{
    namespace planning
    {
        namespace behavior
        {
            namespace condition
            {
                IfNLapsReached::IfNLapsReached(
                    const std::string &condition_name,
                    const BT::NodeConfiguration &conf,
                    const rclcpp::Logger &logger,
                    rclcpp::Clock &clock) : BT::ConditionNode(condition_name, conf), logger_(logger), clock_(clock)
                {
                    RCLCPP_DEBUG(logger_, "IfNLapsReached created");
                    BT::Optional<int> n_laps = getInput<int>("n_laps");
                    if (!n_laps)
                    {
                        RCLCPP_ERROR_STREAM(logger_, "IfNLapsReached: n_laps is not specified, defaulting to [" << n_laps_ << "]");
                    }
                    n_laps_ = n_laps.value();

                    RCLCPP_DEBUG_STREAM(logger_, "n_laps: " << n_laps_);

                    BT::Optional<float> x_init_raw = getInput<float>("x_init");
                    if (!x_init_raw)
                    {
                        RCLCPP_ERROR_STREAM(logger_, "IfNLapsReached: x_init is not specified, defaulting to [" << x_init_ << "]");
                    }
                    x_init_ = x_init_raw.value();

                    BT::Optional<float> y_init_raw = getInput<float>("y_init");
                    if (!y_init_raw)
                    {
                        RCLCPP_ERROR_STREAM(logger_, "IfNLapsReached: y_init is not specified, defaulting to [" << y_init_ << "]");
                    }
                    y_init_ = y_init_raw.value();

                    BT::Optional<float> radius_raw = getInput<float>("radius");
                    if (!radius_raw)
                    {
                        RCLCPP_ERROR_STREAM(logger_, "IfNLapsReached: radius is not specified, defaulting to [" << 0.5 << "]");
                    }
                    radius_ = radius_raw.value();

                    RCLCPP_DEBUG_STREAM(logger_, "x_init: " << x_init_ << ", y_init: " << y_init_ << ", radius: " << radius_);
                }

                BT::NodeStatus IfNLapsReached::tick()
                {
                    RCLCPP_DEBUG(logger_, "IfNLapsReached ticked");
                    // get inputs
                    auto inputs = config().blackboard->get<roar::planning::behavior::BTInputs::ConstSharedPtr>("inputs");
                    if (!inputs)
                    {
                        RCLCPP_ERROR(logger_, "IfNLapsReached: no inputs");
                        return BT::NodeStatus::FAILURE;
                    }

                    if (!inputs->vehicle_state)
                    {
                        RCLCPP_WARN(logger_, "IfNLapsReached: no vehicle state");
                        return BT::NodeStatus::FAILURE;
                    }

                    nav_msgs::msg::Odometry odom = inputs->vehicle_state->odometry;

                    // if within a certain radius of the goal
                    bool is_within_starting_radius = isWithinStartingRadius(odom, radius_, x_init_, y_init_);

                    /**
                     * We have two situations at this point:
                     * 
                     * 1. car is within starting radius due to just entered
                     * 2. car is within starting radius entered, but not exited
                     * 
                    */

                    // check if car has just entered the starting radius
                    if (is_within_starting_radius && state_.has_entered == false && state_.has_exited == false)
                    {
                        state_.has_entered = true;
                    }

                    // check if car has just exited the starting radius
                    if (is_within_starting_radius == false && state_.has_entered == true && state_.has_exited == false)
                    {
                        state_.has_exited = true;
                    }

                    // check if car has completed a lap
                    if (state_.has_entered && state_.has_exited && !is_within_starting_radius)
                    {

                        state_.lap_counts++;
                        RCLCPP_INFO_STREAM(logger_, "[IfNLapsReached]: [" << state_.lap_counts << "] laps reached");

                        state_.reset_state(); // if completed a lap, reset state for next iteration calculation
                    }

                    // check if lap count is reached
                    if (state_.lap_counts >= n_laps_)
                    {
                        RCLCPP_DEBUG_STREAM(logger_, "[IfNLapsReached]:" << n_laps_ << " laps reached");
                        return BT::NodeStatus::SUCCESS;
                    }
                    return BT::NodeStatus::FAILURE;
                }

                BT::PortsList IfNLapsReached::providedPorts()
                {
                    return {
                        BT::InputPort<int>("n_laps"),
                        BT::InputPort<float>("x_init"),
                        BT::InputPort<float>("y_init"),

                        BT::InputPort<std::string>("radius"),
                        BT::InputPort<roar::planning::behavior::BTInputs::ConstSharedPtr>("inputs"),
                    };
                }

            } // namespace condition
        }     // namespace behavior
    }         // namespace planning
} // namespace roar
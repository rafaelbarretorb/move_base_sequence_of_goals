#ifndef MOVE_BASE_SEQUENCE_OF_GOALS_SEQUENCE_OF_GOALS_H_
#define MOVE_BASE_SEQUENCE_OF_GOALS_SEQUENCE_OF_GOALS_H_

#include <ros/ros.h>

// ROS message types
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int16MultiArray.h>
#include <geometry_msgs/PoseArray.h>

// Move Base
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Standard libs
#include <vector>
#include <string>
#include <cmath>


namespace move_base_sequence_of_goals {

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseActionClient;


/**
 * @class SequenceOfGoals
 * @brief Send a sequence of goals to Move Base
 */
class SequenceOfGoals {
 public:
  /**
   * @brief Default constructor of the class
   */
  SequenceOfGoals();

  /** 
   * @brief Treat and send goals to Move Base
   * @param goals vector of poses
   */
  void sendGoalsToMoveBase();

  /** 
   * @brief Active Callback of Move Base 
   */
  void activeCbMoveBase();

  /** 
   * @brief Feedback Callback of Move Base 
   */
  void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

  /** 
   * @brief Done Callback of Move Base 
   */
  void doneCb(const actionlib::SimpleClientGoalState& state,
                      const move_base_msgs::MoveBaseResultConstPtr& result);

  /** 
   * @brief Set the sequence of goals and send to Move Base
   * @param path sequence of goals (poses)
   */
  void moveBaseClient();

  /** 
   * @brief Counter of goals
   * @return the current goal count
   */
  int getGoalCount() const;

  /** 
   * @brief 
   */
  std::vector<geometry_msgs::PoseArray> getGoalsPoses();

  /** 
   * @brief 
   */
  double getDistance(double x1, double y1, double x2, double y2);



  bool isMissionFinished() const;

  bool isMissionCancelled() const;

 private:
  bool cancel_mission_{false};

  int goal_count_{0};

  geometry_msgs::PoseArray path_;

  MoveBaseActionClient move_base_action_client_;

  std::string string_var;

  std::string global_frame_;


  bool mission_finished_{false};
};
};  // namespace move_base_sequence_of_goals

#endif  // MOVE_BASE_SEQUENCE_OF_GOALS_SEQUENCE_OF_GOALS_H_
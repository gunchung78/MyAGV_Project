#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <stdexcept>

struct GoalRow { double x, y, yaw_deg; };

class MultiGoalActionCli {
public:
  using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

  MultiGoalActionCli(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh), ac_("/move_base", /*spin_thread=*/true)
  {
    // 기본 파라미터
    pnh_.param<std::string>("file", file_path_, std::string(""));
    pnh_.param<std::string>("frame_id", frame_id_, std::string("map"));
    pnh_.param<bool>("cycle", cycle_, false);
    pnh_.param<double>("dwell_sec", dwell_sec_, 0.0);
    pnh_.param<double>("server_timeout_sec", server_timeout_sec_, 10.0);

    // 체크포인트/분기 파라미터
    pnh_.param<int>("decision_checkpoint_idx", decision_checkpoint_idx_, -1); // 0-based, -1=off
    pnh_.param<int>("left_idx", left_idx_, -1);
    pnh_.param<int>("right_idx", right_idx_, -1);
    pnh_.param<std::string>("decision_topic", decision_topic_, std::string("/vision_result"));
    pnh_.param<double>("decision_timeout_sec", decision_timeout_sec_, 10.0);
    int default_decision = 0;
    pnh_.param<int>("default_decision", default_decision, 0);
    default_decision_ = (default_decision != 0) ? 1 : 0;

    if (file_path_.empty()) { ROS_FATAL("~file 파라미터 없음"); throw std::runtime_error("file param empty"); }
    if (!loadFile(file_path_)) { ROS_FATAL("목표 파일 읽기 실패: %s", file_path_.c_str()); throw std::runtime_error("file load fail"); }

    ROS_INFO("목표 %zu개 로드 (frame_id=%s, cycle=%s, dwell=%.2fs)",
             rows_.size(), frame_id_.c_str(), cycle_ ? "true" : "false", dwell_sec_);

    // 액션 서버 대기
    ROS_INFO("move_base 액션 서버 대기중(%.1fs)...", server_timeout_sec_);
    if (!ac_.waitForServer(ros::Duration(server_timeout_sec_))) {
      ROS_FATAL("move_base 액션 서버 연결 실패");
      throw std::runtime_error("move_base server timeout");
    }

    // vision 결과 구독(라치값도 수신 가능)
    decision_sub_ = nh_.subscribe<std_msgs::Int32>(decision_topic_, 1, &MultiGoalActionCli::decisionCb, this);

    // 다음 goal 전송용 oneshot 타이머 (처음엔 autostart=false)
    next_goal_timer_ = nh_.createTimer(
        ros::Duration(0.0),
        [this](const ros::TimerEvent&){ this->sendCurrent(); },
        /*oneshot=*/true, /*autostart=*/false);

    // 첫 목표는 약간 지연해서 스케줄 (상태머신 안정화)
    scheduleNext(0.05);
  }

  void spin() { ros::waitForShutdown(); }

private:
  // ---------- 파일 로드 ----------
  bool loadFile(const std::string& path) {
    std::ifstream fin(path.c_str());
    if (!fin.is_open()) return false;
    std::string line; size_t lineno = 0;
    while (std::getline(fin, line)) {
      ++lineno;
      auto trim = [](std::string& s){
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch){ return !std::isspace(ch); }));
        s.erase(std::find_if(s.rbegin(), s.rend(), [](int ch){ return !std::isspace(ch); }).base(), s.end());
      };
      trim(line);
      if (line.empty() || line[0]=='#') continue;
      if (!line.empty() && (line.back()==',' || line.back()==';')) line.pop_back();
      std::replace(line.begin(), line.end(), ',', ' ');

      std::stringstream ss(line);
      GoalRow r{};
      if (!(ss >> r.x >> r.y >> r.yaw_deg)) {
        ROS_WARN("파싱 실패(line %zu): %s", lineno, line.c_str());
        continue;
      }
      rows_.push_back(r);
    }
    return !rows_.empty();
  }

  // ---------- 유틸 ----------
  static geometry_msgs::Quaternion yawToQuat(double yaw_deg) {
    double yaw_rad = yaw_deg * M_PI / 180.0;
    tf::Quaternion q = tf::createQuaternionFromYaw(yaw_rad);
    geometry_msgs::Quaternion o; o.x=q.x(); o.y=q.y(); o.z=q.z(); o.w=q.w(); return o;
  }
  bool isCheckpointIdx(size_t idx) const {
    return (decision_checkpoint_idx_ >= 0) && (static_cast<int>(idx) == decision_checkpoint_idx_);
  }
  void scheduleNext(double delay_s = 0.05) {
    next_goal_timer_.setPeriod(ros::Duration(delay_s), /*reset=*/true);
    next_goal_timer_.start();
  }

  // ---------- goal 전송 ----------
  void sendCurrent() {
    if (rows_.empty()) { ROS_ERROR("목표 없음. 종료."); ros::shutdown(); return; }

    if (idx_ >= rows_.size()) {
      if (cycle_) { ++cycle_cnt_; idx_ = 0; ROS_INFO("사이클 %d 재시작", cycle_cnt_+1); }
      else { ROS_INFO("모든 목표 완료. 종료."); ros::shutdown(); return; }
    }

    const auto& r = rows_[idx_];

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = frame_id_;
    goal.target_pose.pose.position.x = r.x;
    goal.target_pose.pose.position.y = r.y;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation = yawToQuat(r.yaw_deg);

    ROS_INFO("목표 %zu/%zu 전송: x=%.3f y=%.3f yaw=%.1f°",
             idx_+1, rows_.size(), r.x, r.y, r.yaw_deg);

    // 이전 goal이 ACTIVE/PENDING이면 취소 후 약간 대기
    if (ac_.getState() == actionlib::SimpleClientGoalState::ACTIVE ||
        ac_.getState() == actionlib::SimpleClientGoalState::PENDING) {
      ROS_WARN("이전 goal 취소 후 전송");
      ac_.cancelAllGoals();
      ros::Duration(0.05).sleep();
    }

    // 상태 플래그 초기화
    waiting_decision_ = false;
    decision_ready_   = false;

    ac_.sendGoal(goal,
      boost::bind(&MultiGoalActionCli::doneCb, this, _1, _2),
      boost::bind(&MultiGoalActionCli::activeCb, this),
      boost::bind(&MultiGoalActionCli::feedbackCb, this, _1)
    );
  }

  // ---------- 액션 콜백 ----------
  void activeCb() { ROS_INFO("목표 %zu 활성화", idx_+1); }

  void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& fb) {
    (void)fb; // 필요시 현재 위치/오차 활용 가능
  }

  void doneCb(const actionlib::SimpleClientGoalState& state,
              const move_base_msgs::MoveBaseResultConstPtr& result)
  {
    (void)result;
    ROS_INFO("목표 %zu 결과: %s", idx_+1, state.toString().c_str());

    // 성공/실패/LOST 모두 "도착 처리"로 간주 (정책에 맞게 조정 가능)
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED ||
        state == actionlib::SimpleClientGoalState::ABORTED   ||
        state == actionlib::SimpleClientGoalState::PREEMPTED ||
        state == actionlib::SimpleClientGoalState::REJECTED  ||
        state == actionlib::SimpleClientGoalState::LOST)
    {
      // 도착 후 dwell
      if (dwell_sec_ > 0.0) ros::Duration(dwell_sec_).sleep();

      // 체크포인트 처리: vision_result 대기
      if (isCheckpointIdx(idx_)) {
        ROS_WARN("체크포인트 %d 도달. %s 대기 (timeout=%.1fs, default=%d)",
                 decision_checkpoint_idx_+1, decision_topic_.c_str(),
                 decision_timeout_sec_, default_decision_);

        waiting_decision_ = true;
        decision_ready_   = false;

        // AsyncSpinner가 콜백 처리하므로 spinOnce() 금지. 타임아웃만 폴링.
        ros::Time start = ros::Time::now();
        while (ros::ok() && !decision_ready_) {
          if ((ros::Time::now() - start).toSec() >= decision_timeout_sec_) {
            last_decision_  = default_decision_;
            decision_ready_ = true;
            ROS_WARN("vision_result 타임아웃 → default=%d 사용", last_decision_);
            break;
          }
          ros::Duration(0.01).sleep();
        }

        int next = (last_decision_ == 0) ? left_idx_ : right_idx_;
        if (next < 0 || next >= static_cast<int>(rows_.size())) {
          ROS_ERROR("분기 인덱스(%d) 유효하지 않음 → 다음 순번으로 진행", next);
          ++idx_;
        } else {
          ROS_INFO("vision_result=%d → idx=%d 로 점프", last_decision_, next);
          idx_ = static_cast<size_t>(next);
        }
        waiting_decision_ = false;

        // 다음 goal은 타이머로 지연 전송 (DONE 직후 직접 호출 금지)
        scheduleNext(0.05);
        return;
      }

      // 일반 케이스: 다음 순번
      ++idx_;
      scheduleNext(0.05); // DONE 콜백에서 직접 sendCurrent() 호출 금지
      return;
    }

    // 그 밖의 상태면 조금 뒤 재시도(방어적)
    scheduleNext(0.2);
  }

  // ---------- vision 결과 콜백 ----------
  void decisionCb(const std_msgs::Int32::ConstPtr& msg) {
    int v = (msg->data != 0) ? 1 : 0;
    if (waiting_decision_) {
      last_decision_  = v;
      decision_ready_ = true;
      ROS_INFO("vision_result 수신: %d", v);
    } else {
      ROS_DEBUG("vision_result 수신(대기 아님): %d", v);
    }
  }

private:
  ros::NodeHandle nh_, pnh_;
  MoveBaseClient ac_;

  // pub/sub
  ros::Subscriber decision_sub_;

  // 타이머
  ros::Timer next_goal_timer_;

  // 파라미터
  std::string file_path_;
  std::string frame_id_;
  bool  cycle_{false};
  double dwell_sec_{0.0};
  double server_timeout_sec_{10.0};

  int decision_checkpoint_idx_{-1};
  int left_idx_{-1};
  int right_idx_{-1};
  std::string decision_topic_{"/vision_result"};
  double decision_timeout_sec_{10.0};
  int default_decision_{0}; // 0=left, 1=right

  // 상태
  std::vector<GoalRow> rows_;
  size_t idx_{0};
  int    cycle_cnt_{0};

  // 분기 상태
  bool waiting_decision_{false};
  bool decision_ready_{false};
  int  last_decision_{0};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "multi_goals_txt");
  try {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    // 콜백/타이머 동시 처리
    ros::AsyncSpinner spinner(2);
    spinner.start();
    MultiGoalActionCli app(nh, pnh);
    app.spin(); // waitForShutdown
  } catch (const std::exception& e) {
    ROS_FATAL("예외 종료: %s", e.what());
    return 1;
  }
  return 0;
}

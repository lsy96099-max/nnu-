// fcu_mission.cpp
// Modified: add YAML reading for point1~point4, clear missions on takeoff,
// and add mode enable_pos==7 to make UAV1/2/3 draw vertical "N N U" at 1.0m.
// Preservation: original logic preserved where possible.

#include <ros/ros.h>
#include <ros/package.h>
#include <stdio.h>
#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>
#include <geometry_msgs/InertiaStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "quadrotor_msgs/PositionCommand.h"
#include "../mavlink/common/mavlink.h"

// YAML
#include <yaml-cpp/yaml.h>

static std::string mission_yaml_path;
static YAML::Node mission_yaml;

/**
 * 注意：工程中mission_xxx话题为发给飞控的目标值，目标位移应为FRU坐标系，目标姿态应为FRD坐标系
 */
static bool enable_path=false;
static bool enable_track=false;
static bool get_pos_cmd=false;
static bool follow_forward=false;
static bool follow_down=false;
static bool set_goal=false;
static bool use_goal_001=false;
static bool use_goal_002=false;
static bool use_goal_003=false;
static bool use_goal_004=false;
static bool use_goal_005=false;
static bool use_goal_006=false;
static uint8_t enable_pos=0;

float pos_odom_001_x=0.0f; float pos_odom_001_y=0.0f; float pos_odom_001_z=0.0f;
float pos_odom_002_x=0.0f; float pos_odom_002_y=0.0f; float pos_odom_002_z=0.0f;
float pos_odom_003_x=0.0f; float pos_odom_003_y=0.0f; float pos_odom_003_z=0.0f;
float pos_odom_004_x=0.0f; float pos_odom_004_y=0.0f; float pos_odom_004_z=0.0f;
float pos_odom_005_x=0.0f; float pos_odom_005_y=0.0f; float pos_odom_005_z=0.0f;
float pos_odom_006_x=0.0f; float pos_odom_006_y=0.0f; float pos_odom_006_z=0.0f;

float pos_odom_001_roll=0.0f; float pos_odom_001_pitch=0.0f; float pos_odom_001_yaw=0.0f;
float pos_odom_002_roll=0.0f; float pos_odom_002_pitch=0.0f; float pos_odom_002_yaw=0.0f;
float pos_odom_003_roll=0.0f; float pos_odom_003_pitch=0.0f; float pos_odom_003_yaw=0.0f;
float pos_odom_004_roll=0.0f; float pos_odom_004_pitch=0.0f; float pos_odom_004_yaw=0.0f;
float pos_odom_005_roll=0.0f; float pos_odom_005_pitch=0.0f; float pos_odom_005_yaw=0.0f;
float pos_odom_006_roll=0.0f; float pos_odom_006_pitch=0.0f; float pos_odom_006_yaw=0.0f;

float pos_takeoff_001_x=0.0f; float pos_takeoff_001_y=0.0f; float pos_takeoff_001_z=0.0f;
float pos_takeoff_002_x=0.0f; float pos_takeoff_002_y=0.0f; float pos_takeoff_002_z=0.0f;
float pos_takeoff_003_x=0.0f; float pos_takeoff_003_y=0.0f; float pos_takeoff_003_z=0.0f;
float pos_takeoff_004_x=0.0f; float pos_takeoff_004_y=0.0f; float pos_takeoff_004_z=0.0f;
float pos_takeoff_005_x=0.0f; float pos_takeoff_005_y=0.0f; float pos_takeoff_005_z=0.0f;
float pos_takeoff_006_x=0.0f; float pos_takeoff_006_y=0.0f; float pos_takeoff_006_z=0.0f;

typedef enum {
  ReadyToGoal,
  ExecutingGoal
} path_track_flag;
path_track_flag path_track_status = ReadyToGoal;

static std_msgs::Float32MultiArray mission_001;
static ros::Publisher mission_pub_001;

static std_msgs::Float32MultiArray mission_002;
static ros::Publisher mission_pub_002;
static std_msgs::Float32MultiArray mission_003;
static ros::Publisher mission_pub_003;
static std_msgs::Float32MultiArray mission_004;
static ros::Publisher mission_pub_004;
static std_msgs::Float32MultiArray mission_005;
static ros::Publisher mission_pub_005;
static std_msgs::Float32MultiArray mission_006;
static ros::Publisher mission_pub_006;

static float yaw=0.0f, yaw_rate=0.0f;
static float px=0.0f, py=0.0f, pz=0.0f;
static float vx=0.0f, vy=0.0f, vz=0.0f;
static float ax=0.0f, ay=0.0f, az=0.0f;
static float theta=0.0f;

static float  px1=0.0f, py1=0.0f, pz1=0.0f,
              px2=0.0f, py2=0.0f, pz2=0.0f,
              px3=0.0f, py3=0.0f, pz3=0.0f,
              px4=0.0f, py4=0.0f, pz4=0.0f,
              px5=0.0f, py5=0.0f, pz5=0.0f,
              px6=0.0f, py6=0.0f, pz6=0.0f;

static int goal_point = 0;

// ---------- Helper: generate letters N and U (vertical) ----------
struct Pt { float x; float y; float z; };
static std::vector<Pt> gen_N(float height, float width, int samples_per_stroke) {
  // N: left vertical up, diagonal down to bottom-right, right vertical up.
  std::vector<Pt> out;
  // left vertical: from (0,0) up to (0,height)
  for(int i=0;i<samples_per_stroke;i++){
    float t = (float)i/(samples_per_stroke-1);
    out.push_back({0.0f,  t*height,  height});
  }
  // diagonal: from (0,height) to (width,-0.0)
  for(int i=0;i<samples_per_stroke;i++){
    float t = (float)i/(samples_per_stroke-1);
    float x = t*width;
    float y = height*(1.0f - t);
    out.push_back({x, y, height});
  }
  // right vertical: from (width,0) up to (width,height)
  for(int i=0;i<samples_per_stroke;i++){
    float t = (float)i/(samples_per_stroke-1);
    out.push_back({width, t*height, height});
  }
  return out;
}

static std::vector<Pt> gen_U(float height, float width, int samples_side, int samples_bottom) {
  // U: left vertical down, bottom semi-circle, right vertical up (but vertical 'U' upright)
  std::vector<Pt> out;
  // left vertical: top to near bottom
  for(int i=0;i<samples_side;i++){
    float t = (float)i/(samples_side-1);
    out.push_back({0.0f, height*(1.0f - t), height});
  }
  // bottom: half-ellipse from left to right
  for(int i=0;i<samples_bottom;i++){
    float t = (float)i/(samples_bottom-1); // 0..1
    float angle = M_PI * t; // 0..pi
    float cx = width/2.0f;
    float rx = width/2.0f;
    float ry = height*0.25f; // small curvature
    float x = cx + rx * cosf(M_PI - angle); // left->right
    float y = 0.0f - ry * sinf(angle); // small negative dip
    out.push_back({x, y, height});
  }
  // right vertical: near bottom to top
  for(int i=0;i<samples_side;i++){
    float t = (float)i/(samples_side-1);
    out.push_back({width, t*height, height});
  }
  return out;
}

// ---------- End helper ----------

void cmdHandler(const std_msgs::Int16::ConstPtr& cmd){
  switch(cmd->data){
    case 0:
        enable_path=true;
        break;
   case 3: // 起飞并进入原地悬停
    enable_pos = 0;

    // 记录起飞位置
    pos_takeoff_001_x=pos_odom_001_x; pos_takeoff_001_y=pos_odom_001_y; pos_takeoff_001_z=pos_odom_001_z;
    pos_takeoff_002_x=pos_odom_002_x; pos_takeoff_002_y=pos_odom_002_y; pos_takeoff_002_z=pos_odom_002_z;
    pos_takeoff_003_x=pos_odom_003_x; pos_takeoff_003_y=pos_odom_003_y; pos_takeoff_003_z=pos_odom_003_z;
    pos_takeoff_004_x=pos_odom_004_x; pos_takeoff_004_y=pos_odom_004_y; pos_takeoff_004_z=pos_odom_004_z;
    pos_takeoff_005_x=pos_odom_005_x; pos_takeoff_005_y=pos_odom_005_y; pos_takeoff_005_z=pos_odom_005_z;
    pos_takeoff_006_x=pos_odom_006_x; pos_takeoff_006_y=pos_odom_006_y; pos_takeoff_006_z=pos_odom_006_z;

    // 清空旧的 mission 指令，避免飞向旧点
    mission_001.data.assign(11, 0.0f);
    mission_002.data.assign(11, 0.0f);
    mission_003.data.assign(11, 0.0f);
    mission_004.data.assign(11, 0.0f);
    mission_005.data.assign(11, 0.0f);
    mission_006.data.assign(11, 0.0f);

    get_pos_cmd = false;
    use_goal_001 = false;
    use_goal_002 = false;
    use_goal_003 = false;
    use_goal_004 = false;
    use_goal_005 = false;
    use_goal_006 = false;

    ROS_INFO("[CMD] T pressed: takeoff positions recorded + mission cleared.");
    break;
    case 5:
        enable_track=true;
        break;
    case 6:
        enable_track=false;
        enable_path=false;
        enable_pos=255;
        break;
    case 7:
        // 保持原有 point1 功能（不覆盖）
        enable_pos=1;
        use_goal_001=true;
        use_goal_002=true;
        use_goal_003=true;
        use_goal_004=true;
        use_goal_005=true;
        use_goal_006=true;
        break;
    case 8:
        enable_pos=2;
        use_goal_001=true;
        use_goal_002=true;
        use_goal_003=true;
        use_goal_004=true;
        use_goal_005=true;
        use_goal_006=true;
        break;
    case 9:
        enable_pos=3;
        use_goal_001=true;
        use_goal_002=true;
        use_goal_003=true;
        use_goal_004=true;
        use_goal_005=true;
        use_goal_006=true;
        break;
    case 10:
        enable_pos=4;
        use_goal_001=true;
        use_goal_002=true;
        use_goal_003=true;
        use_goal_004=true;
        use_goal_005=true;
        use_goal_006=true;
        break;

    // ===== 新增：case 11 -> 三机画 N N U (enable_pos == 7) =====
    case 11:
        enable_pos = 7; // 我们把 NNU 模式映射到 enable_pos==7
        // 仅激活前三台的 use_goal，这样 4/5/6 保持悬停
        use_goal_001 = true;
        use_goal_002 = true;
        use_goal_003 = true;
        use_goal_004 = false;
        use_goal_005 = false;
        use_goal_006 = false;
        ROS_INFO("[CMD] NNU mode started (enable_pos=7).");
        break;
    // ========================================================

    case 101:
        set_goal=true;
        break;
    case 102:
        get_pos_cmd=true;
        break;
    case 1001:
        follow_forward=true;
        break;
    case 1002:
        follow_down=true;
        break;
    case 1003:
        follow_forward=false;
        follow_down=false;
        break;
    case 1101:
        pos_takeoff_001_x=pos_odom_001_x; pos_takeoff_001_y=pos_odom_001_y; pos_takeoff_001_z=pos_odom_001_z;
        break;
    case 1102:
        pos_takeoff_002_x=pos_odom_002_x; pos_takeoff_002_y=pos_odom_002_y; pos_takeoff_002_z=pos_odom_002_z;
        break;
    case 1103:
        pos_takeoff_003_x=pos_odom_003_x; pos_takeoff_003_y=pos_odom_003_y; pos_takeoff_003_z=pos_odom_003_z;
        break;
    case 1104:
        pos_takeoff_004_x=pos_odom_004_x; pos_takeoff_004_y=pos_odom_004_y; pos_takeoff_004_z=pos_odom_004_z;
        break;
    case 1105:
        pos_takeoff_005_x=pos_odom_005_x; pos_takeoff_005_y=pos_odom_005_y; pos_takeoff_005_z=pos_odom_005_z;
        break;
    case 1106:
        pos_takeoff_006_x=pos_odom_006_x; pos_takeoff_006_y=pos_odom_006_y; pos_takeoff_006_z=pos_odom_006_z;
        break;
    default:
        break;
  }
}

void SetGoal(int id, float target_x,float target_y,float target_z)
{
  switch (id)
  {
  case 1://1号机
    px1=target_x;
    py1=target_y;
    pz1=target_z;
    break;
  case 2://2号机
    px2=target_x;
    py2=target_y;
    pz2=target_z;
    break;
  case 3://3号机
    px3=target_x;
    py3=target_y;
    pz3=target_z;
    break;
  case 4://4号机
    px4=target_x;
    py4=target_y;
    pz4=target_z;
    break;
  case 5://5号机
    px5=target_x;
    py5=target_y;
    pz5=target_z;
    break;
  case 6://6号机
    px6=target_x;
    py6=target_y;
    pz6=target_z;
    break;
  default:
    break;
  }
}

bool IsReachGoal(int id, float dis)
{
  switch (id)
  {
  case 1:
    if(fabs(px1-pos_odom_001_x) < dis && fabs(py1-pos_odom_001_y) < dis){
      return true;
    }
    break;
  case 2:
    if(fabs(px2-pos_odom_002_x) < dis && fabs(py2-pos_odom_002_y) < dis){
      return true;
    }
    break;
  case 3:
    if(fabs(px3-pos_odom_003_x) < dis && fabs(py3-pos_odom_003_y) < dis){
      return true;
    }
    break;
  case 4:
    if(fabs(px4-pos_odom_004_x) < dis && fabs(py4-pos_odom_004_y) < dis){
      return true;
    }
    break;
  case 5:
    if(fabs(px5-pos_odom_005_x) < dis && fabs(py5-pos_odom_005_y) < dis){
      return true;
    }
    break;
  case 6:
    if(fabs(px6-pos_odom_006_x) < dis && fabs(py6-pos_odom_006_y) < dis){
      return true;
    }
    break;
  default:
    break;
  }
  return false;
}

void execute_mission_001(const ros::TimerEvent &event){
  if(get_pos_cmd){
      return;
  }
  if(set_goal&&!use_goal_001){
    return;
  }
  //发布mission
  mission_001.layout.dim.clear();
  mission_001.layout.dim.push_back(std_msgs::MultiArrayDimension());
  mission_001.layout.dim[0].label = "mission_001";
  mission_001.layout.dim[0].size = 11;
  mission_001.layout.dim[0].stride = 1;
  mission_001.data.resize(11);
  mission_001.data[0]=yaw;//rad
  mission_001.data[1]=yaw_rate;//rad/s
  mission_001.data[2]=px1;//x
  mission_001.data[3]=py1;//y
  mission_001.data[4]=pz1;//z
  mission_001.data[5]=vx;//vx
  mission_001.data[6]=vy;//vy
  mission_001.data[7]=vz;//vz
  mission_001.data[8]=ax;//ax
  mission_001.data[9]=ay;//ay
  mission_001.data[10]=az;//az
  mission_pub_001.publish(mission_001);
  use_goal_001=false;     
}

void execute_mission_002(const ros::TimerEvent &event){
  if(get_pos_cmd){
      return;
  }
  if(set_goal&&!use_goal_002){
    return;
  }
  mission_002.layout.dim.clear();
  mission_002.layout.dim.push_back(std_msgs::MultiArrayDimension());
  mission_002.layout.dim[0].label = "mission_002";
  mission_002.layout.dim[0].size = 11;
  mission_002.layout.dim[0].stride = 1;
  mission_002.data.resize(11);
  mission_002.data[0]=0.0f;//rad
  mission_002.data[1]=0.0f;//rad/s
  mission_002.data[2]=px2;//x
  mission_002.data[3]=py2;//y
  mission_002.data[4]=pz2;//z
  mission_002.data[5]=0.0f;//vx
  mission_002.data[6]=0.0f;//vy
  mission_002.data[7]=0.0f;//vz
  mission_002.data[8]=0.0f;//ax
  mission_002.data[9]=0.0f;//ay
  mission_002.data[10]=0.0f;//az
  mission_pub_002.publish(mission_002);
  use_goal_002=false;
}

void execute_mission_003(const ros::TimerEvent &event){
  if(get_pos_cmd){
      return;
  }
  if(set_goal&&!use_goal_003){
    return;
  }
  mission_003.layout.dim.clear();
  mission_003.layout.dim.push_back(std_msgs::MultiArrayDimension());
  mission_003.layout.dim[0].label = "mission_003";
  mission_003.layout.dim[0].size = 11;
  mission_003.layout.dim[0].stride = 1;
  mission_003.data.resize(11);
  mission_003.data[0]=0.0f;//rad
  mission_003.data[1]=0.0f;//rad/s
  mission_003.data[2]=px3;//x
  mission_003.data[3]=py3;//y
  mission_003.data[4]=pz3;//z
  mission_003.data[5]=0.0f;//vx
  mission_003.data[6]=0.0f;//vy
  mission_003.data[7]=0.0f;//vz
  mission_003.data[8]=0.0f;//ax
  mission_003.data[9]=0.0f;//ay
  mission_003.data[10]=0.0f;//az
  mission_pub_003.publish(mission_003);
  use_goal_003=false;
}

void execute_mission_004(const ros::TimerEvent &event){
  if(get_pos_cmd){
      return;
  }
  if(set_goal&&!use_goal_004){
    return;
  }
  mission_004.layout.dim.clear();
  mission_004.layout.dim.push_back(std_msgs::MultiArrayDimension());
  mission_004.layout.dim[0].label = "mission_004";
  mission_004.layout.dim[0].size = 11;
  mission_004.layout.dim[0].stride = 1;
  mission_004.data.resize(11);
  mission_004.data[0]=0.0f;//rad
  mission_004.data[1]=0.0f;//rad/s
  mission_004.data[2]=px4;//x
  mission_004.data[3]=py4;//y
  mission_004.data[4]=pz4;//z
  mission_004.data[5]=0.0f;//vx
  mission_004.data[6]=0.0f;//vy
  mission_004.data[7]=0.0f;//vz
  mission_004.data[8]=0.0f;//ax
  mission_004.data[9]=0.0f;//ay
  mission_004.data[10]=0.0f;//az
  mission_pub_004.publish(mission_004);
  use_goal_004=false;
}

void execute_mission_005(const ros::TimerEvent &event){
  if(get_pos_cmd){
      return;
  }
  if(set_goal&&!use_goal_005){
    return;
  }
  mission_005.layout.dim.clear();
  mission_005.layout.dim.push_back(std_msgs::MultiArrayDimension());
  mission_005.layout.dim[0].label = "mission_005";
  mission_005.layout.dim[0].size = 11;
  mission_005.layout.dim[0].stride = 1;
  mission_005.data.resize(11);
  mission_005.data[0]=0.0f;//rad
  mission_005.data[1]=0.0f;//rad/s
  mission_005.data[2]=px5;//x
  mission_005.data[3]=py5;//y
  mission_005.data[4]=pz5;//z
  mission_005.data[5]=0.0f;//vx
  mission_005.data[6]=0.0f;//vy
  mission_005.data[7]=0.0f;//vz
  mission_005.data[8]=0.0f;//ax
  mission_005.data[9]=0.0f;//ay
  mission_005.data[10]=0.0f;//az
  mission_pub_005.publish(mission_005);
  use_goal_005=false;
}

void execute_mission_006(const ros::TimerEvent &event){
  if(get_pos_cmd){
      return;
  }
  if(set_goal&&!use_goal_006){
    return;
  }
  mission_006.layout.dim.clear();
  mission_006.layout.dim.push_back(std_msgs::MultiArrayDimension());
  mission_006.layout.dim[0].label = "mission_006";
  mission_006.layout.dim[0].size = 11;
  mission_006.layout.dim[0].stride = 1;
  mission_006.data.resize(11);
  mission_006.data[0]=0.0f;//rad
  mission_006.data[1]=0.0f;//rad/s
  mission_006.data[2]=px6;//x
  mission_006.data[3]=py6;//y
  mission_006.data[4]=pz6;//z
  mission_006.data[5]=0.0f;//vx
  mission_006.data[6]=0.0f;//vy
  mission_006.data[7]=0.0f;//vz
  mission_006.data[8]=0.0f;//ax
  mission_006.data[9]=0.0f;//ay
  mission_006.data[10]=0.0f;//az
  mission_pub_006.publish(mission_006);
  use_goal_006=false;
}

void odom_global001_handler(const nav_msgs::Odometry::ConstPtr& odom)
{
  pos_odom_001_x=(float)odom->pose.pose.position.x;//位置点改为FRU坐标
  pos_odom_001_y=-(float)odom->pose.pose.position.y;
  pos_odom_001_z=(float)odom->pose.pose.position.z;
  float quaternion_odom[4]={(float)odom->pose.pose.orientation.w,
                            (float)odom->pose.pose.orientation.x,
                            (float)odom->pose.pose.orientation.y,
                            (float)odom->pose.pose.orientation.z};
  mavlink_quaternion_to_euler(quaternion_odom, &pos_odom_001_roll, &pos_odom_001_pitch, &pos_odom_001_yaw);
}

void odom_global002_handler(const nav_msgs::Odometry::ConstPtr& odom)
{
  pos_odom_002_x=(float)odom->pose.pose.position.x;//位置点改为FRU坐标
  pos_odom_002_y=-(float)odom->pose.pose.position.y;
  pos_odom_002_z=(float)odom->pose.pose.position.z;
  float quaternion_odom[4]={(float)odom->pose.pose.orientation.w,
                            (float)odom->pose.pose.orientation.x,
                            (float)odom->pose.pose.orientation.y,
                            (float)odom->pose.pose.orientation.z};
  mavlink_quaternion_to_euler(quaternion_odom, &pos_odom_002_roll, &pos_odom_002_pitch, &pos_odom_002_yaw);
}

void odom_global003_handler(const nav_msgs::Odometry::ConstPtr& odom)
{
  pos_odom_003_x=(float)odom->pose.pose.position.x;//位置点改为FRU坐标
  pos_odom_003_y=-(float)odom->pose.pose.position.y;
  pos_odom_003_z=(float)odom->pose.pose.position.z;
  float quaternion_odom[4]={(float)odom->pose.pose.orientation.w,
                            (float)odom->pose.pose.orientation.x,
                            (float)odom->pose.pose.orientation.y,
                            (float)odom->pose.pose.orientation.z};
  mavlink_quaternion_to_euler(quaternion_odom, &pos_odom_003_roll, &pos_odom_003_pitch, &pos_odom_003_yaw);
}

void odom_global004_handler(const nav_msgs::Odometry::ConstPtr& odom)
{
  pos_odom_004_x=(float)odom->pose.pose.position.x;//位置点改为FRU坐标
  pos_odom_004_y=-(float)odom->pose.pose.position.y;
  pos_odom_004_z=(float)odom->pose.pose.position.z;
  float quaternion_odom[4]={(float)odom->pose.pose.orientation.w,
                            (float)odom->pose.pose.orientation.x,
                            (float)odom->pose.pose.orientation.y,
                            (float)odom->pose.pose.orientation.z};
  mavlink_quaternion_to_euler(quaternion_odom, &pos_odom_004_roll, &pos_odom_004_pitch, &pos_odom_004_yaw);
}

void odom_global005_handler(const nav_msgs::Odometry::ConstPtr& odom)
{
  pos_odom_005_x=(float)odom->pose.pose.position.x;//位置点改为FRU坐标
  pos_odom_005_y=-(float)odom->pose.pose.position.y;
  pos_odom_005_z=(float)odom->pose.pose.position.z;
  float quaternion_odom[4]={(float)odom->pose.pose.orientation.w,
                            (float)odom->pose.pose.orientation.x,
                            (float)odom->pose.pose.orientation.y,
                            (float)odom->pose.pose.orientation.z};
  mavlink_quaternion_to_euler(quaternion_odom, &pos_odom_005_roll, &pos_odom_005_pitch, &pos_odom_005_yaw);
}

void odom_global006_handler(const nav_msgs::Odometry::ConstPtr& odom)
{
  pos_odom_006_x=(float)odom->pose.pose.position.x;//位置点改为FRU坐标
  pos_odom_006_y=-(float)odom->pose.pose.position.y;
  pos_odom_006_z=(float)odom->pose.pose.position.z;
  float quaternion_odom[4]={(float)odom->pose.pose.orientation.w,
                            (float)odom->pose.pose.orientation.x,
                            (float)odom->pose.pose.orientation.y,
                            (float)odom->pose.pose.orientation.z};
  mavlink_quaternion_to_euler(quaternion_odom, &pos_odom_006_roll, &pos_odom_006_pitch, &pos_odom_006_yaw);
}

void pos_cmd_handler(const quadrotor_msgs::PositionCommand::ConstPtr& pose_plan)//仅机载电脑运行此函数
{
  if(follow_forward||follow_down){
    return;
  }
  //发布mission
  get_pos_cmd=true;
  mission_001.layout.dim.clear();
  mission_001.layout.dim.push_back(std_msgs::MultiArrayDimension());
  mission_001.layout.dim[0].label = "mission_001";
  mission_001.layout.dim[0].size = 11;
  mission_001.layout.dim[0].stride = 1;
  mission_001.data.resize(11);
  mission_001.data[0]=-pose_plan->yaw;//rad
  mission_001.data[1]=-pose_plan->yaw_dot;//rad/s
  mission_001.data[2]=pose_plan->position.x;//x
  mission_001.data[3]=-pose_plan->position.y;//y
  mission_001.data[4]=pose_plan->position.z;//z
  mission_001.data[5]=pose_plan->velocity.x;//vx
  mission_001.data[6]=-pose_plan->velocity.y;//vy
  mission_001.data[7]=pose_plan->velocity.z;//vz
  mission_001.data[8]=pose_plan->acceleration.x;//ax
  mission_001.data[9]=-pose_plan->acceleration.y;//ay
  mission_001.data[10]=pose_plan->acceleration.z;//az
  mission_pub_001.publish(mission_001);
}

void follow_handler(const std_msgs::Float32MultiArray::ConstPtr& follow){
  if(follow_forward){
    get_pos_cmd=true;
    if(follow->data[2]==0.0f&&follow->data[3]==0.0f){
      printf("No tracking!\n");
      return;
    }
    float global_dx = follow->data[2] * cosf(-pos_odom_001_yaw) - follow->data[3] * sinf(-pos_odom_001_yaw);
    float global_dy = follow->data[2] * sinf(-pos_odom_001_yaw) + follow->data[3] * cosf(-pos_odom_001_yaw);
    mission_001.layout.dim.clear();
    mission_001.layout.dim.push_back(std_msgs::MultiArrayDimension());
    mission_001.layout.dim[0].label = "mission_001";
    mission_001.layout.dim[0].size = 11;
    mission_001.layout.dim[0].stride = 1;
    mission_001.data.resize(11);
    mission_001.data[0]=-pos_odom_001_yaw+follow->data[0];//rad
    mission_001.data[1]=0.0f;//rad/s
    mission_001.data[2]=pos_odom_001_x+global_dx;//x
    mission_001.data[3]=pos_odom_001_y+global_dy;//y
    mission_001.data[4]=0.0f;//z
    mission_001.data[5]=0.0f;//vx
    mission_001.data[6]=0.0f;//vy
    mission_001.data[7]=0.0f;//vz
    mission_001.data[8]=0.0f;//ax
    mission_001.data[9]=0.0f;//ay
    mission_001.data[10]=0.0f;//az
    mission_pub_001.publish(mission_001); 
  }else if(follow_down){
    get_pos_cmd=true;
    if(follow->data[2]==0.0f&&follow->data[3]==0.0f){
      printf("No tracking!\n");
      return;
    }
    float global_dx = follow->data[2] * cosf(-pos_odom_001_yaw) - follow->data[3] * sinf(-pos_odom_001_yaw);
    float global_dy = follow->data[2] * sinf(-pos_odom_001_yaw) + follow->data[3] * cosf(-pos_odom_001_yaw);
    mission_001.layout.dim.clear();
    mission_001.layout.dim.push_back(std_msgs::MultiArrayDimension());
    mission_001.layout.dim[0].label = "mission_001";
    mission_001.layout.dim[0].size = 11;
    mission_001.layout.dim[0].stride = 1;
    mission_001.data.resize(11);
    mission_001.data[0]=0.0f;//rad
    mission_001.data[1]=0.0f;//rad/s
    mission_001.data[2]=pos_odom_001_x+global_dx;//x
    mission_001.data[3]=pos_odom_001_y+global_dy;//y
    mission_001.data[4]=0.0f;//z
    mission_001.data[5]=0.0f;//vx
    mission_001.data[6]=0.0f;//vy
    mission_001.data[7]=0.0f;//vz
    mission_001.data[8]=0.0f;//ax
    mission_001.data[9]=0.0f;//ay
    mission_001.data[10]=0.0f;//az
    mission_pub_001.publish(mission_001);
  }
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "fcu_mission");
  ros::NodeHandle nh("~");

  // read mission_yaml parameter (path passed from launch)
  nh.param<std::string>("mission_yaml", mission_yaml_path, std::string(ros::package::getPath("fcu_core") + "/config/mission_points.yaml"));
  try {
    mission_yaml = YAML::LoadFile(mission_yaml_path);
    ROS_INFO("Loaded mission YAML: %s", mission_yaml_path.c_str());
  } catch (const std::exception &e) {
    ROS_WARN("Failed to load mission YAML '%s': %s", mission_yaml_path.c_str(), e.what());
  }

  ros::Subscriber comm=nh.subscribe<std_msgs::Int16>("/fcu_command/command", 100, cmdHandler);
  ros::Subscriber odom001=nh.subscribe<nav_msgs::Odometry>("odom_global_001", 100, odom_global001_handler);
  ros::Subscriber odom002=nh.subscribe<nav_msgs::Odometry>("odom_global_002", 100, odom_global002_handler);
  ros::Subscriber odom003=nh.subscribe<nav_msgs::Odometry>("odom_global_003", 100, odom_global003_handler);
  ros::Subscriber odom004=nh.subscribe<nav_msgs::Odometry>("odom_global_004", 100, odom_global004_handler);
  ros::Subscriber odom005=nh.subscribe<nav_msgs::Odometry>("odom_global_005", 100, odom_global005_handler);
  ros::Subscriber odom006=nh.subscribe<nav_msgs::Odometry>("odom_global_006", 100, odom_global006_handler);
  ros::Subscriber pos_cmd=nh.subscribe<quadrotor_msgs::PositionCommand>("pos_cmd", 100, pos_cmd_handler);
  ros::Subscriber mission_follow=nh.subscribe<std_msgs::Float32MultiArray>("mission_follow", 100, follow_handler);

  mission_pub_001 = nh.advertise<std_msgs::Float32MultiArray>("mission_001",100);
  mission_pub_002 = nh.advertise<std_msgs::Float32MultiArray>("mission_002",100);
  mission_pub_003 = nh.advertise<std_msgs::Float32MultiArray>("mission_003",100);
  mission_pub_004 = nh.advertise<std_msgs::Float32MultiArray>("mission_004",100);
  mission_pub_005 = nh.advertise<std_msgs::Float32MultiArray>("mission_005",100);
  mission_pub_006 = nh.advertise<std_msgs::Float32MultiArray>("mission_006",100);

  ros::Timer timer_mission_001 = nh.createTimer(ros::Duration(0.1),execute_mission_001,false);
  ros::Timer timer_mission_002 = nh.createTimer(ros::Duration(0.1),execute_mission_002,false);
  ros::Timer timer_mission_003 = nh.createTimer(ros::Duration(0.1),execute_mission_003,false);
  ros::Timer timer_mission_004 = nh.createTimer(ros::Duration(0.1),execute_mission_004,false);
  ros::Timer timer_mission_005 = nh.createTimer(ros::Duration(0.1),execute_mission_005,false);
  ros::Timer timer_mission_006 = nh.createTimer(ros::Duration(0.1),execute_mission_006,false);

  // For NNU mode
  static bool nnu_initialized = false;
  static std::vector<Pt> traj1, traj2, traj3;
  static size_t nnu_idx = 0;
  const float nnu_height = 1.0f; // 1m flight height for letters

  ros::Rate loop_rate(200);
  while (ros::ok()) {
    ros::spinOnce();

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(0, 0,0));
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform,  ros::Time::now(), "map", "uwb"));
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform,  ros::Time::now(), "map", "world"));

    if(enable_track){
      theta+=M_PI/20/200;

      px1=1.0*cosf(theta)+2;
      py1=1.0*sinf(theta)+2;
      pz1=0.6;

      px2=1.0*cosf(theta+M_PI*2/6)+2;
      py2=1.0*sinf(theta+M_PI*2/6)+2;
      pz2=0.6;

      px3=1.0*cosf(theta+M_PI*4/6)+2;
      py3=1.0*sinf(theta+M_PI*4/6)+2;
      pz3=0.6;

      px4=1.0*cosf(theta+M_PI*6/6)+2;
      py4=1.0*sinf(theta+M_PI*6/6)+2;
      pz4=0.6;

      px5=1.0*cosf(theta+M_PI*8/6)+2;
      py5=1.0*sinf(theta+M_PI*8/6)+2;
      pz5=0.6;

      px6=1.0*cosf(theta+M_PI*10/6)+2;
      py6=1.0*sinf(theta+M_PI*10/6)+2;
      pz6=0.6;
    } else if(enable_path){
      switch (path_track_status)
      {
        case ReadyToGoal:
          switch(goal_point)
          {
            case 0:
            SetGoal(1,0.5,0.5,0);
            break;
            case 1:
            SetGoal(1,1.0,1.0,0);
            break;
            case 2:
            SetGoal(1,1.5,1.0,0);
            break;
            case 3:
            SetGoal(1,2.0,1.0,0);
            break;
            case 4:
            SetGoal(1,2.5,1.0,0);
            break;
            case 5:
            SetGoal(1,2.5,2.0,0);
            break;
            case 6:
            SetGoal(1,2.0,2.0,0);
            break;
            case 7:
            SetGoal(1,1.5,2.0,0);
            break;
            case 8:
            SetGoal(1,1.0,2.0,0);
            break;
            case 9:
            SetGoal(1,0.5,2.0,0);
            break;
          }
          path_track_status = ExecutingGoal;
          break;
        case ExecutingGoal:
          if(IsReachGoal(1,0.1))
          {
            path_track_status = ReadyToGoal;
            goal_point++;
            break;
          }
          break;
        default:
          break;
      }
    } else {
      // normal enable_pos handling and our new NNU mode
      switch(enable_pos){
        case 0:
            px1=pos_takeoff_001_x; py1=pos_takeoff_001_y; pz1=0.0f;
            px2=pos_takeoff_002_x; py2=pos_takeoff_002_y; pz2=0.0f;
            px3=pos_takeoff_003_x; py3=pos_takeoff_003_y; pz3=0.0f;
            px4=pos_takeoff_004_x; py4=pos_takeoff_004_y; pz4=0.0f;
            px5=pos_takeoff_005_x; py5=pos_takeoff_005_y; pz5=0.0f;
            px6=pos_takeoff_006_x; py6=pos_takeoff_006_y; pz6=0.0f;
            // reset NNU init when leaving mode
            nnu_initialized = false;
            nnu_idx = 0;
            break;

        case 1:
            {
                float tx=1.0f, ty=1.0f, tz=1.0f;
                if(mission_yaml && mission_yaml["point1"] && mission_yaml["point1"].IsSequence() && mission_yaml["point1"].size()>=3){
                    try {
                        tx = mission_yaml["point1"][0].as<float>();
                        ty = mission_yaml["point1"][1].as<float>();
                        tz = mission_yaml["point1"][2].as<float>();
                        ROS_INFO_ONCE("[YAML] Successfully loaded point1: (%.2f, %.2f, %.2f)", tx, ty, tz);

                    } catch(...) {
                        ROS_WARN("mission_yaml: invalid point1 format, using fallback values");
                    }
                } else {
                    ROS_WARN_THROTTLE(5, "mission_yaml: point1 not found, using fallback coords");
                }
                px = tx; py = ty; pz = tz;
                px1=pos_takeoff_001_x+px; py1=pos_takeoff_001_y+py; pz1=pz;
                px2=pos_takeoff_002_x+px; py2=pos_takeoff_002_y+py; pz2=pz;
                px3=pos_takeoff_003_x+px; py3=pos_takeoff_003_y+py; pz3=pz;
                px4=pos_takeoff_004_x+px; py4=pos_takeoff_004_y+py; pz4=pz;
                px5=pos_takeoff_005_x+px; py5=pos_takeoff_005_y+py; pz5=pz;
                px6=pos_takeoff_006_x+px; py6=pos_takeoff_006_y+py; pz6=pz;
            }
            break;

        case 2:
            {
                float tx=1.0f, ty=-1.0f, tz=1.0f;
                if(mission_yaml && mission_yaml["point2"] && mission_yaml["point2"].IsSequence() && mission_yaml["point2"].size()>=3){
                    try {
                        tx = mission_yaml["point2"][0].as<float>();
                        ty = mission_yaml["point2"][1].as<float>();
                        tz = mission_yaml["point2"][2].as<float>();
                        ROS_INFO_ONCE("[YAML] Successfully loaded point2: (%.2f, %.2f, %.2f)", tx, ty, tz);
                    } catch(...) {
                        ROS_WARN("mission_yaml: invalid point2 format, using fallback values");
                    }
                } else {
                    ROS_WARN_THROTTLE(5, "mission_yaml: point2 not found, using fallback coords");
                }
                px = tx; py = ty; pz = tz;
                px1=pos_takeoff_001_x+px; py1=pos_takeoff_001_y+py; pz1=pz;
                px2=pos_takeoff_002_x+px; py2=pos_takeoff_002_y+py; pz2=pz;
                px3=pos_takeoff_003_x+px; py3=pos_takeoff_003_y+py; pz3=pz;
                px4=pos_takeoff_004_x+px; py4=pos_takeoff_004_y+py; pz4=pz;
                px5=pos_takeoff_005_x+px; py5=pos_takeoff_005_y+py; pz5=pz;
                px6=pos_takeoff_006_x+px; py6=pos_takeoff_006_y+py; pz6=pz;
            }
            break;

        case 3:
            {
                float tx=-1.0f, ty=-1.0f, tz=1.0f;
                if(mission_yaml && mission_yaml["point3"] && mission_yaml["point3"].IsSequence() && mission_yaml["point3"].size()>=3){
                    try {
                        tx = mission_yaml["point3"][0].as<float>();
                        ty = mission_yaml["point3"][1].as<float>();
                        tz = mission_yaml["point3"][2].as<float>();
                        ROS_INFO_ONCE("[YAML] Successfully loaded point3: (%.2f, %.2f, %.2f)", tx, ty, tz);
                    } catch(...) {
                        ROS_WARN("mission_yaml: invalid point3 format, using fallback values");
                    }
                } else {
                    ROS_WARN_THROTTLE(5, "mission_yaml: point3 not found, using fallback coords");
                }
                px = tx; py = ty; pz = tz;
                px1=pos_takeoff_001_x+px; py1=pos_takeoff_001_y+py; pz1=pz;
                px2=pos_takeoff_002_x+px; py2=pos_takeoff_002_y+py; pz2=pz;
                px3=pos_takeoff_003_x+px; py3=pos_takeoff_003_y+py; pz3=pz;
                px4=pos_takeoff_004_x+px; py4=pos_takeoff_004_y+py; pz4=pz;
                px5=pos_takeoff_005_x+px; py5=pos_takeoff_005_y+py; pz5=pz;
                px6=pos_takeoff_006_x+px; py6=pos_takeoff_006_y+py; pz6=pz;
            }
            break;

        case 4:
            {
                float tx=0.0f, ty=0.0f, tz=1.0f;
                if(mission_yaml && mission_yaml["point4"] && mission_yaml["point4"].IsSequence() && mission_yaml["point4"].size()>=3){
                    try {
                        tx = mission_yaml["point4"][0].as<float>();
                        ty = mission_yaml["point4"][1].as<float>();
                        tz = mission_yaml["point4"][2].as<float>();
                        ROS_INFO_ONCE("[YAML] Successfully loaded point4: (%.2f, %.2f, %.2f)", tx, ty, tz);
                    } catch(...) {
                        ROS_WARN("mission_yaml: invalid point4 format, using fallback values");
                    }
                } else {
                    ROS_WARN_THROTTLE(5, "mission_yaml: point4 not found, using fallback coords");
                }
                px = tx; py = ty; pz = tz;
                px1=pos_takeoff_001_x+px; py1=pos_takeoff_001_y+py; pz1=pz;
                px2=pos_takeoff_002_x+px; py2=pos_takeoff_002_y+py; pz2=pz;
                px3=pos_takeoff_003_x+px; py3=pos_takeoff_003_y+py; pz3=pz;
                px4=pos_takeoff_004_x+px; py4=pos_takeoff_004_y+py; pz4=pz;
                px5=pos_takeoff_005_x+px; py5=pos_takeoff_005_y+py; pz5=pz;
                px6=pos_takeoff_006_x+px; py6=pos_takeoff_006_y+py; pz6=pz;
            }
            break;

        // ===== 新增 NNU 模式（enable_pos == 7） =====
        case 7:
            {
                // Initialize trajectories once when entering mode
                if(!nnu_initialized){
                  nnu_initialized = true;
                  nnu_idx = 0;
                  traj1.clear(); traj2.clear(); traj3.clear();

                  // Parameters for letter shapes (tweakable)
                  float letter_height = 1.0f;    // vertical letter height (meters)
                  float letter_width  = 0.8f;    // horizontal width of letter
                  int stroke_samples = 18;       // controls smoothness per stroke
                  int bottom_samples = 24;

                  // Generate letter shapes in their local letter frame (x right, y up)
                  std::vector<Pt> N_local = gen_N(letter_height, letter_width, stroke_samples);
                  std::vector<Pt> N_local2 = gen_N(letter_height, letter_width, stroke_samples);
                  std::vector<Pt> U_local = gen_U(letter_height, letter_width, stroke_samples, bottom_samples);

                  // Make them vertical (user asked "竖着放"). We'll align them along y-axis (vertical) already.
                  // Position offsets so three letters are side-by-side vertically centered relative to takeoff.
                  // We want the N N U stacked left-to-right but "竖着放" means letters oriented vertically already.
                  // Place centers at offsets along X (right) direction: -1.0, 0.0, +1.0 (meters)
                  float x_off1 = -1.0f; // UAV1 left letter
                  float x_off2 =  0.0f; // UAV2 middle letter
                  float x_off3 =  1.0f; // UAV3 right letter

                  // Increase per-letter sampling density by duplicating with small interpolation to get smoother path
                  auto densify = [](const std::vector<Pt>& in, int factor)->std::vector<Pt>{
                    std::vector<Pt> out;
                    if(in.empty()) return out;
                    for(size_t i=0;i<in.size()-1;i++){
                      Pt a = in[i];
                      Pt b = in[i+1];
                      for(int k=0;k<factor;k++){
                        float t = (float)k/factor;
                        out.push_back({ a.x + (b.x-a.x)*t, a.y + (b.y-a.y)*t, a.z + (b.z-a.z)*t });
                      }
                    }
                    out.push_back(in.back());
                    return out;
                  };

                  traj1 = densify(N_local, 3); // UAV1 draw N
                  traj2 = densify(N_local2, 3); // UAV2 draw N (same shape)
                  traj3 = densify(U_local, 3); // UAV3 draw U

                  // Translate into relative positions from takeoff
                  for(auto &p : traj1){ p.x += x_off1; }
                  for(auto &p : traj2){ p.x += x_off2; }
                  for(auto &p : traj3){ p.x += x_off3; }

                  // ensure minimum length and synchronized length possibility:
                  size_t maxlen = std::max({traj1.size(), traj2.size(), traj3.size()});
                  // Pad shorter ones by repeating last element (to keep UAVs in sync)
                  auto pad_to = [](std::vector<Pt>& v, size_t N){
                    while(v.size()<N) v.push_back(v.back());
                  };
                  pad_to(traj1, maxlen);
                  pad_to(traj2, maxlen);
                  pad_to(traj3, maxlen);

                  ROS_INFO("[NNU] Generated trajectories: len1=%zu len2=%zu len3=%zu", traj1.size(), traj2.size(), traj3.size());
                } // end init

                // If NNU finished, just hold last point
                if(nnu_idx >= traj1.size()){
                  // hold at final points (relative to takeoff)
                  px1 = pos_takeoff_001_x + traj1.back().x; py1 = pos_takeoff_001_y + traj1.back().y; pz1 = nnu_height;
                  px2 = pos_takeoff_002_x + traj2.back().x; py2 = pos_takeoff_002_y + traj2.back().y; pz2 = nnu_height;
                  px3 = pos_takeoff_003_x + traj3.back().x; py3 = pos_takeoff_003_y + traj3.back().y; pz3 = nnu_height;
                } else {
                  // step through trajectories
                  Pt p1 = traj1[nnu_idx];
                  Pt p2 = traj2[nnu_idx];
                  Pt p3 = traj3[nnu_idx];

                  px1 = pos_takeoff_001_x + p1.x; py1 = pos_takeoff_001_y + p1.y; pz1 = nnu_height;
                  px2 = pos_takeoff_002_x + p2.x; py2 = pos_takeoff_002_y + p2.y; pz2 = nnu_height;
                  px3 = pos_takeoff_003_x + p3.x; py3 = pos_takeoff_003_y + p3.y; pz3 = nnu_height;

                  // increment index at a controlled rate: we run loop 200Hz, but we want slower motion
                  // advance every N loops: define rate divider
                  static int counter = 0;
                  const int advance_every = 6; // 200Hz/6 ~ 33Hz update -> smooth
                  counter++;
                  if(counter >= advance_every){
                    counter = 0;
                    nnu_idx++;
                  }
                }

                // keep 4/5/6 at hover at their takeoff positions (height 0 in your original default; but we can leave Z as 0)
                px4 = pos_takeoff_004_x; py4 = pos_takeoff_004_y; pz4 = pos_takeoff_004_z;
                px5 = pos_takeoff_005_x; py5 = pos_takeoff_005_y; pz5 = pos_takeoff_005_z;
                px6 = pos_takeoff_006_x; py6 = pos_takeoff_006_y; pz6 = pos_takeoff_006_z;
            }
            break;

        default:
            break;
      } // end switch(enable_pos)
    } // end else(not track/path)

    loop_rate.sleep();
  }
  return 0;
}


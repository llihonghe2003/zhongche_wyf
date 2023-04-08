#include <ros/console.h>
#include <ros/ros.h>
#include <vector>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "task_msgs/Task.h"
#include "task_msgs/TaskArray.h"

#include <Python.h>  //前面所做的一切配置都是为了调用这个头文件和相关库
#include "centralised_auction/sequential_auction.h"

#include <thread>

using geometry_msgs::Pose;
using geometry_msgs::PoseArray;
using std::cout;
using std::endl;
using std::string;
using std::vector;
using task_msgs::Task;
using task_msgs::TaskArray;

vector<Task> unallocated_tasks;
vector<Pose> robot_poses;

vector<TaskArray> allocated_tasks;
vector<vector<double>> alloction_results;
vector<ros::Publisher> pubs;
int num_robots;
int seq = 0;
string prefix;
bool return_home;

/**************************************************
 * Helper functions
 **************************************************/
int call_pyfunc() {
  Py_Initialize();
  PyRun_SimpleString("import sys");
  PyRun_SimpleString("import os");

  PyRun_SimpleString("print (sys.path[0])");
  PyRun_SimpleString("print (os.path.abspath('.'))");
  // std::string dirs =
  //     std::string("sys.path.append('/home/xtark/RXY_ws/src/hello_vscode/src')");
  std::string dirs = std::string(
      "sys.path.append('/home/xtark/RXY_ws/src/task_allocation-master/"
      "centralised_auction/scripts')");
  // std::string("sys.path.append('../scripts')");
  PyRun_SimpleString("print (sys.path[0])");
  PyRun_SimpleString(dirs.c_str());
  // PyObject *module = PyString_FromString("drawRef");
  // PyObject *pyModule = PyImport_Import(module);
  PyObject *pyModule = PyImport_ImportModule("drawRef");
  if (pyModule == nullptr)  // failed
  {
    std::cout << "Get python module failed." << std::endl;
    return 0;
  }
  PyObject *pyFunc = PyObject_GetAttrString(pyModule, "main");
  if (!pyFunc || !PyCallable_Check(pyFunc)) {
    std::cout << "Cannot find python function" << std::endl;
    return 0;
  } else {
    std::cout << "load python funtion ok"
              << "\n";
    PyObject *pyParams = PyList_New(0);
    PyObject *args = PyTuple_New(1);

    vector<vector<double>>::iterator iter;
    for (iter = alloction_results.begin(); iter != alloction_results.end();
         ++iter) {
      PyObject *PyList1 = PyList_New(0);

      for (int i = 0; i < (*iter).size(); ++i) {
        cout << (*iter)[i] << " ";
        PyList_Append(PyList1, Py_BuildValue("d", (*iter)[i]));
      }
      PyList_Append(
          pyParams,
          PyList1);  // PyList_Append可以参考Python中，list的append的用处
    }
    PyTuple_SetItem(args, 0, pyParams);
    PyObject *pyRes = PyEval_CallObject(pyFunc, args);
  }
  // sleep(30);
  Py_Finalize();
}

/**************************************************
 * Publisher functions
 **************************************************/

/**************************************************
 *
 **************************************************/

void allocateTasks() {
  if (num_robots <= 0) {
    cout << "No robots specified." << endl;
    return;
  }
  if (unallocated_tasks.size() <= 0) {
    cout << "No tasks specified." << endl;
    return;
  }

  cout << "\nPrepping task allocation. " << endl;
  SequentialAuction solver(unallocated_tasks, robot_poses);
  solver.use_least_contested_bid = false;
  if (return_home) solver.return_home = true;
  cout << "\nAllocating tasks." << endl;
  // allocated_tasks = solver.allocateTasks();
  alloction_results = solver.allocateTasks();
  //迭代器遍历
  vector<vector<double>>::iterator iter;
  for (iter = alloction_results.begin(); iter != alloction_results.end();
       ++iter) {
    for (int i = 0; i < (*iter).size(); ++i) {
      cout << (*iter)[i] << " ";
    }
    cout << endl;
  }
  cout << "\nFinished allocating." << endl;

  ros::Duration(2).sleep();
}

/**************************************************
 * Initialising functions
 **************************************************/

// Loads parameters from the launch file. Uses default values if any values are
// missing.
void loadParams(ros::NodeHandle n_priv) {
  vector<double> waypoint_values;  // position (x,y,z) and orientation (x,y,z,w)
  vector<double> robot_values;     // position (x,y,z)

  // Set default parameters.
  double default_waypoint_values[] = {};
  double default_robot_values[] = {};
  string default_prefix = "robot";
  bool default_return_home = false;

  n_priv.param("return_home", return_home, default_return_home);
  n_priv.param("prefix", prefix, default_prefix);

  // Check parameter server to override defaults.
  XmlRpc::XmlRpcValue v;
  if (n_priv.getParam("tasks", v)) {
    ROS_ASSERT(v.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int i = 0; i < v.size(); i++) {
      if (v[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        waypoint_values.push_back(v[i]);
      }
      if (v[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
        int d = v[i];
        waypoint_values.push_back(d);
      }
    }
  } else {
    waypoint_values.assign(
        default_waypoint_values,
        default_waypoint_values + sizeof(default_waypoint_values) /
                                      sizeof(default_waypoint_values[0]));
  }
  // Convert waypoint values into waypoints.
  if (waypoint_values.size() % 7 != 0) {
    cout << "INCORRECT NUMBER OF WAYPOINT VALUES" << endl;
    return;
  }
  system("clear");
  cout << "当前任务:" << endl;
  for (int i = 0; i < waypoint_values.size(); i += 7) {
    cout << "\ttask" << i / 7 << ":[" << waypoint_values[i] << " "
         << waypoint_values[i + 1] << " " << waypoint_values[i + 2] << "]"
         << endl;
    Task t;
    t.pose.position.x = waypoint_values[i];
    t.pose.position.y = waypoint_values[i + 1];
    t.pose.position.z = waypoint_values[i + 2];
    t.pose.orientation.x = waypoint_values[i + 3];
    t.pose.orientation.y = waypoint_values[i + 4];
    t.pose.orientation.z = waypoint_values[i + 5];
    t.pose.orientation.w = waypoint_values[i + 6];
    unallocated_tasks.push_back(t);
  }
  // sleep(1);
  // Check parameter server to override defaults.
  // XmlRpc::XmlRpcValue v;
  if (n_priv.getParam("robots", v)) {
    ROS_ASSERT(v.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int i = 0; i < v.size(); i++) {
      if (v[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        robot_values.push_back(v[i]);
      }
      if (v[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
        int d = v[i];
        robot_values.push_back(d);
      }
    }
  } else {
    robot_values.assign(
        default_robot_values,
        default_robot_values +
            sizeof(default_robot_values) / sizeof(default_robot_values[0]));
  }
  // Convert waypoint values into waypoints.
  if (robot_values.size() % 3 != 0) {
    cout << "INCORRECT NUMBER OF ROBOT VALUES" << endl;
    return;
  }
  cout << "\n当前机器人:" << endl;
  for (int i = 0; i < robot_values.size(); i += 3) {
    cout << "\t机器人" << i / 3 << ":[" << robot_values[i] << " "
         << robot_values[i + 1] << " " << robot_values[i + 2] << "]" << endl;
    Pose p;
    p.position.x = robot_values[i];
    p.position.y = robot_values[i + 1];
    p.position.z = robot_values[i + 2];
    robot_poses.push_back(p);
  }
  // sleep(3);
  num_robots = robot_poses.size();
  for (int i = 0; i < num_robots; i++) {
    TaskArray ta;
    allocated_tasks.push_back(ta);
  }
}

/**************************************************
 * Main
 **************************************************/
int main(int argc, char **argv) {
  ros::init(argc, argv, "centralised_auction");
  ros::NodeHandle n;
  ros::NodeHandle n_priv("~");

  loadParams(n_priv);

  allocateTasks();
  // std::thread t1(call_pyfunc);
  // t1.join();
  call_pyfunc();
  // execlp("rosrun", "rosrun", "planning_globaltrack", "planning_globaltrack",
  // NULL); sleep(1); execlp("rosrun", "rosrun", "simuctrl", "simuctrl", NULL);
  sleep(1);
  execlp("roslaunch", "roslaunch", "centralised_auction", "start.launch", NULL);
  cout << "1231213121212132121321213213212121321321321";

  ros::Rate r(1);
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
};

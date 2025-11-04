opt/ros/noetic/include/ros/node_handle.h:1828:8: note: candidate: ‘bool ros::NodeHandle::getParam(const string&, std::map<std::__cxx11::basic_string<char>, int>&) const’
 1828 |   bool getParam(const std::string& key, std::map<std::string, int>& map) const;
      |        ^~~~~~~~
/opt/ros/noetic/include/ros/node_handle.h:1828:8: note:   candidate expects 2 arguments, 3 provided
/opt/ros/noetic/include/ros/node_handle.h:1839:8: note: candidate: ‘bool ros::NodeHandle::getParam(const string&, std::map<std::__cxx11::basic_string<char>, bool>&) const’
 1839 |   bool getParam(const std::string& key, std::map<std::string, bool>& map) const;
      |        ^~~~~~~~
/opt/ros/noetic/include/ros/node_handle.h:1839:8: note:   candidate expects 2 arguments, 3 provided
/home/a/catkin/src/mission_master_pkg/lib/mission_master.cpp:129:62: error: no matching function for call to ‘ros::NodeHandle::getParam(const char [19], double&, double)’
  129 |     nh.getParam("trace_start_pose_z", TRACE_START_POSE_Z, 0.0);
      |                                                              ^
In file included from /opt/ros/noetic/include/ros/ros.h:45,
                 from /home/a/catkin/src/mission_master_pkg/include/mission_master_pkg/mission_master.h:13,
                 from /home/a/catkin/src/mission_master_pkg/lib/mission_master.cpp:6:
/opt/ros/noetic/include/ros/node_handle.h:1672:8: note: candidate: ‘bool ros::NodeHandle::getParam(const string&, std::string&) const’
 1672 |   bool getParam(const std::string& key, std::string& s) const;
      |        ^~~~~~~~
/opt/ros/noetic/include/ros/node_handle.h:1672:8: note:   candidate expects 2 arguments, 3 provided
/opt/ros/noetic/include/ros/node_handle.h:1683:8: note: candidate: ‘bool ros::NodeHandle::getParam(const string&, double&) const’
 1683 |   bool getParam(const std::string& key, double& d) const;
      |        ^~~~~~~~
/opt/ros/noetic/include/ros/node_handle.h:1683:8: note:   candidate expects 2 arguments, 3 provided
/opt/ros/noetic/include/ros/node_handle.h:1694:8: note: candidate: ‘bool ros::NodeHandle::getParam(const string&, float&) const’
 1694 |   bool getParam(const std::string& key, float& f) const;
      |        ^~~~~~~~
/opt/ros/noetic/include/ros/node_handle.h:1694:8: note:   candidate expects 2 arguments, 3 provided
/opt/ros/noetic/include/ros/node_handle.h:1705:8: note: candidate: ‘bool ros::NodeHandle::getParam(const string&, int&) const’
 1705 |   bool getParam(const std::string& key, int& i) const;
      |        ^~~~~~~~
/opt/ros/noetic/include/ros/node_handle.h:1705:8: note:   candidate expects 2 arguments, 3 provided
/opt/ros/noetic/include/ros/node_handle.h:1716:8: note: candidate: ‘bool ros::NodeHandle::getParam(const string&, bool&) const’
 1716 |   bool getParam(const std::string& key, bool& b) const;
      |        ^~~~~~~~
/opt/ros/noetic/include/ros/node_handle.h:1716:8: note:   candidate expects 2 arguments, 3 provided
/opt/ros/noetic/include/ros/node_handle.h:1727:8: note: candidate: ‘bool ros::NodeHandle::getParam(const string&, XmlRpc::XmlRpcValue&) const’
 1727 |   bool getParam(const std::string& key, XmlRpc::XmlRpcValue& v) const;
      |        ^~~~~~~~
/opt/ros/noetic/include/ros/node_handle.h:1727:8: note:   candidate expects 2 arguments, 3 provided
/opt/ros/noetic/include/ros/node_handle.h:1739:8: note: candidate: ‘bool ros::NodeHandle::getParam(const string&, std::vector<std::__cxx11::basic_string<char> >&) const’
 1739 |   bool getParam(const std::string& key, std::vector<std::string>& vec) const;
      |        ^~~~~~~~
/opt/ros/noetic/include/ros/node_handle.h:1739:8: note:   candidate expects 2 arguments, 3 provided
/opt/ros/noetic/include/ros/node_handle.h:1750:8: note: candidate: ‘bool ros::NodeHandle::getParam(const string&, std::vector<double>&) const’
 1750 |   bool getParam(const std::string& key, std::vector<double>& vec) const;
      |        ^~~~~~~~
/opt/ros/noetic/include/ros/node_handle.h:1750:8: note:   candidate expects 2 arguments, 3 provided
/opt/ros/noetic/include/ros/node_handle.h:1761:8: note: candidate: ‘bool ros::NodeHandle::getParam(const string&, std::vector<float>&) const’
 1761 |   bool getParam(const std::string& key, std::vector<float>& vec) const;
      |        ^~~~~~~~
/opt/ros/noetic/include/ros/node_handle.h:1761:8: note:   candidate expects 2 arguments, 3 provided
/opt/ros/noetic/include/ros/node_handle.h:1772:8: note: candidate: ‘bool ros::NodeHandle::getParam(const string&, std::vector<int>&) const’
 1772 |   bool getParam(const std::string& key, std::vector<int>& vec) const;
      |        ^~~~~~~~
/opt/ros/noetic/include/ros/node_handle.h:1772:8: note:   candidate expects 2 arguments, 3 provided
/opt/ros/noetic/include/ros/node_handle.h:1783:8: note: candidate: ‘bool ros::NodeHandle::getParam(const string&, std::vector<bool>&) const’
 1783 |   bool getParam(const std::string& key, std::vector<bool>& vec) const;
      |        ^~~~~~~~
/opt/ros/noetic/include/ros/node_handle.h:1783:8: note:   candidate expects 2 arguments, 3 provided
/opt/ros/noetic/include/ros/node_handle.h:1795:8: note: candidate: ‘bool ros::NodeHandle::getParam(const string&, std::map<std::__cxx11::basic_string<char>, std::__cxx11::basic_string<char> >&) const’
 1795 |   bool getParam(const std::string& key, std::map<std::string, std::string>& map) const;
      |        ^~~~~~~~
/opt/ros/noetic/include/ros/node_handle.h:1795:8: note:   candidate expects 2 arguments, 3 provided
/opt/ros/noetic/include/ros/node_handle.h:1806:8: note: candidate: ‘bool ros::NodeHandle::getParam(const string&, std::map<std::__cxx11::basic_string<char>, double>&) const’
 1806 |   bool getParam(const std::string& key, std::map<std::string, double>& map) const;
      |        ^~~~~~~~
/opt/ros/noetic/include/ros/node_handle.h:1806:8: note:   candidate expects 2 arguments, 3 provided
/opt/ros/noetic/include/ros/node_handle.h:1817:8: note: candidate: ‘bool ros::NodeHandle::getParam(const string&, std::map<std::__cxx11::basic_string<char>, float>&) const’
 1817 |   bool getParam(const std::string& key, std::map<std::string, float>& map) const;
      |        ^~~~~~~~
/opt/ros/noetic/include/ros/node_handle.h:1817:8: note:   candidate expects 2 arguments, 3 provided
/opt/ros/noetic/include/ros/node_handle.h:1828:8: note: candidate: ‘bool ros::NodeHandle::getParam(const string&, std::map<std::__cxx11::basic_string<char>, int>&) const’
 1828 |   bool getParam(const std::string& key, std::map<std::string, int>& map) const;
      |        ^~~~~~~~
/opt/ros/noetic/include/ros/node_handle.h:1828:8: note:   candidate expects 2 arguments, 3 provided
/opt/ros/noetic/include/ros/node_handle.h:1839:8: note: candidate: ‘bool ros::NodeHandle::getParam(const string&, std::map<std::__cxx11::basic_string<char>, bool>&) const’
 1839 |   bool getParam(const std::string& key, std::map<std::string, bool>& map) const;
      |        ^~~~~~~~
/opt/ros/noetic/include/ros/node_handle.h:1839:8: note:   candidate expects 2 arguments, 3 provided
/home/a/catkin/src/mission_master_pkg/lib/mission_master.cpp:130:58: error: no matching function for call to ‘ros::NodeHandle::getParam(const char [17], double&, double)’
  130 |     nh.getParam("trace_end_pose_x", TRACE_END_POSE_X, 0.0);
      |                                                          ^
In file included from /opt/ros/noetic/include/ros/ros.h:45,
                 from /home/a/catkin/src/mission_master_pkg/include/mission_master_pkg/mission_master.h:13,
                 from /home/a/catkin/src/mission_master_pkg/lib/mission_master.cpp:6:
/opt/ros/noetic/include/ros/node_handle.h:1672:8: note: candidate: ‘bool ros::NodeHandle::getParam(const string&, std::string&) const’
 1672 |   bool getParam(const std::string& key, std::string& s) const;
      |        ^~~~~~~~
/opt/ros/noetic/include/ros/node_handle.h:1672:8: note:   candidate expects 2 arguments, 3 provided
/opt/ros/noetic/include/ros/node_handle.h:1683:8: note: candidate: ‘bool ros::NodeHandle::getParam(const string&, double&) const’
 1683 |   bool getParam(const std::string& key, double& d) const;
      |        ^~~~~~~~
/opt/ros/noetic/include/ros/node_handle.h:1683:8: note:   candidate expects 2 arguments, 3 provided
/opt/ros/noetic/include/ros/node_handle.h:1694:8: note: candidate: ‘bool ros::NodeHandle::getParam(const string&, float&) const’
 1694 |   bool getParam(const std::string& key, float& f) const;
      |        ^~~~~~~~
/opt/ros/noetic/include/ros/node_handle.h:1694:8: note:   candidate expects 2 arguments, 3 provided
/opt/ros/noetic/include/ros/node_handle.h:1705:8: note: candidate: ‘bool ros::NodeHandle::getParam(const string&, int&) const’
 1705 |   bool getParam(const std::string& key, int& i) const;
      |        ^~~~~~~~
/opt/ros/noetic/include/ros/node_handle.h:1705:8: note:   candidate expects 2 arguments, 3 provided
/opt/ros/noetic/include/ros/node_handle.h:1716:8: note: candidate: ‘bool ros::NodeHandle::getParam(const string&, bool&) const’
 1716 |   bool getParam(const std::string& key, bool& b) const;
      |        ^~~~~~~~
/opt/ros/noetic/include/ros/node_handle.h:1716:8: note:   candidate expects 2 arguments, 3 provided
/opt/ros/noetic/include/ros/node_handle.h:1727:8: note: candidate: ‘bool ros::NodeHandle::getParam(const string&, XmlRpc::XmlRpcValue&) const’
 1727 |   bool getParam(const std::string& key, XmlRpc::XmlRpcValue& v) const;


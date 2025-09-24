//点云处理模块声明
PointcloudProcessor(ros::NodeHandle& nh)
void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
void filterPointcloud()
void transformToBodyFrame()
void generatePolarHistogram()
const PolarHistogram& getHistogram()
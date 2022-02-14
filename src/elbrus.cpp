#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "SlamData.h"
#include "TcpReceiver.h"


/* transformPoints: Функция преобразует точки карты в систему координат, 
 * связанную с роботом. */
void transformPoints(
  const std::vector<tf::Vector3>& allpts, // массив точек СК земли
  tf::Vector3 camera_translation,  // позиция камеры робота на карте
  tf::Quaternion camera_rotation,  // ориентация камеры робота на карте
  std::vector<tf::Vector3>& refpts // массив точек в СК земли
  )
{
  // Смещение камеры относительно начала СК карты
  tf::Transform camera_transform(camera_rotation, camera_translation);
  tf::Transform robot_transform = camera_transform.inverse();
  // Смещение камеры относительно лидара
  tf::Quaternion lidar_rotation;
    double calibAngleRad = 0.0 * M_PI; 
    lidar_rotation[0] = 0; // будет поворот вокруг оси Z
    lidar_rotation[1] = 0;
    lidar_rotation[2] = sin(0.5 * calibAngleRad);
    lidar_rotation[3] = cos(0.5 * calibAngleRad);
  tf::Vector3 lidar_translation(0, 0, 0);
  tf::Transform lidar_transform(lidar_rotation, lidar_translation);
  // Полное смещение камеры
  tf::Transform full_transform = lidar_transform * robot_transform;
  // Выполняем преобразование координат точек к СК лидара
  size_t npts = allpts.size();
  refpts.clear();
  refpts.reserve(npts);
  tf::Vector3 refpt;
  for (size_t i = 0; i < npts; i++)
  {
    refpt = full_transform * allpts[i];
    float angleRad = atan2(refpt[1], refpt[0]);
    if (abs(refpt[2]) < 0.5)
      refpts.push_back(refpt);
  }

}

/* mergePoints: Функция объединяет карту, полученную от камеры, с картой,
 * полученной от лазерного сканера. */
void mergePoints(const sensor_msgs::LaserScan::ConstPtr& scan, 
  std::vector<tf::Vector3>& refpts,
  sensor_msgs::LaserScanPtr& scanMerged)
{
  size_t npts = refpts.size();
  *scanMerged = *scan;
  for (size_t ipt = 0; ipt < npts; ipt++)
  {
    tf::Vector3 refpt = refpts[ipt];
    // Расчет угла направления на точку
    float angleRad = atan2(refpt[1], refpt[0]);
    while (angleRad > scan->angle_max) angleRad -= 2*M_PI;
    while (angleRad < scan->angle_min) angleRad += 2*M_PI;
    size_t iray = // Индекс луча лазера, соответствующего данной точке
      floor((angleRad - scan->angle_min) / scan->angle_increment);
    // Расчет дальности до точки
    float rangeNew2 = refpt[1]*refpt[1] + refpt[0]*refpt[0];
    float rangeCurr = scanMerged->ranges[iray];
    if (rangeCurr > scan->range_min && 
        rangeCurr < scan->range_max &&
        rangeCurr*rangeCurr > rangeNew2)
    {
      /* Оставляем значение дальности лидара */
    }
    else
    {
      /* Заменяем дальность лидара дальностью по видео */
      scanMerged->ranges[iray] = sqrt(rangeNew2);
    }
  }
}


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "elbrus");
  ROS_INFO("Starting elbrus node...");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  SlamData publisher(&n);  // Компонент для публикации данных в топики
  TcpReceiver receiver;    // Компонент для получения данных по TCP/IP

  if (!receiver.Init(false))
  {
    ROS_ERROR("Failed to init TCP socket connection!\n");
    return -1;
  }

  ros::Rate loop_rate(20); // Прием сообщения будет происходит с частотой 10 Гц

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  tf::Vector3 origin;       // позиция камеры
  tf::Quaternion rotation;  // ориентация камеры
  std::vector<tf::Vector3> allpts;   // главное облако точек карты
  std::vector<tf::Vector3> refpts;   // вспомогательное облако точек карты
  ros::Time timestamp;      // временная метка
  while (ros::ok())
  {
    // Проверяем, не пришли ли новые данные
    if (receiver.GetData(origin, rotation, allpts))
    {
        // Получено новое облако точек - 
        timestamp = ros::Time::now();
        publisher.PublishTFForROS(origin, rotation, timestamp);
        publisher.PublishPoseForROS(origin, rotation, timestamp);
        // Создаем облако точек в системе координат, связанной с роботом
        transformPoints(allpts, origin, rotation, refpts);
        // Публикуем облака точек в ROS
        publisher.PublishPointCloudForROS(allpts, refpts, timestamp);
    }
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    //std_msgs::String msg;

    //std::stringstream ss;
    //ss << "hello world " << count;
    //msg.data = ss.str();

    //ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    //chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}



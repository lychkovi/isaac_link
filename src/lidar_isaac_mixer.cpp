// lidar_isaac_mixer.cpp: Программа объединяет данные лидара с данными
// визуальной одометрии.

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>

#include "SlamData.h"
#include "TcpReceiver.h"
#include "TripleBufManager.h"

ros::Publisher scan_pub;

// AsyncBuffer: Структура для передачи данных между главным потом и 
// обработчиком сообщения. 
typedef struct AsyncBufferStruct
{
  tf::Vector3 origin;       // позиция камеры
  tf::Quaternion rotation;  // ориентация камеры
  std::vector<tf::Vector3> allpts;   // главное облако точек карты
  std::vector<tf::Vector3> refpts;   // вспомогательное облако точек карты
  ros::Time timestamp;      // временная метка
  bool valid;
} AsyncBuffer;

// Объект для управления асинхронной передачей данных.
TripleBufManager<AsyncBuffer> buffers;


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
  int tolerance, // диаметр зоны влияния точки (выражено в лучах лазера)
  sensor_msgs::LaserScanPtr& scanMerged)
{
  size_t nrays = scan->ranges.size();
  size_t npts = refpts.size();
  *scanMerged = *scan;
  for (size_t ipt = 0; ipt < npts; ipt++)
  {
    tf::Vector3 refpt = refpts[ipt];
    // Расчет угла направления на точку
    float angleRad = atan2(refpt[1], refpt[0]);
    while (angleRad > scan->angle_max) angleRad -= 2*M_PI;
    while (angleRad < scan->angle_min) angleRad += 2*M_PI;
    size_t iray = // индекс луча лазера, соответствующего данной точке
      floor((angleRad - scan->angle_min) / scan->angle_increment); 
    // Расчет дальности до точки
    float rangeNew2 = refpt[1]*refpt[1] + refpt[0]*refpt[0];
    float rangeCurr = scanMerged->ranges[iray];
    if (rangeCurr > scan->range_min && 
        rangeCurr < scan->range_max &&
        rangeCurr*rangeCurr < rangeNew2)
    {
      /* Оставляем значение дальности лидара */
    }
    else
    {
      /* Заменяем дальность лидара дальностью по видео */
      size_t irayMin = // индекс луча лазера для левой границы зоны влияния
        (iray - tolerance < 0) ? 0 : iray - tolerance;
      size_t irayMax = // индекс луча лазера для правой границы зоны влияния
        (iray + tolerance >= nrays) ? nrays - 1 : iray + tolerance;
      double distance = sqrt(rangeNew2);
      for (size_t i = irayMin; i <= irayMax; i++)
        scanMerged->ranges[i] = distance;
    }
  }
}


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan_0)
{
  sensor_msgs::LaserScanPtr scan(new sensor_msgs::LaserScan);
  AsyncBuffer* async;
  //ROS_INFO("I heard: laser scan with %d measurements!", 
  //  (int)scan_0->ranges.size());
  //*scan = *scan_0;
  //for (size_t i = 0; i < scan->ranges.size(); i++)
  //  scan->ranges[i] = 0.5 * scan->ranges[i];
  buffers.SwapBufTracker(); // если буфер не переключится, то вернет 
  async = buffers.GetBufTracker(); // последнее сообщение
  if (async)
  {
    mergePoints(scan_0, async->refpts, 2, scan);
  }
  else
  {
    *scan = *scan_0;
  }
  scan_pub.publish(scan);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_isaac_mixer");
  ROS_INFO("Started node lidar_isaac_mixer.");
  ros::NodeHandle n;
  
  scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 50);
  ros::Subscriber scan_sub = n.subscribe("scan_0", 50, laserCallback);

  SlamData publisher(&n);  // Компонент для публикации данных VSLAM в топики
  TcpReceiver receiver;    // Компонент для получения данных по TCP/IP

  if (!receiver.Init(false))
  {
    ROS_ERROR("Failed to init TCP socket connection!\n");
    return -1;
  }

  ros::Rate loop_rate(20); // Прием сообщения будет происходит с частотой 10 Гц

  int count = 0;
  AsyncBuffer* async = buffers.GetBufCamera();
  while (ros::ok() && async)
  {
    // Проверяем, не пришли ли новые данные
    if (receiver.GetData(async->origin, async->rotation, async->allpts))
    {
      // Получено новое облако точек - 
      async->timestamp = ros::Time::now();
      publisher.PublishTFForROS(
          async->origin, async->rotation, async->timestamp);
      publisher.PublishPoseForROS(
          async->origin, async->rotation, async->timestamp);
      // Создаем облако точек в системе координат, связанной с роботом
      transformPoints(
          async->allpts, async->origin, async->rotation, async->refpts);
      // Публикуем облака точек в ROS
      publisher.PublishPointCloudForROS(
          async->allpts, async->refpts, async->timestamp);
      // Переключаемся на следующий буфер
      buffers.SwapBufCamera();
      async = buffers.GetBufCamera();
    }
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}



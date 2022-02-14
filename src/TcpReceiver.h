// TcpReceiver.h: Реализация класса, получающего данные от алгоритма Elbrus
// через TCP сокет.

#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <vector>

extern "C"
{
  #include "msg_conn.h"  // работа с сообщениями от ISAAC
}


class TcpReceiver
{
private:
    MsgConnConfig cfg;
    MsgConn conn;

public:
    // Метод инициализации (вызвать один раз в начале)
    bool Init(bool useLocalSocket)
    {
        // Инициализируем структуру конфигурации соединения
        if (useLocalSocket)
        {
            cfg.connRole = MsgConnRoleLocalReceiver;
            strncpy(cfg.servername, "/tmp/my_isaac_receiver", 
                sizeof(cfg.servername));
            cfg.portno = -1;
        }
        else
        {
            cfg.connRole = MsgConnRoleTcpReceiver;
            strncpy(cfg.servername, "localhost", sizeof("localhost"));
            cfg.portno = 10100;
        }
        cfg.mtu = 1460*10;      // максимальный размер одного пакета
        cfg.maxListLength = 10; // максимальная длина очереди сообщений

        // Инициализируем объект соединения
        if (!MsgConnInit(&conn, &cfg))
        {
            printf("Failed to init server connection!\n");
            return false;
        }
        return true;
    }

    // Метод получения данных из TCP сокета
    bool GetData(
        tf::Vector3& origin,       // позиция камеры
        tf::Quaternion& rotation,  // ориентация камеры
        std::vector<tf::Vector3>& pts   // облако точек карты
    )
    {
        // В цикле принимаем одно сообщение
        size_t npts = 0;
        float* ptr = NULL;
        tf::Vector3 pt(0, 0, 0);
        MsgBuffer* pbuf = NULL; // указатель на буфер сообщения
        bool needToExit = false;
        while (!needToExit)
        {
            if (MsgConnReceive(&conn, &pbuf))
            {
                printf("Message no. %04d received!\n", (int)pbuf->msgIndex);
                MsgHeader* msg = (MsgHeader*) pbuf->data;
                
                // Формируем структуры данных для ROS
                switch (msg->type)
                {
                case MsgTypePointCloud: // Облако точек
                    origin.setValue(
                        msg->uni.cloud.translation[0], 
                        msg->uni.cloud.translation[1], 
                        msg->uni.cloud.translation[2]);
                    rotation = tf::Quaternion(
                        msg->uni.cloud.rotation[0], 
                        msg->uni.cloud.rotation[1], 
                        msg->uni.cloud.rotation[2], 
                        msg->uni.cloud.rotation[3]);
                    ptr = (float*) (pbuf->data + sizeof(MsgHeader));
                    npts = msg->uni.cloud.npts;
                    pts.clear();
                    pts.reserve(npts);
                    for (size_t i = 0; i < npts; i++)
                    {
                        pt.setValue((double)ptr[0], (double)ptr[1], (double)ptr[2]);
                        pts.push_back(pt);
                        ptr += 3;
                    }
                    //ptr += 3*(npts - 6);  // переходим к концовке сообщения
                    //if (msg->type == MsgTypePointCloud)
                    //for (size_t i = 0; i < 6; i++)
                    //{
            	    //    printf("x = %12.5f, y = %12.5f, z = %12.5f\n",
            	    //        ptr[0], ptr[1], ptr[2]);
            	    //    ptr += 3;
                    //}
                    break;
                case MsgTypeImage: // Изображение с камеры
                    //TODO: Доделать потом
                    break;
                default:
                    // Неизвестный тип сообщения
                    break;
                }
                // Удаляем обработанное сообщение из списка
                MsgConnBufferRelease(&conn, &pbuf);
                needToExit = true;
                //usleep(50000);
            }
        }
        return true;
    }

    // Метод получения данных для тестирования
    bool GetDataDummy(
        tf::Vector3& origin,       // позиция камеры
        tf::Quaternion& rotation,  // ориентация камеры
        std::vector<tf::Vector3>& pts   // облако точек карты
    )
    {
        //origin.setZero();
        //rotation = tf::Quaternion::getIdentity();
        origin.setValue(0.5, 0, 0);
        rotation.setEuler(0, 0, 0.2);

        // Формируем стенку из точек
        pts.clear();
        const double Vmin = -2.0;  // диапазон по одной оси
        const double Vmax = +2.0;
        const double Wmin = -3.0;  // диапазон по другой оси
        const double Wmax = +3.0;
        const double step = 0.2;   // шаг разбиения диапазона
        size_t Vn = (Vmax - Vmin) / step; // колво точек по одной оси
        size_t Wn = (Wmax - Wmin) / step; // колво точек по другой оси
        tf::Vector3 pt;
        pts.reserve(Vn*Wn);
        for (size_t i = 0; i < Vn; i++)
        {
            double Vij = Vmin + step * i;
            for (size_t j = 0; j < Wn; j++)
            {
                double Wij = Wmin + step * j;
                pt.setValue(0, Vij, Wij);
                pts.push_back(pt);
            }
        }
        return true;
    }
};


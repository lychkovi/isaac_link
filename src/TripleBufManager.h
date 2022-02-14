// TripleBufManager.h: Класс помогает организовать асинхронный прием и 
// обработку данных

template <typename ImageHeader>
class TripleBufManager
{
private:
    ImageHeader buffers[3];
    int buf_track;  // being processed by tracker
    int buf_next;   // free buffer
    int buf_cam;    // being loaded by camera
    pthread_mutex_t mux;

public:
    // Вызывать после загрузки нового кадра в buf_cam
    bool SwapBufCamera()
    {
        pthread_mutex_lock(&mux);
        buffers[buf_cam].valid = true; // готов для обработки tracker'ом
        int tmp = buf_next;
        buf_next = buf_cam;
        buf_cam = tmp;
        pthread_mutex_unlock(&mux);
	    return true;
    }

    ImageHeader* GetBufCamera()
    {
        ImageHeader* header;
        header = buffers + buf_cam;
        return header;
    }

    // Вызывать после обработки текущего кадра в buf_track
    bool SwapBufTracker()
    {
        pthread_mutex_lock(&mux);
        bool success = false;
        if (buffers[buf_next].valid)
        {
            buffers[buf_track].valid = false; // обработан tracker'ом
            int tmp = buf_next;
            buf_next = buf_track;
            buf_track = tmp;
            success = true;
        }
        else
            success = false;
        pthread_mutex_unlock(&mux);
        return success;
    }

    ImageHeader* GetBufTracker()
    {
        ImageHeader* header;
        if (buffers[buf_track].valid)
            header = buffers + buf_track;
        else
            header = NULL;
        return header;
    }

    TripleBufManager()
    {
        buf_track = 0;
        buf_next = 1;
        buf_cam = 2;
        pthread_mutex_init(&mux, NULL);
        for (int i = 0; i < 3; i++)
        {
            buffers[i].valid = false;
        }
    }

    ~TripleBufManager()
    {
        pthread_mutex_destroy(&mux);
    }
};


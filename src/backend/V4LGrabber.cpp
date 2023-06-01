
#include "V4LGrabber.hpp"

/*
 *  V4L2 video capture example
 *
 *  This program can be used and distributed without restrictions.
 *
 *      This program is provided with the V4L2 API
 * see http://linuxtv.org/docs.php for more information
 */

#include <cstdlib>
#include <cstring>
#include <cassert>

#include <fcntl.h> /* low-level i/o */
#include <unistd.h>
#include <cerrno>
#include <sys/stat.h>
#include <sys/types.h>
#include <ctime>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <oatpp/core/macro/component.hpp>

#include <CameraUnit/CameraUnit.hpp>
#include <CameraUnit/CameraUnit_ASI.hpp>

#define CLEAR(x) memset(&(x), 0, sizeof(x))

// #undef OATPP_LOGD
// #define OATPP_LOGD(...) \
//     do                  \
//     {                   \
//     } while (false)

const char *V4LGrabber::TAG = "V4L-Grabber";

int V4LGrabber::errno_report(const char *s)
{
    OATPP_LOGE(TAG, "%s - %s error %d, %s", m_devname, s, errno, strerror(errno));
    return errno;
}

int V4LGrabber::read_frame(CImageData data)
{
    OATPP_LOGD(TAG, "%s - Reading frame", m_devname);
    unsigned char *ptr;
    int sz;
    data.GetJPEGData(ptr, sz);
    OATPP_LOGD(TAG, "%s - Frame size: %d", m_devname, sz);
    std::string first_ten = "";
    for (int i = 0; i < 10 && i < sz; i++)
    {
        first_ten += std::to_string(ptr[i]) + " ";
    }
    OATPP_LOGD(TAG, "%s - First 10 bytes: %s", m_devname, first_ten.c_str());
    m_imagecb(m_userdata, ptr, sz);
    return 0;
}

void V4LGrabber::mainloop(V4LGrabber *parent) // thread function
{

    OATPP_LOGD(TAG, "%s - Created mainloop", parent->m_devname);
    static float exposure = 0.1; // 100 ms
    parent->m_camera->SetGainRaw(20);
    while (parent->m_capturing)
    {
        // do the capture here
        CImageData data = parent->m_camera->CaptureImage();
        data.FindOptimumExposure(exposure, 99.99, 40000, 1, 100, 5000);
        parent->m_camera->SetExposure(exposure);
        OATPP_LOGD(TAG, "%s - New image in mainloop: Exposure %.3f", parent->m_devname, exposure);
        if (parent->read_frame(data)) // failed to get frame
            break;
    }
    OATPP_LOGD(TAG, "%s - Mainloop stopped", parent->m_devname);
    parent->m_capturing = false;
}

int V4LGrabber::stop_capturing()
{
    std::lock_guard<std::mutex> lg(m_mutex);
    int rc;

    if (!m_capturing)
    {
        OATPP_LOGD(TAG, "%s - Not capturing", m_devname);
        return 0;
    }

    OATPP_LOGD(TAG, "%s - Stopping capture", m_devname);

    m_capturing = false;
    if (m_mainloop.joinable())
    {
        m_mainloop.join();
    }

    close_device();

    OATPP_LOGD(TAG, "%s - Stopped capture", m_devname);
    return rc;
}

int V4LGrabber::start_capturing()
{
    std::lock_guard<std::mutex> lg(m_mutex);
    int rc;

    if (m_capturing)
    {
        OATPP_LOGD(TAG, "%s - Already capturing", m_devname);
        return 0;
    }

    OATPP_LOGD(TAG, "%s - Starting capture", m_devname);

    if (!m_initdone)
    {
        rc = open_device();
        if (rc)
        {
            return rc;
        }
        rc = init_device();
        if (rc)
        {
            return rc;
        }
        m_initdone = true;
    }

    if (!m_capturing)
    {
        if (m_mainloop.joinable())
        {
            m_mainloop.join();
        }
        OATPP_LOGD(TAG, "%s - Starting mainloop", m_devname);
        m_capturing = true;
        m_mainloop = std::thread(mainloop, this);
    }

    OATPP_LOGD(TAG, "%s - Started capturing", m_devname);
    return rc;
}

int V4LGrabber::init_device()
{
    if (m_initdone)
        return 0;
    m_camera = nullptr;
    int num_cameras, *cameraIds;
    std::string *cameraNames;
    CCameraUnit_ASI::ListCameras(num_cameras, cameraIds, cameraNames);
    if (num_cameras == 0)
    {
        OATPP_LOGE(TAG, "%s - No cameras found", m_devname);
        return -1;
    }

    try
    {
        m_camera = new CCameraUnit_ASI(cameraIds[0]);
    }
    catch (const std::exception &e)
    {
        OATPP_LOGE(TAG, "%s - Failed to open camera: %s", m_devname, e.what());
        return -1;
    }
    if (!m_camera->CameraReady())
    {
        OATPP_LOGE(TAG, "%s - Camera not ready", m_devname);
        delete m_camera;
        return -1;
    }

    m_initdone = true;

    OATPP_LOGD(TAG, "%s - Device inited", m_devname);

    return 0;
}

int V4LGrabber::close_device()
{
    OATPP_LOGD(TAG, "%s - Closing device", m_devname);

    m_camera->CancelCapture();
    delete m_camera;
    m_initdone = false;

    return 0;
}

int V4LGrabber::open_device()
{
    return init_device();
}

int V4LGrabber::testDevice(const char *device)
{
    OATPP_LOGD(TAG, "Testing device '%s'", device);

    int num_cameras, *cameraIds;
    std::string *cameraNames;
    CCameraUnit_ASI::ListCameras(num_cameras, cameraIds, cameraNames);

    if (num_cameras < 1)
    {
        OATPP_LOGE(TAG, "%s is no device", device);
        return ENODEV;
    }

    OATPP_LOGD(TAG, "Tested device '%s' - ok", device);

    return 0;
}

V4LGrabber::V4LGrabber(const char *device, void (*imagecb)(void *, const void *, int), void *userdata)
    : m_devname(device), m_initdone(false), m_imagecb(imagecb), m_userdata(userdata), m_buffers(nullptr), m_nbuffers(0), m_capturing(false)
{
}

V4LGrabber::~V4LGrabber()
{
    stop_capturing();
}

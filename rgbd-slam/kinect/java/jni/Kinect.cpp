#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <pthread.h>
//#include "Kinect.h"
#include "kinect_Kinect.h"
#include "libfreenect.h"


#include <assert.h>

// Threads
pthread_t kinect_thread;
volatile int die = 0;

pthread_mutex_t video_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t depth_mutex = PTHREAD_MUTEX_INITIALIZER;

// Buffers
freenect_video_format vfmt = FREENECT_VIDEO_RGB;
freenect_depth_format dfmt = FREENECT_DEPTH_11BIT;
freenect_resolution res = FREENECT_RESOLUTION_MEDIUM;
const int W_VID = 640;
const int H_VID = 480;
const int W_DEP = 640;
const int H_DEP = 480;
uint32_t *rgb_buf;
uint16_t *d_buf;
uint32_t rgb_time;
uint32_t d_time;

bool got_rgb = false;
bool got_depth = false;
bool IR_mode = false;

freenect_context *f_ctx = 0;
freenect_device *f_dev = 0;
int freenect_angle = 0;
int freenect_led;

int rgbcnt = 0;
int dcnt = 0;

// Callbacks
void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp)
{
    pthread_mutex_lock(&depth_mutex);
    dcnt++;
    memcpy(d_buf, v_depth, W_DEP*H_DEP*2);
    got_depth = true;
    d_time = timestamp;
    pthread_mutex_unlock(&depth_mutex);
}

void video_cb(freenect_device *dev, void *v_rgb, uint32_t timestamp)
{
    pthread_mutex_lock(&video_mutex);
    rgbcnt++;
    uint8_t* temp = (uint8_t*)v_rgb;
    // RGB Mode
    if (IR_mode == false) {
	for (int i = 0; i < W_VID*H_VID; i++) {
             int argb = 0xff000000;
             argb |= temp[3*i + 0] << 16;
             argb |= temp[3*i + 1] << 8;
             argb |= temp[3*i + 2];
             rgb_buf[i] = argb;
        }
    }
    // IR Mode
    else {
        for (int i = 0; i < W_VID*H_VID; i++) {
             int argb = 0xff000000;
             argb |= temp[i] << 16;
             argb |= temp[i] << 8;
             argb |= temp[i];
	     rgb_buf[i] = argb;
        }
    }
    got_rgb = true;
    rgb_time = timestamp;
    pthread_mutex_unlock(&video_mutex);
}
void *runThread(void *)
{
    freenect_set_led(f_dev, LED_BLINK_GREEN);
    while (!die && freenect_process_events(f_ctx) >= 0) {
        // Spin wildly and process events so the callbacks happen
    }

    freenect_close_device(f_dev);
    freenect_shutdown(f_ctx);

    pthread_exit(NULL);
}

JNIEXPORT jint JNICALL Java_kinect_Kinect_initKinect(JNIEnv *env, jobject obj)
{
    if (freenect_init(&f_ctx, NULL) < 0) {
        printf("freenect_init failed\n");
        return -1;
    }

    freenect_set_log_level(f_ctx, FREENECT_LOG_FATAL);
    int num_devs = freenect_num_devices(f_ctx);
    printf("found %d devices\n", num_devs);
    if (num_devs < 1) {
        printf("did not find any devices\n");
        return -1;
    }

    // Open device 0...later maybe we can specify
    int dnum = 0;
    if (freenect_open_device(f_ctx, &f_dev, dnum) < 0) {
        printf("failed to open device %d\n", dnum);
        return -1;
    }

    // Init device
    //freenect_set_tilt_degs(f_dev, 0);
    freenect_set_led(f_dev, LED_YELLOW);
    freenect_set_depth_callback(f_dev, depth_cb);
    freenect_set_video_callback(f_dev, video_cb);

    freenect_set_video_mode(f_dev, freenect_find_video_mode(res, vfmt));
    freenect_set_depth_mode(f_dev, freenect_find_depth_mode(res, dfmt));

    if (pthread_create(&kinect_thread, NULL, runThread, NULL) < 0) {
        printf("failed to start thread\n");
        return -1;
    }

    return 0;
}

JNIEXPORT jint JNICALL Java_kinect_Kinect_closeKinect(JNIEnv *env, jobject obj)
{
    pthread_mutex_lock(&video_mutex);
    pthread_mutex_lock(&depth_mutex);
    freenect_set_led(f_dev, LED_BLINK_RED_YELLOW);
    die = 1;
    pthread_mutex_unlock(&depth_mutex);
    pthread_mutex_unlock(&video_mutex);

    return 0;
}

JNIEXPORT void JNICALL Java_kinect_Kinect_startVideo(JNIEnv *env, jobject obj)
{
    printf("starting native video\n");
    pthread_mutex_lock(&video_mutex);
    rgb_buf = new uint32_t[W_VID*H_VID];
    freenect_start_video(f_dev);
    pthread_mutex_unlock(&video_mutex);
}

// begin new code segments lifted from https://github.com/OpenKinect/libfreenect/blob/master/examples/hiview.c
// RGB video
JNIEXPORT void JNICALL Java_kinect_Kinect_startRGBVideo(JNIEnv *env, jobject obj)
{
    IR_mode = false;
    printf("starting native RGB video\n");
    pthread_mutex_lock(&video_mutex);
    freenect_set_video_mode(f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB));
    rgb_buf = new uint32_t[W_VID*H_VID];
    freenect_start_video(f_dev);
    pthread_mutex_unlock(&video_mutex);
}

// IR video defaulting to 640x480 to avoid complexity
JNIEXPORT void JNICALL Java_kinect_Kinect_startIRVideo(JNIEnv *env, jobject obj)
{
    IR_mode = true;
    printf("starting native IR video\n");
    pthread_mutex_lock(&video_mutex);
    freenect_set_video_mode(f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_IR_8BIT));
    rgb_buf = new uint32_t[W_VID*H_VID];
    freenect_start_video(f_dev);
    pthread_mutex_unlock(&video_mutex);
}

// end new code

JNIEXPORT void JNICALL Java_kinect_Kinect_stopVideo(JNIEnv *env, jobject obj)
{
    printf("stopping native video\n");
    pthread_mutex_lock(&video_mutex);
    freenect_stop_video(f_dev);
    delete[] rgb_buf;
    pthread_mutex_unlock(&video_mutex);
    printf("native video stopped\n");
}

JNIEXPORT void JNICALL Java_kinect_Kinect_startDepth(JNIEnv *env, jobject obj)
{
    printf("starting native depth\n");
    pthread_mutex_lock(&depth_mutex);
    d_buf = new uint16_t[W_DEP*H_DEP];
    freenect_start_depth(f_dev);
    pthread_mutex_unlock(&depth_mutex);
}

JNIEXPORT void JNICALL Java_kinect_Kinect_stopDepth(JNIEnv *env, jobject obj)
{
    printf("stopping native depth\n");
    pthread_mutex_lock(&depth_mutex);
    freenect_stop_depth(f_dev);
    delete[] d_buf;
    pthread_mutex_unlock(&depth_mutex);
    printf("native depth stopped\n");
}

JNIEXPORT jintArray JNICALL Java_kinect_Kinect_getVideoFrame(JNIEnv *env, jobject obj)
{
    if (got_rgb) {
        //printf("\trgb_time: %d\n", rgb_time);
        pthread_mutex_lock(&video_mutex);
        jintArray argb = env->NewIntArray(W_VID*H_VID);
        env->SetIntArrayRegion(argb, 0, W_VID*H_VID, (jint *)rgb_buf);
        got_rgb = false;
        pthread_mutex_unlock(&video_mutex);

        return argb;
    }

    return 0;   // NULL?
}

JNIEXPORT jshortArray JNICALL Java_kinect_Kinect_getDepthFrame(JNIEnv *env, jobject obj)
{
    if (got_depth) {
        //printf("\td_time: %d\n", d_time);
        pthread_mutex_lock(&depth_mutex);
        jshortArray depth = env->NewShortArray(W_DEP*H_DEP);
        env->SetShortArrayRegion(depth, 0, W_DEP*H_DEP, (jshort *)d_buf);
        got_depth = false;
        pthread_mutex_unlock(&depth_mutex);

        return depth;
    }

    return 0;
}

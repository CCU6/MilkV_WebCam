#ifndef MILKV_WEBCAM_MAIN
#define MILKV_WEBCAM_MAIN
#include "midware_utils.h"
#include "sample_utils.h"
#include "vi_vo_utils.h"

#include <core/utils/vpss_helper.h>
#include <cvi_comm.h>
#include <rtsp.h>
#include <sample_comm.h>
#include "cvi_tdl.h"
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "cviruntime.h"
#include <sys/types.h>
#include <ifaddrs.h>


#include <iostream>
#include <cstring>
#include <sys/ioctl.h>
#include <net/if.h>
#include <arpa/inet.h>

#include <termios.h>
#include <fcntl.h>
#include <sys/stat.h>

#include <semaphore.h>
#include <ctime>
#include <cJSON/cJSON.h>
#include <sstream>

#include "stb_image.h"
#include "stb_image_write.h"
#ifdef __cplusplus
extern "C"
{
#endif
    void MY_RTSP_ON_CONNECT(const char *ip, void *arg);
    void MY_RTSP_ON_DISCONNECT(const char *ip, void *arg);
#ifdef __cplusplus
}
#endif

// #define DEBUG

#define ALSA

typedef struct {
  SAMPLE_TDL_MW_CONTEXT *pstMWContext;
  cvitdl_service_handle_t stServiceHandle;
} SAMPLE_TDL_VENC_THREAD_ARG_S;

typedef struct{
  cvtdl_image_t image;
  int x1;
  int x2;
  int y1;
  int y2;
  int cls;
  char name[128];
  char path[128];
  bool flag;
} IMAGE_SAVER_T;

typedef struct{
  IMAGE_SAVER_T queue[16];
  uint8_t top;
  uint8_t end;
  sem_t sem;
} IMAGE_SAVER_ARGS;

typedef struct{
  const char *serial_port;
  int fd;
} TTY_TYPEDEF;

#endif
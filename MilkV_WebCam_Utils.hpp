#ifndef MILKV_WEBCAM_UTILS
#define MILKV_WEBCAM_UTILS
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
#include <atomic>

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

#define OBJ_STATUS_CHECK(a,b) (((a)&(b)) == (b))
#define OBJ_STATUS_SET(a,b) (a|=(b))
#define OBJ_STATUS_RESET(a,b) (a&=~(b))

#define OBJ_IS_AVAILABLE    0b00000001
#define OBJ_IS_SHOTED       0b00000010
#define OBJ_VIO_SAFEHAT     0b00000100
#define OBJ_VIO_VEST        0b00001000
#define OBJ_IS_NEW          0b00010000
#define OBJ_IS_STABLE       0b00100000

#define OBJ_STABLE_TICKS    100
#define OBJ_NEW_TICKS       10
#define OBJ_CHECK_TICKS     10
#define OBJ_VIOLATION_TICKS 3

#define VIOLATION_SAFE                  1
#define VIOLATION_NO_VEST               2
#define VIOLATION_NO_SAFEHAT            3
#define VIOLATION_NO_VEST_AND_SAFEHAT   4


    /*
    bit 0 ID是否有效
    bit 1 ID是否拍照
    bit 2 ID是否佩戴头盔
    bit 3 ID是否穿戴反光衣
    bit 4 
    */

   /*
  0: Hardhat
  1: Mask
  2: NO-Hardhat
  3: NO-Mask
  4: NO-Safety Vest 
  5: Person
  6: Safety Cone
  7: Safety Vest
  8: machinery 
  9: vehicle
*/
const uint8_t vio_ids[5] = {0b00000000,0b00000000,OBJ_VIO_VEST,OBJ_VIO_SAFEHAT,OBJ_VIO_VEST|OBJ_VIO_SAFEHAT};
class Obj_Status
{
    public:
    std::atomic<uint8_t> obj_status;
    std::atomic<uint8_t> obj_ticks;
    Obj_Status();
    ~Obj_Status();
    uint8_t get_status(uint8_t status);
    uint8_t get_violation(void);  

    void set_status_list(uint8_t status);
    void ticks(uint8_t vio_id);
    void obj_clear(void);
    uint8_t status_list[OBJ_CHECK_TICKS] = {0};
    private:
    
    uint8_t status_list_end = 0;  
   
};

void *network_thread(void *ip);
CVI_S32 init_param(const cvitdl_handle_t tdl_handle);
void set_sample_mot_config(cvtdl_deepsort_config_t *ds_conf);
float* utilis_get_mid(cvtdl_object_info_t x);
cvtdl_service_brush_t get_random_brush(uint64_t seed, int min) ;
void My_CopyObjectInfo(cvtdl_object_info_t *src,cvtdl_object_info_t *dst);
bool utilis_is_in(cvtdl_object_info_t tar,cvtdl_tracker_info_t obj);
bool utilis_wear_safe_hat(cvtdl_object_info_t tar,cvtdl_tracker_info_t obj);
bool utilis_wear_safe_vest(cvtdl_object_info_t tar,cvtdl_tracker_info_t obj);
void uid_reallc(uint64_t *id, uint8_t *stat_map, uint64_t *stat_idmap);
int open_serial_port(const char *dev);
int8_t Network_SendResult(char * ip, uint16_t port, int x1, int x2, int y1, int y2, int cls, char *name, uint32_t pc);
ssize_t write_to_serial(int fd, const std::string &data);
#endif
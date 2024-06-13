#define LOG_TAG "MilkV_WebCam"
#define LOG_LEVEL LOG_LEVEL_INFO

#include "MilkV_WebCam_Main.h"
#include "MilkV_WebCam_Utils.hpp"

#include <unistd.h>
#include <string>
#include <semaphore.h>
#include <ctime>
#include <iostream>
#include <cJSON/cJSON.h>
#include <sstream>

#include "stb_image.h"
#include "stb_image_write.h"
// #include <mongoose/mongoose.h>

// #include <opencv2/opencv.hpp>
static char class_name[][16] = 
  {"Hardhat","Mask","NO-Hardhat","NO-Mask",
  "NO-Safety Vest","Person","Safety Cone",
  "Safety Vest","machinery","vehicle"};
pthread_t NetThread;
char g_tar_ip_addr[16] = {0};
sem_t NetSemphore;
int flag = 0;
pthread_mutex_t ResultMutex;

void MY_RTSP_ON_CONNECT(const char *ip, void *arg) {
  strlcpy(g_tar_ip_addr,ip,16);
  // #ifdef DEBUG
  printf("MY RTSP client connected from: %s\n", g_tar_ip_addr);
  // #endif
  pthread_create(&NetThread, NULL, network_thread, (void *)g_tar_ip_addr);

}

void MY_RTSP_ON_DISCONNECT(const char *ip, void *arg) {
  // #ifdef DEBUG
  printf("MY RTSP client disconnected from: %s\n", ip);
  // #endif
  memset(g_tar_ip_addr, 0, 16);
  pthread_cancel(NetThread);
}

static volatile bool bExit = false;
static cvtdl_object_t g_obj_data = {0};
static cvtdl_object_t g_obj_data2 = {0};
static cvtdl_object_t g_obj_data3 = {0};
static cvtdl_tracker_t g_stTrackerMeta = {0};
static uint32_t g_Personcount = 0;
Obj_Status g_PersonStatus[256];
static uint32_t g_ip_uint32 = 0;
static uint64_t g_idmap[256] = {0};
char g_ip_addr[16] = {0};
char *g_ip_ptr = NULL;
const char* dump_path = "/root/image/";
long int file_count = 0;

static TTY_TYPEDEF huart1 = {
  .serial_port = "/dev/ttyS1",
};

static IMAGE_SAVER_ARGS image_saver_arges = {
  .top = 0,
  .end = 0,
};

void *system_ALSA_thread(void *name){
  char command[128] = {0};
  char *path = (char *)name;
  printf("TinyALSA playing %s\n",path);
  sprintf(command,"./TinyALSA/tinyplay -D 1 -d 0 %s",path);
  system(command);
  printf("end play %s\n",path);
  pthread_exit(NULL);
}

void *image_saver_thread(void *args){
  while(!bExit)
  {
    sem_wait(&image_saver_arges.sem);
    #ifdef DEBUG
    printf("start save 1 image queue top:%3d end:%3d\n",image_saver_arges.top,image_saver_arges.end);
    #endif
    if(file_count>=100){
      #ifdef DEBUG
      printf("over 100 image, delete earlist one...");
      #endif
      system("rm ./image/\"$(ls -t image | tail -1)\"");
      file_count --;
      #ifdef DEBUG
      printf("done\n");
      #endif
    }
    if(image_saver_arges.top != image_saver_arges.end){
      if(image_saver_arges.queue[image_saver_arges.top].flag == true){
        printf("saving 1 image to %s\n",image_saver_arges.queue[image_saver_arges.top].path);
        if(stbi_write_png(image_saver_arges.queue[image_saver_arges.top].path,
                          image_saver_arges.queue[image_saver_arges.top].image.width,
                          image_saver_arges.queue[image_saver_arges.top].image.height,
                          STBI_rgb,
                          image_saver_arges.queue[image_saver_arges.top].image.pix[0],
                          image_saver_arges.queue[image_saver_arges.top].image.stride[0])){                
          printf("save image done\n");
        }
        else{
          #ifdef DEBUG
          printf("Dump image failed!\n");
          #endif
        }
        image_saver_arges.queue[image_saver_arges.top].flag = false;    
        CVI_TDL_FreeImage(&image_saver_arges.queue[image_saver_arges.top].image);
        Network_SendResult(g_tar_ip_addr, 11451, 
        image_saver_arges.queue[image_saver_arges.top].x1, 
        image_saver_arges.queue[image_saver_arges.top].x2,
        image_saver_arges.queue[image_saver_arges.top].y1,
        image_saver_arges.queue[image_saver_arges.top].y2,
        image_saver_arges.queue[image_saver_arges.top].cls,
        image_saver_arges.queue[image_saver_arges.top].name,
        g_Personcount);        
        image_saver_arges.top ++;
        image_saver_arges.top %= 16;
      }
    }
  }
  return 0;
}

void *network_thread(void *ip){
  int sockfd;
  char buffer[1024];
  struct sockaddr_in serveraddr;
  socklen_t addr_len;
  int port = 11451;
  int ret;
  const char *serial_port = "/dev/ttyS1"; 
  int fd = open_serial_port(serial_port);
  if (fd < 0) {
      return 0;
  }
  sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  memset(&serveraddr, 0, sizeof(serveraddr));
  serveraddr.sin_family = AF_INET;
  serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);
  serveraddr.sin_port = htons(port);
  bind(sockfd, (struct sockaddr *)&serveraddr, sizeof(serveraddr));
  addr_len = sizeof(serveraddr);
  while (true) {
      ret = recvfrom(sockfd, buffer, 1024, 0, (struct sockaddr *)&serveraddr, &addr_len);
      if (ret == -1) {
          std::cerr << "Error receiving data" << std::endl;
          break;
      }
      buffer[ret] = '\0'; 
      std::cout << "Received: " << buffer << std::endl;
      if(strstr(buffer,"LED ON")){
          write_to_serial(fd,"LED ON");
          std::cout << "UART Send on" << std::endl;
      }
      else if(strstr(buffer,"LED OFF")){
          write_to_serial(fd,"LED OFF");
          std::cout << "UART Send off" << std::endl;
      }
  }
  close(sockfd); // 关闭套接字
  close(fd);
  return 0;
}

void *run_venc(void *args) {
  printf("Enter encoder thread\n");
  SAMPLE_TDL_VENC_THREAD_ARG_S *pstArgs = (SAMPLE_TDL_VENC_THREAD_ARG_S *)args;
  VIDEO_FRAME_INFO_S stFrame;
  CVI_S32 s32Ret;
  cvtdl_object_t stObjMeta = {0};
  cvtdl_object_t stObjMeta2 = {0};
  cvtdl_tracker_t stTrackerMeta2 = {0};

  // uint32_t s_Personcount = 0;
  uint8_t s_PersonTimeOut[256] = {0};
  uint8_t s_PersonState[256] = {0};
  cvtdl_service_brush_t brush_green = {
    .color={
        .r = 0,
        .g = 255,
        .b = 0,
    },
    .size = 4
  };
  cvtdl_service_brush_t brush_blue = {
    .color={
        .r = 0,
        .g = 0,
        .b = 255,
    },
    .size = 4
  };
  cvtdl_service_brush_t brush_red = {
    .color={
        .r = 255,
        .g = 0,
        .b = 0,
    },
    .size = 4
  };
  cvtdl_service_brush_t brush_yellow = {
    .color={
        .r = 255,
        .g = 255,
        .b = 0,
    },
    .size = 4
  };
  cvtdl_service_brush_t brush_pink = {
    .color={
        .r = 255,
        .g = 0,
        .b = 255,
    },
    .size = 4
  };

  while (bExit == false) {
    s32Ret = CVI_VPSS_GetChnFrame(0, VPSS_CHN0, &stFrame, 2000);
    if (s32Ret != CVI_SUCCESS) {
      printf("CVI_VPSS_GetChnFrame chn0 failed with %#x\n", s32Ret);
      break;
    }

    if(pthread_mutex_trylock(&ResultMutex) != EBUSY)
    {
      CVI_TDL_CopyObjectMeta(&g_obj_data2, &stObjMeta2);
      CVI_TDL_CopyObjectMeta(&g_obj_data, &stObjMeta);
      CVI_TDL_CopyTrackerMeta(&g_stTrackerMeta, &stTrackerMeta2);
      pthread_mutex_unlock(&ResultMutex);
      // s_Personcount = g_Personcount;
    }
    
    cvtdl_service_brush_t *brushes = (cvtdl_service_brush_t *)malloc(stObjMeta2.size * sizeof(cvtdl_service_brush_t));
    for (uint32_t oid = 0; oid < stObjMeta2.size; oid++) {

      if(!OBJ_STATUS_CHECK(g_PersonStatus[stTrackerMeta2.info[oid].id].obj_status,OBJ_IS_STABLE)) {
        brushes[oid] = brush_blue;
        snprintf(stObjMeta2.info[oid].name, sizeof(stObjMeta2.info[oid].name), 
                "ID:%03ld New", stTrackerMeta2.info[oid].id);
      }
      else{
        if(g_PersonStatus[stTrackerMeta2.info[oid].id].get_violation() == VIOLATION_NO_VEST_AND_SAFEHAT) { 
          brushes[oid] = brush_red;
          snprintf(stObjMeta2.info[oid].name, sizeof(stObjMeta2.info[oid].name), 
                  "ID:%03ld No Vest and Safe Hat", stTrackerMeta2.info[oid].id);
        }      
        else if(g_PersonStatus[stTrackerMeta2.info[oid].id].get_violation() == VIOLATION_NO_VEST) {
          brushes[oid] = brush_yellow;
          snprintf(stObjMeta2.info[oid].name, sizeof(stObjMeta2.info[oid].name), 
                  "ID:%03ld No Vest", stTrackerMeta2.info[oid].id);
        }    
        else if(g_PersonStatus[stTrackerMeta2.info[oid].id].get_violation() == VIOLATION_NO_SAFEHAT) {
          brushes[oid] = brush_pink;
          snprintf(stObjMeta2.info[oid].name, sizeof(stObjMeta2.info[oid].name), 
                  "ID:%03ld No Safe Hat", stTrackerMeta2.info[oid].id);
        }
        else{
          brushes[oid] = brush_green;
          snprintf(stObjMeta2.info[oid].name, sizeof(stObjMeta2.info[oid].name), 
                  "ID:%03ld Safe", stTrackerMeta2.info[oid].id);
        }
      }      
    }
    // s32Ret = CVI_TDL_Service_ObjectDrawRect(pstArgs->stServiceHandle, &stObjMeta, &stFrame, true, brush_red);
    s32Ret = CVI_TDL_Service_ObjectDrawRect2(pstArgs->stServiceHandle, &stObjMeta2, &stFrame, true, brushes);
    if (s32Ret != CVI_TDL_SUCCESS) {
      CVI_VPSS_ReleaseChnFrame(0, 0, &stFrame);
      printf("Draw frame fail!, ret=%x\n", s32Ret);
      bExit = true;
    }

    s32Ret = SAMPLE_TDL_Send_Frame_RTSP(&stFrame, pstArgs->pstMWContext);
    if (s32Ret != CVI_SUCCESS) {
      CVI_VPSS_ReleaseChnFrame(0, 0, &stFrame);
      printf("Send Output Frame NG, ret=%x\n", s32Ret);
      bExit = true;
    }
    // CVI_TDL_Free(&obj_data2);
    CVI_VPSS_ReleaseChnFrame(0, 0, &stFrame);
    if (s32Ret != CVI_SUCCESS) {
      bExit = true;
    }
    free(brushes);
  }
  printf("Exit encoder thread\n");
  pthread_exit(NULL);
}

void *run_tdl_thread(void *pHandle) {
  printf("Enter TDL thread\n");
  cvitdl_handle_t tdl_handle = (cvitdl_handle_t)pHandle;
  VIDEO_FRAME_INFO_S fdFrame;
  cvtdl_object_t stObjMeta = {0};
  cvtdl_tracker_t stTrackerMeta = {0};
  cvtdl_object_t stTrackObjMeta = {0};
  cvtdl_object_t stTrackObjMeta2 = {0};
  uint32_t s_Personcount = 0;
  uint8_t s_PersonTimeOut[256] = {0};
  uint8_t s_PersonState[256] = {0};

  // Obj_Status* g_PersonStatus = new Obj_Status[256];
  cvtdl_image_t crop_image;
  int sem;
  bool update = false;

  while (bExit == false) {
    if(CVI_VPSS_GetChnFrame(0, VPSS_CHN1, &fdFrame, 2000) != CVI_SUCCESS)
    {
        CVI_TDL_Free(&stObjMeta);
        pthread_exit(NULL);
    }
    struct timeval yolot0, yolot1;
    struct timeval deepsortt0, deepsortt1;

/*****************************YOLO stage*****************************/    
    gettimeofday(&yolot0, NULL);
    if(CVI_TDL_YOLOV8_Detection(tdl_handle, &fdFrame, &stObjMeta) != CVI_SUCCESS){
      printf("YOLOV8 failed!\n");
      CVI_VPSS_ReleaseChnFrame(0, 1, &fdFrame);
      CVI_TDL_Free(&stObjMeta);
      bExit = true;
    }
    gettimeofday(&yolot1, NULL);
/*****************************DeepSORT stage*****************************/
    gettimeofday(&deepsortt0, NULL);
    CVI_TDL_CopyObjectMeta(&stObjMeta, &stTrackObjMeta);
    
    int j = 0;
    int k = 0;
    for (uint32_t i = 0; i < stObjMeta.size; i++){
      if(stObjMeta.info[i].classes == 5) j++;
    }
    if(j != 0){  
      CVI_TDL_CopyObjectMeta(&stObjMeta,&stTrackObjMeta);
      for (uint32_t i = 0; i < stObjMeta.size; i++){
        if(stTrackObjMeta.info[i].classes == 5){ 
          My_CopyObjectInfo(&stObjMeta.info[i],&stTrackObjMeta.info[k]);
          k++;
        } 
        stTrackObjMeta.size = k;  
      }
      CVI_TDL_CopyObjectMeta(&stTrackObjMeta, &stTrackObjMeta2);
    }
    else CVI_TDL_Free(&stTrackObjMeta2);
    if (CVI_TDL_OSNet(tdl_handle, &fdFrame, &stTrackObjMeta2) != CVI_TDL_SUCCESS) {
      printf("DeepSORT failed!\n");
      CVI_VPSS_ReleaseChnFrame(0, 1, &fdFrame);
      CVI_TDL_Free(&stTrackObjMeta2);
      bExit = true;
    }
    if (CVI_TDL_DeepSORT_Obj(tdl_handle, &stTrackObjMeta2, &stTrackerMeta, true) != CVI_TDL_SUCCESS) {
      printf("DeepSORT failed!\n");
      CVI_VPSS_ReleaseChnFrame(0, 1, &fdFrame);
      CVI_TDL_Free(&stTrackObjMeta);
      CVI_TDL_Free(&stTrackObjMeta2);
      CVI_TDL_Free(&stObjMeta);
      CVI_TDL_Free(&stTrackerMeta);
      bExit = true;
    }      
    gettimeofday(&deepsortt1, NULL);
/***************************** END *****************************/
    unsigned long yolo_execution_time = ((yolot1.tv_sec - yolot0.tv_sec) * 1000000 + yolot1.tv_usec - yolot0.tv_usec)/1000;
    unsigned long deepsort_execution_time = ((deepsortt1.tv_sec - deepsortt0.tv_sec) * 1000000 + deepsortt1.tv_usec - deepsortt0.tv_usec)/1000;
    #ifdef DEBUG
    printf("obj count: %d, exec time=%lu ms\n",stObjMeta.size, yolo_execution_time);
    printf("obj count: %d, exec time=%lu ms\n",stTrackerMeta.size, deepsort_execution_time);
    #endif
    if(stObjMeta.size != 0)
    {
      #ifdef DEBUG
      printf("------------------yolo info-------------------\n");
      #endif
      for (uint32_t i = 0; i < stObjMeta.size; i++){
        strlcpy(stObjMeta.info[i].name,class_name[stObjMeta.info[i].classes],16);
        #ifdef DEBUG
        printf("detect res: %4.1f %4.1f %4.1f %4.1f %1.3f %s\n", stObjMeta.info[i].bbox.x1, stObjMeta.info[i].bbox.y1,
            stObjMeta.info[i].bbox.x2, stObjMeta.info[i].bbox.y2, stObjMeta.info[i].bbox.score,
            class_name[stObjMeta.info[i].classes]);
        #endif
      }
    }
    if(stTrackObjMeta2.size != 0)
    {
      #ifdef DEBUG
      printf("------------------Track obj info-------------------\n");
      #endif
      for (uint32_t i = 0; i < stTrackObjMeta2.size; i++){
        strlcpy(stTrackObjMeta2.info[i].name,class_name[stTrackObjMeta2.info[i].classes],16);
        #ifdef DEBUG
        printf("detect res: %4.1f %4.1f %4.1f %4.1f %1.3f %s\n", stTrackObjMeta2.info[i].bbox.x1, stTrackObjMeta2.info[i].bbox.y1,
            stTrackObjMeta2.info[i].bbox.x2, stTrackObjMeta2.info[i].bbox.y2, stTrackObjMeta2.info[i].bbox.score,
            class_name[stTrackObjMeta2.info[i].classes]);
        #endif
      }
    }
    if(stTrackerMeta.size != 0)
    {
      #ifdef DEBUG
      printf("------------------DeepSORT info-------------------\n");
      #endif
      for (uint32_t i = 0; i < stTrackerMeta.size; i++)
      {
        #ifdef DEBUG
        printf("track res: ");
        printf("%4.1f %4.1f %4.1f %1.3f",stTrackerMeta.info[i].bbox.x1 ,stTrackerMeta.info[i].bbox.y1 ,
                                          stTrackerMeta.info[i].bbox.x2 ,stTrackerMeta.info[i].bbox.y2);
        if(stTrackerMeta.info[i].state == CVI_TRACKER_NEW) printf(" NEW ");
        else if(stTrackerMeta.info[i].state == CVI_TRACKER_UNSTABLE) printf(" UNSTABLE ");
        else if(stTrackerMeta.info[i].state == CVI_TRACKER_STABLE) printf(" STABLE ");
        #endif
        uid_reallc(&stTrackerMeta.info[i].id,s_PersonState,g_idmap);
        #ifdef DEBUG
        printf(" %ld %d\n",stTrackerMeta.info[i].id,stTrackerMeta.info[i].out_num);
        #endif
      }
    }
    if(stTrackerMeta.size != 0)
    {
      #ifdef DEBUG
      printf("------------------Person Count %4d-------------------\n",s_Personcount);
      #endif
      for (uint32_t i = 0; i < stTrackerMeta.size; i++){
        stTrackObjMeta2.info[i].unique_id = stTrackerMeta.info[i].id;
        if (stTrackerMeta.info[i].state == CVI_TRACKER_STABLE){
          OBJ_STATUS_SET(g_PersonStatus[stTrackerMeta.info[i].id].obj_status,OBJ_IS_AVAILABLE);
          uint8_t vio_cat =  0;
          OBJ_STATUS_SET(vio_cat,OBJ_VIO_SAFEHAT|OBJ_VIO_VEST);
          for(uint32_t ii = 0; ii < stObjMeta.size; ii++){
            if(stObjMeta.info[ii].classes == 0){
              if(utilis_wear_safe_hat(stObjMeta.info[ii],stTrackerMeta.info[i]) == true){
                OBJ_STATUS_RESET(vio_cat,OBJ_VIO_SAFEHAT);
              }
            }
            else if(stObjMeta.info[ii].classes == 2){
              if(utilis_is_in(stObjMeta.info[ii],stTrackerMeta.info[i]) == true){
                OBJ_STATUS_SET(vio_cat,OBJ_VIO_SAFEHAT);
                break;
              }
            }
          }
          for(uint32_t ii = 0; ii < stObjMeta.size; ii++){
            if(stObjMeta.info[ii].classes == 7){
              if(utilis_wear_safe_vest(stObjMeta.info[ii],stTrackerMeta.info[i]) == true){
                OBJ_STATUS_RESET(vio_cat,OBJ_VIO_VEST);
              }
            }
            else if(stObjMeta.info[ii].classes == 4){
              if(utilis_is_in(stObjMeta.info[ii],stTrackerMeta.info[i]) == true){
                OBJ_STATUS_SET(vio_cat,OBJ_VIO_VEST);
                break;
              }
            }
          }
          g_PersonStatus[stTrackerMeta.info[i].id].ticks(vio_cat);
        }
        for(uint32_t ii = 0; ii < 256; ii++){
          if(!OBJ_STATUS_CHECK(g_PersonStatus[ii].obj_status,OBJ_IS_AVAILABLE)){
            g_PersonStatus[ii].ticks(0);
          }
          else if((!OBJ_STATUS_CHECK(g_PersonStatus[ii].obj_status,OBJ_IS_SHOTED))&&OBJ_STATUS_CHECK(g_PersonStatus[ii].obj_status,OBJ_IS_STABLE)){
            update = true;
            printf("shot ID:%03d  vio:%1d\n",ii,g_PersonStatus[ii].get_violation());
            if(g_PersonStatus[ii].get_violation() == VIOLATION_NO_VEST_AND_SAFEHAT){
              timeval timenow;
              gettimeofday(&timenow,NULL);
              stTrackObjMeta2.info[i].classes = 14;
              snprintf(stTrackObjMeta2.info[i].name,127,"%010ld_ID%03d_NO_VEST_AND_SAFE_HAT",timenow.tv_sec,(int)stTrackerMeta.info[i].id);
              CVI_TDL_CropImage(&fdFrame,&image_saver_arges.queue[image_saver_arges.end].image,
                                &stTrackObjMeta2.info[i].bbox,false);
              snprintf(image_saver_arges.queue[image_saver_arges.end].path,
                        127,"%s%s%s",dump_path,stTrackObjMeta2.info[i].name,".png");
              image_saver_arges.queue[image_saver_arges.end].x1 = stTrackObjMeta2.info[i].bbox.x1;
              image_saver_arges.queue[image_saver_arges.end].x2 = stTrackObjMeta2.info[i].bbox.x2;
              image_saver_arges.queue[image_saver_arges.end].y1 = stTrackObjMeta2.info[i].bbox.y1;
              image_saver_arges.queue[image_saver_arges.end].y2 = stTrackObjMeta2.info[i].bbox.y2;    
              image_saver_arges.queue[image_saver_arges.end].cls = stTrackObjMeta2.info[i].classes;             
              snprintf(image_saver_arges.queue[image_saver_arges.end].name,127,"%s.png\0",stTrackObjMeta2.info[i].name);
              printf("\ncopy %s\n",image_saver_arges.queue[image_saver_arges.end].name);
              image_saver_arges.queue[image_saver_arges.end].flag = true;                
              image_saver_arges.end ++;
              image_saver_arges.end %= 16;
              #ifdef ALSA
              pthread_t ALSAThread;
              pthread_create(&ALSAThread, NULL, system_ALSA_thread, (void *)"3.wav");
              pthread_detach(ALSAThread);
              #endif
              sem_post(&image_saver_arges.sem);
              OBJ_STATUS_SET(g_PersonStatus[ii].obj_status,OBJ_IS_SHOTED);
              file_count++;
            }
            else if(g_PersonStatus[ii].get_violation() == VIOLATION_NO_SAFEHAT){
              timeval timenow;
              gettimeofday(&timenow,NULL);
              stTrackObjMeta2.info[i].classes = 12;
              snprintf(stTrackObjMeta2.info[i].name,127,"%010ld_ID%03d_NO_SAFE_HAT",timenow.tv_sec,(int)stTrackerMeta.info[i].id);
              CVI_TDL_CropImage(&fdFrame,&image_saver_arges.queue[image_saver_arges.end].image,
                                &stTrackObjMeta2.info[i].bbox,false);
              snprintf(image_saver_arges.queue[image_saver_arges.end].path,
                        128,"%s%s%s",dump_path,stTrackObjMeta2.info[i].name,".png");                
              image_saver_arges.queue[image_saver_arges.end].x1 = stTrackObjMeta2.info[i].bbox.x1;
              image_saver_arges.queue[image_saver_arges.end].x2 = stTrackObjMeta2.info[i].bbox.x2;
              image_saver_arges.queue[image_saver_arges.end].y1 = stTrackObjMeta2.info[i].bbox.y1;
              image_saver_arges.queue[image_saver_arges.end].y2 = stTrackObjMeta2.info[i].bbox.y2;    
              image_saver_arges.queue[image_saver_arges.end].cls = stTrackObjMeta2.info[i].classes;             
              snprintf(image_saver_arges.queue[image_saver_arges.end].name,127,"%s.png\0",stTrackObjMeta2.info[i].name);
              printf("\ncopy %s\n",image_saver_arges.queue[image_saver_arges.end].name);
              image_saver_arges.queue[image_saver_arges.end].flag = true;                
              image_saver_arges.end ++;
              image_saver_arges.end %= 16;
              #ifdef DEBUG
              printf("sended save request\n");
              #endif
              sem_post(&image_saver_arges.sem);
              OBJ_STATUS_SET(g_PersonStatus[ii].obj_status,OBJ_IS_SHOTED);
              file_count++;
            }
            else if(g_PersonStatus[ii].get_violation() == VIOLATION_NO_VEST){
              timeval timenow;
              gettimeofday(&timenow,NULL);
              stTrackObjMeta2.info[i].classes = 13;
              snprintf(stTrackObjMeta2.info[i].name,127,"%010ld_ID%03d_NO_VEST",timenow.tv_sec,(int)stTrackerMeta.info[i].id);
              CVI_TDL_CropImage(&fdFrame,&image_saver_arges.queue[image_saver_arges.end].image,
                                &stTrackObjMeta2.info[i].bbox,false);
              snprintf(image_saver_arges.queue[image_saver_arges.end].path,
                        128,"%s%s%s",dump_path,stTrackObjMeta2.info[i].name,".png");
              image_saver_arges.queue[image_saver_arges.end].x1 = stTrackObjMeta2.info[i].bbox.x1;
              image_saver_arges.queue[image_saver_arges.end].x2 = stTrackObjMeta2.info[i].bbox.x2;
              image_saver_arges.queue[image_saver_arges.end].y1 = stTrackObjMeta2.info[i].bbox.y1;
              image_saver_arges.queue[image_saver_arges.end].y2 = stTrackObjMeta2.info[i].bbox.y2;    
              image_saver_arges.queue[image_saver_arges.end].cls = stTrackObjMeta2.info[i].classes;             
              snprintf(image_saver_arges.queue[image_saver_arges.end].name,127,"%s.png\0",stTrackObjMeta2.info[i].name);
              printf("\ncopy %s\n",image_saver_arges.queue[image_saver_arges.end].name);
              image_saver_arges.queue[image_saver_arges.end].flag = true;                
              image_saver_arges.end ++;
              image_saver_arges.end %= 16;
              #ifdef DEBUG
              printf("sended save request\n");
              #endif
              sem_post(&image_saver_arges.sem);
              OBJ_STATUS_SET(g_PersonStatus[ii].obj_status,OBJ_IS_SHOTED);
              file_count++;              
            }
            else{
              snprintf(stTrackObjMeta2.info[i].name,127,"SAFE");
              OBJ_STATUS_SET(g_PersonStatus[ii].obj_status,OBJ_IS_SHOTED);
              stTrackObjMeta2.info[i].classes = 11;
            }
          }
        }
      }
      if(update == true){
        update = false;
        pthread_mutex_lock(&ResultMutex);
        g_Personcount = s_Personcount;
        pthread_mutex_unlock(&ResultMutex);
      }
      // printf("---------------------------------------------------\n");
      for (uint32_t i = 0; i < 256; i++){
        if(g_PersonStatus[i].obj_ticks != 10){
          printf("ID:%03d  vio:%1d        ",i,g_PersonStatus[i].get_violation());
          for(uint32_t ii = 0; ii < 10; ii++){
            printf(" %1d",(int)g_PersonStatus[i].status_list[ii]);
          }
           printf("\n");
        }
        OBJ_STATUS_RESET(g_PersonStatus[i].obj_status,OBJ_IS_AVAILABLE);
      }
      // printf("---------------------------------------------------\n");
    }

    pthread_mutex_lock(&ResultMutex);
    CVI_TDL_CopyObjectMeta(&stObjMeta, &g_obj_data);
    CVI_TDL_CopyObjectMeta(&stTrackObjMeta2, &g_obj_data2);
    CVI_TDL_CopyTrackerMeta(&stTrackerMeta, &g_stTrackerMeta);
    g_Personcount = s_Personcount;
    pthread_mutex_unlock(&ResultMutex);
    CVI_VPSS_ReleaseChnFrame(0, 1, &fdFrame);
    CVI_TDL_Free(&stObjMeta);
    CVI_TDL_Free(&stTrackObjMeta);
    CVI_TDL_Free(&stTrackObjMeta2);
    CVI_TDL_Free(&stTrackerMeta);
    #ifdef DEBUG
    printf("------------------ END -------------------\n\n\n");
    #endif
  }
  printf("Exit TDL thread\n");
  pthread_exit(NULL);
}

static void SampleHandleSig(CVI_S32 signo) {
  signal(SIGINT, SIG_IGN);
  signal(SIGTERM, SIG_IGN);
  printf("handle signal, signo: %d\n", signo);
  if (SIGINT == signo || SIGTERM == signo) {
    bExit = true;
  }
}
/*                                        */
int main(int argc, char *argv[]) {
  if (argc != 3) {
    printf(
        "\nUsage: %s YOLOV8_PATH OSNET_PATH\n\n"
        "\tYOLOV8_PATH\n"
        "\tOSNET_PATH\n",
        argv[0]);
    return CVI_TDL_FAILURE;
  }
  signal(SIGINT, SampleHandleSig);
  signal(SIGTERM, SampleHandleSig);
  struct ifaddrs *interfaces = nullptr;
  struct ifaddrs *addr = nullptr;
  char *outptr;
  char outbuf[1024];

  huart1.fd = open_serial_port(huart1.serial_port);
  if (huart1.fd < 0) {
      printf("serial open failed\n"); 
      return -1;
  }
  if(system("mkdir image") == 0){
    printf("create image path\n");
  }
  else{
    printf("image is exist\n");
  }
  FILE * filep = popen("ls ./image -l | grep \"^-\" | wc -l","r");
  while(fgets(outbuf,1024,filep)!=NULL);
  file_count = strtol(outbuf,&outptr,10);
  printf("image file count:%ld\n",file_count);
  pthread_mutex_init(&ResultMutex, NULL);
  sem_init(&NetSemphore, 0, 0);
  sem_init(&image_saver_arges.sem,0,0);
  SAMPLE_TDL_MW_CONFIG_S stMWConfig;
  memset(&stMWConfig,0,sizeof(stMWConfig));

  CVI_S32 s32Ret = SAMPLE_TDL_Get_VI_Config(&stMWConfig.stViConfig);
  if (s32Ret != CVI_SUCCESS || stMWConfig.stViConfig.s32WorkingViNum <= 0) {
    printf("Failed to get senor infomation from ini file (/mnt/data/sensor_cfg.ini).\n");
    return -1;
  }

  // Get VI size
  PIC_SIZE_E enPicSize;
  s32Ret = SAMPLE_COMM_VI_GetSizeBySensor(stMWConfig.stViConfig.astViInfo[0].stSnsInfo.enSnsType,
                                          &enPicSize);
  if (s32Ret != CVI_SUCCESS) {
    printf("Cannot get senor size\n");
    return -1;
  }

  SIZE_S stSensorSize;
  s32Ret = SAMPLE_COMM_SYS_GetPicSize(enPicSize, &stSensorSize);
  if (s32Ret != CVI_SUCCESS) {
    printf("Cannot get senor size\n");
    return -1;
  }

  // Setup frame size of video encoder to 1080p
  SIZE_S stVencSize = {
      .u32Width = 1920,
      .u32Height = 1080,
  };

  PIXEL_FORMAT_E enInputFormat;
  enInputFormat = PIXEL_FORMAT_RGB_888;

  stMWConfig.stVBPoolConfig.u32VBPoolCount = 3;

  // VBPool 0 for VPSS Grp0 Chn0
  stMWConfig.stVBPoolConfig.astVBPoolSetup[0].enFormat = VI_PIXEL_FORMAT;
  stMWConfig.stVBPoolConfig.astVBPoolSetup[0].u32BlkCount = 3;
  stMWConfig.stVBPoolConfig.astVBPoolSetup[0].u32Height = stSensorSize.u32Height;
  stMWConfig.stVBPoolConfig.astVBPoolSetup[0].u32Width = stSensorSize.u32Width;
  stMWConfig.stVBPoolConfig.astVBPoolSetup[0].bBind = true;
  stMWConfig.stVBPoolConfig.astVBPoolSetup[0].u32VpssChnBinding = VPSS_CHN0;
  stMWConfig.stVBPoolConfig.astVBPoolSetup[0].u32VpssGrpBinding = (VPSS_GRP)0;

  // VBPool 1 for VPSS Grp0 Chn1
  stMWConfig.stVBPoolConfig.astVBPoolSetup[1].enFormat = enInputFormat;
  stMWConfig.stVBPoolConfig.astVBPoolSetup[1].u32BlkCount = 3;
  stMWConfig.stVBPoolConfig.astVBPoolSetup[1].u32Height = stVencSize.u32Height;
  stMWConfig.stVBPoolConfig.astVBPoolSetup[1].u32Width = stVencSize.u32Width;
  stMWConfig.stVBPoolConfig.astVBPoolSetup[1].bBind = true;
  stMWConfig.stVBPoolConfig.astVBPoolSetup[1].u32VpssChnBinding = VPSS_CHN1;
  stMWConfig.stVBPoolConfig.astVBPoolSetup[1].u32VpssGrpBinding = (VPSS_GRP)0;

  // VBPool 2 for TDL preprocessing
  stMWConfig.stVBPoolConfig.astVBPoolSetup[2].enFormat = PIXEL_FORMAT_RGB_888_PLANAR;
  stMWConfig.stVBPoolConfig.astVBPoolSetup[2].u32BlkCount = 1;
  stMWConfig.stVBPoolConfig.astVBPoolSetup[2].u32Height = 720;
  stMWConfig.stVBPoolConfig.astVBPoolSetup[2].u32Width = 1280;
  stMWConfig.stVBPoolConfig.astVBPoolSetup[2].bBind = false;

  // Setup VPSS Grp0
  stMWConfig.stVPSSPoolConfig.u32VpssGrpCount = 1;
#ifndef CV186X
  stMWConfig.stVPSSPoolConfig.stVpssMode.aenInput[0] = VPSS_INPUT_MEM;
  stMWConfig.stVPSSPoolConfig.stVpssMode.enMode = VPSS_MODE_DUAL;
  stMWConfig.stVPSSPoolConfig.stVpssMode.ViPipe[0] = 0;
  stMWConfig.stVPSSPoolConfig.stVpssMode.aenInput[1] = VPSS_INPUT_ISP;
  stMWConfig.stVPSSPoolConfig.stVpssMode.ViPipe[1] = 0;
#endif

  SAMPLE_TDL_VPSS_CONFIG_S *pstVpssConfig = &stMWConfig.stVPSSPoolConfig.astVpssConfig[0];
  pstVpssConfig->bBindVI = true;

  // Assign device 1 to VPSS Grp0, because device1 has 3 outputs in dual mode.
  VPSS_GRP_DEFAULT_HELPER2(&pstVpssConfig->stVpssGrpAttr, stSensorSize.u32Width,
                           stSensorSize.u32Height, VI_PIXEL_FORMAT, 1);
  pstVpssConfig->u32ChnCount = 2;
  pstVpssConfig->u32ChnBindVI = 0;
  VPSS_CHN_DEFAULT_HELPER(&pstVpssConfig->astVpssChnAttr[0], stVencSize.u32Width,
                          stVencSize.u32Height, VI_PIXEL_FORMAT, true);

  // Prepare Vpss Chn1 for TDL inference
  VPSS_CHN_DEFAULT_HELPER(&pstVpssConfig->astVpssChnAttr[1], stVencSize.u32Width,
                          stVencSize.u32Height, enInputFormat, true);

  // Get default VENC configurations
  SAMPLE_TDL_Get_Input_Config(&stMWConfig.stVencConfig.stChnInputCfg);
  stMWConfig.stVencConfig.u32FrameWidth = stVencSize.u32Width;
  stMWConfig.stVencConfig.u32FrameHeight = stVencSize.u32Height;

  // Get default RTSP configurations
  SAMPLE_TDL_Get_RTSP_Config(&stMWConfig.stRTSPConfig.stRTSPConfig);

  SAMPLE_TDL_MW_CONTEXT stMWContext = {0};
  s32Ret = SAMPLE_TDL_Init_WM(&stMWConfig, &stMWContext);
  if (s32Ret != CVI_SUCCESS) {
    printf("init middleware failed! ret=%x\n", s32Ret);
    return -1;
  }

  cvitdl_handle_t stTDLHandle = NULL;

  if(CVI_TDL_CreateHandle2(&stTDLHandle, 1, 0) != CVI_SUCCESS)
  {
    SAMPLE_TDL_Destroy_MW(&stMWContext);
    return -1;
  }

  if(CVI_TDL_SetVBPool(stTDLHandle, 0, 2) != CVI_SUCCESS)
  {
    CVI_TDL_DestroyHandle(stTDLHandle);
    SAMPLE_TDL_Destroy_MW(&stMWContext);
    return -1;
  }

  CVI_TDL_SetVpssTimeout(stTDLHandle, 1000);

  cvitdl_service_handle_t stServiceHandle = NULL;
  if(CVI_TDL_Service_CreateHandle(&stServiceHandle, stTDLHandle) != CVI_SUCCESS)
  {
    CVI_TDL_DestroyHandle(stTDLHandle);
    SAMPLE_TDL_Destroy_MW(&stMWContext);
    return -1;
  }

  printf("---------------------open yolo model-----------------------\n");
  init_param(stTDLHandle);
  int ret2 = (int)CVI_TDL_OpenModel(stTDLHandle, CVI_TDL_SUPPORTED_MODEL_YOLOV8_DETECTION, argv[1]);
  if(ret2 !=CVI_SUCCESS)
  {
    printf("openmodel failed ret=%X\n",ret2);
    CVI_TDL_Service_DestroyHandle(stServiceHandle);
    CVI_TDL_DestroyHandle(stTDLHandle);
    SAMPLE_TDL_Destroy_MW(&stMWContext);    
  }
  printf("---------------------setup deepsort-----------------------\n");
  ret2 = (int)CVI_TDL_OpenModel(stTDLHandle, CVI_TDL_SUPPORTED_MODEL_OSNET, argv[2]);
  if(ret2 !=CVI_SUCCESS)
  {
    printf("openmodel failed ret=%X\n",ret2);
    CVI_TDL_Service_DestroyHandle(stServiceHandle);
    CVI_TDL_DestroyHandle(stTDLHandle);
    SAMPLE_TDL_Destroy_MW(&stMWContext);    
  }
  CVI_TDL_DeepSORT_Init(stTDLHandle, true);
  cvtdl_deepsort_config_t ds_conf;
  CVI_TDL_DeepSORT_GetDefaultConfig(&ds_conf);
  set_sample_mot_config(&ds_conf);
  ret2 = (int)CVI_TDL_DeepSORT_SetConfig(stTDLHandle, &ds_conf, -1, false);
  if(ret2 !=CVI_SUCCESS){
    printf("DeepSORT config failed ret=%X\n",ret2);
  }
  else printf("DeepSORT config Success\n");
  printf("---------------------all finish-----------------------\n"); 

  pthread_t stVencThread,stTDLThread,saverThread;

  SAMPLE_TDL_VENC_THREAD_ARG_S args = {
      .pstMWContext = &stMWContext,
      .stServiceHandle = stServiceHandle,
  };
  pthread_create(&saverThread, NULL, image_saver_thread, &args);
  pthread_create(&stVencThread, NULL, run_venc, &args);
  pthread_create(&stTDLThread, NULL, run_tdl_thread, stTDLHandle);
  // pthread_create(&ALSAThread, NULL, system_ALSA_thread, &args);
  sleep(1);
  
  pthread_join(stVencThread, NULL);
  pthread_join(stTDLThread, NULL);
  pthread_cancel(saverThread);

  // pthread_join(stHttpThread, NULL);

  CVI_TDL_Service_DestroyHandle(stServiceHandle);
  CVI_TDL_DestroyHandle(stTDLHandle);
  SAMPLE_TDL_Destroy_MW(&stMWContext);

  return 0;
}

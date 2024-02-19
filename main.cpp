//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
// CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
// SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//

#include "livox_lidar_api.h"
#include "livox_lidar_def.h"


#include <arpa/inet.h>
#include <unistd.h>

#include <chrono>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>

void PointCloudCallback(uint32_t handle,
                        const uint8_t dev_type,
                        LivoxLidarEthernetPacket *data,
                        void *client_data) {
  if(data == nullptr) {
    return;
  }

  LivoxLidarCartesianHighRawPoint *p_point_data
      = (LivoxLidarCartesianHighRawPoint *) data->data;

  printf("point cloud data: \n");
  
  // for(uint32_t i = 0; i < data->dot_num; i++) {
  //   printf("x: %d, y: %d, z: %d\n",
  //          p_point_data[i].x,
  //          p_point_data[i].y,
  //          p_point_data[i].z);
  // }

  

}

void WorkModeCallback(livox_status status,
                      uint32_t handle,
                      LivoxLidarAsyncControlResponse *response,
                      void *client_data) {
  if(response == nullptr) {
    return;
  }
  printf(
      "WorkModeCallack, status:%u, handle:%u, ret_code:%u, error_key:%u",
      status,
      handle,
      response->ret_code,
      response->error_key);
}

void RebootCallback(livox_status status,
                    uint32_t handle,
                    LivoxLidarRebootResponse *response,
                    void *client_data) {
  if(response == nullptr) {
    return;
  }
  printf("RebootCallback, status:%u, handle:%u, ret_code:%u",
         status,
         handle,
         response->ret_code);
}

void SetIpInfoCallback(livox_status status,
                       uint32_t handle,
                       LivoxLidarAsyncControlResponse *response,
                       void *client_data) {
  if(response == nullptr) {
    return;
  }
  printf(
      "LivoxLidarIpInfoCallback, status:%u, handle:%u, ret_code:%u, "
      "error_key:%u",
      status,
      handle,
      response->ret_code,
      response->error_key);

  if(response->ret_code == 0 && response->error_key == 0) {
    LivoxLidarRequestReboot(handle, RebootCallback, nullptr);
  }
}

void QueryInternalInfoCallback(
    livox_status status,
    uint32_t handle,
    LivoxLidarDiagInternalInfoResponse *response,
    void *client_data) {
  if(status != kLivoxLidarStatusSuccess) {
    printf("Query lidar internal info failed.\n");
    QueryLivoxLidarInternalInfo(
        handle, QueryInternalInfoCallback, nullptr);
    return;
  }

  if(response == nullptr) {
    return;
  }

  uint8_t host_point_ipaddr[4] {0};
  uint16_t host_point_port = 0;
  uint16_t lidar_point_port = 0;

  uint8_t host_imu_ipaddr[4] {0};
  uint16_t host_imu_data_port = 0;
  uint16_t lidar_imu_data_port = 0;

  uint16_t off = 0;
  for(uint8_t i = 0; i < response->param_num; ++i) {
    LivoxLidarKeyValueParam *kv
        = (LivoxLidarKeyValueParam *) &response->data[off];
    if(kv->key == kKeyLidarPointDataHostIpCfg) {
      memcpy(host_point_ipaddr, &(kv->value[0]), sizeof(uint8_t) * 4);
      memcpy(&(host_point_port), &(kv->value[4]), sizeof(uint16_t));
      memcpy(&(lidar_point_port), &(kv->value[6]), sizeof(uint16_t));
    } else if(kv->key == kKeyLidarImuHostIpCfg) {
      memcpy(host_imu_ipaddr, &(kv->value[0]), sizeof(uint8_t) * 4);
      memcpy(&(host_imu_data_port), &(kv->value[4]), sizeof(uint16_t));
      memcpy(&(lidar_imu_data_port), &(kv->value[6]), sizeof(uint16_t));
    }
    off += sizeof(uint16_t) * 2;
    off += kv->length;
  }

  printf(
      "Host point cloud ip addr:%u.%u.%u.%u, host point cloud port:%u, "
      "lidar point cloud port:%u.\n",
      host_point_ipaddr[0],
      host_point_ipaddr[1],
      host_point_ipaddr[2],
      host_point_ipaddr[3],
      host_point_port,
      lidar_point_port);

  printf(
      "Host imu ip addr:%u.%u.%u.%u, host imu port:%u, lidar imu "
      "port:%u.\n",
      host_imu_ipaddr[0],
      host_imu_ipaddr[1],
      host_imu_ipaddr[2],
      host_imu_ipaddr[3],
      host_imu_data_port,
      lidar_imu_data_port);
}

void LidarInfoChangeCallback(const uint32_t handle,
                             const LivoxLidarInfo *info,
                             void *client_data) {
  if(info == nullptr) {
    printf("lidar info change callback failed, the info is nullptr.\n");
    return;
  }
  printf("LidarInfoChangeCallback Lidar handle: %u SN: %s\n",
         handle,
         info->sn);

  SetLivoxLidarWorkMode(
      handle, kLivoxLidarNormal, WorkModeCallback, nullptr);

  QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);
}

int main(int argc, const char *argv[]) {
  if(argc != 2) {
    printf("Params Invalid, must input config path.\n");
    return -1;
  }
  const std::string path = argv[1];

  // REQUIRED, to init Livox SDK2
  if(!LivoxLidarSdkInit(path.c_str())) {
    printf("Livox Init Failed\n");
    LivoxLidarSdkUninit();
    return -1;
  }

  // REQUIRED, to get point cloud data via 'PointCloudCallback'
  SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);

  // REQUIRED, to get a handle to targeted lidar and set its work mode to
  // NORMAL
  SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

  sleep(5);
  LivoxLidarSdkUninit();
  printf("Livox Quick Start Demo End!\n");
  return 0;
}

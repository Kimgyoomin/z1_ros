#include "unitree_arm_sdk/unitree_arm.h"
#include <stdio.h>
#include <cmath>
#include <iostream>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <vector>
#include <thread>
#include <Eigen/Dense>
#include <eigen3/Eigen/Dense>
#include <atomic>
#include <pthread.h>

// 전역변수로 UNITREE_ARM_SDK::UnitreeArm 객체와 동기화에 필요한 변수 선언
UNITREE_ARM_SDK::UnitreeArm* z1Ptr;
std::vector<double> save_ref5, save_enc5, save_ref4, save_enc4; // 값 저장을 위한 벡터
using std::ofstream;


void* recvsendData(void* arg) {
  int cnt = 0;
  double jntSpeed = 45 * M_PI / 180;
  Vec6 q = z1Ptr->armState.getQ_d();
  
  while(1) {
    z1Ptr->armCmd.mode = (mode_t)UNITREE_ARM_SDK::ArmMode::JointPositionCtrl;
    auto start = std::chrono::high_resolution_clock::now();
    z1Ptr->recv();
    // 관절 제어부
    z1Ptr->armCmd.setQ(q);
    q(5) = M_PI / 8 * (1 - cos(jntSpeed * 0.0064 * cnt));
    z1Ptr->armCmd.dq_d[5] = M_PI / 8 * sin(jntSpeed * 0.0064);
    
    q(4) = M_PI / 8 * (1 - cos(jntSpeed * 0.0064 * cnt));
    z1Ptr->armCmd.dq_d[4] = M_PI / 8 * sin(jntSpeed * 0.0064);

    z1Ptr->send();

    // 값을 배열에 저장
    save_ref5.push_back(q(5));
    save_enc5.push_back(z1Ptr->armState.q[5]);
    save_ref4.push_back(q(4));
    save_enc4.push_back(z1Ptr->armState.q[4]);

    // usleep(4000); // 4000 microseconds
    auto next_cycle = start + std::chrono::milliseconds(4);
    std::this_thread::sleep_until(next_cycle);
    if (cnt >= 1250) {
      z1Ptr->armCmd.mode = (mode_t)UNITREE_ARM_SDK::ArmMode::Passive;
      z1Ptr->sendRecv();
      break;
    }
    cnt++;
  }
  return NULL;
}

// void saveDataToFile() {
//   // 파일 입출력문 불러오기
//   std::ofstream file("/home/kim/rc_lab_datafoldeer/240219_1.txt");
//   for (size_t i = 0; i < save_ref5.size(); ++i) {
//     file << std::fixed << std::setprecision(6) << save_ref5[i]
//     << "\t" << save_enc5[i] << "\t" << save_ref4[i] << "\t" << save_enc4[i]
//     << std::endl;
//   }
//   file.close();
// }

int main(int argc, char** argv)
{
  /* Connect to z1_controller */
  std::string controller_IP = argc > 1 ? argv[1] : "127.0.0.1";
  UNITREE_ARM_SDK::UnitreeArm z1(controller_IP);
  auto start_m = std::chrono::high_resolution_clock::now();
  z1Ptr = &z1;
  z1Ptr->init();

  /* For p_thread */
  pthread_t recvsendThread;
  pthread_create(&recvsendThread, NULL, recvsendData, NULL);

  pthread_join(recvsendThread, NULL); // 이 쓰레드가 종료될때까지 대기

  /* 작업 끝난 후 arm모드를 Passive로 변경 */
  // z1.armCmd.mode = (mode_t)UNITREE_ARM_SDK::ArmMode::Passive;
  // z1.sendRecv();

  std::ofstream file("/home/kim/rc_lab_datafolder/240219_1.txt");
  for (size_t i = 0; i < 1250; ++i)
  {
    file << std::fixed << std::setprecision(6) << save_ref5[i]
    << "\t" << save_enc5[i] << "\t" << save_ref4[i] << "\t" << save_enc4[i]
    << std::endl;
  }
  file.close();
  // saveDataToFile(); // 스레드 완료 후 데이터 저장 함수 실행

  /* 전체 실행 시간 확인 */
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> execution_time = end - start_m;
  std::cout << "Execution Time (chrono) " << execution_time.count() << "ms"
  << std::endl;

  return 0;
}
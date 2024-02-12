#include "unitree_arm_sdk/unitree_arm.h"
#include <stdio.h>
#include <cmath>
#include <iostream>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <pthread.h>


/* 그래프 그리기 위해 데이터 뽑기 */
// using std::ofstream;

using namespace UNITREE_ARM_SDK;



// 관절 위치 제어를 수행하는 thread func
void* controlLoop(void* arg) {
  UnitreeArm* z1 = static_cast<UnitreeArm*>(arg); // 스레드 인자로 받은 로봇 객체 포인터

  // 로봇의 초기 상태 수신
  z1->recv();


  double jntSpeed = 5 * M_PI / 180; // 관절 속도 (rad/s)
  Vec6 q; // 관절 위치를 저장할 벡터
  int cnt = 0; // 카운터 초기화

  while(1) {
    if (cnt >= 10250) break; // 반복 횟수 제한
    z1->armCmd.mode = static_cast<mode_t>(ArmMode::JointPositionCtrl);
    z1->recv();

    q = z1->armState.getQ_d();

    // JT5, JT4 위치 변화
    q(5) = M_PI / 8 * (1 - cos(jntSpeed * cnt * 0.00328));
    z1->armCmd.dq_d[5] = M_PI / 8 * sin(jntSpeed * cnt * 0.00328);
    
    // q(4) = M_PI / 8 * (1 - cos(jntSpeed * cnt * 0.00328));
    // z1->armCmd.dq_d[4] = M_PI / 8 * sin(jntSpeed * cnt * 0.00328);

    z1->send();
    z1->recv();

    cnt++;
  }

  // 스레드 종료 시 로봇을 Passive 모드로 설정
  z1->armCmd.mode = static_cast<mode_t>(ArmMode::Passive);
  z1->sendRecv();

  return NULL;
}




int main(int argc, char** argv)
{
  /* Connect to z1_controller */
  std::string controller_IP = argc > 1 ? argv[1] : "127.0.0.1";
  UNITREE_ARM_SDK::UnitreeArm z1(controller_IP);
  z1.init();

  // ofstream file("/home/kim/rc_lab_datafolder/z1_jt5jt4_test1.txt");
  z1.recv();
  Timer timer(z1.dt);

  pthread_t threadID;
  pthread_create(&threadID, NULL, controlLoop, &z1);
  pthread_join(threadID, NULL);




  /* Joint velocity control */
  // double jntSpeed = 5 * M_PI / 180;

  /* chrono func start*/
  // auto start = std::chrono::high_resolution_clock::now();

  /* Joint position control */
  // Get current desired q
  // Vec6 q = z1.armState.getQ_d();
  // for (size_t i = 0; i < 10250; i++)
  // {
  //   z1.armCmd.mode = (mode_t)UNITREE_ARM_SDK::ArmMode::JointPositionCtrl;
  //   q(5) = M_PI / 8 * (1 - cos(jntSpeed * 2 * 0.00328 * i));
  //   z1.armCmd.setQ(q);
  //   z1.send();
  //   z1.armCmd.dq_d[5] = M_PI / 8 * sin(jntSpeed * 2 * 0.00328);
  //   q(4) = M_PI / 8 * (1 - cos(jntSpeed * 2 * 0.00328 * i));
  //   z1.armCmd.setQ(q);
  //   z1.armCmd.dq_d[4] = M_PI / 8 * sin(jntSpeed * 2 * 0.00328);
  //   // z1.armCmd.gripperCmd.angle -= jntSpeed*z1.dt;
  //   z1.send();
  //   z1.recv();
  //   // timer.sleep();

  //   file << std::fixed << std::setprecision(6) << q[5] << "\t"
  //   << z1.armState.q[5] << "\t" << q[4] << "\t" << z1.armState.q[4]
  //   << std::endl;
  // }

  /* chrono func finish */
  // auto end = std::chrono::high_resolution_clock::now();
  // std::chrono::duration<double, std::milli> execution_time = end - start;
  // std::cout << "Execution Time (chorno) " << execution_time.count() << "ms"
  // << std::endl;
  // Vec6 q = z1.armState.getQ_d();
  // for (size_t i = 0; i < 1250; i ++)
  // {
  //   z1.armCmd.mode = (mode_t)UNITREE_ARM_SDK::ArmMode::JointPositionCtrl;
  //   q(4) = M_PI / 8 * (1 - cos(jntSpeed * 2 * 0.00328 * i));
  //   z1.armCmd.setQ(q);
  //   z1.armCmd.dq_d[4] = M_PI / 8 * sin(jntSpeed * 2 * 0.00328);
  //   // z1.armCmd.gripperCmd.angle -= jntSpeed*z1.dt;
  //   z1.send();
  //   z1.recv();
  //   // timer.sleep();
  // }
  // z1.armCmd.mode = (mode_t)UNITREE_ARM_SDK::ArmMode::Passive;
  // z1.sendRecv();

  /* end for writing file */
  // file.close();
  return 0;
}
#include "unitree_arm_sdk/unitree_arm.h"
#include <stdio.h>
#include <cmath>
#include <iostream>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <vector>

/**
 * @example example_joint_ctrl.cpp
 * An example showing how to control joint velocity and position.
 * 
 * Run `roslaunch z1_bringup **_ctrl.launch` first.
 */

/* 그래프 그리기 위해 데이터 뽑기 */
using std::ofstream;

int main(int argc, char** argv)
{
  /* Connect to z1_controller */
  std::string controller_IP = argc > 1 ? argv[1] : "127.0.0.1";
  UNITREE_ARM_SDK::UnitreeArm z1(controller_IP);
  z1.init();

  ofstream file("/home/kim/rc_lab_datafolder/240215_1.txt");

  // Timer timer(z1.dt);
  double jntSpeed = 45 * M_PI / 180;
  std::vector<float> save_ref_5;
  std::vector<float> save_enc_5;
  std::vector<float> save_ref_4;
  std::vector<float> save_enc_4;


  /* Joint position control */
  // Get current desired q
  Vec6 q = z1.armState.getQ_d();

  int cnt = 0;
  auto start = std::chrono::high_resolution_clock::now();
  while(1) {
    z1.armCmd.mode = (mode_t)UNITREE_ARM_SDK::ArmMode::JointPositionCtrl;
    z1.recv();
    if (cnt >= 1250) break;

    // 관절 제어부
    z1.armCmd.setQ(q);
    q(5) = M_PI / 8 * (1 - cos(jntSpeed * 0.0064 * cnt));
    z1.armCmd.dq_d[5] = M_PI / 8 * sin(jntSpeed * 0.0064);

    q(4) = M_PI / 8 * (1 - cos(jntSpeed * 0.0064 * cnt));
    z1.armCmd.dq_d[4] = M_PI / 8 * sin(jntSpeed * 0.0064);

    z1.send();
    save_ref_5.push_back(q(5));
    save_enc_5.push_back(z1.armState.q[5]);
    save_ref_4.push_back(q(4));
    save_enc_4.push_back(z1.armState.q[4]);

    // 고해상도 타이머를 통해 정확하게 한 iteration당 4ms 되도록 대기
    auto next_cycle = start + std::chrono::milliseconds(4 * (cnt+1));
    std::this_thread::sleep_until(next_cycle);
    cnt++;
  }

  /* chrono func finish 
  checking for time */
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> execution_time = end - start;
  std::cout << "Execution Time (chorno) " << execution_time.count() << "ms"
  << std::endl;

  /* 작업 끝난 후 arm모드를 Passive로 변경 */
  z1.armCmd.mode = (mode_t)UNITREE_ARM_SDK::ArmMode::Passive;
  z1.sendRecv();

  for (size_t i = 0; i < 1250; i++)
  {
    file << std::fixed << std::setprecision(6) << save_ref_5[i]
    << "\t" << save_enc_5[i] << "\t" << save_ref_4[i] << "\t" <<
    save_enc_4[i] << std::endl;
  }

  /* end for writing file */
  // file.close();
  return 0;
}
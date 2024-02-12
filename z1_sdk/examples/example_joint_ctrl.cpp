#include "unitree_arm_sdk/unitree_arm.h"
#include <stdio.h>
#include <cmath>
#include <iostream>
#include <chrono>
#include <fstream>
#include <iomanip>

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

  ofstream file("/home/kim/rc_lab_datafolder/z1_jt5jt4_test1.txt");

  Timer timer(z1.dt);

  /* Joint velocity control */
  double jntSpeed = 5 * M_PI / 180;
  // Vec6 dq;
  // dq << jntSpeed, 0, 0, 0, 0, 0;
  // for (size_t i = 0; i < 400; i++)
  // {
  //   z1.armCmd.mode = (mode_t)UNITREE_ARM_SDK::ArmMode::JointSpeedCtrl;
  //   z1.armCmd.setDq(dq);
  //   z1.armCmd.gripperCmd.angle -= jntSpeed*z1.dt;
  //   z1.sendRecv();
  //   timer.sleep();
  // }

  /* chrono func start*/
  auto start = std::chrono::high_resolution_clock::now();

  /* Joint position control */
  // Get current desired q
  Vec6 q = z1.armState.getQ_d();
  for (size_t i = 0; i < 10250; i++)
  {
    z1.armCmd.mode = (mode_t)UNITREE_ARM_SDK::ArmMode::JointPositionCtrl;
    q(5) = M_PI / 8 * (1 - cos(jntSpeed * 2 * 0.00328 * i));
    z1.armCmd.setQ(q);
    z1.send();
    z1.armCmd.dq_d[5] = M_PI / 8 * sin(jntSpeed * 2 * 0.00328);
    q(4) = M_PI / 8 * (1 - cos(jntSpeed * 2 * 0.00328 * i));
    z1.armCmd.setQ(q);
    z1.armCmd.dq_d[4] = M_PI / 8 * sin(jntSpeed * 2 * 0.00328);
    // z1.armCmd.gripperCmd.angle -= jntSpeed*z1.dt;
    z1.send();
    z1.recv();
    // timer.sleep();

    file << std::fixed << std::setprecision(6) << q[5] << "\t"
    << z1.armState.q[5] << "\t" << q[4] << "\t" << z1.armState.q[4]
    << std::endl;
  }

  /* chrono func finish */
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> execution_time = end - start;
  std::cout << "Execution Time (chorno) " << execution_time.count() << "ms"
  << std::endl;
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
  z1.armCmd.mode = (mode_t)UNITREE_ARM_SDK::ArmMode::Passive;
  z1.sendRecv();

  /* end for writing file */
  file.close();
  return 0;
}
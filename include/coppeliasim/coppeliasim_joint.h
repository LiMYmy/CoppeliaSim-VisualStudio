#ifndef _COPPELIASIM_JOINT_H
#define _COPPELIASIM_JOINT_H

#include "coppeliasim_base.h"

#include <vector>
#include <map>
#include <math.h>

enum class JointMode
{
  Idle,
  Position,
  Velocity,
  Torque
};

class CoppeliaSimJoint : public CoppeliaSimBase
{
  friend class CoppeliaSimJointManager;

public:
  CoppeliaSimJoint(simxInt client_id, std::string name);

  /**
  * @brief ���ùؽ�Ŀ��λ�ã���λ��rad��ʵ��ִ��Ч����PID����Ӱ�졣
  */
  CoppeliaSimJoint& SetPosition(float pos);

  /**
  * @brief ���ùؽ�Ŀ�����أ���λ��N*m
  */
  CoppeliaSimJoint& SetTorque(float torque);

  /**
  * @brief ���ùؽ�Ŀ���ٶȣ���λ��rpm
  */
  CoppeliaSimJoint& SetVelocity(float vel);
  

  /**
   * @brief ����PID����
   * 
   * @param p
   * @param i
   * @param d
   */
  CoppeliaSimJoint& SetPID(float p, float i, float d);

  /**
   * @brief �������������أ���λ��N*m��Ĭ��ֵ��300
   * 
   * @param torque ����
   */
  CoppeliaSimJoint& SetMaxTorque(float torque);

  /**
   * @brief ����������ת�٣���λ��rpm��Ĭ��ֵ��5000
   *
   * @param velocity ����
   */
  CoppeliaSimJoint& SetMaxVelocity(float velocity);

  /**
   * @brief ��ȡʵ��λ�ã���λ��rad
   * 
   * @return [float]
   */
  float GetPosition();
  /**
   * @brief ��ȡʵ�����أ���λ��N*m
   *
   * @return [float]
   */
  float GetTorque();
  /**
   * @brief ��ȡʵ���ٶȣ���λ��rpm
   *
   * @return [float]
   */
  float GetVelocity();
  

private:
  void RefreshData(int mode);
  void SetMode(JointMode mode);

  /**
   * @brief ת�ٵ�λ���㡣rad/s -> rpm
   * 
   * @param vel ת�٣���λrad/s
   * @return [float] ת�٣���λrpm
   */
  inline float RadS2RPM(float vel);

  /**
   * @brief ת�ٵ�λ���㡣rpm -> rad/s
   * 
   * @param vel ת�٣���λrpm
   * @return [float] ת�٣���λrad/s
   */
  inline float RPM2RadS(float vel);

  std::string name_;

  JointMode mode_ = JointMode::Idle;

  float position_ = 0;
  float torque_ = 0;
  float velocity_ = 0;

  float max_torque_ = 300;
  float max_velocity_ = 5000;
};

using CoppeliaSimJointSharedPtr = std::shared_ptr<CoppeliaSimJoint>;

#endif // !_COPPELIASIM_JOINT_H

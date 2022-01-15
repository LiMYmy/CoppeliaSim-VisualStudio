#include "coppeliasim_joint.h"

CoppeliaSimJoint::CoppeliaSimJoint(simxInt client_id, std::string name)
  : name_(name)
{
  client_id_ = client_id;
  handler_ = GetHandler(name_);
  simxSetObjectIntParameter(client_id_, handler_, sim_jointintparam_ctrl_enabled, 1, simx_opmode_oneshot);
  simxSetJointMaxForce(client_id_, handler_, max_torque_, simx_opmode_oneshot);
}

/**
* @brief 设置关节目标位置，单位：rad。实际执行效果受PID参数影响。
*/
CoppeliaSimJoint& CoppeliaSimJoint::SetPosition(float pos)
{
  SetMode(JointMode::Position);
  simxSetJointTargetPosition(client_id_, handler_, (simxFloat)pos, simx_opmode_oneshot);
  return *this;
}

/**
* @brief 设置关节目标力矩，单位：N*m
*/
CoppeliaSimJoint& CoppeliaSimJoint::SetTorque(float torque)
{
  SetMode(JointMode::Torque);
  simxSetJointMaxForce(client_id_, handler_, torque, simx_opmode_oneshot);
  return *this;
}

/**
* @brief 设置关节目标速度，单位：rpm
*/
CoppeliaSimJoint& CoppeliaSimJoint::SetVelocity(float vel)
{
  SetMode(JointMode::Velocity);
  simxSetJointTargetVelocity(client_id_, handler_, (simxFloat)RPM2RadS(vel), simx_opmode_oneshot);
  return *this;
}


/**
 * @brief 设置PID参数
 *
 * @param p
 * @param i
 * @param d
 */
CoppeliaSimJoint& CoppeliaSimJoint::SetPID(float p, float i, float d)
{
  simxSetObjectFloatParameter(client_id_, handler_, sim_jointfloatparam_pid_p, (simxFloat)p, simx_opmode_oneshot);
  simxSetObjectFloatParameter(client_id_, handler_, sim_jointfloatparam_pid_i, (simxFloat)i, simx_opmode_oneshot);
  simxSetObjectFloatParameter(client_id_, handler_, sim_jointfloatparam_pid_d, (simxFloat)d, simx_opmode_oneshot);
  return *this;
}

/**
 * @brief 设置最大输出力矩，单位：N*m。默认值：300
 *
 * @param torque 力矩
 */
CoppeliaSimJoint& CoppeliaSimJoint::SetMaxTorque(float torque)
{
  max_torque_ = torque;
  simxSetJointMaxForce(client_id_, handler_, max_torque_, simx_opmode_oneshot);
  return *this;
}

/**
 * @brief 设置最大输出转速，单位：rpm。默认值：5000
 *
 * @param velocity 力矩
 */
CoppeliaSimJoint& CoppeliaSimJoint::SetMaxVelocity(float velocity)
{
  max_velocity_ = velocity;
  simxSetJointTargetVelocity(client_id_, handler_, max_velocity_, simx_opmode_oneshot);
  return *this;
}

/**
 * @brief 获取实际位置，单位：rad
 *
 * @return [float]
 */
float CoppeliaSimJoint::GetPosition()
{
  return position_;
}
/**
 * @brief 获取实际力矩，单位：N*m
 *
 * @return [float]
 */
float CoppeliaSimJoint::GetTorque()
{
  return torque_;
}
/**
 * @brief 获取实际速度，单位：rpm
 *
 * @return [float]
 */
float CoppeliaSimJoint::GetVelocity()
{
  return velocity_;
}

void CoppeliaSimJoint::RefreshData(int mode)
{
  simxGetJointPosition(client_id_, handler_, &position_, mode);
  simxGetObjectFloatParameter(client_id_, handler_, sim_jointfloatparam_velocity, &velocity_, mode);
  simxGetJointForce(client_id_, handler_, &torque_, mode);

  velocity_ = RadS2RPM(velocity_);
}

void CoppeliaSimJoint::SetMode(JointMode mode)
{
  if (mode_ == mode)
    return;

  if (mode == JointMode::Position)
  {
    simxSetObjectIntParameter(client_id_, handler_, sim_jointintparam_ctrl_enabled, 1, simx_opmode_oneshot);
    simxSetJointMaxForce(client_id_, handler_, max_torque_, simx_opmode_oneshot);
  }
  else if (mode == JointMode::Torque)
  {
    simxSetObjectIntParameter(client_id_, handler_, sim_jointintparam_ctrl_enabled, 0, simx_opmode_oneshot);
    simxSetJointTargetVelocity(client_id_, handler_, max_velocity_, simx_opmode_oneshot);
  }
  else if (mode == JointMode::Velocity)
  {
    simxSetObjectIntParameter(client_id_, handler_, sim_jointintparam_ctrl_enabled, 0, simx_opmode_oneshot);
    simxSetJointMaxForce(client_id_, handler_, max_torque_, simx_opmode_oneshot);
  }

  mode_ = mode;
}

/**
 * @brief 转速单位换算。rad/s -> rpm
 *
 * @param vel 转速，单位rad/s
 * @return [float] 转速，单位rpm
 */
inline float CoppeliaSimJoint::RadS2RPM(float vel)
{
  return vel * 30.0 / M_PI;
}

/**
 * @brief 转速单位换算。rpm -> rad/s
 *
 * @param vel 转速，单位rpm
 * @return [float] 转速，单位rad/s
 */
inline float CoppeliaSimJoint::RPM2RadS(float vel)
{
  return vel * M_PI / 30.0;
}



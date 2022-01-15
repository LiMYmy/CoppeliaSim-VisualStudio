#ifndef _COPPELIASIM_JOINT_MANAGER_H
#define _COPPELIASIM_JOINT_MANAGER_H

#include "coppeliasim_joint.h"

class CoppeliaSimJointManager : public CoppeliaSimBase
{
  friend class CoppeliaSim;
public:
  CoppeliaSimJointManager(simxInt client_id);

  /**
   * @brief ��ȡ�ؽ�Joint
   *
   * @param id ��Ҫ��ȡ�Ĺؽ�id
   * @return [CoppeliaSimJoint&] ��id����������Χ�����ص�0�Źؽ�
   */
  CoppeliaSimJointSharedPtr Joint(int id);

  /**
   * @brief ��ȡ�ؽ�Joint�������ٶȱ� Joint(int id) ��������
   *
   * @param id ��Ҫ��ȡ�Ĺؽ�����
   * @return [CoppeliaSimJoint&] ����������Ӧ�Ĺؽڲ����ڣ����ص�0�Źؽ�
   */
  CoppeliaSimJointSharedPtr Joint(std::string name);

private:

  void RegisterJoints(std::vector<std::string> names);

  void RefreshData(int mode = simx_opmode_oneshot);

  //simxInt client_id_;

  std::vector<CoppeliaSimJointSharedPtr> joints_;
  std::map<std::string, int> name_id_map_;

};

using CoppeliaSimJointManagerSharedPtr = std::shared_ptr<CoppeliaSimJointManager>;

#endif // !_COPPELIASIM_JOINT_MANAGER_H

#include "coppeliasim_joint_manager.h"

CoppeliaSimJointManager::CoppeliaSimJointManager(simxInt client_id)
{
  client_id_ = client_id;
}

void CoppeliaSimJointManager::RegisterJoints(std::vector<std::string> names)
{
  for (auto i : names)
  {
    CoppeliaSimJointSharedPtr joint = std::make_shared<CoppeliaSimJoint>(client_id_, i);
    joints_.push_back(joint);
    name_id_map_.insert(std::pair<std::string, int>(i, joints_.size() + 1));
  }
}

void CoppeliaSimJointManager::RefreshData(int mode)
{
  for (int i = 0; i < joints_.size(); i++)
  {
    joints_[i]->RefreshData(mode);
  }
}

/**
 * @brief ��ȡ�ؽ�Joint
 *
 * @param id ��Ҫ��ȡ�Ĺؽ�id
 * @return [CoppeliaSimJoint&] ��id����������Χ�����ص�0�Źؽ�
 */
CoppeliaSimJointSharedPtr CoppeliaSimJointManager::Joint(int id)
{
  using namespace std;
  if (id < joints_.size())
  {
    return joints_[id];
  }
  else
  {
    logger->error("Link id {} is out of range", id);
    return joints_[0];
  }
}

/**
 * @brief ��ȡ�ؽ�Joint�������ٶȱ� Joint(int id) ��������
 *
 * @param id ��Ҫ��ȡ�Ĺؽ�����
 * @return [CoppeliaSimJoint&] ����������Ӧ�Ĺؽڲ����ڣ����ص�0�Źؽ�
 */
CoppeliaSimJointSharedPtr CoppeliaSimJointManager::Joint(std::string name)
{
  using namespace std;

  auto val = name_id_map_.find(name);
  if (val != name_id_map_.end())
  {
    return Joint(val->second);
  }
  else
  {
    logger->error("Can't find Link name: {}" , name);
    return joints_[0];
  }
}

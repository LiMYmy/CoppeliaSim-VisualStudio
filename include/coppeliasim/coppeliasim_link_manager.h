#ifndef _COPPELIASIM_LINK_MANAGER_H
#define _COPPELIASIM_LINK_MANAGER_H

#include "coppeliasim_link.h"

class CoppeliaSimLinkManager : public CoppeliaSimBase
{
  friend class CoppeliaSim;
public:
  CoppeliaSimLinkManager(simxInt client_id);

  /**
   * @brief ��ȡ����Link
   *
   * @param id ��Ҫ��ȡ������id
   * @return [CoppeliaSimLink&] ��id����������Χ�����ص�0�����ˡ�
   */
  CoppeliaSimLinkSharedPtr Link(int id);

  /**
   * @brief ��ȡ����Link�������ٶȱ� Link(int id) ��������
   *
   * @param id ��Ҫ��ȡ����������
   * @return [CoppeliaSimLink&] ����������Ӧ�����˲����ڣ����ص�0�����ˡ�
   */
  CoppeliaSimLinkSharedPtr Link(std::string name);

private:
  void RegisterLinks(std::vector<std::string> names);

  void RefreshData(int mode = simx_opmode_oneshot);

  std::vector<CoppeliaSimLinkSharedPtr> links_;
  std::map<std::string, int> name_id_map_;
};

using CoppeliaSimLinkManagerSharedPtr = std::shared_ptr<CoppeliaSimLinkManager>;

#endif // !_COPPELIASIM_LINK_MANAGER_H


#include "coppeliasim_link_manager.h"

CoppeliaSimLinkManager::CoppeliaSimLinkManager(simxInt client_id)
{
  client_id_ = client_id;
}

void CoppeliaSimLinkManager::RegisterLinks(std::vector<std::string> names)
{
  for (auto i : names)
  {
    CoppeliaSimLinkSharedPtr link = std::make_shared<CoppeliaSimLink>(client_id_, i);
    links_.push_back(link);
    name_id_map_.insert(std::pair<std::string, int>(i, links_.size() + 1));
  }
}

void CoppeliaSimLinkManager::RefreshData(int mode)
{
  for (int i = 0; i < links_.size(); i++)
  {
    links_[i]->RefreshData(mode);
  }
}

/**
 * @brief ��ȡ����Link
 *
 * @param id ��Ҫ��ȡ������id
 * @return [CoppeliaSimLink&] ��id����������Χ�����ص�0�����ˡ�
 */
CoppeliaSimLinkSharedPtr CoppeliaSimLinkManager::Link(int id)
{
  using namespace std;
  if (id < links_.size())
  {
    return links_[id];
  }
  else
  {
    logger->error("Link id {} is out of range", id);
    return links_[0];
  }
}

/**
 * @brief ��ȡ����Link�������ٶȱ� Link(int id) ��������
 *
 * @param id ��Ҫ��ȡ����������
 * @return [CoppeliaSimLink&] ����������Ӧ�����˲����ڣ����ص�0�����ˡ�
 */
CoppeliaSimLinkSharedPtr CoppeliaSimLinkManager::Link(std::string name)
{
  using namespace std;

  auto val = name_id_map_.find(name);
  if (val != name_id_map_.end())
  {
    return Link(val->second);
  }
  else
  {
    logger->error("Can't find Link name: {}", name);
    return links_[0];
  }
}

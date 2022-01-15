#ifndef _COPPELIASIM_BODY_H
#define _COPPELIASIM_BODY_H

#include "coppeliasim_base.h"

#include <vector>
#include <map>
#include <memory>

class CoppeliaSimLink : protected CoppeliaSimBase
{
  friend class CoppeliaSimLinkManager;
public:
  CoppeliaSimLink(simxInt client_id, std::string name);

  /**
   * @brief ��ȡ�������������ϵ��λ��
   * 
   * @return [float[3]] ����(x, y, z)
   */
  float* GetPosition();

  /**
   * @brief ��ȡ�������������ϵ����̬
   *
   * @return [float[3]] ��̬(alpha, beta, gamma)
   */
  float* GetOrientation();

private:
  void RefreshData(int mode);

  std::string name_;

  float position_[3] = { 0 };
  float orientation_[3] = { 0 };
};

using CoppeliaSimLinkSharedPtr = std::shared_ptr<CoppeliaSimLink>;

#endif // !_COPPELIASIM_BODY_H


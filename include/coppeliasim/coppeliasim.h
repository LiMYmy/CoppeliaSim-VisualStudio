#ifndef _COPPELIASIM_H
#define _COPPELIASIM_H

#include "coppeliasim_link.h"
#include "coppeliasim_link_manager.h"
#include "coppeliasim_joint.h"
#include "coppeliasim_joint_manager.h"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"

#include <Windows.h>
#include <iostream>
#include <string>
#include <vector>
#include <memory>

using loggerSharedPtr = std::shared_ptr<spdlog::logger>;

using ConfigFunction = void(*)(CoppeliaSimLinkManagerSharedPtr, CoppeliaSimJointManagerSharedPtr);
using CycleFunction = void(*)(uint64_t, CoppeliaSimLinkManagerSharedPtr, CoppeliaSimJointManagerSharedPtr, int, loggerSharedPtr, loggerSharedPtr);

class CoppeliaSim : public CoppeliaSimBase
{
public:
	CoppeliaSim();
	CoppeliaSim(std::string ip, int32_t port);

	void Init();

	float GetSimulationTimeStep();
	float GetDynamicStepSize();

	/**
	* @brief Can only be written when using a custom dt value (which can be set in the simulation settings dialog), and when simulation is stopped.
	*/
	void SetSimulationTimeStep(float ms);

	/**
	* @brief Can only be written when using a customized configuration for the physics engine, and when simulation is stopped.
	*/
	void SetDynamicStepSize(float ms);

	/**
	 * @brief ���÷������������
	 * 
	 * @param link_name ��������
	 */
	void SetLinks(std::vector<std::string> link_name);

	/**
	 * @brief ���÷�������Ĺؽ�
	 * 
	 * @param joint_name �ؽ�����
	 */
	void SetJoints(std::vector<std::string> joint_name);

	/**
	 * @brief ���������������������Ὣ���õ�������ؽڴ��룬�û�������в���
	 * 
	 * @param func �û��Զ������к���
	 */
	void Run(ConfigFunction config_func, CycleFunction cycle_func);

	/**
	 * @brief ��ȡѭ������
	 * 
	 * @return [uint64_t] ѭ������
	 */
	uint64_t GetCycleTimes();

	std::shared_ptr<spdlog::logger> logger;
	std::shared_ptr<spdlog::logger> file_recorder;

private:
	void Pause();
	void Stop();
	void Trigger();

	std::string ip_ = "127.0.0.1";
	int32_t port_ = 19997;
	simxInt client_id_ = 0;
	
	uint64_t cycle_times_ = 0;

	CoppeliaSimLinkManagerSharedPtr links_;
	CoppeliaSimJointManagerSharedPtr joints_;

	int keyboard_ = 0;

};


#endif // !_COPPELIASIM_H


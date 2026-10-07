//
// Created by cosmosmount on 2025/8/29.
//

#ifndef RM26_PID_HPP
#define RM26_PID_HPP

#include "math.hpp"
#include <cstdint>

/**
 * @brief PID模式
 */
enum PidModeType
{
    PID_POSITION = 0x01, // 位置式 PID
    PID_DELTA = 0x02,    // 增量式 PID
};

/**
 * @brief PID类
 */
class PID
{
public:
    /* 参考值、反馈值和误差均须使用相同物理量单位。 */
    uint8_t mode;

    float kp;
    float ki;
    float kd;

    float ref;
    float fdb;
    float err[3]{}; // err[0] 最新误差；err[1]/err[2] 保存前两次误差。

    float pResult;
    float iResult;
    float dResult;
    float result;

    float maxOut;  // 最终输出绝对限幅。
    float maxIOut; // 积分项绝对限幅。

    /** 构造 PID 控制器；mode 选择位置式或增量式算法。 */
    PID(float kp, float ki, float kd, float maxOut, float maxIOut, int mode = PID_POSITION);
    /** 在线更新比例、积分和微分系数。 */
    void Tuning(float tuning_kp, float tuning_ki, float tuning_kd);
    /** 根据 ref-fdb 更新本周期控制输出及误差历史。 */
    void UpdateResult();
    /** 清零输出、积分状态和历史误差。 */
    void Clear();
};

#endif //RM26_PID_HPP

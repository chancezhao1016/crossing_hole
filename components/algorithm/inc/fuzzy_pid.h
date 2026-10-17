#ifndef FUZZY_PID_H
#define FUZZY_PID_H

#include "stdint.h"

/*************************** 模糊集（7 级） ***************************/
#define NB -3  // 负大
#define NM -2  // 负中
#define NS -1  // 负小
#define ZE  0  // 零
#define PS  1  // 正小
#define PM  2  // 正中
#define PB  3  // 正大

typedef struct
{
    uint8_t enable;   // 开关：1=使能 0=禁用

    float KpFuzzy;  // 模糊输出：Kp 增量
    float KiFuzzy;  // 模糊输出：Ki 增量
    float KdFuzzy;  // 模糊输出：Kd 增量

    float (*FuzzyRuleKp)[7];  // 7x7 Kp 规则表
    float (*FuzzyRuleKi)[7];  // 7x7 Ki 规则表
    float (*FuzzyRuleKd)[7];  // 7x7 Kd 规则表

    float KpRatio;  // Kp 增量缩放
    float KiRatio;  // Ki 增量缩放
    float KdRatio;  // Kd 增量缩放

    float eStep;   // 误差量化步长
    float ecStep;  // 误差率量化步长

    float e;       // 误差
    float ec;      // 误差率
    float eLast;   // 上次误差

    uint32_t last_tick;  // HAL_GetTick 时间戳(ms)
    float dt;            // 控制周期(s)
} FuzzyRule_t;

/* 初始化：规则表可为 NULL（用默认表），ratio 为增量缩放，eStep/ecStep 为量化步长 */
void Fuzzy_Rule_Init(FuzzyRule_t *fuzzyRule, float (*fuzzyRuleKp)[7], float (*fuzzyRuleKi)[7], float (*fuzzyRuleKd)[7],
                     float kpRatio, float kiRatio, float kdRatio,
                     float eStep, float ecStep);

/* 实现：输入 measure(实际值) ref(目标值)，输出 KpFuzzy/KiFuzzy/KdFuzzy */
void Fuzzy_Rule_Implementation(FuzzyRule_t *fuzzyRule, float measure, float ref);

#endif

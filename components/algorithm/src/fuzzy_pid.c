#include "fuzzy_pid.h"
#include "main.h"  // HAL_GetTick

/*************************** 默认模糊规则表（7x7，沿用 crossing_hole） ***************************/
static float FuzzyRuleKpRAW[7][7] = {
    PB, PB, PM, PM, PS, ZE, ZE,
    PB, PB, PM, PS, PS, ZE, PS,
    PM, PM, PM, PS, ZE, PS, PS,
    PM, PM, PS, ZE, PS, PM, PM,
    PS, PS, ZE, PS, PS, PM, PM,
    PS, ZE, PS, PM, PM, PM, PB,
    ZE, ZE, PM, PM, PM, PB, PB};

static float FuzzyRuleKiRAW[7][7] = {
    PB, PB, PM, PM, PS, ZE, ZE,
    PB, PB, PM, PS, PS, ZE, ZE,
    PB, PM, PM, PS, ZE, PS, PS,
    PM, PM, PS, ZE, PS, PM, PM,
    PS, PS, ZE, PS, PS, PM, PB,
    ZE, ZE, PS, PS, PM, PB, PB,
    ZE, ZE, PS, PM, PM, PB, PB};

static float FuzzyRuleKdRAW[7][7] = {
    PS, PS, PB, PB, PB, PM, PS,
    PS, PS, PB, PM, PM, PS, ZE,
    ZE, PS, PM, PM, PS, PS, ZE,
    ZE, PS, PS, PS, PS, PS, ZE,
    ZE, ZE, ZE, ZE, ZE, ZE, ZE,
    PB, PS, PS, PS, PS, PS, PB,
    PB, PM, PM, PM, PS, PS, PB};

void Fuzzy_Rule_Init(FuzzyRule_t *fuzzyRule, float (*fuzzyRuleKp)[7], float (*fuzzyRuleKi)[7], float (*fuzzyRuleKd)[7],
                     float kpRatio, float kiRatio, float kdRatio,
                     float eStep, float ecStep)
{
    fuzzyRule->FuzzyRuleKp = (fuzzyRuleKp == NULL) ? FuzzyRuleKpRAW : fuzzyRuleKp;
    fuzzyRule->FuzzyRuleKi = (fuzzyRuleKi == NULL) ? FuzzyRuleKiRAW : fuzzyRuleKi;
    fuzzyRule->FuzzyRuleKd = (fuzzyRuleKd == NULL) ? FuzzyRuleKdRAW : fuzzyRuleKd;

    fuzzyRule->KpRatio = kpRatio;
    fuzzyRule->KiRatio = kiRatio;
    fuzzyRule->KdRatio = kdRatio;

    if (eStep < 0.00001f) eStep = 1.0f;
    if (ecStep < 0.00001f) ecStep = 1.0f;
    fuzzyRule->eStep = eStep;
    fuzzyRule->ecStep = ecStep;

    fuzzyRule->e = 0.0f;
    fuzzyRule->ec = 0.0f;
    fuzzyRule->eLast = 0.0f;
    fuzzyRule->last_tick = 0;
    fuzzyRule->dt = 0.001f;
}

void Fuzzy_Rule_Implementation(FuzzyRule_t *fuzzyRule, float measure, float ref)
{
    float eLeftTemp, ecLeftTemp, eRightTemp, ecRightTemp;
    int eLeftIndex, ecLeftIndex, eRightIndex, ecRightIndex;

    /* 控制周期（HAL_GetTick，ms -> s） */
    uint32_t now = HAL_GetTick();
    fuzzyRule->dt = (float)(now - fuzzyRule->last_tick) / 1000.0f;
    fuzzyRule->last_tick = now;
    if (fuzzyRule->dt <= 0.0f || fuzzyRule->dt > 0.1f)
        fuzzyRule->dt = 0.001f;  // 兜底按 1kHz

    /* 误差和误差率 */
    fuzzyRule->e = ref - measure;
    fuzzyRule->ec = (fuzzyRule->e - fuzzyRule->eLast) / fuzzyRule->dt;
    fuzzyRule->eLast = fuzzyRule->e;

    /* 隶属区间（索引 0~6 对应 NB~PB） */
    eLeftIndex  = fuzzyRule->e >= 3 * fuzzyRule->eStep  ? 6 : (fuzzyRule->e <= -3 * fuzzyRule->eStep ? 0 : (fuzzyRule->e >= 0 ? ((int)(fuzzyRule->e / fuzzyRule->eStep) + 3) : ((int)(fuzzyRule->e / fuzzyRule->eStep) + 2)));
    eRightIndex = fuzzyRule->e >= 3 * fuzzyRule->eStep  ? 6 : (fuzzyRule->e <= -3 * fuzzyRule->eStep ? 0 : (fuzzyRule->e >= 0 ? ((int)(fuzzyRule->e / fuzzyRule->eStep) + 4) : ((int)(fuzzyRule->e / fuzzyRule->eStep) + 3)));
    ecLeftIndex  = fuzzyRule->ec >= 3 * fuzzyRule->ecStep ? 6 : (fuzzyRule->ec <= -3 * fuzzyRule->ecStep ? 0 : (fuzzyRule->ec >= 0 ? ((int)(fuzzyRule->ec / fuzzyRule->ecStep) + 3) : ((int)(fuzzyRule->ec / fuzzyRule->ecStep) + 2)));
    ecRightIndex = fuzzyRule->ec >= 3 * fuzzyRule->ecStep ? 6 : (fuzzyRule->ec <= -3 * fuzzyRule->ecStep ? 0 : (fuzzyRule->ec >= 0 ? ((int)(fuzzyRule->ec / fuzzyRule->ecStep) + 4) : ((int)(fuzzyRule->ec / fuzzyRule->ecStep) + 3)));

    /* 隶属度 */
    eLeftTemp  = fuzzyRule->e >= 3 * fuzzyRule->eStep  ? 0 : (fuzzyRule->e <= -3 * fuzzyRule->eStep ? 1 : (eRightIndex - fuzzyRule->e / fuzzyRule->eStep - 3));
    eRightTemp = fuzzyRule->e >= 3 * fuzzyRule->eStep  ? 1 : (fuzzyRule->e <= -3 * fuzzyRule->eStep ? 0 : (fuzzyRule->e / fuzzyRule->eStep - eLeftIndex + 3));
    ecLeftTemp  = fuzzyRule->ec >= 3 * fuzzyRule->ecStep ? 0 : (fuzzyRule->ec <= -3 * fuzzyRule->ecStep ? 1 : (ecRightIndex - fuzzyRule->ec / fuzzyRule->ecStep - 3));
    ecRightTemp = fuzzyRule->ec >= 3 * fuzzyRule->ecStep ? 1 : (fuzzyRule->ec <= -3 * fuzzyRule->ecStep ? 0 : (fuzzyRule->ec / fuzzyRule->ecStep - ecLeftIndex + 3));

    /* 双线性插值求模糊输出（注意：修正了 crossing_hole 中间两项索引写反的问题） */
    fuzzyRule->KpFuzzy = fuzzyRule->KpRatio * (eLeftTemp * ecLeftTemp * fuzzyRule->FuzzyRuleKp[eLeftIndex][ecLeftIndex] +
                                               eLeftTemp * ecRightTemp * fuzzyRule->FuzzyRuleKp[eLeftIndex][ecRightIndex] +
                                               eRightTemp * ecLeftTemp * fuzzyRule->FuzzyRuleKp[eRightIndex][ecLeftIndex] +
                                               eRightTemp * ecRightTemp * fuzzyRule->FuzzyRuleKp[eRightIndex][ecRightIndex]);

    fuzzyRule->KiFuzzy = fuzzyRule->KiRatio * (eLeftTemp * ecLeftTemp * fuzzyRule->FuzzyRuleKi[eLeftIndex][ecLeftIndex] +
                                               eLeftTemp * ecRightTemp * fuzzyRule->FuzzyRuleKi[eLeftIndex][ecRightIndex] +
                                               eRightTemp * ecLeftTemp * fuzzyRule->FuzzyRuleKi[eRightIndex][ecLeftIndex] +
                                               eRightTemp * ecRightTemp * fuzzyRule->FuzzyRuleKi[eRightIndex][ecRightIndex]);

    fuzzyRule->KdFuzzy = fuzzyRule->KdRatio * (eLeftTemp * ecLeftTemp * fuzzyRule->FuzzyRuleKd[eLeftIndex][ecLeftIndex] +
                                               eLeftTemp * ecRightTemp * fuzzyRule->FuzzyRuleKd[eLeftIndex][ecRightIndex] +
                                               eRightTemp * ecLeftTemp * fuzzyRule->FuzzyRuleKd[eRightIndex][ecLeftIndex] +
                                               eRightTemp * ecRightTemp * fuzzyRule->FuzzyRuleKd[eRightIndex][ecRightIndex]);
}

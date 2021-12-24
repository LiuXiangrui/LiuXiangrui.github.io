---
title: VTM代码阅读：xCheckRDCostMerge2Nx2N
date: 2021-12-13 00:30:25
tags: 
- 帧间预测
- VVC
- 代码阅读
categories: 视频编码
mathjax: true
---

# void EncCu::xCheckRDCostMerge2Nx2N
## 输入
- `CodingStructure *&tempCS`
- `CodingStructure *&bestCS`
- `Partitioner &partitioner`
- `const EncTestMode& encTestMode`

## 功能
`xCheckRDCostMerge2Nx2N` 函数根据SATD-Cost选择Merge/Skip模式的最佳候选MVP，所涉及的帧间预测模式包括常规Merge模式(Regular Merge)，带有运动矢量差的Merge模式(MMVD)以及帧内帧间联合预测模式(CIIP)

## 流程
### 获取Merge模式候选列表
   1. 调用 `PU::getInterMergeCandidates` 函数获取常规Merge模式候选列表
   2. 调用 `PU::getInterMMVDMergeCandidates` 函数获取MMVD模式候选列表
### 初始化候选Merge模式信息列表 `RdModeList`

#### 候选Merge模式信息列表长度=常规Merge候选数量 `MRG_MAX_NUM_CANDS` + MMVD候选数量 `MMVD_ADD_NUM`

#### 模式信息 `ModeInfo` 定义
```c++
struct ModeInfo
{
    uint32_t mergeCand;  // Merge候选模式索引
    bool     isRegularMerge;  // 常规Merge Flag
    bool     isMMVD;  // MMVD Flag
    bool     isCIIP;  // CIIP Flag
    // ... 省略构造函数
};
```

#### 使用SATD-Cost进行粗选，缩减候选数量至 `uiNumMrgSATDCand`
1. 获取当前块在缓存中的最佳编码模式，根据其最佳模式确定缩减后模式数量 
2. 若最佳模式为Skip模式，则 `uiNumMrgSATDCand` 为可用常规Merge模式数量 `numValidMergeCand`
3. 若最佳模式为MMVD Skip模式，则`uiNumMrgSATDCand` 为可用常规Merge模式数量 + MMVD模式数量
4. 若最佳模式不是Skip模式，则根据候选模式的SATD-Cost确定 `uiNumMrgSATDCand` 
   1. 遍历常规Merge候选模式
      1. 调用函数 `InterPrediction::motionCompensation` 进行运动补偿获得预测值，并使用DMVR对MV进行细化。
      2. 根据预测值的亮度分量计算失真 `uiSad`,并根据 `uiSad` 和比特率 `fracBits` 计算损失 `cost`
      3. 调用函数 `TU::updateCandList` 更新 `RdModeList`，将代价小的模式移植列表前面
   2. 若CIIP模式可用，则遍历RD模式列表中的前几个模式（最多遍历4个模式）
      1. 调用函数 `IntraPrediction::geneWeightedPred` 计算CIIP预测像素
      2. 计算失真 `uiSad` 和损失 `cost`
      3. 调用函数 `TU::updateCandList` 更新 `RdModeList`，将代价小的模式移植列表前面
   3. 遍历MMVD候选模式
      1. 根据初始MV派生扩展MVMV
      2. 调用函数 `InterPrediction::motionCompensation` 进行运动补偿获得预测值
      3. 计算失真 `uiSad`和损失 `cost`
      4. 调用函数 `TU::updateCandList` 更新 `RdModeList`，将代价小的模式移植列表前面
   4. 若 `RdModeList[i] > RdModeList[0] * MRG_FAST_RATIO`， 则将 `uiNumMrgSATDCand` 设置为 `i`

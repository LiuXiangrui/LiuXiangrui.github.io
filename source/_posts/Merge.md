---
title: Merge模式
date: 2021-12-13 00:30:25
tags: 
- 帧间预测
- VVC
categories: 视频编码
mathjax: true
---

# 原理

Merge模式利用空时域相邻MV的相关性，为当前PU建立候选MVP列表。编码器使用RDO选出最优的预测MV并将其作为当前PU的MV，**仅需传输预测MV的索引**

# 工作流程

1. 构建MVP候选列表
2. 遍历候选列表，根据RDCost选择Merge模式最优候选MVP
3. 利用运动估计寻找最优MV
4. 更新帧间预测最优MVP
5. 编码MVP索引

## VTM平台中对应函数

- `CommonAnalyserLib::PU::getInterMergeCandidates()`： {% post_link 'getInterMergeCandidates' '建立Merge候选列表'%}
- `EcoderLib::EncCU::xCheckRDCostMerge2Nx2N()`：{% post_link 'xCheckRDCostMerge2Nx2N'  '根据RDCost选择Merge/Skip模式中最佳候选MV' %}



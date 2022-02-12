---
title: VVC中的Merge模式
date: 2021-12-13 00:30:25
tags: 
- 帧间预测
- VVC
categories: 视频编码
mathjax: true
---

# 原理
- Merge模式是一种对运动信息进行预测编码的方法
- Merge模式利用空时域MV的相关性，为PU建立候选MVP列表
- 编码器根据率失真代价从候选列表中选出最优MPV，仅传输最优MVP的索引

# 工作流程

## 构建MVP候选列表

### 概述
- VVC中候选列表长度设置为 `MRG_MAX_NUM_CANDS = 6`，包括：
  - 空域候选项 (最多4个)
  - 时域候选项 (最多1个)
  - HMVP候选项 (最多填充至 `MRG_MAX_NUM_CANDS-1`)
  - 逐对的平均MVP候选项 (最多1个)
  - 零向量 (填充至列表长度)
- 若构建过程中参考列表被填满，则提前终止构建过程

### 空域候选项

![Merge-Candidate-List-for-3D-HEVC-The-difference-between-3DHEVC-and-HEVC-lies-in-the.png](https://s2.loli.net/2021/12/19/Hq4Q2OPtygeTMdo.png)

- 按照$B_1\rightarrow A_1\rightarrow B_0\rightarrow A_0\rightarrow B_2$选取**最多4个MV作为空域候选MVP**
- 当候选列表加入$B_1$后，新加入的候选MVP需要进行冗余性检查

### 时域候选项 (TMVP)

- 当启用时域候选且 $\text{PU\_Size} \geq 8\times 8$ 时，将相邻已编码帧中同位PU的MV作为时域候选项
- 时域候选列表长度为1，按照$RB \rightarrow  \text{ColCenter}$的顺序查询同位PU的MV
- 时域候选项需要根据当前帧到其参考帧的距离 $\Delta POC_{cur}$ 与同位帧到其参考帧的距离 $\Delta POC_{col}$ 的比例对MV进行缩放，即
$$
MV_{cur} = \frac{\Delta POC_{cur}}{\Delta POC_{col}} *MV_{col}
$$

### 历史候选项 (HMVP)
- 历史候选项来源于同一CTU中模式为帧间预测的已编码CU
- CTU维护一个基于FIFO的查找表LUT，用于记录已编码CU的运动信息，需要查询时从后向前遍历LUT，并在冗余性检查通过后加入Merge候选列表
- 当候选列表长度为 `MRG_MAX_NUM_CANDS-1` 时，结束HMVP查询

### 逐对的平均MVP 
- 根据当前候选列表前两项 `p0Cand` 和 `p1Cand` 计算平均MV并加入候选列表
- 若仅有一个MV可用，则直接将该MV加入Merge候选列表

## 遍历候选列表，根据RDCost选择Merge模式最优候选MVP

## 利用运动估计寻找最优MV

## 更新帧间预测最优MVP

## 编码MVP索引

## VTM平台中对应函数

- `PU::getInterMergeCandidates()`： {% post_link 'getInterMergeCandidates' '建立Merge候选列表'%}
- `PU::getInterMMVDMergeCandidates`：{% post_link 'MMVD' '建立MMVD模式候选列表'%}
- `EncCU::xCheckRDCostMerge2Nx2N()`：{% post_link 'xCheckRDCostMerge2Nx2N'  '根据RDCost选择Merge/Skip模式中最佳候选MV' %}



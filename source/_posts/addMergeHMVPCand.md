---
title: VTM代码阅读：PU::addMergeHMVPCand
date: 2021-12-13 00:30:25
tags: 
- 帧间预测
- VVC
- 代码阅读
categories: 视频编码
mathjax: false
---

# 定义

```c++
bool PU::addMergeHMVPCand(const CodingStructure &cs, MergeCtx &mrgCtx, const int &mrgCandIdx,
                          const uint32_t maxNumMergeCandMin1, int &cnt, const bool isAvailableA1,
                          const MotionInfo miLeft, const bool isAvailableB1, const MotionInfo miAbove,
                          const bool ibcFlag, const bool isGt4x4
```

# 输入
- `cconst CodingStructure &cs`
- `MergeCtx& mrgCtx`
- `const int &mrgCandIdx`
- `const uint32_t maxNumMergeCandMin1`
- `int &cnt`
- `const bool isAvailableA1`
- `const MotionInfo miLeft`
- `const bool isAvailableB1`
- `const MotionInfo miAbove`
- `const bool ibcFlag`
- `const bool isGt4x4`

# 功能
`addMergeHMVPCand` 函数用于查询历史MVP候选

# 流程
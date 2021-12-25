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
`addMergeHMVPCand` 函数用于向Merge候选列表中添加历史MVP(HMVP)候选

# 流程

## CTU维护一个基于FIFO的查找表LUT，用于记录已编码CU的运动信息
### LUT定义
```c++
struct LutMotionCand
{
  static_vector<MotionInfo, MAX_NUM_HMVP_CANDS> lut;
  static_vector<MotionInfo, MAX_NUM_HMVP_CANDS> lutIbc;  // IBC模式的LUT
};
```

### LUT更新
```c++
void CodingStructure::addMiToLut(static_vector<MotionInfo, MAX_NUM_HMVP_CANDS> &lut, const MotionInfo &mi)
{
  size_t currCnt = lut.size();

  bool pruned      = false;
  int  sameCandIdx = 0;

  for (int idx = 0; idx < currCnt; idx++)
  {
    if (lut[idx] == mi)
    {
      sameCandIdx = idx;
      pruned      = true;
      break;
    }
  }

  if (pruned || currCnt == lut.capacity())
  {
    lut.erase(lut.begin() + sameCandIdx);  // 若新增项与已有项运动信息相同，则弹出已有项
  }

  lut.push_back(mi);
}
```

### 从后向前查询LUT，并在冗余性检查通过后加入Merge候选列表
```c++
   for (int mrgIdx = 1; mrgIdx <= num_avai_candInLUT; mrgIdx++)
   {
      miNeighbor = lut[num_avai_candInLUT - mrgIdx];  // 从后向前查询LUT
      if ( mrgIdx > 2 || ((mrgIdx > 1 || !isGt4x4) && ibcFlag)  // Merge候选列表冗余性检查
      || ((!isAvailableA1 || (miLeft != miNeighbor)) && (!isAvailableB1 || (miAbove != miNeighbor))) )
      {
        //... 加入Merge候选列表
      }
   }
```

### 当候选列表长度等于 `maxNumMergeCand-1` 时，结束查询HMVP候选
```c++
   if (cnt  == maxNumMergeCandMin1)
   {
      break;
   }
```

# 代码
```c++
bool PU::addMergeHMVPCand(const CodingStructure &cs, MergeCtx &mrgCtx, const int &mrgCandIdx,
                          const uint32_t maxNumMergeCandMin1, int &cnt, const bool isAvailableA1,
                          const MotionInfo miLeft, const bool isAvailableB1, const MotionInfo miAbove,
                          const bool ibcFlag, const bool isGt4x4
)
{
  const Slice& slice = *cs.slice;
  MotionInfo miNeighbor;

  auto &lut = ibcFlag ? cs.motionLut.lutIbc : cs.motionLut.lut;  // 历史CU运动信息查找表
  int num_avai_candInLUT = (int)lut.size();


  for (int mrgIdx = 1; mrgIdx <= num_avai_candInLUT; mrgIdx++)
  {
    miNeighbor = lut[num_avai_candInLUT - mrgIdx];  // 从后向前查询LUT


    if ( mrgIdx > 2 || ((mrgIdx > 1 || !isGt4x4) && ibcFlag)  // Merge候选列表冗余性检查
      || ((!isAvailableA1 || (miLeft != miNeighbor)) && (!isAvailableB1 || (miAbove != miNeighbor))) )
    {
      mrgCtx.interDirNeighbours[cnt] = miNeighbor.interDir;  // 向Merge候选列表加入HMVP
      mrgCtx.useAltHpelIf      [cnt] = !ibcFlag && miNeighbor.useAltHpelIf;
      mrgCtx.BcwIdx            [cnt] = (miNeighbor.interDir == 3) ? miNeighbor.BcwIdx : BCW_DEFAULT;

      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miNeighbor.mv[0], miNeighbor.refIdx[0]);

      if (slice.isInterB())  // B帧
      {
        mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miNeighbor.mv[1], miNeighbor.refIdx[1]);
      }

      if (mrgCandIdx == cnt)
      {
        return true;
      }
      cnt ++;

      if (cnt  == maxNumMergeCandMin1)  // 当Merge列表长度为最大长度-1时，停止HMVP
      {
        break;
      }
    }
  }

  if (cnt < maxNumMergeCandMin1)
  {
    mrgCtx.useAltHpelIf[cnt] = false;
  }

  return false;
}
```
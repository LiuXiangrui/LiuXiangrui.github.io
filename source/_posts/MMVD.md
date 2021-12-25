---
title: 带有运动矢量差的Merge(MMVD)
date: 2021-12-13 00:30:25
tags: 
- 帧间预测
- VVC
categories: 视频编码
mathjax: true
---

# 原理
## MMVD对常规Merge模式的MV进行细化以提高Merge模式的预测精度

- MMVD在常规Merge候选列表选取 `MMVD_BASE_MV_NUM=2` 个初始MV进行细化，获得 `MMVD_BASE_MV_NUM * MMVD_REFINE_STEP*4=64` 个细化MV
- MMVD使用初始MV的索引，细化搜索步长索引，细化搜索方向索引三个语法元素表示细化后的MV

## MMVD定义了 `MMVD_REFINE_STEP=8` 种搜索步长
| 偏移量索引 | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
|:----:|:----:|:----:|:----:|:----:|:----:|:----:|:----:|:----:|
| 偏移量(Y分量) | 1/4 | 1/2 | 1 | 2 | 4 | 8 | 16 | 32 |

## MMVD定义了四种搜索方向
| 偏移方向索引 | 00 | 01 | 10 | 11 | 
|:----:|:----:|:----:|:----:|:----:|
| x轴偏移 | + | - | N/A | N/A |
| y轴偏移 | N/A | N/A | + | - |


# 流程
## 在 `EncCu::xCheckRDCostMerge2Nx2N` 函数中获取MMVD候选并进行率失真选择
```c++
void EncCu::xCheckRDCostMerge2Nx2N( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
   // ...
       PU::getInterMMVDMergeCandidates(pu, mergeCtx);  // 取得MMVD模式候选

   // ... 常规Merge模式和CIIP模式
   if ( pu.cs->sps->getUseMMVD() )
   {
      cu.mmvdSkip = true;
      pu.regularMergeFlag = true;  // 启用普通Merge列表以获取初始MV
      for (int mmvdMergeCand = 0; mmvdMergeCand < tempNum; mmvdMergeCand++)
      {
         int baseIdx = mmvdMergeCand / MMVD_MAX_REFINE_NUM;
         int refineStep = (mmvdMergeCand - (baseIdx * MMVD_MAX_REFINE_NUM)) / 4;
         if (refineStep >= m_pcEncCfg->getMmvdDisNum())  // 提前终止
         {
            continue;
         }
         mergeCtx.setMmvdMergeCandiInfo(pu, mmvdMergeCand);  // 进行细化搜索，得到MMVD候选
         // ... 运动补偿计算预测值
         // ... 计算SATD-Cost
         // ... 更新候选列表
      }
   }
   // ... 后续操作
}
```


### 调用函数 `PU::getInterMMVDMergeCandidates` 从常规Merge候选列表选取 `MMVD_BASE_MV_NUM=2` 个初始MV

```c++
void PU::getInterMMVDMergeCandidates(const PredictionUnit &pu, MergeCtx& mrgCtx, const int& mrgCandIdx)
{
  int refIdxList0, refIdxList1;
  int k;
  int currBaseNum = 0;
  const uint16_t maxNumMergeCand = mrgCtx.numValidMergeCand;
 
  for (k = 0; k < maxNumMergeCand; k++) //遍历Merge候选列表
  {
    if (mrgCtx.mrgTypeNeighbours[k] == MRG_TYPE_DEFAULT_N)  // 只取常规Merge候选
    {
      refIdxList0 = mrgCtx.mvFieldNeighbours[(k << 1)].refIdx;
      refIdxList1 = mrgCtx.mvFieldNeighbours[(k << 1) + 1].refIdx;
 
      if ((refIdxList0 >= 0) && (refIdxList1 >= 0)) 
      {
        mrgCtx.mmvdBaseMv[currBaseNum][0] = mrgCtx.mvFieldNeighbours[(k << 1)];
        mrgCtx.mmvdBaseMv[currBaseNum][1] = mrgCtx.mvFieldNeighbours[(k << 1) + 1];
      }
      else if (refIdxList0 >= 0)
      {
        mrgCtx.mmvdBaseMv[currBaseNum][0] = mrgCtx.mvFieldNeighbours[(k << 1)];
        mrgCtx.mmvdBaseMv[currBaseNum][1] = MvField(Mv(0, 0), -1);
      }
      else if (refIdxList1 >= 0)
      {
        mrgCtx.mmvdBaseMv[currBaseNum][0] = MvField(Mv(0, 0), -1);
        mrgCtx.mmvdBaseMv[currBaseNum][1] = mrgCtx.mvFieldNeighbours[(k << 1) + 1];
      }
      mrgCtx.mmvdUseAltHpelIf[currBaseNum] = mrgCtx.useAltHpelIf[k];
 
      currBaseNum++;
 
      if (currBaseNum == MMVD_BASE_MV_NUM)  // 跳出
      {
        break;
      }
    }
  }
}
```

### 调用函数 `MergeCtx::setMmvdMergeCandiInfo` 进行细化搜索，得到MMVD候选

```c++
void MergeCtx::setMmvdMergeCandiInfo(PredictionUnit& pu, int candIdx)
{
   int fPosBaseIdx = 0;  // 初始MV索引
   int fPosStep = 0;  // 搜索步长
   int fPosPosition = 0;  // 搜索方向
   // ... 根据MMVD索引candIdx确定初始MV，搜索方向和步长
   // ... 搜索MMVD并加入候选列表
}
```

#### MMVD候选的偏移根据POC距离进行缩放
- 若前后向参考帧到当前帧的POC距离相同，则无需缩放
- 若前向参考帧到当前帧的POC距离更远，则对后向MMVD候选的偏移进行缩放
```c++
   tempMv[1] = tempMv[0].scaleMv(scale);
```

- 若后向参考帧到当前帧的POC距离更远，则对前向MMVD候选的偏移进行缩放
```c++
   tempMv[0] = tempMv[1].scaleMv(scale);
```

#### MMVD候选的偏移符号由初始MV的信息决定
- 当初始MV为单向预测，或者为双向预测且前后双向的参考帧均在当前帧的同一侧时，将搜索方向的符号添加到MMVD候选的偏移值

```c++
   // ... 双向MV
   else if (refList0 != -1)  // 只有前向MV
   {
      if (fPosPosition == 0)
      {
         tempMv[0] = Mv(offset, 0);
      }
      else if (fPosPosition == 1)
      {
      tempMv[0] = Mv(-offset, 0);
      }
      else if (fPosPosition == 2)
      {
         tempMv[0] = Mv(0, offset);
      }
      else
      {
         tempMv[0] = Mv(0, -offset);
      }
    else  // 只有后向MV
    {
        // ... 省略
    }

```

- 当初始MV为双向预测且前后双向的参考帧在当前帧的两侧时，则将搜索方向的符号添加到前向MMVD候选的偏移值，而后向MMVD候选的偏移值取与前向相反的符号

```c++
   // ... 前向偏移
   tempMv[1].set(-1 * tempMv[0].getHor(), -1 * tempMv[0].getVer());
```

#### 将MMVD候选的偏移量加到初始MV，生成MMVD候选
```c++
   pu.mv[REF_PIC_LIST_0] = mmvdBaseMv[fPosBaseIdx][0].mv + tempMv[0];
   pu.refIdx[REF_PIC_LIST_0] = refList0;
   pu.mv[REF_PIC_LIST_1] = mmvdBaseMv[fPosBaseIdx][1].mv + tempMv[1];
   pu.refIdx[REF_PIC_LIST_1] = refList1;
```

# 代码

```c++
void MergeCtx::setMmvdMergeCandiInfo(PredictionUnit& pu, int candIdx)
{
  const Slice &slice = *pu.cs->slice;
  const int mvShift = MV_FRACTIONAL_BITS_DIFF;  // = 2
  // 步长
  const int refMvdCands[8] = { 1 << mvShift , 2 << mvShift , 4 << mvShift , 8 << mvShift , 16 << mvShift , 32 << mvShift,  64 << mvShift , 128 << mvShift };
  int fPosGroup = 0;  // 候选列表初始位置
  int fPosBaseIdx = 0;  // 初始MV索引
  int fPosStep = 0;  // 搜索步长索引
  int tempIdx = 0; 
  int fPosPosition = 0;  // 搜索方向索引
  Mv tempMv[2];  // 存储双向MV的临时变量

#if GDR_ENABLED
  const CodingStructure &cs = *pu.cs;
  const bool isEncodeGdrClean = cs.sps->getGDREnabledFlag() && cs.pcv->isEncoder && ((cs.picHeader->getInGdrInterval() && cs.isClean(pu.Y().topRight(), CHANNEL_TYPE_LUMA)) || (cs.picHeader->getNumVerVirtualBoundaries() == 0));
#endif

  tempIdx = candIdx;
  fPosGroup = tempIdx / (MMVD_BASE_MV_NUM * MMVD_MAX_REFINE_NUM);
  tempIdx = tempIdx - fPosGroup * (MMVD_BASE_MV_NUM * MMVD_MAX_REFINE_NUM);
  fPosBaseIdx = tempIdx / MMVD_MAX_REFINE_NUM;
  tempIdx = tempIdx - fPosBaseIdx * (MMVD_MAX_REFINE_NUM);
  fPosStep = tempIdx / 4;
  fPosPosition = tempIdx - fPosStep * (4);
  int offset = refMvdCands[fPosStep];
  if ( pu.cu->slice->getPicHeader()->getDisFracMMVD() )
  {
    offset <<= 2;
  }
  // 读取初始MV
  const int refList0 = mmvdBaseMv[fPosBaseIdx][0].refIdx;
  const int refList1 = mmvdBaseMv[fPosBaseIdx][1].refIdx;

  if ((refList0 != -1) && (refList1 != -1))
  {
    const int poc0 = slice.getRefPOC(REF_PIC_LIST_0, refList0);
    const int poc1 = slice.getRefPOC(REF_PIC_LIST_1, refList1);
    const int currPoc = slice.getPOC();
    if (fPosPosition == 0)
    {
      tempMv[0] = Mv(offset, 0);  // 向右搜索
    }
    else if (fPosPosition == 1)
    {
      tempMv[0] = Mv(-offset, 0);  // 向左搜索
    }
    else if (fPosPosition == 2)
    {
      tempMv[0] = Mv(0, offset);  // 向下搜索
    }
    else
    {
      tempMv[0] = Mv(0, -offset);  // 向上搜索
    }
    if ((poc0 - currPoc) == (poc1 - currPoc))  // 前后参考帧为同一帧
    {
      tempMv[1] = tempMv[0]; 
    } 
    else if (abs(poc1 - currPoc) > abs(poc0 - currPoc))  // 后向参考帧剧当前帧更远
    {
      const int scale = PU::getDistScaleFactor(currPoc, poc0, currPoc, poc1);  // 缩放因子
      tempMv[1] = tempMv[0];  // 后向MV设置为原前向MV
      const bool isL0RefLongTerm = slice.getRefPic(REF_PIC_LIST_0, refList0)->longTerm;
      const bool isL1RefLongTerm = slice.getRefPic(REF_PIC_LIST_1, refList1)->longTerm;
      if (isL0RefLongTerm || isL1RefLongTerm)  // 存在长期参考帧
      {
        if ((poc1 - currPoc)*(poc0 - currPoc) > 0)  // 前后向参考帧在一侧
        {
          tempMv[0] = tempMv[1];
        }
        else
        {
          tempMv[0].set(-1 * tempMv[1].getHor(), -1 * tempMv[1].getVer());
        }
      }
      else
      {
        tempMv[0] = tempMv[1].scaleMv(scale);  // 前向MV=缩放后的原前向MV
      }
    }
    else  // 前向参考帧剧当前帧更远
    {
      const int scale = PU::getDistScaleFactor(currPoc, poc1, currPoc, poc0);
      const bool isL0RefLongTerm = slice.getRefPic(REF_PIC_LIST_0, refList0)->longTerm;
      const bool isL1RefLongTerm = slice.getRefPic(REF_PIC_LIST_1, refList1)->longTerm;
      if (isL0RefLongTerm || isL1RefLongTerm)
      {
        if ((poc1 - currPoc)*(poc0 - currPoc) > 0)  //若前后向参考帧都来自当前帧的时域的同一侧，后向MMVDoffet与前向的相等
        {
          tempMv[1] = tempMv[0];
        }
        else
        {
          tempMv[1].set(-1 * tempMv[0].getHor(), -1 * tempMv[0].getVer());  // 前向和后向的MMVDoffset相对称
        }
      }
      else  // 都不是长期参考帧
      {
        tempMv[1] = tempMv[0].scaleMv(scale);  // 后向MV=缩放后的原前向MV
      }
    }

    pu.interDir = 3;  // 双向MV
    pu.mv[REF_PIC_LIST_0] = mmvdBaseMv[fPosBaseIdx][0].mv + tempMv[0];
    pu.refIdx[REF_PIC_LIST_0] = refList0;
    pu.mv[REF_PIC_LIST_1] = mmvdBaseMv[fPosBaseIdx][1].mv + tempMv[1];
    pu.refIdx[REF_PIC_LIST_1] = refList1;
#if GDR_ENABLED
    if (isEncodeGdrClean)
    {
      Mv mv0 = pu.mv[REF_PIC_LIST_0];
      Mv mv1 = pu.mv[REF_PIC_LIST_1];

      int refIdx0 = pu.refIdx[REF_PIC_LIST_0];
      int refIdx1 = pu.refIdx[REF_PIC_LIST_1];

      mmvdValid[fPosBaseIdx][0] = cs.isClean(pu.Y().topRight(), mv0, REF_PIC_LIST_0, refIdx0);
      mmvdValid[fPosBaseIdx][1] = cs.isClean(pu.Y().topRight(), mv1, REF_PIC_LIST_1, refIdx1);

      pu.mvSolid[REF_PIC_LIST_0] = mmvdSolid[fPosBaseIdx][0];
      pu.mvSolid[REF_PIC_LIST_1] = mmvdSolid[fPosBaseIdx][1];

      pu.mvValid[REF_PIC_LIST_0] = mmvdValid[fPosBaseIdx][0];
      pu.mvValid[REF_PIC_LIST_1] = mmvdValid[fPosBaseIdx][1];
    }
#endif
  }
  else if (refList0 != -1)  // 只有前向MV
  {
    if (fPosPosition == 0)
    {
      tempMv[0] = Mv(offset, 0);
    }
    else if (fPosPosition == 1)
    {
      tempMv[0] = Mv(-offset, 0);
    }
    else if (fPosPosition == 2)
    {
      tempMv[0] = Mv(0, offset);
    }
    else
    {
      tempMv[0] = Mv(0, -offset);
    }
    pu.interDir = 1;
    pu.mv[REF_PIC_LIST_0] = mmvdBaseMv[fPosBaseIdx][0].mv + tempMv[0];
    pu.refIdx[REF_PIC_LIST_0] = refList0;
    pu.mv[REF_PIC_LIST_1] = Mv(0, 0);
    pu.refIdx[REF_PIC_LIST_1] = -1;

#if GDR_ENABLED
    if (isEncodeGdrClean)
    {
      Mv mv0 = pu.mv[REF_PIC_LIST_0];
      //Mv mv1 = pu.mv[REF_PIC_LIST_1];

      int refIdx0 = pu.refIdx[REF_PIC_LIST_0];
      //int refIdx1 = pu.refIdx[REF_PIC_LIST_1];

      pu.mvSolid[REF_PIC_LIST_0] = mmvdSolid[fPosBaseIdx][0];
      pu.mvSolid[REF_PIC_LIST_1] = true;

      mmvdValid[fPosBaseIdx][0] = cs.isClean(pu.Y().topRight(), mv0, REF_PIC_LIST_0, refIdx0);
      mmvdValid[fPosBaseIdx][1] = true;

      pu.mvValid[REF_PIC_LIST_0] = mmvdValid[fPosBaseIdx][0];
      pu.mvValid[REF_PIC_LIST_1] = true;
    }
#endif
  }
  else if (refList1 != -1)
  {
    if (fPosPosition == 0)
    {
      tempMv[1] = Mv(offset, 0);
    }
    else if (fPosPosition == 1)
    {
      tempMv[1] = Mv(-offset, 0);
    }
    else if (fPosPosition == 2)
    {
      tempMv[1] = Mv(0, offset);
    }
    else
    {
      tempMv[1] = Mv(0, -offset);
    }
    pu.interDir = 2;
    pu.mv[REF_PIC_LIST_0] = Mv(0, 0);
    pu.refIdx[REF_PIC_LIST_0] = -1;
    pu.mv[REF_PIC_LIST_1] = mmvdBaseMv[fPosBaseIdx][1].mv + tempMv[1];
    pu.refIdx[REF_PIC_LIST_1] = refList1;
#if GDR_ENABLED
    if (isEncodeGdrClean)
    {
      // Mv mv0 = pu.mv[REF_PIC_LIST_0];
      Mv mv1 = pu.mv[REF_PIC_LIST_1];

      // int refIdx0 = pu.refIdx[REF_PIC_LIST_0];
      int refIdx1 = pu.refIdx[REF_PIC_LIST_1];

      mmvdValid[fPosBaseIdx][0] = true;
      mmvdValid[fPosBaseIdx][1] = cs.isClean(pu.Y().topRight(), mv1, REF_PIC_LIST_1, refIdx1);

      pu.mvSolid[REF_PIC_LIST_0] = true;
      pu.mvSolid[REF_PIC_LIST_1] = mmvdSolid[fPosBaseIdx][1];

      pu.mvValid[REF_PIC_LIST_0] = true;
      pu.mvValid[REF_PIC_LIST_1] = mmvdValid[fPosBaseIdx][1];
    }
#endif
  }

  pu.mmvdMergeFlag = true;
  pu.mmvdMergeIdx = candIdx;
  pu.mergeFlag = true;
  pu.regularMergeFlag = true;
  pu.mergeIdx = candIdx;
  pu.mergeType = MRG_TYPE_DEFAULT_N;
  pu.mvd[REF_PIC_LIST_0] = Mv();
  pu.mvd[REF_PIC_LIST_1] = Mv();
  pu.mvpIdx[REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpIdx[REF_PIC_LIST_1] = NOT_VALID;
  pu.mvpNum[REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpNum[REF_PIC_LIST_1] = NOT_VALID;
  pu.cu->imv = mmvdUseAltHpelIf[fPosBaseIdx] ? IMV_HPEL : 0;

  pu.cu->BcwIdx = (interDirNeighbours[fPosBaseIdx] == 3) ? BcwIdx[fPosBaseIdx] : BCW_DEFAULT;

  for (int refList = 0; refList < 2; refList++)
  {
    if (pu.refIdx[refList] >= 0)
    {
      pu.mv[refList].clipToStorageBitDepth();
    }
  }

  PU::restrictBiPredMergeCandsOne(pu);
}
```
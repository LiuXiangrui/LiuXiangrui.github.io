---
title: 帧内帧间联合预测(CIIP))
date: 2021-12-13 00:30:25
tags: 
- 帧间预测
- VVC
- 代码阅读
categories: 视频编码
mathjax: true
---
# 原理
CIIP预测根据帧间Merge模式预测与帧内Planar模式预测值加权生成预测信号
$$
P_{CIIP} = (wInter * P_{inter} + wIntra * P_{intra} + 2) >> 2 
$$

## 条件
- 当前CU以Merge模式编码
- CU的亮度分量尺寸大于64，且宽高均小于`MAX_CU_SIZE=128`

```c++
void EncCu::xCheckRDCostMerge2Nx2N( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
   bool isIntrainterEnabled = sps.getUseCiip();  // CIIP标志
   if (bestCS->area.lwidth() * bestCS->area.lheight() < 64 || bestCS->area.lwidth() >= MAX_CU_SIZE || bestCS->area.lheight() >= MAX_CU_SIZE)
   {
      isIntrainterEnabled = false;  // CU的亮度分量尺寸大于64，且宽高均小于128
   }
}
```

# 流程

## 调用函数 `IntraPrediction::geneWeightedPred` 计算CIIP预测

```c++
void EncCu::xCheckRDCostMerge2Nx2N( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
   // ...
   if( !bestIsSkip )  // 最佳模式非Skip模式
   {
      // ... 常规Merge模式
      if (isIntrainterEnabled)  // CIIP模式可用
      {
        // ...
        // 遍历CIIP模式需要测试的候选Merge模式
         for (uint32_t mergeCnt = 0; mergeCnt < std::min(std::min(NUM_MRG_SATD_CAND, (const int)mergeCtx.numValidMergeCand), 4); mergeCnt++)
         {
            pu.intraDir[0] = PLANAR_IDX;  // 帧内用Planar模式
            // 产生帧内帧间联合预测像素
            if (mergeCnt == 0)  // 帧内预测只需计算一次
            {
               // ... 计算帧内预测值
            }
            // 计算CIIP预测像素
            m_pcIntraSearch->geneWeightedPred(COMPONENT_Y, pu.cs->getPredBuf(pu).Y(), pu, m_pcIntraSearch->getPredictorPtr2(COMPONENT_Y, intraCnt));

            // ... 计算SATD-Cost
            // ... 更新Merge候选列表
      // ... MMVD模式等其他模式
   }
}
```

### 计算CIIP权重
- 权重 `wIntra` 和 `wMerge` 取决于当前CU的顶部和左侧相邻块的编码模式
- 若top和left相邻块可用且都是帧内编码，则 `wIntra=3, wMerge=1` 
- 若top和left相邻块仅有一个是帧内编码，则 `wIntra=2, wMerge=2` 
- 若top和left相邻块均为帧间编码，将wt设置为1

```c++
void IntraPrediction::geneWeightedPred(const ComponentID compId, PelBuf &pred, const PredictionUnit &pu, Pel *srcBuf)
{
   // ...
   const PredictionUnit *neigh0 = pu.cs->getPURestricted(posBL.offset(-1, 0), pu, CHANNEL_TYPE_LUMA);  // 当前PU的左下相邻块
   const PredictionUnit *neigh1 = pu.cs->getPURestricted(posTR.offset(0, -1), pu, CHANNEL_TYPE_LUMA);  // 当前PU的右上相邻块
   bool isNeigh0Intra = neigh0 && (CU::isIntra(*neigh0->cu));
   bool isNeigh1Intra = neigh1 && (CU::isIntra(*neigh1->cu));
 
   if (isNeigh0Intra && isNeigh1Intra)  // 左侧和上侧相邻块都是帧内编码
   {
      wIntra = 3; wMerge = 1;
   }
   else
   {
      if (!isNeigh0Intra && !isNeigh1Intra)  // 左侧和上侧相邻块均为帧间编码
      {
         wIntra = 1; wMerge = 3;
      }
      else
      {
         wIntra = 2; wMerge = 2;  // 仅有一个是帧内编码
      }
   }
   // ...
}
```

### 计算CIIP预测
```c++
   // ... 计算CIIP权重
   for (int y = 0; y < height; y++)  //对帧内帧间预测信号进行加权平均
   {
      for (int x = 0; x < width; x++) 
      {
         dstBuf[y*dstStride + x] = (wMerge * dstBuf[y*dstStride + x] + wIntra * srcBuf[y*srcStride + x] + 2) >> 2;
      }
   }
```


# 代码
```c++
void IntraPrediction::geneWeightedPred(const ComponentID compId, PelBuf &pred, const PredictionUnit &pu, Pel *srcBuf)
{
  const int            width = pred.width;
  CHECK(width == 2, "Width of 2 is not supported");
  const int            height = pred.height;
  const int            srcStride = width;
  const int            dstStride = pred.stride;
 
  Pel*                 dstBuf = pred.buf;
  int wIntra, wMerge;
 
  const Position posBL = pu.Y().bottomLeft(); //当前PU的左下位置
  const Position posTR = pu.Y().topRight(); //当前PU的右上位置
  /// 当前PU的左下相邻块和右下相邻块
  const PredictionUnit *neigh0 = pu.cs->getPURestricted(posBL.offset(-1, 0), pu, CHANNEL_TYPE_LUMA);
  const PredictionUnit *neigh1 = pu.cs->getPURestricted(posTR.offset(0, -1), pu, CHANNEL_TYPE_LUMA);
  bool isNeigh0Intra = neigh0 && (CU::isIntra(*neigh0->cu));
  bool isNeigh1Intra = neigh1 && (CU::isIntra(*neigh1->cu));
 
  if (isNeigh0Intra && isNeigh1Intra) //左侧和上侧相邻块都是帧内编码
  {
    wIntra = 3; wMerge = 1;
  }
  else
  {
    if (!isNeigh0Intra && !isNeigh1Intra)//左侧和上侧相邻块都不是帧内编码
    {
      wIntra = 1; wMerge = 3;
    }
    else
    {
      wIntra = 2; wMerge = 2;
    }
  }
  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++) //对帧内帧间预测信号进行加权平均
    {
      dstBuf[y*dstStride + x] = (wMerge * dstBuf[y*dstStride + x] + wIntra * srcBuf[y*srcStride + x] + 2) >> 2;
    }
  }
}
```
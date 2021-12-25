---
title: VTM代码阅读：EncCu::xCheckRDCostMerge2Nx2N
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
   void EncCu::xCheckRDCostMerge2Nx2N( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
```

# 输入
- `CodingStructure *&tempCS`
- `CodingStructure *&bestCS`
- `Partitioner &partitioner`
- `const EncTestMode& encTestMode`

# 功能
`xCheckRDCostMerge2Nx2N` 函数根据RD-Cost选择Merge/Skip模式的最佳候选MVP，所涉及的帧间预测模式包括常规Merge模式(Regular Merge)，带有运动矢量差的Merge模式(MMVD)以及帧内帧间联合预测模式(CIIP)

# 流程
## 获取Merge模式候选列表
### 1. 调用 `PU::getInterMergeCandidates` 函数{% post_link 'getInterMergeCandidates' '建立常规Merge模式候选列表'%}

```c++
   PU::getInterMergeCandidates(pu, mergeCtx, 0);
```

### 2. 调用 `PU::getInterMMVDMergeCandidates` 函数获取MMVD模式候选列表

```c++
   PU::getInterMMVDMergeCandidates(pu, mergeCtx);
```

## 初始化候选Merge模式信息列表

### 定义

```c++
  static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  RdModeList;  
```

### 模式信息 `ModeInfo` 定义

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

### 遍历常规Merge和MMVD的候选MVP，加入候选模式列表

```c++
   const int candNum = mergeCtx.numValidMergeCand + (tempCS->sps->getUseMMVD() ? std::min<int>(MMVD_BASE_MV_NUM, mergeCtx.numValidMergeCand) * MMVD_MAX_REFINE_NUM : 0);  

   for (int i = 0; i < candNum; i++) 
   {
      if (i < mergeCtx.numValidMergeCand)
      {
         RdModeList.push_back(ModeInfo(i, true, false, false));
      }
      else
      {
         RdModeList.push_back(ModeInfo(std::min(MMVD_ADD_NUM, i - mergeCtx.numValidMergeCand), false, true, false));
      }
   }
```

## 使用SATD-Cost进行粗选，缩减候选数量至 `uiNumMrgSATDCand`
### 1. 获取当前块在缓存中的最佳编码模式

```c++
   bestIsSkip       = false;

   if( auto blkCache = dynamic_cast< CacheBlkInfoCtrl* >( m_modeCtrl ) )
   {
   if (slice.getSPS()->getIBCFlag())
   {
      ComprCUCtx cuECtx = m_modeCtrl->getComprCUCtx();
      bestIsSkip = blkCache->isSkip(tempCS->area) && cuECtx.bestCU;
   }
   else
   bestIsSkip = blkCache->isSkip( tempCS->area );  // 此处缩进有问题，判断当前块在缓存中的最佳编码模式是否为Skip
   bestIsMMVDSkip = blkCache->isMMVDSkip(tempCS->area);
   }

   if (isIntrainterEnabled) // always perform low complexity check
   {
      bestIsSkip = false;
   }
```

### 2. 若最佳模式为Skip模式，则 `uiNumMrgSATDCand` 为可用常规Merge模式数量 `numValidMergeCand`

```c++
   uiNumMrgSATDCand = mergeCtx.numValidMergeCand;
```
### 3. 若最佳模式为MMVD Skip模式，则`uiNumMrgSATDCand` 为可用常规Merge模式数量 + MMVD模式数量

```c++
   if (bestIsMMVDSkip  ) 
   {
      uiNumMrgSATDCand = mergeCtx.numValidMergeCand + ((mergeCtx.numValidMergeCand > 1) ? MMVD_ADD_NUM : MMVD_ADD_NUM >> 1);
   }
```
### 4. 若最佳模式不是Skip模式，则根据候选模式的SATD-Cost确定 `uiNumMrgSATDCand` 
#### 1. 遍历常规Merge候选模式
1. 调用函数 `InterPrediction::motionCompensation` 进行运动补偿获得预测值，并使用DMVR对MV进行细化。

```c++
   m_pcInterSearch->motionCompensation(pu, *singleMergeTempBuffer, REF_PIC_LIST_X, true, true, &(acMergeTmpBuffer[uiMergeCand]));
   pu.mvRefine = false;
   // 判断是否为双向Merge，进行DMVR
   if( mergeCtx.interDirNeighbours[uiMergeCand] == 3 && mergeCtx.mrgTypeNeighbours[uiMergeCand] == MRG_TYPE_DEFAULT_N )
   {
      mergeCtx.mvFieldNeighbours[2*uiMergeCand].mv   = pu.mv[0];
      mergeCtx.mvFieldNeighbours[2*uiMergeCand+1].mv = pu.mv[1];
      {
         int dx, dy, i, j, num = 0;
         dy = std::min<int>(pu.lumaSize().height, DMVR_SUBCU_HEIGHT);
         dx = std::min<int>(pu.lumaSize().width, DMVR_SUBCU_WIDTH);
         if (PU::checkDMVRCondition(pu))
         {
            for (i = 0; i < (pu.lumaSize().height); i += dy)
            {
               for (j = 0; j < (pu.lumaSize().width); j += dx)
               {
               refinedMvdL0[num][uiMergeCand] = pu.mvdL0SubPu[num];
               num++;
               }
            }
         }
      }
   }
```

2. 根据预测值的亮度分量计算SAD失真 `uiSad`,并根据 `uiSad` 和比特率 `fracBits` 计算损失 `cost`

```c++
   Distortion uiSad = distParam.distFunc(distParam);  // 计算SAD
   m_CABACEstimator->getCtx() = ctxStart;
   uint64_t fracBits = m_pcInterSearch->xCalcPuMeBits(pu);  // 计算比特率
   double cost = (double)uiSad + (double)fracBits * sqrtLambdaForFirstPassIntra;
```

3. 调用函数 `TU::updateCandList` 更新 `RdModeList`，将代价小的模式移植列表前面

```c++
   insertPos = -1;  // 当前模式在Merge候选Cost列表中的位置
   updateCandList(ModeInfo(uiMergeCand, true, false, false), cost, RdModeList, candCostList, uiNumMrgSATDCand, &insertPos);
   if (insertPos != -1)  // 发生了更新
   {
      if (insertPos == RdModeList.size() - 1)
      {
         swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
      }
      else
      {
         for (uint32_t i = uint32_t(RdModeList.size()) - 1; i > insertPos; i--)
         {
            swap(acMergeTempBuffer[i - 1], acMergeTempBuffer[i]);  // 将代价小的模式移植列表前面
         }
         swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
      }
   }
```

#### 2. 若CIIP模式启用标志`isIntrainterEnabled=True`，则遍历 `RdModeList` 前 `NUM_MRG_SATD_CAND` 个模式

1. 生成帧内帧间联合预测

```c++
   if (mergeCnt == 0)
   {
      m_pcIntraSearch->initIntraPatternChType(*pu.cu, pu.Y());
      m_pcIntraSearch->predIntraAng(COMPONENT_Y, pu.cs->getPredBuf(pu).Y(), pu);
      m_pcIntraSearch->switchBuffer(pu, COMPONENT_Y, pu.cs->getPredBuf(pu).Y(), m_pcIntraSearch->getPredictorPtr2(COMPONENT_Y, intraCnt));
   }
   pu.cs->getPredBuf(pu).copyFrom(acMergeTmpBuffer[mergeCand]);
```

2. 调用函数 `IntraPrediction::geneWeightedPred` 计算CIIP预测像素

```c++
   m_pcIntraSearch->geneWeightedPred(COMPONENT_Y, pu.cs->getPredBuf(pu).Y(), pu, m_pcIntraSearch->getPredictorPtr2(COMPONENT_Y, intraCnt));
```

3. 计算SAD失真 `uiSad` 和损失 `cost`
4. 调用函数 `TU::updateCandList` 更新 `RdModeList`，将代价小的模式移植列表前面

#### 3. 遍历MMVD候选模式
1. 调用函数 `MergeCtx::setMmvdMergeCandiInfo` 根据初始MV派生扩展MV

```c++
   mergeCtx.setMmvdMergeCandiInfo(pu, mmvdMergeCand);
```

2. 调用函数 `InterPrediction::motionCompensation` 进行运动补偿获得预测值

```c++
   m_pcInterSearch->motionCompensation(pu, *singleMergeTempBuffer, REF_PIC_LIST_X, true, false);
```

1. 计算SAD失真 `uiSad`和损失 `cost`
2. 调用函数 `TU::updateCandList` 更新 `RdModeList`，将代价小的模式移植列表前面

#### 4. 若 `RdModeList[i] > RdModeList[0] * MRG_FAST_RATIO`， 则将 `uiNumMrgSATDCand` 设置为 `i`

```c++
   for( uint32_t i = 1; i < uiNumMrgSATDCand; i++ )
   {
      if( candCostList[i] > MRG_FAST_RATIO * candCostList[0] )
      {
         uiNumMrgSATDCand = i;
         break;
      }
   }
```

## 两次遍历 `RdModeList` 以确定常规Merge模式和Skip模式，选择最优Merge候选
### 1. 初始化当前Merge候选的模式
   1. TODO

### 2. 调用 `EncCu::xEncodeInterResidual` 函数编码当前Merge候选的预测残差，并完成率失真比较

```c++
   xEncodeInterResidual( tempCS, bestCS, partitioner, encTestMode, uiNoResidualPass, uiNoResidualPass == 0 ? &candHasNoResidual[uiMrgHADIdx] : NULL );
```

# 完整代码

```c++
void EncCu::xCheckRDCostMerge2Nx2N( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  const Slice &slice = *tempCS->slice;

  CHECK( slice.getSliceType() == I_SLICE, "Merge modes not available for I-slices" );  // 检查是否为I-Slice

  tempCS->initStructData( encTestMode.qp );

  MergeCtx mergeCtx;
  const SPS &sps = *tempCS->sps;

#if GDR_ENABLED
  bool isEncodeGdrClean = false;
  CodingStructure *cs;
#endif
  if (sps.getSbTMVPEnabledFlag())  // 使用基于子块的TMVP(SbTMVP)
  {
    Size bufSize = g_miScaling.scale( tempCS->area.lumaSize() );
    mergeCtx.subPuMvpMiBuf    = MotionBuf( m_SubPuMiBuf,    bufSize );
  }

  Mv   refinedMvdL0[MAX_NUM_PARTS_IN_CTU][MRG_MAX_NUM_CANDS];
  setMergeBestSATDCost( MAX_DOUBLE );  // 初始化SATDCost为最大值

  {
    // first get merge candidates  获取Merge候选
    CodingUnit cu( tempCS->area );
    cu.cs       = tempCS;
    cu.predMode = MODE_INTER;
    cu.slice    = tempCS->slice;
    cu.tileIdx  = tempCS->pps->getTileIdx( tempCS->area.lumaPos() );

    PredictionUnit pu( tempCS->area );
    pu.cu = &cu;
    pu.cs = tempCS;
    PU::getInterMergeCandidates(pu, mergeCtx, 0);  // 取得常规Merge模式候选
    PU::getInterMMVDMergeCandidates(pu, mergeCtx);  // 取得MMVD模式候选
    pu.regularMergeFlag = true;  // 常规Merge模式
#if GDR_ENABLED
    cs = pu.cs;
    isEncodeGdrClean = cs->sps->getGDREnabledFlag() && cs->pcv->isEncoder && ((cs->picHeader->getInGdrInterval() && cs->isClean(pu.Y().topRight(), CHANNEL_TYPE_LUMA)) || (cs->picHeader->getNumVerVirtualBoundaries() == 0));
#endif
  }
  bool candHasNoResidual[MRG_MAX_NUM_CANDS + MMVD_ADD_NUM];
  for (uint32_t ui = 0; ui < MRG_MAX_NUM_CANDS + MMVD_ADD_NUM; ui++)
  {
    candHasNoResidual[ui] = false;
  }

  bool        bestIsSkip     = false;
  bool        bestIsMMVDSkip = true;
  PelUnitBuf  acMergeBuffer[MRG_MAX_NUM_CANDS];  // 存储常规Merge模式预测像素
  PelUnitBuf  acMergeTmpBuffer[MRG_MAX_NUM_CANDS];  // 存储常规Merge模式预测像素的临时空间
  PelUnitBuf  acMergeRealBuffer[MMVD_MRG_MAX_RD_BUF_NUM];  // 存储常规MMVD Merge模式预测像素
  PelUnitBuf *acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM];  // 存储MMVD Merge模式的临时空间
  PelUnitBuf *singleMergeTempBuffer;  // 存储候选列表Cost最大的项
  int         insertPos;
  unsigned    uiNumMrgSATDCand = mergeCtx.numValidMergeCand + MMVD_ADD_NUM;  // 计算SATD Cost的模式数目

  struct ModeInfo
  {
    uint32_t mergeCand;  // Merge候选
    bool     isRegularMerge;  // 常规Merge Flag
    bool     isMMVD;  // MMVD Flag
    bool     isCIIP;  // CIIP Flag
    ModeInfo() : mergeCand(0), isRegularMerge(false), isMMVD(false), isCIIP(false) {}
    ModeInfo(const uint32_t mergeCand, const bool isRegularMerge, const bool isMMVD, const bool isCIIP) :
      mergeCand(mergeCand), isRegularMerge(isRegularMerge), isMMVD(isMMVD), isCIIP(isCIIP) {}
  };

  static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  RdModeList;  // 保存候选模式信息
  bool                                        mrgTempBufSet = false;
  const int candNum = mergeCtx.numValidMergeCand + (tempCS->sps->getUseMMVD() ? std::min<int>(MMVD_BASE_MV_NUM, mergeCtx.numValidMergeCand) * MMVD_MAX_REFINE_NUM : 0);  

  for (int i = 0; i < candNum; i++)  // 遍历常规Merge和MMVD的候选MVP，加入候选模式列表
  {
    if (i < mergeCtx.numValidMergeCand)
    {
      RdModeList.push_back(ModeInfo(i, true, false, false));
    }
    else
    {
      RdModeList.push_back(ModeInfo(std::min(MMVD_ADD_NUM, i - mergeCtx.numValidMergeCand), false, true, false));
    }
  }

  const UnitArea localUnitArea(tempCS->area.chromaFormat, Area(0, 0, tempCS->area.Y().width, tempCS->area.Y().height));  // 初始化一个和当前CU相同大小的区域
  for (unsigned i = 0; i < MMVD_MRG_MAX_RD_BUF_NUM; i++)  // ????
  {
    acMergeRealBuffer[i] = m_acMergeBuffer[i].getBuf(localUnitArea);
    if (i < MMVD_MRG_MAX_RD_NUM)
    {
      acMergeTempBuffer[i] = acMergeRealBuffer + i;
    }
    else
    {
      singleMergeTempBuffer = acMergeRealBuffer + i;
    }
  }

  bool isIntrainterEnabled = sps.getUseCiip();  // CIIP标志
  if (bestCS->area.lwidth() * bestCS->area.lheight() < 64 || bestCS->area.lwidth() >= MAX_CU_SIZE || bestCS->area.lheight() >= MAX_CU_SIZE)
  {
    isIntrainterEnabled = false;  // 尺寸小于8x8的块不适用于CIIP
  }
  bool isTestSkipMerge[MRG_MAX_NUM_CANDS]; // record if the merge candidate has tried skip mode  记录Merge候选是否尝试Skip模式
  for (uint32_t idx = 0; idx < MRG_MAX_NUM_CANDS; idx++)
  {
    isTestSkipMerge[idx] = false;
  }
  if( m_pcEncCfg->getUseFastMerge() || isIntrainterEnabled)
  {
    uiNumMrgSATDCand = NUM_MRG_SATD_CAND;  // 计算SATD候选数量
    if (isIntrainterEnabled)
    {
      uiNumMrgSATDCand += 1;
    }
    bestIsSkip       = false;

    if( auto blkCache = dynamic_cast< CacheBlkInfoCtrl* >( m_modeCtrl ) )
    {
      if (slice.getSPS()->getIBCFlag())
      {
        ComprCUCtx cuECtx = m_modeCtrl->getComprCUCtx();
        bestIsSkip = blkCache->isSkip(tempCS->area) && cuECtx.bestCU;
      }
      else
      bestIsSkip = blkCache->isSkip( tempCS->area );  // 此处缩进有问题，判断当前块在缓存中的最佳编码模式是否为Skip
      bestIsMMVDSkip = blkCache->isMMVDSkip(tempCS->area);
    }

    if (isIntrainterEnabled) // always perform low complexity check
    {
      bestIsSkip = false;
    }

    static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM> candCostList;  // Merge候选的RDCost列表

    // 1. Pass: get SATD-cost for selected candidates and reduce their count  粗选
    if( !bestIsSkip )  // 最佳模式非Skip模式
    {
      RdModeList.clear();
      mrgTempBufSet       = true;
      const TempCtx ctxStart(m_CtxCache, m_CABACEstimator->getCtx());
      // 初始化TempCS的CU
      CodingUnit &cu      = tempCS->addCU( tempCS->area, partitioner.chType );
      const double sqrtLambdaForFirstPassIntra = m_pcRdCost->getMotionLambda( ) * FRAC_BITS_SCALE;
      partitioner.setCUData( cu );
      cu.slice            = tempCS->slice;
      cu.tileIdx          = tempCS->pps->getTileIdx( tempCS->area.lumaPos() );
      cu.skip             = false;
      cu.mmvdSkip = false;
      cu.geoFlag          = false;
    //cu.affine
      cu.predMode         = MODE_INTER;
    //cu.LICFlag
      cu.chromaQpAdj      = m_cuChromaQpOffsetIdxPlus1;
      cu.qp               = encTestMode.qp;
    //cu.emtFlag  is set below

      PredictionUnit &pu  = tempCS->addPU( cu, partitioner.chType );

      DistParam distParam;  // 初始化Distortion计算方式
      const bool bUseHadamard = !tempCS->slice->getDisableSATDForRD();  // 是否使用HAD进行RDCost计算
      m_pcRdCost->setDistParam (distParam, tempCS->getOrgBuf().Y(), m_acMergeBuffer[0].Y(), sps.getBitDepth (CHANNEL_TYPE_LUMA), COMPONENT_Y, bUseHadamard);

      const UnitArea localUnitArea( tempCS->area.chromaFormat, Area( 0, 0, tempCS->area.Y().width, tempCS->area.Y().height) );
      for( uint32_t uiMergeCand = 0; uiMergeCand < mergeCtx.numValidMergeCand; uiMergeCand++ )  // 遍历Merge候选，计算SAD
      {
        mergeCtx.setMergeInfo( pu, uiMergeCand );

        PU::spanMotionInfo( pu, mergeCtx );
        pu.mvRefine = true;
        distParam.cur = singleMergeTempBuffer->Y();
        acMergeTmpBuffer[uiMergeCand] = m_acMergeTmpBuffer[uiMergeCand].getBuf(localUnitArea);
        // ME
        m_pcInterSearch->motionCompensation(pu, *singleMergeTempBuffer, REF_PIC_LIST_X, true, true, &(acMergeTmpBuffer[uiMergeCand]));
        acMergeBuffer[uiMergeCand] = m_acRealMergeBuffer[uiMergeCand].getBuf(localUnitArea);
        acMergeBuffer[uiMergeCand].copyFrom(*singleMergeTempBuffer);
        pu.mvRefine = false;
        // 判断是否为双向Merge，进行DMVR
        if( mergeCtx.interDirNeighbours[uiMergeCand] == 3 && mergeCtx.mrgTypeNeighbours[uiMergeCand] == MRG_TYPE_DEFAULT_N )
        {
          mergeCtx.mvFieldNeighbours[2*uiMergeCand].mv   = pu.mv[0];
          mergeCtx.mvFieldNeighbours[2*uiMergeCand+1].mv = pu.mv[1];
          {
            int dx, dy, i, j, num = 0;
            dy = std::min<int>(pu.lumaSize().height, DMVR_SUBCU_HEIGHT);
            dx = std::min<int>(pu.lumaSize().width, DMVR_SUBCU_WIDTH);
            if (PU::checkDMVRCondition(pu))
            {
              for (i = 0; i < (pu.lumaSize().height); i += dy)
              {
                for (j = 0; j < (pu.lumaSize().width); j += dx)
                {
                  refinedMvdL0[num][uiMergeCand] = pu.mvdL0SubPu[num];
                  num++;
                }
              }
            }
          }
        }

        Distortion uiSad = distParam.distFunc(distParam);  // 计算SAD
        m_CABACEstimator->getCtx() = ctxStart;
        uint64_t fracBits = m_pcInterSearch->xCalcPuMeBits(pu);  // 计算比特率
        double cost = (double)uiSad + (double)fracBits * sqrtLambdaForFirstPassIntra;
        insertPos = -1;  // 在Merge候选Cost列表中的位置

#if GDR_ENABLED
        // Non-RD cost for regular merge
        if (isEncodeGdrClean)
        {
          bool isSolid = true;
          bool isValid = true;

          if (mergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 0].refIdx >= 0)
          {
            Mv mv = mergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 0].mv;
            int ridx = mergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 0].refIdx;

            mergeCtx.mvValid[(uiMergeCand << 1) + 0] = cs->isClean(pu.Y().bottomRight(), mv, REF_PIC_LIST_0, ridx);

            isSolid = isSolid && mergeCtx.mvSolid[(uiMergeCand << 1) + 0];
            isValid = isValid && mergeCtx.mvValid[(uiMergeCand << 1) + 0];
          }

          if (mergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 1].refIdx >= 0) \
          {
            Mv mv = mergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 1].mv;
            int ridx = mergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 1].refIdx;

            mergeCtx.mvValid[(uiMergeCand << 1) + 1] = cs->isClean(pu.Y().bottomRight(), mv, REF_PIC_LIST_1, ridx);

            isSolid = isSolid && mergeCtx.mvSolid[(uiMergeCand << 1) + 1];
            isValid = isValid && mergeCtx.mvValid[(uiMergeCand << 1) + 1];
          }

          if (!isValid || !isSolid)
          {
            cost = MAX_DOUBLE;
          }
        }
#endif
        // 更新Merge候选Cost列表
        updateCandList(ModeInfo(uiMergeCand, true, false, false), cost, RdModeList, candCostList, uiNumMrgSATDCand, &insertPos);
        if (insertPos != -1)  // 发生了更新
        {
          if (insertPos == RdModeList.size() - 1)
          {
            swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
          }
          else
          {
            for (uint32_t i = uint32_t(RdModeList.size()) - 1; i > insertPos; i--)
            {
              swap(acMergeTempBuffer[i - 1], acMergeTempBuffer[i]);  // 将acMergeTempBuffer最后一项移到insertPos前
            }
            swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
          }
        }
#if !GDR_ENABLED
        CHECK(std::min(uiMergeCand + 1, uiNumMrgSATDCand) != RdModeList.size(), "");
#endif
      }

      if (isIntrainterEnabled)  // CIIP模式可用
      {
        // prepare for Intra bits calculation
        pu.ciipFlag = true;

        // save the to-be-tested merge candidates  保存要测试的CIIP候选模式
        uint32_t CiipMergeCand[NUM_MRG_SATD_CAND];
        for (uint32_t mergeCnt = 0; mergeCnt < std::min(NUM_MRG_SATD_CAND, (const int)mergeCtx.numValidMergeCand); mergeCnt++)
        {
          CiipMergeCand[mergeCnt] = RdModeList[mergeCnt].mergeCand;
        }
        // 遍历CIIP模式需要测试的候选Merge模式
        for (uint32_t mergeCnt = 0; mergeCnt < std::min(std::min(NUM_MRG_SATD_CAND, (const int)mergeCtx.numValidMergeCand), 4); mergeCnt++)
        {
          uint32_t mergeCand = CiipMergeCand[mergeCnt];
          acMergeTmpBuffer[mergeCand] = m_acMergeTmpBuffer[mergeCand].getBuf(localUnitArea);

          // estimate merge bits  估计Merge的bit数
          mergeCtx.setMergeInfo(pu, mergeCand);

          // first round
          pu.intraDir[0] = PLANAR_IDX;  // 帧内用Planar模式
          uint32_t intraCnt = 0;
          // generate intrainter Y prediction  产生帧内帧间联合预测像素
          if (mergeCnt == 0)
          {
            m_pcIntraSearch->initIntraPatternChType(*pu.cu, pu.Y());
            m_pcIntraSearch->predIntraAng(COMPONENT_Y, pu.cs->getPredBuf(pu).Y(), pu);
            m_pcIntraSearch->switchBuffer(pu, COMPONENT_Y, pu.cs->getPredBuf(pu).Y(), m_pcIntraSearch->getPredictorPtr2(COMPONENT_Y, intraCnt));
          }
          pu.cs->getPredBuf(pu).copyFrom(acMergeTmpBuffer[mergeCand]);
          if (pu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
          {
            pu.cs->getPredBuf(pu).Y().rspSignal(m_pcReshape->getFwdLUT());
          }
          // 计算CIIP预测像素
          m_pcIntraSearch->geneWeightedPred(COMPONENT_Y, pu.cs->getPredBuf(pu).Y(), pu, m_pcIntraSearch->getPredictorPtr2(COMPONENT_Y, intraCnt));

          // calculate cost
          if (pu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
          {
            pu.cs->getPredBuf(pu).Y().rspSignal(m_pcReshape->getInvLUT());
          }
          distParam.cur = pu.cs->getPredBuf(pu).Y();
          Distortion sadValue = distParam.distFunc(distParam);
          if (pu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
          {
            pu.cs->getPredBuf(pu).Y().rspSignal(m_pcReshape->getFwdLUT());
          }
          m_CABACEstimator->getCtx() = ctxStart;
          pu.regularMergeFlag = false;
          uint64_t fracBits = m_pcInterSearch->xCalcPuMeBits(pu);
          double cost = (double)sadValue + (double)fracBits * sqrtLambdaForFirstPassIntra;
#if GDR_ENABLED
          // Non-RD cost for CIIP merge
          if (isEncodeGdrClean)
          {
            bool isSolid = true;
            bool isValid = true;

            if (mergeCtx.mvFieldNeighbours[(mergeCand << 1) + 0].refIdx >= 0)
            {
              Mv mv = mergeCtx.mvFieldNeighbours[(mergeCand << 1) + 0].mv;
              int ridx = mergeCtx.mvFieldNeighbours[(mergeCand << 1) + 0].refIdx;

              mergeCtx.mvValid[(mergeCand << 1) + 0] = cs->isClean(pu.Y().bottomRight(), mv, REF_PIC_LIST_0, ridx);

              isSolid = isSolid && mergeCtx.mvSolid[(mergeCand << 1) + 0];
              isValid = isValid && mergeCtx.mvValid[(mergeCand << 1) + 0];
            }

            if (mergeCtx.mvFieldNeighbours[(mergeCand << 1) + 1].refIdx >= 0)
            {
              Mv mv = mergeCtx.mvFieldNeighbours[(mergeCand << 1) + 1].mv;
              int ridx = mergeCtx.mvFieldNeighbours[(mergeCand << 1) + 1].refIdx;

              mergeCtx.mvValid[(mergeCand << 1) + 1] = cs->isClean(pu.Y().bottomRight(), mv, REF_PIC_LIST_1, ridx);

              isSolid = isSolid && mergeCtx.mvSolid[(mergeCand << 1) + 1];
              isValid = isValid && mergeCtx.mvValid[(mergeCand << 1) + 1];
            }

            if (!isValid || !isSolid)
            {
              cost = MAX_DOUBLE;
            }
          }
#endif

          insertPos = -1;
          // 更新候选模式列表
          updateCandList(ModeInfo(mergeCand, false, false, true), cost, RdModeList, candCostList, uiNumMrgSATDCand, &insertPos);
          if (insertPos != -1)
          {
            for (int i = int(RdModeList.size()) - 1; i > insertPos; i--)
            {
              swap(acMergeTempBuffer[i - 1], acMergeTempBuffer[i]);
            }
            swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
          }
        }
        pu.ciipFlag = false;
      }
      if ( pu.cs->sps->getUseMMVD() )  // 使用MMVD模式，这里选出最优的MMVD的Merge候选（初始MV+搜索方向+搜索步长）
      {
        cu.mmvdSkip = true;
        pu.regularMergeFlag = true;  // 普通Merge列表依旧可用，因为初始的MV还是要从普通Merge中获取
        const int tempNum = (mergeCtx.numValidMergeCand > 1) ? MMVD_ADD_NUM : MMVD_ADD_NUM >> 1;
        // 对MMVD候选循环遍历，由于每个初始MV共有4种搜索方向，8种搜索步长，即每个初始MV产生32种细化MV
        for (int mmvdMergeCand = 0; mmvdMergeCand < tempNum; mmvdMergeCand++)
        {
          int baseIdx = mmvdMergeCand / MMVD_MAX_REFINE_NUM;
          int refineStep = (mmvdMergeCand - (baseIdx * MMVD_MAX_REFINE_NUM)) / 4;
          if (refineStep >= m_pcEncCfg->getMmvdDisNum())
          {
            continue;
          }
#if GDR_ENABLED
          if (isEncodeGdrClean)
          {
            pu.mvSolid[REF_PIC_LIST_0] = true;
            pu.mvSolid[REF_PIC_LIST_1] = true;

            pu.mvValid[REF_PIC_LIST_0] = true;
            pu.mvValid[REF_PIC_LIST_1] = true;
          }
#endif
          // 设置MMVD候选的信息，得到每个扩展MV具体的搜索初始点以及每个方向以及对应的步长
          mergeCtx.setMmvdMergeCandiInfo(pu, mmvdMergeCand);

          PU::spanMotionInfo(pu, mergeCtx);
          pu.mvRefine = true;
          distParam.cur = singleMergeTempBuffer->Y();
          pu.mmvdEncOptMode = (refineStep > 2 ? 2 : 1);
          CHECK(!pu.mmvdMergeFlag, "MMVD merge should be set");
          // Don't do chroma MC here
          // 运动补偿计算预测值
          m_pcInterSearch->motionCompensation(pu, *singleMergeTempBuffer, REF_PIC_LIST_X, true, false);
          pu.mmvdEncOptMode = 0;
          pu.mvRefine = false;
          Distortion uiSad = distParam.distFunc(distParam);

          m_CABACEstimator->getCtx() = ctxStart;
          uint64_t fracBits = m_pcInterSearch->xCalcPuMeBits(pu);  // Bit
          double cost = (double)uiSad + (double)fracBits * sqrtLambdaForFirstPassIntra;  // RDCost
          insertPos = -1;

#if GDR_ENABLED
          if (isEncodeGdrClean)
          {
            bool isSolid = true;
            bool isValid = true;

            if (pu.refIdx[0] >= 0)
            {
              isSolid = isSolid && pu.mvSolid[0];
              isValid = isValid && pu.mvValid[0];
            }

            if (pu.refIdx[1] >= 0)
            {
              isSolid = isSolid && pu.mvSolid[1];
              isValid = isValid && pu.mvValid[1];
            }

            if (!isSolid || !isValid)
            {
              cost = MAX_DOUBLE;
            }
          }
#endif
          // 更新候选列表
          updateCandList(ModeInfo(mmvdMergeCand, false, true, false), cost, RdModeList, candCostList, uiNumMrgSATDCand, &insertPos);
          if (insertPos != -1)
          {
            for (int i = int(RdModeList.size()) - 1; i > insertPos; i--)
            {
              swap(acMergeTempBuffer[i - 1], acMergeTempBuffer[i]);
            }
            swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
          }
        }
      }
      // Try to limit number of candidates using SATD-costs
      // 尽量限制使用SATD-Cost的模式数
      // 如果某个模式的Cost大于候选列表中的第一个模式的Cost*1.25，则后面的模式不再进行SATD-Cost计算
      for( uint32_t i = 1; i < uiNumMrgSATDCand; i++ )
      {
        if( candCostList[i] > MRG_FAST_RATIO * candCostList[0] )
        {
          uiNumMrgSATDCand = i;
          break;
        }
      }

      setMergeBestSATDCost( candCostList[0] );  // 初始化Merge模式的最佳SATD Cost

      if (isIntrainterEnabled && isChromaEnabled(pu.cs->pcv->chrFormat))  // CIIP模式可用且存在色度分量
      {
        pu.ciipFlag = true;
        for (uint32_t mergeCnt = 0; mergeCnt < uiNumMrgSATDCand; mergeCnt++)
        {
          if (RdModeList[mergeCnt].isCIIP) // 对候选列表中的CIIP模式计算色度分量的Planar预测像素
          {
            pu.intraDir[0] = PLANAR_IDX;
            pu.intraDir[1] = DM_CHROMA_IDX;
            if (pu.chromaSize().width == 2)
            {
              continue;
            }
            uint32_t bufIdx = 0;
            m_pcIntraSearch->initIntraPatternChType(*pu.cu, pu.Cb());
            m_pcIntraSearch->predIntraAng(COMPONENT_Cb, pu.cs->getPredBuf(pu).Cb(), pu);
            m_pcIntraSearch->switchBuffer(pu, COMPONENT_Cb, pu.cs->getPredBuf(pu).Cb(), m_pcIntraSearch->getPredictorPtr2(COMPONENT_Cb, bufIdx));

            m_pcIntraSearch->initIntraPatternChType(*pu.cu, pu.Cr());
            m_pcIntraSearch->predIntraAng(COMPONENT_Cr, pu.cs->getPredBuf(pu).Cr(), pu);
            m_pcIntraSearch->switchBuffer(pu, COMPONENT_Cr, pu.cs->getPredBuf(pu).Cr(), m_pcIntraSearch->getPredictorPtr2(COMPONENT_Cr, bufIdx));
          }
        }
        pu.ciipFlag = false;
      }

      tempCS->initStructData( encTestMode.qp );
      m_CABACEstimator->getCtx() = ctxStart;
    }
    else
    {
      if (bestIsMMVDSkip  )  // 如果最佳的模式是MMVD Skip模式
      {
        uiNumMrgSATDCand = mergeCtx.numValidMergeCand + ((mergeCtx.numValidMergeCand > 1) ? MMVD_ADD_NUM : MMVD_ADD_NUM >> 1);
      }
      else  // 最佳模式是Skip模式
      {
        uiNumMrgSATDCand = mergeCtx.numValidMergeCand;
      }
    }
  }
  m_bestModeUpdated = tempCS->useDbCost = bestCS->useDbCost = false;
  uint32_t iteration;
  uint32_t iterationBegin = 0;
  iteration = 2;  // 两次迭代，一次是Skip模式跳过残差编码；另一次是常规Merge模式
  for (uint32_t uiNoResidualPass = iterationBegin; uiNoResidualPass < iteration; ++uiNoResidualPass)
  {
    for( uint32_t uiMrgHADIdx = 0; uiMrgHADIdx < uiNumMrgSATDCand; uiMrgHADIdx++ )
    {
      uint32_t uiMergeCand = RdModeList[uiMrgHADIdx].mergeCand;  // 当前HAD列表中对应的Merge候选模式

      if (uiNoResidualPass != 0 && RdModeList[uiMrgHADIdx].isCIIP) // intrainter does not support skip mode CIIP不支持Skip模式
      {
        if (isTestSkipMerge[uiMergeCand])
        {
          continue;
        }
      }

      if (((uiNoResidualPass != 0) && candHasNoResidual[uiMrgHADIdx])
       || ( (uiNoResidualPass == 0) && bestIsSkip ) )
      {
        continue;
      }

      // first get merge candidates
      CodingUnit &cu      = tempCS->addCU( tempCS->area, partitioner.chType );

      partitioner.setCUData( cu );
      cu.slice            = tempCS->slice;
      cu.tileIdx          = tempCS->pps->getTileIdx( tempCS->area.lumaPos() );
      cu.skip             = false;
      cu.mmvdSkip = false;
      cu.geoFlag          = false;
    //cu.affine
      cu.predMode         = MODE_INTER;
    //cu.LICFlag
      cu.chromaQpAdj      = m_cuChromaQpOffsetIdxPlus1;
      cu.qp               = encTestMode.qp;
      PredictionUnit &pu  = tempCS->addPU( cu, partitioner.chType );

      if (uiNoResidualPass == 0 && RdModeList[uiMrgHADIdx].isCIIP)  // 常规Merge模式并且是CIIP模式
      {
        cu.mmvdSkip = false;
        mergeCtx.setMergeInfo(pu, uiMergeCand);
        pu.ciipFlag = true;
        pu.regularMergeFlag = false;
        pu.intraDir[0] = PLANAR_IDX;
        CHECK(pu.intraDir[0]<0 || pu.intraDir[0]>(NUM_LUMA_MODE - 1), "out of intra mode");
        pu.intraDir[1] = DM_CHROMA_IDX;
      }
      else if (RdModeList[uiMrgHADIdx].isMMVD)  // 待测模式是MMVD模式
      {
        cu.mmvdSkip = true;
        pu.regularMergeFlag = true;
        mergeCtx.setMmvdMergeCandiInfo(pu, uiMergeCand);
      }
      else  //常规merge模式
      {
        cu.mmvdSkip = false;
        pu.regularMergeFlag = true;
        mergeCtx.setMergeInfo(pu, uiMergeCand);
      }
      PU::spanMotionInfo( pu, mergeCtx );

      if( m_pcEncCfg->getMCTSEncConstraint() )  // DMVR模式
      {
        bool isDMVR = PU::checkDMVRCondition( pu );
        if( ( isDMVR && MCTSHelper::isRefBlockAtRestrictedTileBoundary( pu ) ) || ( !isDMVR && !( MCTSHelper::checkMvBufferForMCTSConstraint( pu ) ) ) )
        {
          // Do not use this mode
          tempCS->initStructData( encTestMode.qp );
          continue;
        }
      }
      if( mrgTempBufSet ) // 设置Merge预测的临时Buffer
      {
        {
          int dx, dy, i, j, num = 0;
          dy = std::min<int>(pu.lumaSize().height, DMVR_SUBCU_HEIGHT);
          dx = std::min<int>(pu.lumaSize().width, DMVR_SUBCU_WIDTH);
          if (PU::checkDMVRCondition(pu))
          {
            for (i = 0; i < (pu.lumaSize().height); i += dy)
            {
              for (j = 0; j < (pu.lumaSize().width); j += dx)
              {
                pu.mvdL0SubPu[num] = refinedMvdL0[num][uiMergeCand];
                num++;
              }
            }
          }
        }
        if (pu.ciipFlag)  // CIIP模式
        {
          uint32_t bufIdx = 0;
          PelBuf tmpBuf = tempCS->getPredBuf(pu).Y();
          tmpBuf.copyFrom(acMergeTmpBuffer[uiMergeCand].Y());
          if (pu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
          {
            tmpBuf.rspSignal(m_pcReshape->getFwdLUT());
          }
          m_pcIntraSearch->geneWeightedPred(COMPONENT_Y, tmpBuf, pu, m_pcIntraSearch->getPredictorPtr2(COMPONENT_Y, bufIdx));
          if (isChromaEnabled(pu.chromaFormat))
          {
            if (pu.chromaSize().width > 2)
            {
              tmpBuf = tempCS->getPredBuf(pu).Cb();
              tmpBuf.copyFrom(acMergeTmpBuffer[uiMergeCand].Cb());
              m_pcIntraSearch->geneWeightedPred(COMPONENT_Cb, tmpBuf, pu,
                                                m_pcIntraSearch->getPredictorPtr2(COMPONENT_Cb, bufIdx));
              tmpBuf = tempCS->getPredBuf(pu).Cr();
              tmpBuf.copyFrom(acMergeTmpBuffer[uiMergeCand].Cr());
              m_pcIntraSearch->geneWeightedPred(COMPONENT_Cr, tmpBuf, pu,
                                                m_pcIntraSearch->getPredictorPtr2(COMPONENT_Cr, bufIdx));
            }
            else
            {
              tmpBuf = tempCS->getPredBuf(pu).Cb();
              tmpBuf.copyFrom(acMergeTmpBuffer[uiMergeCand].Cb());
              tmpBuf = tempCS->getPredBuf(pu).Cr();
              tmpBuf.copyFrom(acMergeTmpBuffer[uiMergeCand].Cr());
            }
          }
        }
        else
        {
          if (RdModeList[uiMrgHADIdx].isMMVD)
          {
            pu.mmvdEncOptMode = 0;
            m_pcInterSearch->motionCompensation(pu);
          }
          else if (uiNoResidualPass != 0 && RdModeList[uiMrgHADIdx].isCIIP)
          {
            tempCS->getPredBuf().copyFrom(acMergeBuffer[uiMergeCand]);
          }
          else
          {
            tempCS->getPredBuf().copyFrom(*acMergeTempBuffer[uiMrgHADIdx]);
          }
        }
      }
      else
      {
        pu.mvRefine = true;
        m_pcInterSearch->motionCompensation( pu );
        pu.mvRefine = false;
      }
      if (!cu.mmvdSkip && !pu.ciipFlag && uiNoResidualPass != 0)
      {
        CHECK(uiMergeCand >= mergeCtx.numValidMergeCand, "out of normal merge");
        isTestSkipMerge[uiMergeCand] = true;
      }

#if GDR_ENABLED
      if (isEncodeGdrClean)
      {
        bool isSolid = true;
        bool isValid = true;

        if (pu.refIdx[0] >= 0)
        {
          isSolid = isSolid && pu.mvSolid[0];
          isValid = isValid && pu.mvValid[0];
        }

        if (pu.refIdx[1] >= 0)
        {
          isSolid = isSolid && pu.mvSolid[1];
          isValid = isValid && pu.mvValid[1];
        }

        if (isSolid && isValid)
        {
          xEncodeInterResidual(tempCS, bestCS, partitioner, encTestMode, uiNoResidualPass, uiNoResidualPass == 0 ? &candHasNoResidual[uiMrgHADIdx] : NULL);
        }
      }
      else
      {
        xEncodeInterResidual(tempCS, bestCS, partitioner, encTestMode, uiNoResidualPass, uiNoResidualPass == 0 ? &candHasNoResidual[uiMrgHADIdx] : NULL);
      }
#else
      // 编码残差
      xEncodeInterResidual( tempCS, bestCS, partitioner, encTestMode, uiNoResidualPass, uiNoResidualPass == 0 ? &candHasNoResidual[uiMrgHADIdx] : NULL );
#endif

      if( m_pcEncCfg->getUseFastDecisionForMerge() && !bestIsSkip && !pu.ciipFlag)
      {
        bestIsSkip = !bestCS->cus.empty() && bestCS->getCU( partitioner.chType )->rootCbf == 0;
      }
      tempCS->initStructData( encTestMode.qp );
    }// end loop uiMrgHADIdx

    if( uiNoResidualPass == 0 && m_pcEncCfg->getUseEarlySkipDetection() )
    {
      const CodingUnit     &bestCU = *bestCS->getCU( partitioner.chType );
      const PredictionUnit &bestPU = *bestCS->getPU( partitioner.chType );

      if( bestCU.rootCbf == 0 )
      {
        if( bestPU.mergeFlag )
        {
          m_modeCtrl->setEarlySkipDetected();
        }
        else if( m_pcEncCfg->getMotionEstimationSearchMethod() != MESEARCH_SELECTIVE )
        {
          int absolute_MV = 0;

          for( uint32_t uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
          {
            if( slice.getNumRefIdx( RefPicList( uiRefListIdx ) ) > 0 )
            {
              absolute_MV += bestPU.mvd[uiRefListIdx].getAbsHor() + bestPU.mvd[uiRefListIdx].getAbsVer();
            }
          }

          if( absolute_MV == 0 )
          {
            m_modeCtrl->setEarlySkipDetected();
          }
        }
      }
    }
  }
  if ( m_bestModeUpdated && bestCS->cost != MAX_DOUBLE )
  {
    xCalDebCost( *bestCS, partitioner );
  }
}
```

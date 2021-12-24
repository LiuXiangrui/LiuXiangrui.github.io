---
title: 高级运动矢量预测(AMVP)
date: 2021-12-13 00:30:25
tags: 
- 帧间预测
- VVC
categories: 视频编码
mathjax: true
---

## 原理
AMVP利用空时域相邻MV的相关性，为当前PU建立MVP候选列表。编码器使用RDO选出最优MVP，仅需传输MV预测残差和最优MVP索引

## 工作流程
1. 构建MVP候选列表
2. 遍历候选列表，根据RDCost选择AMVP的最优候选MVP
3. 利用运动估计寻找最优MV
4. 更新帧间预测最优MVP
5. 编码MVP预测残差和MVP索引

## 空域候选列表构建
![空域候选MV位置](https://s2.loli.net/2021/12/16/KyAlG2Ms9kRJHQ6.jpg)

- 空域候选列表长度为2，在当前PU的左侧和上方各产生一个候选
- 当左侧或上方查询到第一个可用MV时，直接将该MV作为候选，同时终止在该方向上的查询
- 左侧候选顺序为 $A_0 \rightarrow A_1 \rightarrow \text{scaled}\, A_0 \rightarrow \text{scaled}\, A_1$
- 上方的候选顺序为 $B_0 \rightarrow B_1 \rightarrow B_2$
- 当左侧无法产生候选时（PU不可用或模式为帧内预测），缩放的候选块 $\text{scaled}\, B_0 \rightarrow \text{scaled}B_1, \rightarrow \text{scaled}B_2$ 参与空域候选列表构建

## 时域候选列表构建
- 时域候选列表长度为1，构建方式与Merge模式相同
- 将空域候选列表与时域候选列表合并，去除重复的候选MV
- 若合并后MV候选列表长度不足2，则使用$(0,0)$填补

## AMVP与Merge模式的区别：
- Merge模式是一种帧间编码模式，当前PU的MV直接由空域或时域上邻近的PU预测得到，不存在MVD
- AMVP是一种MV预测技术，存在MVD
- Merge候选列表长度为6，AMVP候选列表长度为2，且Merge模式额外包含HMVP和逐对的平均MVP两种候选MV

## VTM平台对应代码

### 涉及函数
- `EncoderLib::InterSearch::xEstimateMvPredAMVP()`：选出最优MVP
- `CommonAnalyserLib::PU::fillMvpCand()`：建立候选列表
- `EncoderLib::InterSearch::xMotionEstimation()`：运动估计寻找最优MV
- `EncoderLib::InterSearch::xCheckBestMVP()`：选出最优MV对应的最优MVP

### xEstimateMvPredAMVP
```c++
void InterSearch::xEstimateMvPredAMVP( PredictionUnit& pu, PelUnitBuf& origBuf, RefPicList eRefPicList, int iRefIdx, Mv& rcMvPred, AMVPInfo& rAMVPInfo, bool bFilled, Distortion* puiDistBiP )
{
  Mv         cBestMv;
  int        iBestIdx   = 0;
  Distortion uiBestCost = std::numeric_limits<Distortion>::max();
  int        i;

  AMVPInfo*  pcAMVPInfo = &rAMVPInfo;  // 获取AMVP信息

  // Fill the MV Candidates
  if (!bFilled)
  {
    PU::fillMvpCand( pu, eRefPicList, iRefIdx, *pcAMVPInfo );  // 构造AMVP候选列表
  }

  // initialize Mvp index & Mvp
  iBestIdx = 0;
  cBestMv  = pcAMVPInfo->mvCand[0]; // 初始化最优MVP为第一个MVP

  PelUnitBuf predBuf = m_tmpStorageLCU.getBuf( UnitAreaRelative(*pu.cu, pu) );  

  //-- Check Minimum Cost.
  for( i = 0 ; i < pcAMVPInfo->numCand; i++)  // 遍历所有候选MVP，选择RDCost最小的MVP作为最佳候选
  {
    Distortion uiTmpCost = xGetTemplateCost( pu, origBuf, predBuf, pcAMVPInfo->mvCand[i], i, AMVP_MAX_NUM_CANDS, eRefPicList, iRefIdx );

    if( uiBestCost > uiTmpCost )  // 找到RDCost更小的候选
    {
      uiBestCost     = uiTmpCost;
      cBestMv        = pcAMVPInfo->mvCand[i];
      iBestIdx       = i;
      (*puiDistBiP)  = uiTmpCost;
    }
  }

  // Setting Best MVP
  rcMvPred = cBestMv;  // 设置最优MV
  pu.mvpIdx[eRefPicList] = iBestIdx;  // 设置最优索引
  pu.mvpNum[eRefPicList] = pcAMVPInfo->numCand;  // 设置参考数量

  return;
}
```

### fillMvpCand
```c++
void PU::fillMvpCand(PredictionUnit &pu, const RefPicList &eRefPicList, const int &refIdx, AMVPInfo &amvpInfo)
{
  CodingStructure &cs = *pu.cs;

  AMVPInfo *pInfo = &amvpInfo;

  pInfo->numCand = 0;

  if (refIdx < 0)
  {
    return;
  }

  //-- Get Spatial MV
  Position posLT = pu.Y().topLeft();  // 左上侧参考MV，即B2
  Position posRT = pu.Y().topRight();  // 右上侧参考MV，即B0
  Position posLB = pu.Y().bottomLeft();  // 左下侧参考MV，即A0


  bool bAdded = addMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_BELOW_LEFT, *pInfo );  // 检查A0是否可用
  if( !bAdded )
  {
    bAdded = addMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_LEFT, *pInfo );  // 检查A1是否可用

  }
  

  // Above predictor search
  {
    bool bAdded = addMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE_RIGHT, *pInfo );  // 检查B0是否可用

    if( !bAdded )
    {
      bAdded = addMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE, *pInfo );  // 检查B1是否可用

      if( !bAdded )
      {
        addMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_ABOVE_LEFT, *pInfo );  // 检查B2是否可用
      }
    }
  }

  for( int i = 0; i < pInfo->numCand; i++ )
  {
    pInfo->mvCand[i].roundTransPrecInternal2Amvr(pu.cu->imv);
  }

  if( pInfo->numCand == 2 )
  {
    if( pInfo->mvCand[0] == pInfo->mvCand[1] )  // 去除重复空域候选MV
    {
      pInfo->numCand = 1;
    }
  }
  // 当启用时域候选，空域候选数量少于2个，CU尺寸大于4x4时，建立时域候选列表
  if (cs.picHeader->getEnableTMVPFlag() && pInfo->numCand < AMVP_MAX_NUM_CANDS && (pu.lumaSize().width + pu.lumaSize().height > 12))  
  {
    // Get Temporal Motion Predictor
    const int refIdx_Col = refIdx;

    Position posRB = pu.Y().bottomRight().offset(-3, -3);

    const PreCalcValues& pcv = *cs.pcv;

    Position posC0;  // 右下位置
    bool C0Avail = false;
    Position posC1 = pu.Y().center();  // 中心位置
    Mv cColMv;  // 同位MV

    bool boundaryCond = ((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight); 
    const SubPic &curSubPic = pu.cs->slice->getPPS()->getSubPicFromPos(pu.lumaPos());
    if (curSubPic.getTreatedAsPicFlag())
    {
      boundaryCond = ((posRB.x + pcv.minCUWidth) <= curSubPic.getSubPicRight() &&
                      (posRB.y + pcv.minCUHeight) <= curSubPic.getSubPicBottom()); // 判断是否为边界，即最后一行且最后一列CU
    }
    if (boundaryCond)
    {
      int posYInCtu = posRB.y & pcv.maxCUHeightMask;
      if (posYInCtu + 4 < pcv.maxCUHeight)  // 判断是否为最后一列，但不是最后一行
      {
        posC0 = posRB.offset(4, 4);
        C0Avail = true;
      }
    }
    if ( ( C0Avail && getColocatedMVP( pu, eRefPicList, posC0, cColMv, refIdx_Col, false ) ) || getColocatedMVP( pu, eRefPicList, posC1, cColMv, refIdx_Col, false ) )
    {
      cColMv.roundTransPrecInternal2Amvr(pu.cu->imv);
      pInfo->mvCand[pInfo->numCand++] = cColMv;
    }
  }

  if (pInfo->numCand < AMVP_MAX_NUM_CANDS)
  {
    const int currRefPOC = cs.slice->getRefPic(eRefPicList, refIdx)->getPOC();
    addAMVPHMVPCand(pu, eRefPicList, currRefPOC, *pInfo);  // 加入时域候选MV
  }

  if (pInfo->numCand > AMVP_MAX_NUM_CANDS)  // 处理边界条件
  {
    pInfo->numCand = AMVP_MAX_NUM_CANDS;
  }

  while (pInfo->numCand < AMVP_MAX_NUM_CANDS)
  {
    pInfo->mvCand[pInfo->numCand] = Mv( 0, 0 );  // 候选MVP不足2个，使用(0,0)填充
    pInfo->numCand++;
  }

  for (Mv &mv : pInfo->mvCand)
  {
    mv.roundTransPrecInternal2Amvr(pu.cu->imv);
  }
}
```

### xCheckBestMVP
```c++
void InterSearch::xCheckBestMVP ( RefPicList eRefPicList, Mv cMv, Mv& rcMvPred, int& riMVPIdx, AMVPInfo& amvpInfo, uint32_t& ruiBits, Distortion& ruiCost, const uint8_t imv )
{
  if ( imv > 0 && imv < 3 )  // 异常情况
  {
    return;
  }

  AMVPInfo* pcAMVPInfo = &amvpInfo;

  CHECK(pcAMVPInfo->mvCand[riMVPIdx] != rcMvPred, "Invalid MV prediction candidate");

  if (pcAMVPInfo->numCand < 2)  // //候选数小于2直接返回
  {
    return;
  }

  m_pcRdCost->setCostScale ( 0    );

  int iBestMVPIdx = riMVPIdx;

  Mv pred = rcMvPred;
  pred.changeTransPrecInternal2Amvr(imv);
  m_pcRdCost->setPredictor( pred );
  Mv mv = cMv;
  mv.changeTransPrecInternal2Amvr(imv);
  int iOrgMvBits = m_pcRdCost->getBitsOfVectorWithPredictor(mv.getHor(), mv.getVer(), 0);  // 当前MV的比特数
  iOrgMvBits += m_auiMVPIdxCost[riMVPIdx][AMVP_MAX_NUM_CANDS];  // 加上编码MVP索引的比特数
  int iBestMvBits = iOrgMvBits;  // 初始化比特数

  for (int iMVPIdx = 0; iMVPIdx < pcAMVPInfo->numCand; iMVPIdx++)  // 遍历所有MVP
  {
    if (iMVPIdx == riMVPIdx)
    {
      continue;
    }
    // 计算第iMVPIdx个MVP的比特消耗
    pred = pcAMVPInfo->mvCand[iMVPIdx];
    pred.changeTransPrecInternal2Amvr(imv);
    m_pcRdCost->setPredictor( pred );
    int iMvBits = m_pcRdCost->getBitsOfVectorWithPredictor(mv.getHor(), mv.getVer(), 0);
    iMvBits += m_auiMVPIdxCost[iMVPIdx][AMVP_MAX_NUM_CANDS];  
    if (iMvBits < iBestMvBits)  // 比较当前MVP和最优MVP，选择最优MVP
    {
      iBestMvBits = iMvBits;
      iBestMVPIdx = iMVPIdx;
    }
  }

  if (iBestMVPIdx != riMVPIdx)  //if changed  更新最优MVP信息
  {
    rcMvPred = pcAMVPInfo->mvCand[iBestMVPIdx];

    riMVPIdx = iBestMVPIdx;
    uint32_t uiOrgBits = ruiBits;
    ruiBits = uiOrgBits - iOrgMvBits + iBestMvBits;
    ruiCost = (ruiCost - m_pcRdCost->getCost( uiOrgBits ))  + m_pcRdCost->getCost( ruiBits );
  }
}

```



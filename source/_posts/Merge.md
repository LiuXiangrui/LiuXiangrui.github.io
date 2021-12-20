---
title: Merge模式
date: 2021-12-13 00:30:25
tags: 
- 帧间预测
- VVC
categories: 视频编码
mathjax: true
---

## 原理
- Merge模式利用空时域相邻MV的相关性，为当前PU建立候选MVP列表。编码器使用RDO选出最优的预测MV并将其作为当前PU的MV，**仅需传输预测MV的索引**

## 工作流程
1. 构建MVP候选列表
2. 遍历候选列表，根据RDCost选择Merge模式最优候选MVP
3. 利用运动估计寻找最优MV
4. 更新帧间预测最优MVP
5. 编码MVP索引

## VVC中Merge模式MV候选列表构建
![Merge-Candidate-List-for-3D-HEVC-The-difference-between-3DHEVC-and-HEVC-lies-in-the.png](https://s2.loli.net/2021/12/19/Hq4Q2OPtygeTMdo.png)

### 总览
- Merge候选项包括MV和参考帧索引，B-Slice的候选项包括前向/后向MV和相应的参考帧索引
- VVC中**候选列表长度设置为6**，按顺序包括空域候选项（最多4个），时域候选项（最多1个），HMVP候选项（最多填充至列表长度-1），逐对的平均MVP候选项（最多1个），零向量（填充至列表长度）
- 若中途参考列表被填满，则提前终止

### 基于相邻块的空域MVP
- VVC从5个空域相邻块$B_1, A_1, B_0, A_0, B_2$中选取**最多4个MV作为候选MVP**
- 按照$B_1\rightarrow A_1\rightarrow B_0\rightarrow A_0\rightarrow (B_2)$的顺序构建空域候选列表
- 当且仅当空域候选列表长度不足4时，将$B_2$作为替补加入空域候选列表
- 当候选列表加入$B_1$后，新加入的候选MVP需要进行冗余性检查
  
### 基于同位块的时域MVP
- **时域候选列表长度为1**，由相邻已编码帧中同位PU的MV按照$RB \rightarrow  \text{ColCenter}$的顺序缩放填充
- 需要根据当前帧到其参考帧的距离$\Delta POC_{cur}$与同位帧到其参考帧的距离$\Delta POC_{col}$的比例对同位PU的MV进行缩放，即
$$
MV_{cur} = \frac{\Delta POC_{cur}}{\Delta POC_{col}} *MV_{col}
$$

### 基于历史信息的MVP(HMVP)
- 历史候选来源于同一CTU中已帧间编码CU的运动信息
- 在空域候选和时域候选过程完成后，若候选列表长度小于 `maxNumMergeCand-1`，将HMVP候选MV加入Merge候选列表
- CTU维护一个基于FIFO的查找表LUT，用于记录帧间编码的CU的运动信息。若新增项与已有项运动信息相同，则弹出已有项
- 在使用HMVP候选项构建Merge候选列表时，从后向前查询LUT中HMVP候选，并在冗余性检查通过后加入Merge候选列表
- 当候选列表长度等于 `maxNumMergeCand-1` 时，结束查询HMVP候选

### 逐对的平均MVP
- 根据Merge候选列表前两项 `p0Cand` 和 `p1Cand` 计算平均MV
- 若为双向候选列表，分别计算L0和L1的平均MV
- 若仅有一个MV可用，则直接将该MV加入Merge候选列表
- 若 `p0Cand` 和 `p1Cand` 的半像素插值滤波器指数不同，则将平均MV的滤波器指数置为0

### 零运动矢量

## VTM平台对应代码

### 涉及函数
- `CommonAnalyserLib::PU::getInterMergeCandidates()`：建立Merge候选列表
- `CommonAnalyserLib::PU::addMergeHMVPCand()`：添加HMVP候选
- `EcoderLib::EncCU::xCheckRDCostMerge2Nx2N()`：根据RDCost选择Merge/Skip模式中最佳候选MV


### getInterMergeCandidates
```c++
void PU::getInterMergeCandidates( const PredictionUnit &pu, MergeCtx& mrgCtx,
                                 int mmvdList,
                                 const int& mrgCandIdx )
{
  const unsigned plevel = pu.cs->sps->getLog2ParallelMergeLevelMinus2() + 2;
  const CodingStructure &cs  = *pu.cs;
  const Slice &slice         = *pu.cs->slice;
  const uint32_t maxNumMergeCand = pu.cs->sps->getMaxNumMergeCand();
  CHECK (maxNumMergeCand > MRG_MAX_NUM_CANDS, "selected maximum number of merge candidate exceeds global limit");
#if GDR_ENABLED
  const bool isEncodeGdrClean = cs.sps->getGDREnabledFlag() && cs.pcv->isEncoder && ((cs.picHeader->getInGdrInterval() && cs.isClean(pu.Y().topRight(), CHANNEL_TYPE_LUMA)) || (cs.picHeader->getNumVerVirtualBoundaries() == 0));
  bool  allCandSolidInAbove = true;
#endif
  for (uint32_t ui = 0; ui < maxNumMergeCand; ++ui)
  {
    mrgCtx.BcwIdx[ui] = BCW_DEFAULT;
    mrgCtx.interDirNeighbours[ui] = 0;
    mrgCtx.mrgTypeNeighbours [ui] = MRG_TYPE_DEFAULT_N;
    mrgCtx.mvFieldNeighbours[(ui << 1)    ].refIdx = NOT_VALID;
    mrgCtx.mvFieldNeighbours[(ui << 1) + 1].refIdx = NOT_VALID;
#if GDR_ENABLED
    if (isEncodeGdrClean)
    {
      mrgCtx.mvSolid[(ui << 1) + 0] = true;
      mrgCtx.mvSolid[(ui << 1) + 1] = true;
      mrgCtx.mvValid[(ui << 1) + 0] = true;
      mrgCtx.mvValid[(ui << 1) + 1] = true;
      mrgCtx.mvPos[(ui << 1) + 0] = Position(0, 0);
      mrgCtx.mvPos[(ui << 1) + 1] = Position(0, 0);
    }
#endif
    mrgCtx.useAltHpelIf[ui] = false;
  }

  mrgCtx.numValidMergeCand = maxNumMergeCand;
  // compute the location of the current PU

  int cnt = 0;

  const Position posLT = pu.Y().topLeft();  // 当前PU左上角坐标
  const Position posRT = pu.Y().topRight();  // 当前PU右上角坐标
  const Position posLB = pu.Y().bottomLeft();  // 当前PU左下角坐标
  MotionInfo miAbove, miLeft, miAboveLeft, miAboveRight, miBelowLeft;  // 当前PU相邻块运动信息 B0, A0, B2, B1, A1,  

  // above B0->A0->B1->A1->(B2)
  const PredictionUnit *puAbove = cs.getPURestricted(posRT.offset(0, -1), pu, pu.chType);  // 上方PU，即B1  

  bool isAvailableB1 = puAbove && isDiffMER(pu.lumaPos(), posRT.offset(0, -1), plevel) && pu.cu != puAbove->cu && CU::isInter(*puAbove->cu);

  if (isAvailableB1)
  {
    miAbove = puAbove->getMotionInfo(posRT.offset(0, -1));

    // get Inter Dir
    mrgCtx.interDirNeighbours[cnt] = miAbove.interDir;  // B1的参考列表方向(前向，后向，双向)
    mrgCtx.useAltHpelIf[cnt] = miAbove.useAltHpelIf;  // B1的半像素插值滤波器指数
    // get Mv from Above
    mrgCtx.BcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puAbove->cu->BcwIdx : BCW_DEFAULT;  // 当B1为双向预测时，读取B1的双向加权预测(BCW)权值索引
    mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miAbove.mv[0], miAbove.refIdx[0]);  // 加入B1的前向MV和前向参考帧索引

#if GDR_ENABLED
    if (isEncodeGdrClean)
    {
      Position pos = puAbove->lumaPos();
      mrgCtx.mvPos[(cnt << 1) + 0] = pos;
      mrgCtx.mvSolid[(cnt << 1) + 0] = cs.isClean(pos, pu.chType);
      mrgCtx.mvValid[(cnt << 1) + 0] = cs.isClean(pu.Y().bottomRight(), miAbove.mv[0], REF_PIC_LIST_0, miAbove.refIdx[0]);
    }
#endif
    if (slice.isInterB())  // 判断是否为双向参考
    {
      mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miAbove.mv[1], miAbove.refIdx[1]);  // 加入B1的后向MV和后向参考帧索引
#if GDR_ENABLED
      if (isEncodeGdrClean)
      {
        Position pos = puAbove->lumaPos();
        mrgCtx.mvPos[(cnt << 1) + 1] = pos;
        mrgCtx.mvSolid[(cnt << 1) + 1] = cs.isClean(pos, pu.chType);
        mrgCtx.mvValid[(cnt << 1) + 1] = cs.isClean(pu.Y().bottomRight(), miAbove.mv[1], REF_PIC_LIST_1, miAbove.refIdx[1]);
      }
#endif
    }
    if (mrgCandIdx == cnt)
    {
      return;
    }

    cnt++;
  }

  // early termination
  if (cnt == maxNumMergeCand)   // 候选列表满
  {
    return;
  }

  //left
  const PredictionUnit* puLeft = cs.getPURestricted(posLB.offset(-1, 0), pu, pu.chType);  // 左侧PU，即A1

  const bool isAvailableA1 = puLeft && isDiffMER(pu.lumaPos(), posLB.offset(-1, 0), plevel) && pu.cu != puLeft->cu && CU::isInter(*puLeft->cu);

  if (isAvailableA1)
  {
    miLeft = puLeft->getMotionInfo(posLB.offset(-1, 0));

    if (!isAvailableB1 || (miAbove != miLeft))  // 冗余性检查，检查A1是否与B1相同
    {
      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miLeft.interDir; // A1的参考列表方向(前向，后向，双向)
      mrgCtx.useAltHpelIf[cnt] = miLeft.useAltHpelIf;  // A1的半像素插值滤波器指数
      mrgCtx.BcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puLeft->cu->BcwIdx : BCW_DEFAULT;  // 当A1为双向预测时，读取B1的双向加权预测(BCW)权值索引
      // get Mv from Left
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miLeft.mv[0], miLeft.refIdx[0]);  // 加入A1的前向MV和前向参考帧索引
#if GDR_ENABLED
      if (isEncodeGdrClean)
      {
        Position pos = puLeft->lumaPos();
        mrgCtx.mvPos[(cnt << 1) + 0] = pos;
        mrgCtx.mvSolid[(cnt << 1) + 0] = cs.isClean(pos, pu.chType);
        mrgCtx.mvValid[(cnt << 1) + 0] = cs.isClean(pu.Y().bottomRight(), miLeft.mv[0], REF_PIC_LIST_0, miLeft.refIdx[0]);
      }
#endif

      if (slice.isInterB())  // 判断是否为双向参考
      {
        mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miLeft.mv[1], miLeft.refIdx[1]);  // 加入A1的后向MV和后向参考帧索引
#if GDR_ENABLED
        if (isEncodeGdrClean)
        {
          Position pos = puLeft->lumaPos();
          mrgCtx.mvPos[(cnt << 1) + 1] = pos;
          mrgCtx.mvSolid[(cnt << 1) + 1] = cs.isClean(pos, pu.chType);
          mrgCtx.mvValid[(cnt << 1) + 1] = cs.isClean(pu.Y().bottomRight(), miLeft.mv[1], REF_PIC_LIST_1, miLeft.refIdx[1]);
        }
#endif
      }
      if (mrgCandIdx == cnt)
      {
        return;
      }

      cnt++;
    }
  }

  // early termination
  if( cnt == maxNumMergeCand )
  {
    return;
  }

  // above right
  const PredictionUnit *puAboveRight = cs.getPURestricted( posRT.offset( 1, -1 ), pu, pu.chType );  // 右上侧PU，即B0

  bool isAvailableB0 = puAboveRight && isDiffMER( pu.lumaPos(), posRT.offset(1, -1), plevel) && CU::isInter( *puAboveRight->cu );

  if( isAvailableB0 )
  {
    miAboveRight = puAboveRight->getMotionInfo( posRT.offset( 1, -1 ) );

    if( !isAvailableB1 || ( miAbove != miAboveRight ) )  // 冗余性检查，检查B0是否与B1相同
    {

      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miAboveRight.interDir;  // 将B1加入候选列表
      mrgCtx.useAltHpelIf[cnt] = miAboveRight.useAltHpelIf;
      // get Mv from Above-right
      mrgCtx.BcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puAboveRight->cu->BcwIdx : BCW_DEFAULT;
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField( miAboveRight.mv[0], miAboveRight.refIdx[0] );
#if GDR_ENABLED
      if (isEncodeGdrClean)
      {
        Position pos = puAboveRight->lumaPos();
        mrgCtx.mvPos[(cnt << 1) + 0] = pos;
        mrgCtx.mvSolid[(cnt << 1) + 0] = cs.isClean(pos, pu.chType);
        mrgCtx.mvValid[(cnt << 1) + 0] = cs.isClean(pu.Y().bottomRight(), miAboveRight.mv[0], REF_PIC_LIST_0, miAboveRight.refIdx[0]);
      }
#endif

      if( slice.isInterB() )
      {
        mrgCtx.mvFieldNeighbours[( cnt << 1 ) + 1].setMvField( miAboveRight.mv[1], miAboveRight.refIdx[1] );
#if GDR_ENABLED
        if (isEncodeGdrClean)
        {
          Position pos = puAboveRight->lumaPos();
          mrgCtx.mvPos[(cnt << 1) + 1] = pos;
          mrgCtx.mvSolid[(cnt << 1) + 1] = cs.isClean(pos, pu.chType);
          mrgCtx.mvValid[(cnt << 1) + 1] = cs.isClean(pu.Y().bottomRight(), miAboveRight.mv[1], REF_PIC_LIST_1, miAboveRight.refIdx[1]);
        }
#endif
      }

      if (mrgCandIdx == cnt)
      {
        return;
      }

      cnt++;
    }
  }
  // early termination
  if( cnt == maxNumMergeCand )
  {
    return;
  }

  //left bottom
  const PredictionUnit *puLeftBottom = cs.getPURestricted( posLB.offset( -1, 1 ), pu, pu.chType );  // 左下侧PU，即A0

  bool isAvailableA0 = puLeftBottom && isDiffMER( pu.lumaPos(), posLB.offset(-1, 1), plevel) && CU::isInter( *puLeftBottom->cu );

  if( isAvailableA0 )
  {
    miBelowLeft = puLeftBottom->getMotionInfo( posLB.offset( -1, 1 ) );

    if( !isAvailableA1 || ( miBelowLeft != miLeft ) )  // 冗余性检查，检查A0是否与A1相同
    {
      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miBelowLeft.interDir;
      mrgCtx.useAltHpelIf[cnt] = miBelowLeft.useAltHpelIf;
      mrgCtx.BcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puLeftBottom->cu->BcwIdx : BCW_DEFAULT;
      // get Mv from Bottom-Left
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField( miBelowLeft.mv[0], miBelowLeft.refIdx[0] );
#if GDR_ENABLED
      if (isEncodeGdrClean)
      {
        Position pos = puLeftBottom->lumaPos();
        mrgCtx.mvPos[(cnt << 1) + 0] = pos;
        mrgCtx.mvSolid[(cnt << 1) + 0] = cs.isClean(pos, pu.chType);
        mrgCtx.mvValid[(cnt << 1) + 0] = cs.isClean(pu.Y().bottomRight(), miBelowLeft.mv[0], REF_PIC_LIST_0, miBelowLeft.refIdx[0]);
      }
#endif

      if( slice.isInterB() )
      {
        mrgCtx.mvFieldNeighbours[( cnt << 1 ) + 1].setMvField( miBelowLeft.mv[1], miBelowLeft.refIdx[1] );
#if GDR_ENABLED
        if (isEncodeGdrClean)
        {
          Position pos = puLeftBottom->lumaPos();
          mrgCtx.mvPos[(cnt << 1) + 1] = pos;
          mrgCtx.mvSolid[(cnt << 1) + 1] = cs.isClean(pos, pu.chType);
          mrgCtx.mvValid[(cnt << 1) + 1] = cs.isClean(pu.Y().bottomRight(), miBelowLeft.mv[1], REF_PIC_LIST_1, miBelowLeft.refIdx[1]);
        }
#endif
      }

      if (mrgCandIdx == cnt)
      {
        return;
      }

      cnt++;
    }
  }
  // early termination
  if( cnt == maxNumMergeCand )
  {
    return;
  }

  // above left
  if ( cnt < 4 )  // 判断候选列表未满，则查询左上侧PU，即B2
  {
    const PredictionUnit *puAboveLeft = cs.getPURestricted( posLT.offset( -1, -1 ), pu, pu.chType );

    bool isAvailableB2 = puAboveLeft && isDiffMER( pu.lumaPos(), posLT.offset(-1, -1), plevel ) && CU::isInter( *puAboveLeft->cu );

    if( isAvailableB2 ) 
    {
      miAboveLeft = puAboveLeft->getMotionInfo( posLT.offset( -1, -1 ) );

      if( ( !isAvailableA1 || ( miLeft != miAboveLeft ) ) && ( !isAvailableB1 || ( miAbove != miAboveLeft ) ) )  // 冗余性检查，检查B2是否与B1和A1相同
      {

        // get Inter Dir
        mrgCtx.interDirNeighbours[cnt] = miAboveLeft.interDir;
        mrgCtx.useAltHpelIf[cnt] = miAboveLeft.useAltHpelIf;
        mrgCtx.BcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puAboveLeft->cu->BcwIdx : BCW_DEFAULT;
        // get Mv from Above-Left
        mrgCtx.mvFieldNeighbours[cnt << 1].setMvField( miAboveLeft.mv[0], miAboveLeft.refIdx[0] );
#if GDR_ENABLED
        if (isEncodeGdrClean)
        {
          Position pos = puAboveLeft->lumaPos();
          mrgCtx.mvPos[(cnt << 1) + 0] = pos;
          mrgCtx.mvSolid[(cnt << 1) + 0] = cs.isClean(pos, pu.chType);
          mrgCtx.mvValid[(cnt << 1) + 0] = cs.isClean(pu.Y().bottomRight(), miAboveLeft.mv[0], REF_PIC_LIST_0, miAboveLeft.refIdx[0]);
        }
#endif

        if( slice.isInterB() )
        {
          mrgCtx.mvFieldNeighbours[( cnt << 1 ) + 1].setMvField( miAboveLeft.mv[1], miAboveLeft.refIdx[1] );
#if GDR_ENABLED
          if (isEncodeGdrClean)
          {
            Position pos = puAboveLeft->lumaPos();
            mrgCtx.mvPos[(cnt << 1) + 1] = pos;
            mrgCtx.mvSolid[(cnt << 1) + 1] = cs.isClean(pos, pu.chType);
            mrgCtx.mvValid[(cnt << 1) + 1] = cs.isClean(pu.Y().bottomRight(), miAboveLeft.mv[1], REF_PIC_LIST_1, miAboveLeft.refIdx[1]);
          }
#endif
        }

        if (mrgCandIdx == cnt)
        {
          return;
        }

        cnt++;
      }
    }
  }
  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }
  // 时域候选列表
  if (slice.getPicHeader()->getEnableTMVPFlag() && (pu.lumaSize().width + pu.lumaSize().height > 12))  // 当启用时域MVP且PU尺寸大于等于8x8
  {
    //>> MTK colocated-RightBottom
    // offset the pos to be sure to "point" to the same position the uiAbsPartIdx would've pointed to
    Position posRB = pu.Y().bottomRight().offset( -3, -3 );
    const PreCalcValues& pcv = *cs.pcv;
#if GDR_ENABLED
    bool posC0inCurPicSolid = true;
    bool posC1inCurPicSolid = true;
    bool posC0inRefPicSolid = true;
    bool posC1inRefPicSolid = true;
#endif

    Position posC0;
    Position posC1 = pu.Y().center();
    bool C0Avail = false;
    bool boundaryCond = ((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight);
    const SubPic& curSubPic = pu.cs->slice->getPPS()->getSubPicFromPos(pu.lumaPos());
    if (curSubPic.getTreatedAsPicFlag())  // 判断在不包括环路内滤波操作的解码处理中，是否将子图视为图
    {
      boundaryCond = ((posRB.x + pcv.minCUWidth) <= curSubPic.getSubPicRight() &&
                      (posRB.y + pcv.minCUHeight) <= curSubPic.getSubPicBottom());
    }
    if (boundaryCond)
    {
      int posYInCtu = posRB.y & pcv.maxCUHeightMask;
      if (posYInCtu + 4 < pcv.maxCUHeight)  // C0可用
      {
        posC0 = posRB.offset(4, 4);
        C0Avail = true;
      }
    }

    Mv        cColMv;  // 同位MV
    int       iRefIdx     = 0;  // 候选的参考帧
    int       dir         = 0;  // 单向/双向
    unsigned  uiArrayAddr = cnt;
    bool      bExistMV    = ( C0Avail && getColocatedMVP(pu, REF_PIC_LIST_0, posC0, cColMv, iRefIdx, false ) )
                              || getColocatedMVP( pu, REF_PIC_LIST_0, posC1, cColMv, iRefIdx, false );   //检查是否存在同位MV，若存在则获取相应的MV到cColMv
    if (bExistMV)
    {
      dir     |= 1;
      mrgCtx.mvFieldNeighbours[2 * uiArrayAddr].setMvField(cColMv, iRefIdx);  // 加入邻居MV列表
#if GDR_ENABLED
      if (isEncodeGdrClean)
      {
        Mv ccMv;

        posC0inCurPicSolid = cs.isClean(posC0, CHANNEL_TYPE_LUMA);
        posC1inCurPicSolid = cs.isClean(posC1, CHANNEL_TYPE_LUMA);
        posC0inRefPicSolid = cs.isClean(posC0, REF_PIC_LIST_0, iRefIdx);
        posC1inRefPicSolid = cs.isClean(posC1, REF_PIC_LIST_0, iRefIdx);

        bool isMVP0exist = C0Avail && getColocatedMVP(pu, REF_PIC_LIST_0, posC0, ccMv, iRefIdx, false);

        Position pos = isMVP0exist ? posC0 : posC1;
        mrgCtx.mvPos[2 * uiArrayAddr] = pos;
        mrgCtx.mvSolid[2 * uiArrayAddr] = isMVP0exist ? (posC0inCurPicSolid && posC0inRefPicSolid) : (posC1inCurPicSolid && posC1inRefPicSolid);
        mrgCtx.mvValid[2 * uiArrayAddr] = cs.isClean(pu.Y().bottomRight(), ccMv, REF_PIC_LIST_0, iRefIdx);
      }
#endif
    }

    if (slice.isInterB())
    {
      bExistMV = ( C0Avail && getColocatedMVP(pu, REF_PIC_LIST_1, posC0, cColMv, iRefIdx, false ) )
                   || getColocatedMVP( pu, REF_PIC_LIST_1, posC1, cColMv, iRefIdx, false );
      if (bExistMV)
      {
        dir     |= 2;  // 双向候选
        mrgCtx.mvFieldNeighbours[2 * uiArrayAddr + 1].setMvField(cColMv, iRefIdx);
#if GDR_ENABLED
        if (isEncodeGdrClean)
        {
          Mv ccMv;

          posC0inCurPicSolid = cs.isClean(posC0, CHANNEL_TYPE_LUMA);
          posC1inCurPicSolid = cs.isClean(posC1, CHANNEL_TYPE_LUMA);
          posC0inRefPicSolid = cs.isClean(posC0, REF_PIC_LIST_1, iRefIdx);
          posC1inRefPicSolid = cs.isClean(posC1, REF_PIC_LIST_1, iRefIdx);

          bool isMVP0exist = C0Avail && getColocatedMVP(pu, REF_PIC_LIST_1, posC0, ccMv, iRefIdx, false);

          Position pos = isMVP0exist ? posC0 : posC1;
          mrgCtx.mvPos[2 * uiArrayAddr + 1] = pos;
          mrgCtx.mvSolid[2 * uiArrayAddr + 1] = isMVP0exist ? (posC0inCurPicSolid && posC0inRefPicSolid) : (posC1inCurPicSolid && posC1inRefPicSolid);
          mrgCtx.mvValid[2 * uiArrayAddr + 1] = cs.isClean(pu.Y().bottomRight(), ccMv, REF_PIC_LIST_1, iRefIdx);
        }
#endif
      }
    }

    if( dir != 0 )
    {
      bool addTMvp = true;
      if( addTMvp )
      {
        mrgCtx.interDirNeighbours[uiArrayAddr] = dir;  // 加入候选列表
        mrgCtx.BcwIdx[uiArrayAddr] = BCW_DEFAULT;
        mrgCtx.useAltHpelIf[uiArrayAddr] = false;
        if (mrgCandIdx == cnt)
        {
          return;
        }

        cnt++;
      }
    }
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }

  // 基于历史信息构建候选
  int maxNumMergeCandMin1 = maxNumMergeCand - 1;
  if (cnt != maxNumMergeCandMin1)  // 检查Merge列表是否到达最大数目减1，如果到达则不需要进行HMV候选
  {
    bool isGt4x4 = true;
#if GDR_ENABLED
    allCandSolidInAbove = true;
#endif
#if GDR_ENABLED
    bool bFound  = addMergeHMVPCand(cs, mrgCtx, mrgCandIdx, maxNumMergeCandMin1, cnt, isAvailableA1, miLeft,
                                   isAvailableB1, miAbove, CU::isIBC(*pu.cu), isGt4x4, pu, allCandSolidInAbove);
#else
    bool bFound  = addMergeHMVPCand(cs, mrgCtx, mrgCandIdx, maxNumMergeCandMin1, cnt, isAvailableA1, miLeft,
                                   isAvailableB1, miAbove, CU::isIBC(*pu.cu), isGt4x4);  // 添加基于历史信息的候选(HMVP)
#endif

    if (bFound)
    {
      return;
    }
  }

  // pairwise-average candidates 成对平均候选MV
  {
    if (cnt > 1 && cnt < maxNumMergeCand)
    {
      mrgCtx.mvFieldNeighbours[cnt * 2].setMvField( Mv( 0, 0 ), NOT_VALID );
      mrgCtx.mvFieldNeighbours[cnt * 2 + 1].setMvField( Mv( 0, 0 ), NOT_VALID );
      // calculate average MV for L0 and L1 seperately 分别计算L0和L1的平均MV
      unsigned char interDir = 0;
      // 设置半像素插值滤波器指数，若p0Cand和p1Cand半像素插值滤波器指数不同，则将平均MV的滤波器指数置为0
      mrgCtx.useAltHpelIf[cnt] = (mrgCtx.useAltHpelIf[0] == mrgCtx.useAltHpelIf[1]) ? mrgCtx.useAltHpelIf[0] : false;
      for( int refListId = 0; refListId < (slice.isInterB() ? 2 : 1); refListId++ )
      {
        const short refIdxI = mrgCtx.mvFieldNeighbours[0 * 2 + refListId].refIdx;
        const short refIdxJ = mrgCtx.mvFieldNeighbours[1 * 2 + refListId].refIdx;

#if GDR_ENABLED
        // GDR: Pairwise average candidate
        bool mvISolid = mrgCtx.mvSolid[0 * 2 + refListId];
        bool mvJSolid = mrgCtx.mvSolid[1 * 2 + refListId];
        bool mvSolid = true;
#endif
        // both MVs are invalid, skip  两个MV都无效
        if( (refIdxI == NOT_VALID) && (refIdxJ == NOT_VALID) )
        {
          continue;
        }

        interDir += 1 << refListId;  // 帧间预测方向
        // both MVs are valid, average these two MVs  两个MV都有效
        if( (refIdxI != NOT_VALID) && (refIdxJ != NOT_VALID) )
        {
          const Mv& MvI = mrgCtx.mvFieldNeighbours[0 * 2 + refListId].mv;
          const Mv& MvJ = mrgCtx.mvFieldNeighbours[1 * 2 + refListId].mv;

          // average two MVs
          Mv avgMv = MvI;
          avgMv += MvJ;
          roundAffineMv(avgMv.hor, avgMv.ver, 1);

          mrgCtx.mvFieldNeighbours[cnt * 2 + refListId].setMvField( avgMv, refIdxI );
#if GDR_ENABLED
          // GDR: Pairwise single I,J candidate
          if (isEncodeGdrClean)
          {
            mvSolid = mvISolid && mvJSolid && allCandSolidInAbove;

            mrgCtx.mvPos[cnt * 2 + refListId] = Position(0, 0);
            mrgCtx.mvSolid[cnt * 2 + refListId] = mvSolid && allCandSolidInAbove;
            mrgCtx.mvValid[cnt * 2 + refListId] = cs.isClean(pu.Y().bottomRight(), avgMv, (RefPicList)refListId, refIdxI);
            allCandSolidInAbove = mvSolid && allCandSolidInAbove;
          }
#endif
        }
        // only one MV is valid, take the only one MV  只有一个向量有效
        else if( refIdxI != NOT_VALID )
        {
          Mv singleMv = mrgCtx.mvFieldNeighbours[0 * 2 + refListId].mv;
          mrgCtx.mvFieldNeighbours[cnt * 2 + refListId].setMvField( singleMv, refIdxI );
#if GDR_ENABLED
          // GDR: Pairwise single I,J candidate
          if (isEncodeGdrClean)
          {
            mvSolid = mvISolid && allCandSolidInAbove;

            mrgCtx.mvPos[cnt * 2 + refListId] = Position(0, 0);
            mrgCtx.mvSolid[cnt * 2 + refListId] = mvSolid && allCandSolidInAbove;
            mrgCtx.mvValid[cnt * 2 + refListId] = cs.isClean(pu.Y().bottomRight(), singleMv, (RefPicList)refListId, refIdxI);
            allCandSolidInAbove = mvSolid && allCandSolidInAbove;
          }
#endif
        }
        else if( refIdxJ != NOT_VALID )
        {
          Mv singleMv = mrgCtx.mvFieldNeighbours[1 * 2 + refListId].mv;
          mrgCtx.mvFieldNeighbours[cnt * 2 + refListId].setMvField( singleMv, refIdxJ );
#if GDR_ENABLED
          // GDR: Pairwise single I,J candidate
          if (isEncodeGdrClean)
          {
            mvSolid = mvJSolid && allCandSolidInAbove;

            mrgCtx.mvPos[cnt * 2 + refListId] = Position(0, 0);
            mrgCtx.mvSolid[cnt * 2 + refListId] = mvSolid && allCandSolidInAbove;
            mrgCtx.mvValid[cnt * 2 + refListId] = cs.isClean(pu.Y().bottomRight(), singleMv, (RefPicList)refListId, refIdxJ);
            allCandSolidInAbove = mvSolid && allCandSolidInAbove;
          }
#endif
        }
      }

      mrgCtx.interDirNeighbours[cnt] = interDir;
      if( interDir > 0 )
      {
        cnt++;
      }
    }

    // early termination
    if( cnt == maxNumMergeCand )
    {
      return;
    }
  }

  uint32_t uiArrayAddr = cnt;

  int iNumRefIdx = slice.isInterB() ? std::min(slice.getNumRefIdx(REF_PIC_LIST_0), slice.getNumRefIdx(REF_PIC_LIST_1)) : slice.getNumRefIdx(REF_PIC_LIST_0);

  // 补充零MV
  int r = 0;
  int refcnt = 0;
  while (uiArrayAddr < maxNumMergeCand)  // 填充前向(0,0)
  {
    mrgCtx.interDirNeighbours [uiArrayAddr     ] = 1;
    mrgCtx.BcwIdx             [uiArrayAddr     ] = BCW_DEFAULT;
    mrgCtx.mvFieldNeighbours  [uiArrayAddr << 1].setMvField(Mv(0, 0), r);
    mrgCtx.useAltHpelIf[uiArrayAddr] = false;

#if GDR_ENABLED
    // GDR: zero mv(0,0)
    if (isEncodeGdrClean)
    {
      mrgCtx.mvPos[uiArrayAddr << 1] = Position(0, 0);
      mrgCtx.mvSolid[uiArrayAddr << 1] = true && allCandSolidInAbove;
      mrgCtx.mvValid[uiArrayAddr << 1] = cs.isClean(pu.Y().bottomRight(), Mv(0, 0), REF_PIC_LIST_0, r);
      allCandSolidInAbove = true && allCandSolidInAbove;
    }
#endif
    if (slice.isInterB())  // B帧填充后向(0,0)
    {
      mrgCtx.interDirNeighbours [ uiArrayAddr          ] = 3;
      mrgCtx.mvFieldNeighbours  [(uiArrayAddr << 1) + 1].setMvField(Mv(0, 0), r);
#if GDR_ENABLED
      // GDR: zero mv(0,0)
      if (isEncodeGdrClean)
      {
        mrgCtx.mvPos[(uiArrayAddr << 1) + 1] = Position(0, 0);
        mrgCtx.mvSolid[(uiArrayAddr << 1) + 1] = true && allCandSolidInAbove;
        mrgCtx.mvValid[(uiArrayAddr << 1) + 1] = cs.isClean(pu.Y().bottomRight(), Mv(0, 0), (RefPicList)REF_PIC_LIST_1, r);
        allCandSolidInAbove = true && allCandSolidInAbove;
      }
#endif
    }

    if ( mrgCtx.interDirNeighbours[uiArrayAddr] == 1 && pu.cs->slice->getRefPic(REF_PIC_LIST_0, mrgCtx.mvFieldNeighbours[uiArrayAddr << 1].refIdx)->getPOC() == pu.cs->slice->getPOC())
    {
      mrgCtx.mrgTypeNeighbours[uiArrayAddr] = MRG_TYPE_IBC;
    }

    uiArrayAddr++;

    if (refcnt == iNumRefIdx - 1)
    {
      r = 0;
    }
    else
    {
      ++r;
      ++refcnt;
    }
  }
  mrgCtx.numValidMergeCand = uiArrayAddr;
}
```

### addMergeHMVPCand
```c++
// 添加HMVP候选
bool PU::addMergeHMVPCand(const CodingStructure &cs, MergeCtx &mrgCtx, const int &mrgCandIdx,
                          const uint32_t maxNumMergeCandMin1, int &cnt, const bool isAvailableA1,
                          const MotionInfo miLeft, const bool isAvailableB1, const MotionInfo miAbove,
                          const bool ibcFlag, const bool isGt4x4
#if GDR_ENABLED
                         ,const PredictionUnit &pu
                         ,bool &allCandSolidInAbove
#endif
)
{
  const Slice& slice = *cs.slice;
  MotionInfo miNeighbor;

  auto &lut = ibcFlag ? cs.motionLut.lutIbc : cs.motionLut.lut;  // 历史CU运动信息查找表
  int num_avai_candInLUT = (int)lut.size();

#if GDR_ENABLED
  const bool isEncodeGdrClean = cs.sps->getGDREnabledFlag() && cs.pcv->isEncoder && ((cs.picHeader->getInGdrInterval() && cs.isClean(pu.Y().topRight(), CHANNEL_TYPE_LUMA)) || (cs.picHeader->getNumVerVirtualBoundaries() == 0));

  bool  vbOnCtuBoundary = true;
  if (isEncodeGdrClean)
  {
    vbOnCtuBoundary = (pu.cs->picHeader->getNumVerVirtualBoundaries() == 0) || (pu.cs->picHeader->getVirtualBoundariesPosX(0) % pu.cs->sps->getMaxCUWidth() == 0);
    allCandSolidInAbove = allCandSolidInAbove && vbOnCtuBoundary;
  }
#endif
  for (int mrgIdx = 1; mrgIdx <= num_avai_candInLUT; mrgIdx++)
  {
    miNeighbor = lut[num_avai_candInLUT - mrgIdx];  // 从后向前查询LUT
#if GDR_ENABLED
    Position sourcePos = Position(0, 0);
    if (isEncodeGdrClean)
    {
      sourcePos = miNeighbor.sourcePos;
    }
#endif

    if ( mrgIdx > 2 || ((mrgIdx > 1 || !isGt4x4) && ibcFlag)  // Merge候选列表冗余性检查
      || ((!isAvailableA1 || (miLeft != miNeighbor)) && (!isAvailableB1 || (miAbove != miNeighbor))) )
    {
      mrgCtx.interDirNeighbours[cnt] = miNeighbor.interDir;  // 向Merge候选列表加入HMVP
      mrgCtx.useAltHpelIf      [cnt] = !ibcFlag && miNeighbor.useAltHpelIf;
      mrgCtx.BcwIdx            [cnt] = (miNeighbor.interDir == 3) ? miNeighbor.BcwIdx : BCW_DEFAULT;

      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miNeighbor.mv[0], miNeighbor.refIdx[0]);
#if GDR_ENABLED
      if (isEncodeGdrClean)
      {
        // note : cannot gaurantee the order/value in the lut if any of the lut is in dirty area
        mrgCtx.mvPos[(cnt << 1) + 0]   = sourcePos;
        mrgCtx.mvSolid[(cnt << 1) + 0] = allCandSolidInAbove && vbOnCtuBoundary;
        mrgCtx.mvValid[(cnt << 1) + 0] = cs.isClean(pu.Y().bottomRight(), miNeighbor.mv[0], REF_PIC_LIST_0, miNeighbor.refIdx[0]);
        allCandSolidInAbove = allCandSolidInAbove && vbOnCtuBoundary;
      }
#endif
      if (slice.isInterB())  // B帧
      {
        mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miNeighbor.mv[1], miNeighbor.refIdx[1]);
#if GDR_ENABLED
        if (isEncodeGdrClean)
        {
          mrgCtx.mvPos[(cnt << 1) + 1]   = sourcePos;
          mrgCtx.mvSolid[(cnt << 1) + 1] = allCandSolidInAbove && vbOnCtuBoundary;
          mrgCtx.mvValid[(cnt << 1) + 1] = cs.isClean(pu.Y().bottomRight(), miNeighbor.mv[1], REF_PIC_LIST_1, miNeighbor.refIdx[1]);
          allCandSolidInAbove = allCandSolidInAbove && vbOnCtuBoundary;
        }
#endif
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
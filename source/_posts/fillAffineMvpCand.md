

# `PU::addAffineMVPCandUnscaled`



```c++
bool PU::addAffineMVPCandUnscaled( const PredictionUnit &pu, const RefPicList &refPicList, const int &refIdx, const Position &pos, const MvpDir &dir, AffineAMVPInfo &affiAMVPInfo )
{
  CodingStructure &cs = *pu.cs;
  const PredictionUnit *neibPU = NULL;
  Position neibPos;

  switch ( dir )  // 根据方向生成相邻CU的偏移量
  {
  case MD_LEFT:
    neibPos = pos.offset( -1, 0 );
    break;
  case MD_ABOVE:
    neibPos = pos.offset( 0, -1 );
    break;
  case MD_ABOVE_RIGHT:
    neibPos = pos.offset( 1, -1 );
    break;
  case MD_BELOW_LEFT:
    neibPos = pos.offset( -1, 1 );
    break;
  case MD_ABOVE_LEFT:
    neibPos = pos.offset( -1, -1 );
    break;
  default:
    break;
  }

  neibPU = cs.getPURestricted( neibPos, pu, pu.chType );  // 相邻CU块

  if (neibPU == NULL || !CU::isInter(*neibPU->cu) || !neibPU->cu->affine || neibPU->mergeType != MRG_TYPE_DEFAULT_N)
  {
    return false;  // 相邻CU块不可用 
  }

  Mv outputAffineMv[3];  // CPMV

  const MotionInfo& neibMi = neibPU->getMotionInfo( neibPos );

  const int        currRefPOC = cs.slice->getRefPic( refPicList, refIdx )->getPOC();
  const RefPicList refPicList2nd = (refPicList == REF_PIC_LIST_0) ? REF_PIC_LIST_1 : REF_PIC_LIST_0;

  for ( int predictorSource = 0; predictorSource < 2; predictorSource++ ) // examine the indicated reference picture list, then if not available, examine the other list.
  {
    const RefPicList eRefPicListIndex = (predictorSource == 0) ? refPicList : refPicList2nd;
    const int        neibRefIdx = neibMi.refIdx[eRefPicListIndex];  // 相邻CU块的参考帧索引

    if ( ((neibPU->interDir & (eRefPicListIndex + 1)) == 0) || pu.cu->slice->getRefPOC( eRefPicListIndex, neibRefIdx ) != currRefPOC )
    {
      continue;  // 相邻CU与当前CU的参考帧不同，则跳过
    }

    xInheritedAffineMv( pu, neibPU, eRefPicListIndex, outputAffineMv );  // 继承相邻CU块CPMV
    outputAffineMv[0].roundAffinePrecInternal2Amvr(pu.cu->imv);
    outputAffineMv[1].roundAffinePrecInternal2Amvr(pu.cu->imv);
    affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand] = outputAffineMv[0];  // 添加左上CPMV候选项
    affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand] = outputAffineMv[1];  // 添加右上CPMV候选项

    if ( pu.cu->affineType == AFFINEMODEL_6PARAM )  // 6参数模型
    {
      outputAffineMv[2].roundAffinePrecInternal2Amvr(pu.cu->imv);
      affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand] = outputAffineMv[2];  // 添加左下CPMV
    }
    affiAMVPInfo.numCand++;
    return true;
  }

  return false;
}
```

# `PU::xInheritedAffineMv`
xInheritedAffineMv函数在getAffineMergeCand函数构造AffineMerge候选列表中被调用，来得到AffineMerge模式的“继承的MVP”，即在从Affine模式的相邻pu继承得到CPMVP。
pu为当前pu，puNeighbour为当前块的相邻pu，eRefPicList为L0或L1，
rcMv[3]即为需要计算的当前块的Affine继承候选，即3个控制点的mv。
#
```c++
// 继承相邻块puNeighbour的CPMV
void PU::xInheritedAffineMv( const PredictionUnit &pu, const PredictionUnit* puNeighbour, RefPicList eRefPicList, Mv rcMv[3] )
{
  int posNeiX = puNeighbour->Y().pos().x;
  int posNeiY = puNeighbour->Y().pos().y;
  int posCurX = pu.Y().pos().x;
  int posCurY = pu.Y().pos().y;  // 获取当前PU和相邻PU的坐标

  int neiW = puNeighbour->Y().width;
  int curW = pu.Y().width;
  int neiH = puNeighbour->Y().height;
  int curH = pu.Y().height;  // 获取当前PU和相邻PU的尺寸

  Mv mvLT, mvRT, mvLB;  // //相邻PU的Affine模式三个控制点MV
  mvLT = puNeighbour->mvAffi[eRefPicList][0];
  mvRT = puNeighbour->mvAffi[eRefPicList][1];
  mvLB = puNeighbour->mvAffi[eRefPicList][2];

  bool isTopCtuBoundary = false;  // 相邻PU为当前块左上或右上CTU的最底层边沿CU
  if ( (posNeiY + neiH) % pu.cs->sps->getCTUSize() == 0 && (posNeiY + neiH) == posCurY )
  {
    // use bottom-left and bottom-right sub-block MVs for inheritance  使用左下和右下子块MVs进行继承
    const Position posRB = puNeighbour->Y().bottomRight();
    const Position posLB = puNeighbour->Y().bottomLeft();
    mvLT = puNeighbour->getMotionInfo( posLB ).mv[eRefPicList];
    mvRT = puNeighbour->getMotionInfo( posRB ).mv[eRefPicList];
    posNeiY += neiH;
    isTopCtuBoundary = true;
  }

  int shift = MAX_CU_DEPTH;
  int iDMvHorX, iDMvHorY, iDMvVerX, iDMvVerY;

  iDMvHorX = (mvRT - mvLT).getHor() << (shift - floorLog2(neiW));
  iDMvHorY = (mvRT - mvLT).getVer() << (shift - floorLog2(neiW));
  if ( puNeighbour->cu->affineType == AFFINEMODEL_6PARAM && !isTopCtuBoundary )
  {
    iDMvVerX = (mvLB - mvLT).getHor() << (shift - floorLog2(neiH));
    iDMvVerY = (mvLB - mvLT).getVer() << (shift - floorLog2(neiH));
  }
  else
  {
    iDMvVerX = -iDMvHorY;
    iDMvVerY = iDMvHorX;
  }

  int iMvScaleHor = mvLT.getHor() << shift;
  int iMvScaleVer = mvLT.getVer() << shift;
  int horTmp, verTmp;

  // v0 第一个控制点的mv
  horTmp = iMvScaleHor + iDMvHorX * (posCurX - posNeiX) + iDMvVerX * (posCurY - posNeiY);
  verTmp = iMvScaleVer + iDMvHorY * (posCurX - posNeiX) + iDMvVerY * (posCurY - posNeiY);
  roundAffineMv( horTmp, verTmp, shift );
  rcMv[0].hor = horTmp;
  rcMv[0].ver = verTmp;
  rcMv[0].clipToStorageBitDepth();

  // v1 第二个控制点的mv
  horTmp = iMvScaleHor + iDMvHorX * (posCurX + curW - posNeiX) + iDMvVerX * (posCurY - posNeiY);
  verTmp = iMvScaleVer + iDMvHorY * (posCurX + curW - posNeiX) + iDMvVerY * (posCurY - posNeiY);
  roundAffineMv( horTmp, verTmp, shift );
  rcMv[1].hor = horTmp;
  rcMv[1].ver = verTmp;
  rcMv[1].clipToStorageBitDepth();

  // v2 第三个控制点的mv
  if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
  {
    horTmp = iMvScaleHor + iDMvHorX * (posCurX - posNeiX) + iDMvVerX * (posCurY + curH - posNeiY);
    verTmp = iMvScaleVer + iDMvHorY * (posCurX - posNeiX) + iDMvVerY * (posCurY + curH - posNeiY);
    roundAffineMv( horTmp, verTmp, shift );
    rcMv[2].hor = horTmp;
    rcMv[2].ver = verTmp;
    rcMv[2].clipToStorageBitDepth();
  }
}
```

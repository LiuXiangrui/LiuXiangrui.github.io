---
title: VVC新技术与概念
date: 2021-12-12 21:08:13
tags:
categories: 视频编码
---
本文整理了在阅读VVC技术文档和代码过程中遇到的新技术与概念

## GDR（Gradual Decoding Refresh）
### 概念
将一帧完整的I帧分成若干I-Slice，并放置到其前面若干连续的P帧中，这些包含I-Slice的P帧称为GDR帧

### 作用
- 通过传输GDR帧，能够为后续帧提供I帧参考，进而阻断误差累积
- 与直接传输完整I帧相比，能够避免码率剧烈波动，减少对网络带宽的冲击，降低网络延时，起到平滑码率的作用
- GDR适合用于广播/会议等不需要频繁随机访问，且对网络延时要求较高的场景

### 备注
- 在H.264与H.265中GDR作为SEI的可选项提供，在H.266中作为一种新的NAL单元类型提供

## IBC (Intra Block Copy)
Intra block copy(IBC)是屏幕内容编码(Screen Content Coding,SCC)的主要技术之一。IBC是块级的编码模式，在编码端会使用块匹配(Block Matching,BM)技术为每个CU寻找最优匹配块，并计算块向量(block vector)。块向量表明当前块和最优匹配块的位置关系。

当CU使用IBC模式编码，其亮度分量的块向量使用整数精度表示，色度分量的块向量四舍五入到整数精度。当结合AMVR技术时，块向量可以在1像素精度和4像素精度间切换。

IBC模式只用于亮度分量的宽和高都小于等于64的CU。

在编码端，可以对IBC模式使用基于哈希的运动估计技术。编码器会为宽或高不大于16的块进行RD检查。对于non-merge模式，首先使用基于哈希的搜索进行块向量搜索，如果哈希搜索没有返回有效的候选项则使用基于局部搜索的BM。

在基于哈希的搜索中，当前块和参考块间的哈希键值匹配(32-bit CRC)操作对所有允许尺寸的块进行。对当前图像所有位置的哈希键值计算都是基于4x4子块进行的。对于较大的当前块，只有其所有4x4子块的哈希键值与参考块对应位置哈希键值都匹配时，才将其作为匹配块。如果多个块的哈希键值都和当前块匹配，则选择块向量代价最小的一个。

在BM搜索中，搜索范围包括前面的CTU和当前CTU。

在CU级上有一个IBC标志位表示IBC AMVP模式或IBC skip/merge模式：

IBC skip/merge模式：一个merge候选项索引用于表示使用哪个相邻IBC块的块向量作为预测值。merge列表由空域、HMVP和pairwise候选项构成。

IBC AMVP模式：BVD(block vector difference)和MVD(motion vector difference)编码方式一样。块向量有两个候选预测值分别来自左侧和上侧IBC块。当两个候选项都不可用时，使用一个默认块向量作为预测值。需要传输一个标志位表示使用的块向量预测值的索引。

## MER (Merge Estimation Region)
对于当前CU的Merge候选列表推导过程中，不包括与当前CU在同一MER内的候选块，即判断相邻块可用时，仅使用不同MER内的相邻块。在编码器侧选择MER大小，并在序列参数集中以log2_parallel_merge_level_minus2表示。对于当前CU的Merge候选列表推导过程中，不包括与当前CU在同一MER内的候选块，即判断相邻块可用时，仅使用不同MER内的相邻块。在编码器侧选择MER大小，并在序列参数集中以log2_parallel_merge_level_minus2表示。


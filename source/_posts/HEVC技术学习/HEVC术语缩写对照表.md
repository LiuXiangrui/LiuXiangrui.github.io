---
title: HEVC术语缩写对照表
date: 2021-12-12 21:08:13
tags:
categories: 视频编码
---
本文整理了在阅读HEVC技术文档和代码过程中遇到的术语缩写	

|  <center>作用域</center> | <center>缩写</center> | <center>全称</center> | <center>中文名称</center> | <center>简介</center> | <center>详解</center> |
|:-----:|:----:|----:|----:|----:|----:|
| 高层语法 | NALU | Network Abstraction Layer Unit | 网络抽象层单元 | | |
| 高层语法 | IRAP | Intra Random Access Point | 帧内随机接入点 | 一种帧内图像，包含IDR，CRA和BLA三种子类型 | {% post_link '视频编码中的帧类型' %} |
| 高层语法 | IDR | Instantaneous Decoding Refresh | 即时解码刷新 | 切断后续图像对IDR前已解码图像的参考 | {% post_link '视频编码中的帧类型' %} |
| 高层语法 | CRA | Clean Random Access | 纯随机接入 | 比特流GOP的第一帧 | {% post_link '视频编码中的帧类型' %} |
| 高层语法 | BLA | Broken Link Access | 断点连接接入 | 用于实现不同视频码流的拼接 | {% post_link '视频编码中的帧类型' %} |
| 高层语法 | TSA | Temporal Sub-layer Access | 时域子层接入点 | 比特流可以由此开始解码更高时域层 | TODO |
| 高层语法 | STSA | Step-wise Temporal Sub-layer Access | 逐步时域子层接入点 | TODO | TODO |
| 高层语法 |  | |  |  | TODO |
| 帧间预测 | AMP | Asymmetric Motion Partition |  非对称运动分割 | TODO | TODO |
| 帧间预测 | AMVP | Advanced Motion Vector Prediction | 高级运动向量预测技术 | 利用空时域相关性预测运动矢量 | TODO |




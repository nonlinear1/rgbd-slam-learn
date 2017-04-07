# rgbd-slam-learn
learning rgbd-slam by gaoxiang12
数据集地址：http://yun.baidu.com/s/1i33uvw5

修改地方：
1. 暂时无法选择特征提取方式，只是简单按照字符匹配，选择ORB
2. pnp函数在opencv3.2中已经不一样了，请参考最新的参数列表修改
3. 点云图好像有点问题，对Z和X同时取反了
4. g2o的c++11编译问题，已添加了那段程序，有效果，但是会出现一些警告
5. 待说明

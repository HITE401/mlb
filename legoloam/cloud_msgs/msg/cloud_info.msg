Header header 

int32[] startRingIndex		# segMsg.startRingIndex[i]表示在第i条激光线中计算点曲率时的开始点在点云segmentedCloud中的索引
int32[] endRingIndex		# segMsg.endRingIndex[i]表示在第i条激光线中计算点曲率时的最后一点在点云segmentedCloud中的索引

float32 startOrientation	# 当前帧点云初始点的水平角度（约为-pi/2）
float32 endOrientation		# 当前帧点云结束点的水平角度（约为3/2*pi）
float32 orientationDiff		# 当前帧点云初始点与结束点之间的角度差值（约为2pi）

bool[]    segmentedCloudGroundFlag 	# 用于标记点云segmentedCloud中的点是否是地面点，true-地面点，false-分割点
uint32[]  segmentedCloudColInd 		# point column index in range image
float32[] segmentedCloudRange 		# point range





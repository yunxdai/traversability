#include "utility.h"
#include <pcl/features/normal_3d.h>
class TraversabilityFilter{
    
private:

    // ROS node handler
    ros::NodeHandle nh;
    // ROS subscriber
    ros::Subscriber subCloud;
    // ROS publisher
    ros::Publisher pubCloud;
    ros::Publisher pubCloudVisualHiRes;
    ros::Publisher pubCloudVisualLowRes;
    ros::Publisher pubLaserScan;
    // Point Cloud
    pcl::PointCloud<PointType>::Ptr laserCloudRaw; // raw cloud from /velodyne_points
    pcl::PointCloud<PointType>::Ptr laserCloudIn; // projected full velodyne cloud
    pcl::PointCloud<PointXYZIR>::Ptr laserCloudInRing; // full cloud with ring 
    pcl::PointCloud<PointType>::Ptr laserCloudOut; // filtered and downsampled point cloud
    pcl::PointCloud<PointType>::Ptr laserCloudObstacles; // cloud for saving points that are classified as obstables, convert them to laser scan
    // Transform Listener
    tf::TransformListener listener;
    tf::StampedTransform transform;
    // A few points
    PointType nanPoint;
    PointType robotPoint;
    PointType localMapOrigin;
    // point cloud saved as N_SCAN * Horizon_SCAN form
    vector<vector<PointType>> laserCloudMatrix;
    // Matrice
    cv::Mat obstacleMatrix; // -1 - invalid, 0 - free, 1 - obstacle
    cv::Mat rangeMatrix; // -1 - invalid, >0 - valid range value
    cv::Mat intensityMatrix;
    // laser scan message
    sensor_msgs::LaserScan laserScan;
    // for downsample
    float **minHeight;
    float **maxHeight;
    bool **obstFlag;
    bool **initFlag;

    ros::Time pcMsgTimeStamp;

public:
    TraversabilityFilter():
        nh("~"){
        subCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 5, &TraversabilityFilter::cloudHandler, this);
        pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/filtered_pointcloud", 5);
        pubCloudVisualHiRes = nh.advertise<sensor_msgs::PointCloud2> ("/filtered_pointcloud_visual_high_res", 5);
        pubCloudVisualLowRes = nh.advertise<sensor_msgs::PointCloud2> ("/filtered_pointcloud_visual_low_res", 5);

        allocateMemory();
    }

    void allocateMemory(){
        
        laserCloudRaw.reset(new pcl::PointCloud<PointType>());
        laserCloudInRing.reset(new pcl::PointCloud<PointXYZIR>());

        laserCloudIn.reset(new pcl::PointCloud<PointType>());
        laserCloudIn->points.resize(N_SCAN * Horizon_SCAN);

        laserCloudOut.reset(new pcl::PointCloud<PointType>());
        laserCloudObstacles.reset(new pcl::PointCloud<PointType>());

        obstacleMatrix = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(-1));
        rangeMatrix =  cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(-1));
        intensityMatrix =  cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(-1));
        laserCloudMatrix.resize(N_SCAN);
        for (int i = 0; i < N_SCAN; ++i)
            laserCloudMatrix[i].resize(Horizon_SCAN);

        initFlag = new bool*[filterHeightMapArrayLength];
        for (int i = 0; i < filterHeightMapArrayLength; ++i)
            initFlag[i] = new bool[filterHeightMapArrayLength];

        obstFlag = new bool*[filterHeightMapArrayLength];
        for (int i = 0; i < filterHeightMapArrayLength; ++i)
            obstFlag[i] = new bool[filterHeightMapArrayLength];

        minHeight = new float*[filterHeightMapArrayLength];
        for (int i = 0; i < filterHeightMapArrayLength; ++i)
            minHeight[i] = new float[filterHeightMapArrayLength];

        maxHeight = new float*[filterHeightMapArrayLength];
        for (int i = 0; i < filterHeightMapArrayLength; ++i)
            maxHeight[i] = new float[filterHeightMapArrayLength];


        nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = -1;

        resetParameters();
    }

    void resetParameters(){
        laserCloudRaw->clear();

        laserCloudOut->clear();
        laserCloudObstacles->clear();

        obstacleMatrix = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(-1));
        rangeMatrix =  cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(-1));
        intensityMatrix =  cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(-1));
        for (int i = 0; i < filterHeightMapArrayLength; ++i){
            for (int j = 0; j < filterHeightMapArrayLength; ++j){
                initFlag[i][j] = false;
                obstFlag[i][j] = false;
            }
        }
        
        std::fill(laserCloudIn->points.begin(), laserCloudIn->points.end(), nanPoint);
    }

    ~TraversabilityFilter(){}


    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
        
        velodyne2RangeCloud(laserCloudMsg);
        
        if (transformCloud() == false) return;

        cloud2Matrix();

        applyFilter();

        extractFilteredCloud();

        downsampleCloud();

        predictCloudBGK();

        publishCloud();

        resetParameters();
    }
    
    void velodyne2RangeCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg) {
        
        // transform raw point cloud into range image format
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudRaw);
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*laserCloudRaw,*laserCloudRaw,indices);
        if (useCloudRing == true) {
            pcl::fromROSMsg(*laserCloudMsg, *laserCloudInRing);
            if (laserCloudInRing->is_dense == false) {
                ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
                ros::shutdown();
            }
        }
        // save timestamp (for transform)
        pcMsgTimeStamp = laserCloudMsg->header.stamp;
        size_t cloudSize = laserCloudRaw->points.size();
        PointType thisPoint;
        float verticalAngle, horizonAngle, range;
        size_t rowIdn, columnIdn, index;

        for (size_t i = 0; i < cloudSize; i++) {
            thisPoint.x = laserCloudRaw->points[i].x;
            thisPoint.y = laserCloudRaw->points[i].y;
            thisPoint.z = laserCloudRaw->points[i].z;
            thisPoint.intensity = laserCloudRaw->points[i].intensity;
            if (useCloudRing == true) {
                rowIdn = laserCloudInRing->points[i].ring;
            }
            else {
                verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
                rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
            }
            
            if (rowIdn < 0 || rowIdn >= N_SCAN) continue;

            horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
            columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;

            if (columnIdn < 0 || columnIdn >= Horizon_SCAN) continue;

            range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);

            if (range < sensorMinimumRange) continue;
            index = columnIdn + rowIdn * Horizon_SCAN;

            PointType tempPoint;
            tempPoint.x = thisPoint.x;
            tempPoint.y = thisPoint.y;
            tempPoint.z = thisPoint.z;
            laserCloudIn->points[index] = thisPoint;

            intensityMatrix.at<float>(rowIdn, columnIdn) = thisPoint.intensity;
            rangeMatrix.at<float>(rowIdn, columnIdn) = range;
            obstacleMatrix.at<int>(rowIdn, columnIdn) = 0;
        }
    }

    bool transformCloud(){
        // Listen to the TF transform and prepare for point cloud transformation
        try{listener.lookupTransform("map","base_link", ros::Time(0), transform); }
        // try{listener.lookupTransform("map","base_link", pcMsgTimeStamp, transform); }
        catch (tf::TransformException ex){ /*ROS_ERROR("Transfrom Failure.");*/ return false; }

        robotPoint.x = transform.getOrigin().x();
        robotPoint.y = transform.getOrigin().y();
        robotPoint.z = transform.getOrigin().z();

        laserCloudIn->header.frame_id = "base_link";
        laserCloudIn->header.stamp = 0; // don't use the latest time, we don't have that transform in the queue yet

        pcl::PointCloud<PointType> laserCloudTemp;
        pcl_ros::transformPointCloud("map", *laserCloudIn, laserCloudTemp, listener);
        *laserCloudIn = laserCloudTemp;

        return true;
    }

    void cloud2Matrix(){

        for (int i = 0; i < N_SCAN; ++i){
            for (int j = 0; j < Horizon_SCAN; ++j){
                int index = j  + i * Horizon_SCAN;
                PointType p = laserCloudIn->points[index];
                laserCloudMatrix[i][j] = p;
            }
        }
    }

    void applyFilter(){

        if (urbanMapping == true){
            positiveCurbFilter2();
            negativeCurbFilter();
        }
        // negativeCurbFilter();

        // slopeFilter();
        // intensityFilter();
    }

    void positiveCurbFilter2() // Detect Positive Curbs by Normal Vector
    {
        int rangeNormCalculation = 6;
        for (int i = 0; i < scanNumCurbFilter; i++) {
            for (int j = rangeNormCalculation; j < Horizon_SCAN - rangeNormCalculation; j++) {
                if (obstacleMatrix.at<int>(i, j) == 1) {
                    continue;
                }
                if (rangeMatrix.at<float>(i, j) > sensorRangeLimit || rangeMatrix.at<float>(i,j) == -1) {
                    continue;
                }
                // int index = j  + i * Horizon_SCAN;
                vector<float> neighborArray;
                for (int k = -rangeNormCalculation; k <= rangeNormCalculation; k++) {
                    if (rangeMatrix.at<float>(i,j+k) != -1) {
                        neighborArray.push_back(laserCloudMatrix[i][j+k].x);
                        neighborArray.push_back(laserCloudMatrix[i][j+k].y);
                        neighborArray.push_back(laserCloudMatrix[i][j+k].z);
                    }
                }
                if (i - 1 >= 0) {
                    for (int k = -rangeNormCalculation; k <= rangeNormCalculation; k++) {
                        if (rangeMatrix.at<float>(i-1,j+k) != -1) {
                            neighborArray.push_back(laserCloudMatrix[i-1][j+k].x);
                            neighborArray.push_back(laserCloudMatrix[i-1][j+k].y);
                            neighborArray.push_back(laserCloudMatrix[i-1][j+k].z);
                        }
                    }
                }
                if (i + 1 < scanNumCurbFilter) {
                    for (int k = -rangeNormCalculation; k <= rangeNormCalculation; k++) {
                        if (rangeMatrix.at<float>(i+1,j+k) != -1) {
                            neighborArray.push_back(laserCloudMatrix[i+1][j+k].x);
                            neighborArray.push_back(laserCloudMatrix[i+1][j+k].y);
                            neighborArray.push_back(laserCloudMatrix[i+1][j+k].z);
                        }
                    }
                }
                Eigen::MatrixXf matPoints = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(neighborArray.data(), neighborArray.size() / 3, 3);
                Eigen::MatrixXf centered = matPoints.rowwise() - matPoints.colwise().mean();
                Eigen::MatrixXf cov = (centered.adjoint() * centered); //协方差矩阵
                Eigen::EigenSolver<Eigen::Matrix3f> es(cov);
                Eigen::Matrix3f D = es.pseudoEigenvalueMatrix();
                Eigen::Matrix3f V = es.pseudoEigenvectors();
                int rowIdx, colIdx;
                D.minCoeff(&rowIdx, &colIdx);
                Eigen::Vector3f normal = V.col(colIdx);
                normal /= normal.norm();
                float slopeAngle = std::acos(std::fabs(normal(2))) / M_PI * 180;
                ///////////////// USING SVD //////////////
                // Eigen::Matrix3Xf points = matPoints.transpose();
                // Eigen::Matrix3Xf center = points.colwise() - points.rowwise().mean();
                // int setting = Eigen::ComputeFullU | Eigen::ComputeThinV;
                // Eigen::JacobiSVD<Eigen::Matrix3Xf> svd = center.jacobiSvd(setting);
                // Eigen::Matrix3Xf U = svd.matrixU();
                // Eigen::Vector3f normal = U.col(2);
                // float slopeAngle = getDegAngle(normal, Eigen::Vector3f(0,0,1));
                if(fabs(slopeAngle - 90) > 10.0) {obstacleMatrix.at<int>(i,j) = 1;}
            }
        }
    }
    double getDegAngle(Eigen::Vector3f v1, Eigen::Vector3f v2) 
    {
        double radian_angle = atan2(v1.cross(v2).norm(), v1.transpose() * v2);
        return radian_angle * 180 / M_PI;
    }

    void positiveCurbFilter() // Reimplement the idea in original code
    {
        int rangeCompareNeighborNum = 3;
        float diff[Horizon_SCAN - 1];

        for (int i = 0; i < scanNumCurbFilter; ++i){

            for (int j = rangeCompareNeighborNum; j < Horizon_SCAN - rangeCompareNeighborNum; ++j){
                // Point that has been verified by other filters
                if (obstacleMatrix.at<int>(i, j) == 1) {
                    continue;
                }
                bool breakFlag = false;
                // point is too far away, skip comparison since it can be inaccurate
                if (rangeMatrix.at<float>(i, j) > sensorRangeLimit || rangeMatrix.at<float>(i,j) == -1) {
                    continue;
                }
                vector<float> neighborArray; // valid neighboring points (in order)
                float minZ = FLT_MAX;
                float maxZ = FLT_MIN;
                for (int k = -rangeCompareNeighborNum; k <= rangeCompareNeighborNum; k++) {
                    if (rangeMatrix.at<float>(i,j+k) != -1) {
                        neighborArray.push_back(rangeMatrix.at<float>(i,j+k));
                        minZ = min(laserCloudMatrix[i][j].z, minZ);
                        maxZ = max(laserCloudMatrix[i][j].z, maxZ);
                    }
                }

                float minR = *std::min_element(neighborArray.begin(),neighborArray.end());
                float maxR = *std::max_element(neighborArray.begin(),neighborArray.end());
                float max_diff = maxR - minR;
                // check monotonic
                if (!isMonotonic(neighborArray)) continue;
                if (max_diff / rangeMatrix.at<float>(i, j) < 0.02 ) {
                    continue;
                }
                obstacleMatrix.at<int>(i, j) = 1;
            }
        }
    }
    
    bool isMonotonic(vector<float>& num) {
        return checkMonotonic(num, true) || checkMonotonic(num, false);
    }
    bool checkMonotonic(vector<float>& num, bool flag) {
        for (int i = 0; i < num.size() - 1; i++) {
            if (flag) {
                if (num[i] > num[i+1]) return false;
            }
            else {
                if (num[i] < num[i+1]) return false;
            }
        }
        return true;
    }


    void negativeCurbFilter() {
        int rangeCompareNeighborNum = 3;

        for (int i = 0; i < scanNumCurbFilter; ++i){
            for (int j = 0; j < Horizon_SCAN; ++j){
                // Point that has been verified by other filters
                if (obstacleMatrix.at<int>(i, j) == 1)
                    continue;
                // point without range value cannot be verified
                if (rangeMatrix.at<float>(i, j) == -1)
                    continue;
                // point is too far away, skip comparison since it can be inaccurate
                if (rangeMatrix.at<float>(i, j) > sensorRangeLimit)
                    continue;
                // check neighbors
                for (int m = -rangeCompareNeighborNum; m <= rangeCompareNeighborNum; ++m){
                    int k = j + m;
                    if (k < 0 || k >= Horizon_SCAN)
                        continue;
                    if (rangeMatrix.at<float>(i, k) == -1)
                        continue;
                    // height diff greater than threshold, might be a negative curb

                    if (fabs(laserCloudMatrix[i][j].z - laserCloudMatrix[i][k].z) > 0.1
                        && pointDistance(laserCloudMatrix[i][j], laserCloudMatrix[i][k]) <= 1.0){
                        obstacleMatrix.at<int>(i, j) = 1; 
                        break;
                    }
                }
            }
        }
    }

    void slopeFilter(){
        
        for (int i = 0; i < scanNumSlopeFilter; ++i){
            // 为什么slope也要限制scanNum
            for (int j = 0; j < Horizon_SCAN; ++j){
                // Point that has been verified by other filters
                if (obstacleMatrix.at<int>(i, j) == 1)
                    continue;
                // point without range value cannot be verified
                if (rangeMatrix.at<float>(i, j) == -1 || rangeMatrix.at<float>(i+1, j) == -1)
                    continue;
                // point is too far away, skip comparison since it can be inaccurate
                if (rangeMatrix.at<float>(i, j) > sensorRangeLimit)
                    continue;
                // Two range filters here:
                // 1. if a point's range is larger than scanNumSlopeFilter th ring point's range
                // 2. if a point's range is larger than the upper point's range
                // then this point is very likely on obstacle. i.e. a point under the car or on a pole
                // if (  (rangeMatrix.at<float>(scanNumSlopeFilter, j) != -1 && rangeMatrix.at<float>(i, j) > rangeMatrix.at<float>(scanNumSlopeFilter, j))
                //     || (rangeMatrix.at<float>(i, j) > rangeMatrix.at<float>(i+1, j)) ){
                //     obstacleMatrix.at<int>(i, j) = 1;
                //     continue;
                // }
                // Calculate slope angle
                float diffX = laserCloudMatrix[i+1][j].x - laserCloudMatrix[i][j].x; // 前后rings同一径向上比较
                float diffY = laserCloudMatrix[i+1][j].y - laserCloudMatrix[i][j].y;
                float diffZ = laserCloudMatrix[i+1][j].z - laserCloudMatrix[i][j].z;
                float angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY)) * 180 / M_PI;
                // Slope angle is larger than threashold, mark as obstacle point
                if (angle < -filterAngleLimit || angle > filterAngleLimit){
                    obstacleMatrix.at<int>(i, j) = 1;
                    continue;
                }
            }
        }
    }

    void intensityFilter() {
        for (int i = 0; i < N_SCAN; i++) {
            for (int j = 0; j < Horizon_SCAN; j++) {
                // Point that has been verified by other filters
                if (obstacleMatrix.at<int>(i, j) == 1)
                    continue;
                if (rangeMatrix.at<float>(i, j) == -1 || rangeMatrix.at<float>(i+1, j) == -1)
                    continue;
                // point is too far away, skip comparison since it can be inaccurate
                if (rangeMatrix.at<float>(i, j) > sensorRangeLimit)
                    continue;
                if (intensityMatrix.at<float>(i,j) > intensityLimit) {
                    obstacleMatrix.at<int>(i,j) = 1;
                    continue;
                }
            }
        }
    }

    void extractFilteredCloud(){
        for (int i = 0; i < scanNumMax; ++i){
            for (int j = 0; j < Horizon_SCAN; ++j){
                // invalid points and points too far are skipped
                if (rangeMatrix.at<float>(i, j) > sensorRangeLimit ||
                    rangeMatrix.at<float>(i, j) == -1)
                    continue;
                // update point intensity (occupancy) into
                PointType p = laserCloudMatrix[i][j];
                p.intensity = obstacleMatrix.at<int>(i,j) == 1 ? 100 : 0; // 不应该让intensity表示occupancy
                // save updated points
                laserCloudOut->push_back(p);
                // extract obstacle points and convert them to laser scan
                if (p.intensity == 100)
                    laserCloudObstacles->push_back(p);
            }
        }

        // Publish laserCloudOut for visualization (before downsample and BGK prediction)
        if (pubCloudVisualHiRes.getNumSubscribers() != 0){
            sensor_msgs::PointCloud2 laserCloudTemp;
            pcl::toROSMsg(*laserCloudOut, laserCloudTemp);
            laserCloudTemp.header.stamp = ros::Time::now();
            // laserCloudTemp.header.stamp = pcMsgTimeStamp;
            laserCloudTemp.header.frame_id = "map";
            pubCloudVisualHiRes.publish(laserCloudTemp);
        }
    }

    void downsampleCloud(){

        float roundedX = float(int(robotPoint.x * 10.0f)) / 10.0f;
        float roundedY = float(int(robotPoint.y * 10.0f)) / 10.0f;
        // height map origin
        localMapOrigin.x = roundedX - sensorRangeLimit;
        localMapOrigin.y = roundedY - sensorRangeLimit;
        unordered_map<int, vector<float>> heightmap;
        int helper = filterHeightMapArrayLength + 1;
        // convert from point cloud to height map
        int cloudSize = laserCloudOut->points.size();
        for (int i = 0; i < cloudSize; ++i){

            // 这里的idx, idy是二维平面的坐标；之前的i是row idx
            int idx = (laserCloudOut->points[i].x - localMapOrigin.x) / mapResolution; 
            int idy = (laserCloudOut->points[i].y - localMapOrigin.y) / mapResolution;
            
            int linear_idx = idx * (filterHeightMapArrayLength + 1) + idy; // easy from linear_idx to idx / idy
            // points out of boundry
            if (idx < 0 || idy < 0 || idx >= filterHeightMapArrayLength || idy >= filterHeightMapArrayLength)
                continue;
            // obstacle point (decided by curb or slope filter)
            if (laserCloudOut->points[i].intensity == 100)
                obstFlag[idx][idy] = true;

            heightmap[linear_idx].push_back(laserCloudOut->points[i].z);

            if (initFlag[idx][idy] == false){
                minHeight[idx][idy] = laserCloudOut->points[i].z;
                maxHeight[idx][idy] = laserCloudOut->points[i].z;
                initFlag[idx][idy] = true;
            } else {
                minHeight[idx][idy] = std::min(minHeight[idx][idy], laserCloudOut->points[i].z);
                maxHeight[idx][idy] = std::max(maxHeight[idx][idy], laserCloudOut->points[i].z);
            }
        }
        float vehicleHeight = 1.0;
        // intermediate cloud (adding process for hanging over structure)
        pcl::PointCloud<PointType>::Ptr laserCloudTemp(new pcl::PointCloud<PointType>());
        // convert from height map to point cloud
        for (int i = 0; i < filterHeightMapArrayLength; ++i){
            for (int j = 0; j < filterHeightMapArrayLength; ++j){
                // no point at this grid
                if (initFlag[i][j] == false)
                    continue;
                // convert grid to point
                PointType thisPoint;
                thisPoint.x = localMapOrigin.x + i * mapResolution + mapResolution / 2.0;
                thisPoint.y = localMapOrigin.y + j * mapResolution + mapResolution / 2.0;
                thisPoint.z = maxHeight[i][j]; // downsample时只记录最大

                // if (obstFlag[i][j] == true || maxHeight[i][j] - minHeight[i][j] > filterHeightLimit)
                if (maxHeight[i][j] - minHeight[i][j] > filterHeightLimit) {
                    obstFlag[i][j] = true;
                    
                    int linear_idx = i * helper + j;
                    if (heightmap.find(linear_idx) != heightmap.end()) {
                        // multiply height value exist
                        auto& heightVec = heightmap[linear_idx];
                        sort(heightVec.begin(),heightVec.end());
                        float lowestHeight = 0;
                        for (int k = 0; k < heightVec.size() - 1; k++) {
                            if (k == 0) lowestHeight = heightVec[k];
                            else {
                                if (heightVec[k+1] - heightVec[k] > vehicleHeight && heightVec[k] - lowestHeight < filterHeightLimit) {
                                    // cout << "Hanging over at (" << i << "," << j << ")" << endl;
                                    obstFlag[i][j] = false; // 若存在hanging over，则重新置为false
                                }
                            }
                        }
                    }
                    
                }
                if (obstFlag[i][j] == true)
                {
                    thisPoint.intensity = 100; // obstacle
                    laserCloudTemp->push_back(thisPoint); // 这里这个把occupancy附给intensity的问题也需要解决
                }else{
                    thisPoint.intensity = 0; // free
                    laserCloudTemp->push_back(thisPoint);
                    // 如果分成两个point cloud发送呢？
                }
            }
        }

        *laserCloudOut = *laserCloudTemp; // 只有前8线的

        // Publish laserCloudOut for visualization (after downsample but beforeBGK prediction)
        if (pubCloudVisualLowRes.getNumSubscribers() != 0){
            sensor_msgs::PointCloud2 laserCloudTemp;
            pcl::toROSMsg(*laserCloudOut, laserCloudTemp);
            laserCloudTemp.header.stamp = ros::Time::now();
            laserCloudTemp.header.frame_id = "map";
            pubCloudVisualLowRes.publish(laserCloudTemp);
        }
    }

    void predictCloudBGK(){

        if (predictionEnableFlag == false)
            return;

        int kernelGridLength = int(predictionKernalSize / mapResolution);

        for (int i = 0; i < filterHeightMapArrayLength; ++i){
            for (int j = 0; j < filterHeightMapArrayLength; ++j){
                // skip observed point
                if (initFlag[i][j] == true)
                    continue;
                PointType testPoint;
                testPoint.x = localMapOrigin.x + i * mapResolution + mapResolution / 2.0;
                testPoint.y = localMapOrigin.y + j * mapResolution + mapResolution / 2.0;
                testPoint.z = robotPoint.z; // this value is not used except for computing distance with robotPoint
                // skip grids too far
                if (pointDistance(testPoint, robotPoint) > sensorRangeLimit)
                    continue;
                // Training data
                vector<float> xTrainVec; // training data x and y coordinates
                vector<float> yTrainVecElev; // training data elevation
                vector<float> yTrainVecOccu; // training data occupancy
                // Fill trainig data (vector)
                for (int m = -kernelGridLength; m <= kernelGridLength; ++m){
                    for (int n = -kernelGridLength; n <= kernelGridLength; ++n){
                        // skip grids too far
                        if (std::sqrt(float(m*m + n*n)) * mapResolution > predictionKernalSize)
                            continue;
                        int idx = i + m;
                        int idy = j + n;
                        // index out of boundry
                        if (idx < 0 || idy < 0 || idx >= filterHeightMapArrayLength || idy >= filterHeightMapArrayLength)
                            continue;
                        // save only observed grid in this scan
                        if (initFlag[idx][idy] == true){
                            xTrainVec.push_back(localMapOrigin.x + idx * mapResolution + mapResolution / 2.0);
                            xTrainVec.push_back(localMapOrigin.y + idy * mapResolution + mapResolution / 2.0);
                            yTrainVecElev.push_back(maxHeight[idx][idy]);
                            yTrainVecOccu.push_back(obstFlag[idx][idy] == true ? 1 : 0);
                        }
                    }
                }
                // no training data available, continue
                if (xTrainVec.size() == 0)
                    continue;
                // convert from vector to eigen
                Eigen::MatrixXf xTrain = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(xTrainVec.data(), xTrainVec.size() / 2, 2);
                Eigen::MatrixXf yTrainElev = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(yTrainVecElev.data(), yTrainVecElev.size(), 1);
                Eigen::MatrixXf yTrainOccu = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(yTrainVecOccu.data(), yTrainVecOccu.size(), 1);
                // Test data (current grid)
                vector<float> xTestVec;
                xTestVec.push_back(testPoint.x);
                xTestVec.push_back(testPoint.y);
                Eigen::MatrixXf xTest = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(xTestVec.data(), xTestVec.size() / 2, 2);
                // Predict
                Eigen::MatrixXf Ks; // covariance matrix
                covSparse(xTest, xTrain, Ks); // sparse kernel

                Eigen::MatrixXf ybarElev = (Ks * yTrainElev).array();
                Eigen::MatrixXf ybarOccu = (Ks * yTrainOccu).array();
                Eigen::MatrixXf kbar = Ks.rowwise().sum().array();

                // Update Elevation with Prediction
                if (std::isnan(ybarElev(0,0)) || std::isnan(ybarOccu(0,0)) || std::isnan(kbar(0,0)))
                    continue;

                if (kbar(0,0) == 0)
                    continue;

                float elevation = ybarElev(0,0) / kbar(0,0);
                float occupancy = ybarOccu(0,0) / kbar(0,0);

                PointType p;
                p.x = xTestVec[0];
                p.y = xTestVec[1];
                p.z = elevation;
                p.intensity = (occupancy > 0.7) ? 100 : 0;

                laserCloudOut->push_back(p);
            }
        }
    }

    void dist(const Eigen::MatrixXf &xStar, const Eigen::MatrixXf &xTrain, Eigen::MatrixXf &d) const {
        d = Eigen::MatrixXf::Zero(xStar.rows(), xTrain.rows());
        for (int i = 0; i < xStar.rows(); ++i) {
            d.row(i) = (xTrain.rowwise() - xStar.row(i)).rowwise().norm();
        }
    }

    void covSparse(const Eigen::MatrixXf &xStar, const Eigen::MatrixXf &xTrain, Eigen::MatrixXf &Kxz) const {
        dist(xStar/(predictionKernalSize+0.1), xTrain/(predictionKernalSize+0.1), Kxz);
        Kxz = (((2.0f + (Kxz * 2.0f * 3.1415926f).array().cos()) * (1.0f - Kxz.array()) / 3.0f) +
              (Kxz * 2.0f * 3.1415926f).array().sin() / (2.0f * 3.1415926f)).matrix() * 1.0f;
        // Clean up for values with distance outside length scale, possible because Kxz <= 0 when dist >= predictionKernalSize
        for (int i = 0; i < Kxz.rows(); ++i)
            for (int j = 0; j < Kxz.cols(); ++j)
                if (Kxz(i,j) < 0) Kxz(i,j) = 0;
    }

    void publishCloud(){
        sensor_msgs::PointCloud2 laserCloudTemp;
        pcl::toROSMsg(*laserCloudOut, laserCloudTemp);
        laserCloudTemp.header.stamp = ros::Time::now();
        // laserCloudTemp.header.stamp = pcMsgTimeStamp;
        laserCloudTemp.header.frame_id = "map";
        pubCloud.publish(laserCloudTemp);
    }

};






int main(int argc, char** argv){

    ros::init(argc, argv, "traversability_mapping");
    
    TraversabilityFilter TFilter;

    ROS_INFO("\033[1;32m---->\033[0m Traversability Filter Started.");

    ros::spin();

    return 0;
}

#include "utility.h"
#include "elevation_msgs/occupancyLocal.h"
class TraversabilityMapping{

private:

    // ROS Node Handle
    ros::NodeHandle nh;
    // Mutex Memory Lock
    std::mutex mtx;
    // Transform Listener
    tf::TransformListener listener;
    tf::StampedTransform transform;
    // Subscriber
    ros::Subscriber subFilteredGroundCloud;
    // Publisher
    ros::Publisher pubOccupancyMapLocal;
    ros::Publisher pubOccupancyMapLocalHeight;
    ros::Publisher pubElevationCloud;
    // Point Cloud Pointer
    pcl::PointCloud<PointType>::Ptr laserCloud; // save input filtered laser cloud for mapping
    pcl::PointCloud<PointType>::Ptr laserCloudElevation; // a cloud for publishing elevation map
    // Occupancy Grid Map
    nav_msgs::OccupancyGrid occupancyMap2D; // local occupancy grid map
    elevation_msgs::OccupancyElevation occupancyMap2DHeight; // customized message that includes occupancy map and elevation info

    int pubCount;
    
    // Map Arrays
    int mapArrayCount;
    int **mapArrayInd; // it saves the index of this submap in vector mapArray
    // int **predictionArrayFlag;
    vector<childMap_t*> mapArray;

    // Local Map Extraction
    PointType robotPoint;
    PointType localMapOriginPoint;
    grid_t localMapOriginGrid;

    // Global Variables for Traversability Calculation
    cv::Mat matCov, matEig, matVec;

    // Lists for New Scan
    vector<mapCell_t*> observingList1; // thread 1: save new observed cells
    vector<mapCell_t*> observingList2; // thread 2: calculate traversability of new observed cells
    
    ros::Time initialTime_;
    
    // Service
    // ros::ServiceServer service;
public:
    TraversabilityMapping():
        nh("~"),
        pubCount(1),
        mapArrayCount(0){
        // subscribe to traversability filter
        subFilteredGroundCloud = nh.subscribe<sensor_msgs::PointCloud2>("/filtered_pointcloud", 5, &TraversabilityMapping::cloudHandler, this);
        // subscribe directly to raw pointcloud (for elevation mapping test)
        // subFilteredGroundCloud = nh.subscribe<sensor_msgs::PointCloud2>("/filtered_pointcloud_visual_high_res", 5, &TraversabilityMapping::cloudHandler, this);
        // publish local occupancy and elevation grid map
        pubOccupancyMapLocal = nh.advertise<nav_msgs::OccupancyGrid> ("/occupancy_map_local", 5);
        pubOccupancyMapLocalHeight = nh.advertise<elevation_msgs::OccupancyElevation> ("/occupancy_map_local_height", 5);
        // publish elevation map for visualization
        pubElevationCloud = nh.advertise<sensor_msgs::PointCloud2> ("/elevation_pointcloud", 5);
        initialTime_ = ros::Time::now();    
        // Service
        // service = nh.advertiseService("/test", &TraversabilityMapping::serviceCallback, this);
        allocateMemory(); 
    }

    ~TraversabilityMapping(){}
    /*
    bool serviceCallback(elevation_msgs::occupancyLocal::Request &req, elevation_msgs::occupancyLocal::Response &res) {
        // do traversability computation
        // res.occupancy = occupancyMap2DHeight.occupancy;
        if (getRobotPosition() == false) return false;


        
        // local map origin x and y
        localMapOriginPoint.x = robotPoint.x - localMapLength / 2;
        localMapOriginPoint.y = robotPoint.y - localMapLength / 2;
        localMapOriginPoint.z = robotPoint.z;
        // local map origin cube id (in global map)
        localMapOriginGrid.cubeX = int((localMapOriginPoint.x + mapCubeLength/2.0) / mapCubeLength) + rootCubeIndex;
        localMapOriginGrid.cubeY = int((localMapOriginPoint.y + mapCubeLength/2.0) / mapCubeLength) + rootCubeIndex;
        if (localMapOriginPoint.x + mapCubeLength/2.0 < 0)  --localMapOriginGrid.cubeX;
        if (localMapOriginPoint.y + mapCubeLength/2.0 < 0)  --localMapOriginGrid.cubeY;
        // local map origin grid id (in sub-map)
        float originCubeOriginX, originCubeOriginY; // the orign of submap that the local map origin belongs to (note the submap may not be created yet, cannot use originX and originY)
        originCubeOriginX = (localMapOriginGrid.cubeX - rootCubeIndex) * mapCubeLength - mapCubeLength/2.0;
        originCubeOriginY = (localMapOriginGrid.cubeY - rootCubeIndex) * mapCubeLength - mapCubeLength/2.0;
        localMapOriginGrid.gridX = int((localMapOriginPoint.x - originCubeOriginX) / mapResolution);
        localMapOriginGrid.gridY = int((localMapOriginPoint.y - originCubeOriginY) / mapResolution);
        
        // Initialize response
        res.occupancy.header.frame_id = "map";
        res.occupancy.header.stamp = ros::Time::now();
        res.occupancy.info.width = localMapArrayLength;
        res.occupancy.info.height = localMapArrayLength;
        res.occupancy.info.resolution = mapResolution;
        res.occupancy.info.origin.orientation.x = 0.0;
        res.occupancy.info.origin.orientation.y = 0.0;
        res.occupancy.info.origin.orientation.z = 0.0;
        res.occupancy.info.origin.orientation.w = 1.0;

        res.occupancy.info.origin.position.x = localMapOriginPoint.x;
        res.occupancy.info.origin.position.y = localMapOriginPoint.y;
        res.occupancy.info.origin.position.z = localMapOriginPoint.z + 10; // add 10, just for visualization
        res.occupancy.data.resize(occupancyMap2DHeight.occupancy.info.width * occupancyMap2DHeight.occupancy.info.height);
        std::fill(res.occupancy.data.begin(), res.occupancy.data.end(), -1);

        for (int i = 0; i < localMapArrayLength; i++) {
            for (int j = 0; j < localMapArrayLength; j++) {
                // only compute local traversability
                int indX = localMapOriginGrid.gridX + i;
                int indY = localMapOriginGrid.gridY + j;

                grid_t thisGrid;

                thisGrid.cubeX = localMapOriginGrid.cubeX + indX / mapCubeArrayLength;
                thisGrid.cubeY = localMapOriginGrid.cubeY + indY / mapCubeArrayLength;

                thisGrid.gridX = indX % mapCubeArrayLength;
                thisGrid.gridY = indY % mapCubeArrayLength;

                // if sub-map is not created yet
                if (mapArrayInd[thisGrid.cubeX][thisGrid.cubeY] == -1) {
                    continue;
                }
                
                mapCell_t *thisCell = grid2Cell(&thisGrid);

                PointType thisPoint;
                thisPoint.x = thisCell->xyz->x;
                thisPoint.y = thisCell->xyz->y;
                thisPoint.z = thisCell->xyz->z;
                // too far, not accurate
                if (pointDistance(thisPoint, robotPoint) >= traversabilityCalculatingDistance)
                    continue;
                // Find neighbor cells of this center cell
                
                vector<mapCell_t*> mapCellVector = findNeighborElevations(thisCell);

                vector<float> xyzVector;
                for (auto& cell : mapCellVector) {
                    xyzVector.push_back(cell->xyz->x);
                    xyzVector.push_back(cell->xyz->y);
                    xyzVector.push_back(cell->xyz->z);
                }
                if (xyzVector.size() <= 2)
                    continue;
                // matPoints: n * 3 matrix
                Eigen::MatrixXf matPoints = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(xyzVector.data(), xyzVector.size() / 3, 3);
                
                
                // min and max elevation
                float minElevation = matPoints.col(2).minCoeff();
                float maxElevation = matPoints.col(2).maxCoeff();
                float maxDifference = maxElevation - minElevation;
                thisCell->step_diff = maxDifference;

                // slope
                Eigen::MatrixXf centered = matPoints.rowwise() - matPoints.colwise().mean(); // 中心化
                Eigen::MatrixXf cov = (centered.adjoint() * centered); //协方差矩阵
                cv::eigen2cv(cov, matCov); // copy data from eigen to cv::Mat
                cv::eigen(matCov, matEig, matVec); // find eigenvalues and eigenvectors for the covariance matrix
                float slopeAngle = std::acos(std::abs(matVec.at<float>(2, 2))) / M_PI * 180;
                if (slopeAngle < filterAngleLimit) {
                    thisCell->slope_traversability = 1 - slopeAngle / filterAngleLimit;
                }
                else {
                    thisCell->slope_traversability = 0;
                }
                
                // roughness
                Eigen::Vector3f norm(matVec.at<float>(2,0),matVec.at<float>(2,1),matVec.at<float>(2,2)); // normal vector
                Eigen::Vector3f mean = matPoints.colwise().mean();
                float planeParam = mean.transpose() * norm;
                Eigen::MatrixXf res = matPoints * norm;
                res.array() -= planeParam;
                double roughness = sqrt(res.squaredNorm() / (xyzVector.size() - 1));
                double roughnessThresh_ = 0.01;
                if (roughness < roughnessThresh_) {
                    thisCell->roughness_traversability = 1 - roughness / roughnessThresh_;
                }
                else {
                    thisCell->roughness_traversability = 0;
                }
            }
        }
        
        // second iteration
        for (int i = 0; i < localMapArrayLength; i++) {
            for (int j = 0; j < localMapArrayLength; j++) {
                // only compute local traversability
                int indX = localMapOriginGrid.gridX + i;
                int indY = localMapOriginGrid.gridY + j;

                grid_t thisGrid;

                thisGrid.cubeX = localMapOriginGrid.cubeX + indX / mapCubeArrayLength;
                thisGrid.cubeY = localMapOriginGrid.cubeY + indY / mapCubeArrayLength;

                thisGrid.gridX = indX % mapCubeArrayLength;
                thisGrid.gridY = indY % mapCubeArrayLength;

                // if sub-map is not created yet
                if (mapArrayInd[thisGrid.cubeX][thisGrid.cubeY] == -1) {
                    continue;
                }
                
                mapCell_t *thisCell = grid2Cell(&thisGrid);
                int stepThresh_ = 0.3;
                int nCellCritical_ = 5;
                PointType thisPoint;
                thisPoint.x = thisCell->xyz->x;
                thisPoint.y = thisCell->xyz->y;
                thisPoint.z = thisCell->xyz->z;
                // too far, not accurate
                if (pointDistance(thisPoint, robotPoint) >= traversabilityCalculatingDistance)
                    continue;
                // Find neighbor cells of this center cell
                vector<mapCell_t*> mapCellVector = findNeighborElevations(thisCell); // 替换掉，第二次遍历需要返回mapCell_t
                if (mapCellVector.size() <= 2)
                    continue;

                double stepMax = 0.0;
                int nCells = 0;
                for (auto& cell : mapCellVector) {
                    if (cell->step_diff > stepMax) {stepMax = cell->step_diff;}
                    if (cell->step_diff > stepThresh_) {nCells++;}
                }
                double step;
                // cout << "step max = " << stepMax << endl;
                step = std::min(stepMax, (double) nCells / (double) nCellCritical_ * stepMax);
                if (step < stepThresh_) {
                    thisCell->step_traversability = 1.0 - step / stepThresh_;
                }
                else {
                    thisCell->step_traversability = 0.0;
                }

                double w_step = 0.5;
                double w_slope = 0.25;
                double w_roughness = 0.25;
                double traversability = w_step * thisCell->step_traversability + w_slope * thisCell->slope_traversability + w_roughness * thisCell->roughness_traversability;
                
                if (thisCell->elevation != -FLT_MAX) {
                    int index = i + j * localMapArrayLength;
                    res.occupancy.data[index] = traversability * 100 < 20 ? 100 : 0;
                }
                
            }
        }
        
        return true;
    }
    */
    void allocateMemory(){
        // allocate memory for point cloud
        laserCloud.reset(new pcl::PointCloud<PointType>());
        laserCloudElevation.reset(new pcl::PointCloud<PointType>());
        
        // initialize array for cmap
        mapArrayInd = new int*[mapArrayLength];
        for (int i = 0; i < mapArrayLength; ++i)
            mapArrayInd[i] = new int[mapArrayLength];

        for (int i = 0; i < mapArrayLength; ++i)
            for (int j = 0; j < mapArrayLength; ++j)
                mapArrayInd[i][j] = -1;

        // initialize array for predicting elevation sub-maps
        // predictionArrayFlag = new int*[mapArrayLength];
        // for (int i = 0; i < mapArrayLength; ++i)
        //     predictionArrayFlag[i] = new int[mapArrayLength];

        // for (int i = 0; i < mapArrayLength; ++i)
        //     for (int j = 0; j < mapArrayLength; ++j)
        //         predictionArrayFlag[i][j] = false;

        // Matrix Initialization
        matCov = cv::Mat (3, 3, CV_32F, cv::Scalar::all(0));
        matEig = cv::Mat (1, 3, CV_32F, cv::Scalar::all(0));
        matVec = cv::Mat (3, 3, CV_32F, cv::Scalar::all(0));

        initializeLocalOccupancyMap();
    }


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////// Register Cloud /////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
        // Lock thread
        std::lock_guard<std::mutex> lock(mtx);
        // Get Robot Position
        if (getRobotPosition() == false) 
            return;
        // Convert Point Cloud
        pcl::fromROSMsg(*laserCloudMsg, *laserCloud);
        ros::Time timestamp = laserCloudMsg->header.stamp;
        const float scanTimeSinceInitialization = (timestamp - initialTime_).toSec();
        // Register New Scan
        updateElevationMap(scanTimeSinceInitialization);
        // publish local occupancy grid map
        publishMap();
    }

    void updateElevationMap(const float timestamp){
        int cloudSize = laserCloud->points.size();
        for (int i = 0; i < cloudSize; ++i){
            laserCloud->points[i].z -= 0.2; // for visualization
            updateElevationMap(&laserCloud->points[i], timestamp);
        }
    }

    void updateElevationMap(PointType *point, const float timestamp){
        // Find point index in global map
        grid_t thisGrid;
        if (findPointGridInMap(&thisGrid, point) == false) return;
        // Get current cell pointer
        mapCell_t *thisCell = grid2Cell(&thisGrid);
        // update elevation
        updateCellElevation(thisCell, point, timestamp);
        // update occupancy
        updateCellOccupancy(thisCell, point); // 待改进：occupancy信息也需要考虑方差（或者距离），太远的会有很多误检（大部分远的curb都会）
        // update observation time
        updateCellObservationTime(thisCell);
    }

    void updateCellObservationTime(mapCell_t *thisCell){
        ++thisCell->observeTimes;
        // 在当前输入中有对应点的cell，且被观测到的时间超过阈值，就会被加到这个list（然后被丢去计算traversability）
        if (thisCell->observeTimes >= traversabilityObserveTimeTh)
            observingList1.push_back(thisCell);
    }
    
    void updateCellOccupancy(mapCell_t *thisCell, PointType *point)
    // 这个updating函数可能也有问题
    {
        // Update log_odds
        float p;  // Probability of being occupied knowing current measurement.
        if (point->intensity == 100)
            p = p_occupied_when_laser;
        else
            p = p_occupied_when_no_laser;
        thisCell->log_odds += std::log(p / (1 - p));

        if (thisCell->log_odds < -large_log_odds)
            thisCell->log_odds = -large_log_odds;
        else if (thisCell->log_odds > large_log_odds)
            thisCell->log_odds = large_log_odds;
        // Update occupancy
        float occupancy;
        if (thisCell->log_odds < -max_log_odds_for_belief)
            occupancy = 0;
        else if (thisCell->log_odds > max_log_odds_for_belief)
            occupancy = 100;
        else
            occupancy = (int)(lround((1 - 1 / (1 + std::exp(thisCell->log_odds))) * 100));
        // update cell
        thisCell->updateOccupancy(occupancy);
    }
    /*
    // 试一下完全从elevation map计算traversability
    void updateCellOccupancy(mapCell_t *thisCell, PointType *point) {
        float occupancy = point->intensity;
        thisCell->updateOccupancy(occupancy);
    }
    */
    void updateCellElevation(mapCell_t *thisCell, PointType *point, const float timestamp){
        // Kalman Filter: update cell elevation using Kalman filter
        // https://www.cs.cornell.edu/courses/cs4758/2012sp/materials/MI63slides.pdf

        // cell is observed for the first time, no need to use Kalman filter
        if (thisCell->elevation == -FLT_MAX){
            thisCell->elevation = point->z;
            thisCell->elevationVar = pointDistance(robotPoint, *point);
            return;
        }
        /*
        // added by dyx @  6.26: adopt updating rules in ETH elevation mapping 
        float height_p = point->z;
        float elevation_pre = thisCell->elevation;
        float elevationVar_pre = thisCell->elevationVar;
        float dist_mahalanobis = fabs(elevation_pre - height_p) / sqrt(elevationVar_pre);
        float elevation_thresh = 2.5;
        if (dist_mahalanobis > elevation_thresh) {
            if (timestamp - thisCell->time <= 1.0 && elevation_pre > height_p) {
                ;
            }
            else if (timestamp - thisCell->time <= 1.0) {
                thisCell->updateElevation(point->z, pointDistance(robotPoint, *point));
            }
            else {
                thisCell->updateElevation(elevation_pre, elevationVar_pre + pow(0.003, 2));
            }
            
        }
        else {
            float R = pointDistance(robotPoint, *point);
            float elevation_final = (elevation_pre * R + height_p * elevationVar_pre) / (R + elevationVar_pre);
            float elevationVar_final = R * elevationVar_pre / (R + elevationVar_pre);
            thisCell->updateElevation(elevation_final, elevationVar_final);
        }
        */

        // previous kalman filter
        // Predict:
        float x_pred = thisCell->elevation; // x = F * x + B * u
        float P_pred = thisCell->elevationVar + 0.01; // P = F*P*F + Q
        // Update:
        float R_factor = (thisCell->observeTimes > 20) ? 10 : 1;
        float R = pointDistance(robotPoint, *point) * R_factor; // measurement noise: R, scale it with dist and observed times
        float K = P_pred / (P_pred + R);// Gain: K  = P * H^T * (HPH + R)^-1
        float y = point->z; // measurement: y
        float x_final = x_pred + K * (y - x_pred); // x_final = x_pred + K * (y - H * x_pred)
        float P_final = (1 - K) * P_pred; // P_final = (I - K * H) * P_pred
        // Update cell
        thisCell->updateElevation(x_final, P_final);
    }

    mapCell_t* grid2Cell(grid_t *thisGrid){
        return mapArray[mapArrayInd[thisGrid->cubeX][thisGrid->cubeY]]->cellArray[thisGrid->gridX][thisGrid->gridY];
    }

    bool findPointGridInMap(grid_t *gridOut, PointType *point){
        // Calculate the cube index that this point belongs to. (Array dimension: mapArrayLength * mapArrayLength)
        grid_t thisGrid;
        getPointCubeIndex(&thisGrid.cubeX, &thisGrid.cubeY, point);
        // Decide whether a point is out of pre-allocated map
        if (thisGrid.cubeX >= 0 && thisGrid.cubeX < mapArrayLength && 
            thisGrid.cubeY >= 0 && thisGrid.cubeY < mapArrayLength){
            // Point is in the boundary, but this sub-map is not allocated before
            // Allocate new memory for this sub-map and save it to mapArray
            if (mapArrayInd[thisGrid.cubeX][thisGrid.cubeY] == -1){
                childMap_t *thisChildMap = new childMap_t(mapArrayCount, thisGrid.cubeX, thisGrid.cubeY);
                mapArray.push_back(thisChildMap);
                mapArrayInd[thisGrid.cubeX][thisGrid.cubeY] = mapArrayCount;
                ++mapArrayCount;
            }
        }else{
            // Point is out of pre-allocated boundary, report error (you should increase map size)
            ROS_ERROR("Point cloud is out of elevation map boundary. Change params ->mapArrayLength<-. The program will crash!");
            return false;
        }
        // sub-map id
        thisGrid.mapID = mapArrayInd[thisGrid.cubeX][thisGrid.cubeY];
        // Find the index for this point in this sub-map (grid index)
        thisGrid.gridX = (int)((point->x - mapArray[thisGrid.mapID]->originX) / mapResolution);
        thisGrid.gridY = (int)((point->y - mapArray[thisGrid.mapID]->originY) / mapResolution);
        if (thisGrid.gridX < 0 || thisGrid.gridY < 0 || thisGrid.gridX >= mapCubeArrayLength || thisGrid.gridY >= mapCubeArrayLength)
            return false;

        *gridOut = thisGrid;
        return true;
    }

    void getPointCubeIndex(int *cubeX, int *cubeY, PointType *point){
        *cubeX = int((point->x + mapCubeLength/2.0) / mapCubeLength) + rootCubeIndex;
        *cubeY = int((point->y + mapCubeLength/2.0) / mapCubeLength) + rootCubeIndex;

        if (point->x + mapCubeLength/2.0 < 0)  --*cubeX;
        if (point->y + mapCubeLength/2.0 < 0)  --*cubeY;
    }


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////// Traversability Calculation ///////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    void TraversabilityThread(){

        ros::Rate rate(10); // Hz
        
        while (ros::ok()){

            traversabilityMapCalculation();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            // rate.sleep();
        }
    }
    

    
    void traversabilityMapCalculation(){

        // no new scan, return
        if (observingList1.size() == 0) {
            // ROS_INFO_STREAM("RETURN!");
            return;
        }
        // ROS_INFO_STREAM("Traversability Calculating!");
        //对于每一帧观测到的点云，检查是否在过往被多次观测到，若是则计算traversability（问题在于内圈的点可能没有被观测到但是周围的elevation信息发生了改变会导致其traversability改变？）
        observingList2 = observingList1;
        observingList1.clear();

        int listSize = observingList2.size();

        for (int i = 0; i < listSize; ++i){

            mapCell_t *thisCell = observingList2[i];
            // convert this cell to a point for convenience
            PointType thisPoint;
            thisPoint.x = thisCell->xyz->x;
            thisPoint.y = thisCell->xyz->y;
            thisPoint.z = thisCell->xyz->z;
            // too far, not accurate
            if (pointDistance(thisPoint, robotPoint) >= traversabilityCalculatingDistance)
                continue;
            // Find neighbor cells of this center cell
            vector<float> xyzVector = findNeighborElevations(thisCell);
            
            if (xyzVector.size() <= 2)
                continue;

            // matPoints: n * 3 matrix
            Eigen::MatrixXf matPoints = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(xyzVector.data(), xyzVector.size() / 3, 3);
            
            // min and max elevation
            // float minElevation = matPoints.col(2).minCoeff();
            // float maxElevation = matPoints.col(2).maxCoeff();
            // float maxDifference = maxElevation - minElevation;
            
            // if (maxDifference > filterHeightLimit){
            //     thisPoint.intensity = 100;
            //     updateCellOccupancy(thisCell, &thisPoint);
            //     continue;
            // }
            
            
            

            
            // slope
            Eigen::MatrixXf centered = matPoints.rowwise() - matPoints.colwise().mean(); // 中心化
            Eigen::MatrixXf cov = (centered.adjoint() * centered); //协方差矩阵
            cv::eigen2cv(cov, matCov); // copy data from eigen to cv::Mat
            cv::eigen(matCov, matEig, matVec); // find eigenvalues and eigenvectors for the covariance matrix
            float slopeAngle = std::acos(std::abs(matVec.at<float>(2, 2))) / M_PI * 180;
            
            if (slopeAngle > filterAngleLimit) {
                thisPoint.intensity = 100;
                // updateCellOccupancy(thisCell, &thisPoint);
                continue;
            }
            /*
            else {
                if (thisPoint.intensity = 100) {
                    thisPoint.intensity = 0;
                }
            }
            */
            
            // updateCellOccupancy(thisCell,&thisPoint);
            
            

            /*
            // roughness
            Eigen::Vector3f norm(matVec.at<float>(2,0),matVec.at<float>(2,1),matVec.at<float>(2,2)); // normal vector
            Eigen::Vector3f mean = matPoints.colwise().mean();
            float planeParam = mean.transpose() * norm;
            Eigen::MatrixXf res = matPoints * norm;
            res.array() -= planeParam;
            double roughness = sqrt(res.squaredNorm() / (xyzVector.size() - 1));
            double roughnessThresh_ = 0.01;

            updateCellOccupancy(thisCell, &thisPoint);
            
            if (roughness > roughnessThresh_) {
                thisPoint.intensity = 100;
                updateCellOccupancy(thisCell, &thisPoint);    
            }
            */
            
            

            // Debug
            // std::cout <<"center point : (" << thisPoint.x << "," << thisPoint.y << ")";
            // std::cout << " roughness = " << roughness << endl;
            // if (thisPoint.y >= 1.5 && thisPoint.y <= 2.5 && std::abs(thisPoint.x) <= 5.0) {
            //     std::cout <<"center point : (" << thisPoint.x << "," << thisPoint.y << ")";
            //     std::cout << " neighbor points:" << endl << matPoints << endl;
            //     std::cout << " slopeAngle = " << slopeAngle << endl;
            // }
            // // float occupancy = 1.0f / (1.0f + exp(-(slopeAngle - filterAngleLimit)));

            // float occupancy = 0.5 * (slopeAngle / filterAngleLimit)
            //                 + 0.5 * (maxDifference / filterHeightLimit);
            // thisPoint.intensity = slopeAngle;
            

            // if (slopeAngle > filterAngleLimit) {
            //     thisPoint.intensity = 100;
            //     updateCellOccupancy(thisCell, &thisPoint);
            // }
        }
    }
    

    
    vector<float> findNeighborElevations(mapCell_t *centerCell){

        vector<float> xyzVector;

        grid_t centerGrid = centerCell->grid;
        grid_t thisGrid;

        int footprintRadiusLength = int(robotRadius / mapResolution);

        for (int k = -footprintRadiusLength; k <= footprintRadiusLength; ++k){
            for (int l = -footprintRadiusLength; l <= footprintRadiusLength; ++l){
                // skip grids too far
                if (std::sqrt(float(k*k + l*l)) * mapResolution > robotRadius)
                    continue;
                // the neighbor grid
                thisGrid.cubeX = centerGrid.cubeX;
                thisGrid.cubeY = centerGrid.cubeY;
                thisGrid.gridX = centerGrid.gridX + k;
                thisGrid.gridY = centerGrid.gridY + l;
                // If the checked grid is in another sub-map, update it's indexes
                if(thisGrid.gridX < 0){ --thisGrid.cubeX; thisGrid.gridX = thisGrid.gridX + mapCubeArrayLength;
                }else if(thisGrid.gridX >= mapCubeArrayLength){ ++thisGrid.cubeX; thisGrid.gridX = thisGrid.gridX - mapCubeArrayLength; }
                if(thisGrid.gridY < 0){ --thisGrid.cubeY; thisGrid.gridY = thisGrid.gridY + mapCubeArrayLength;
                }else if(thisGrid.gridY >= mapCubeArrayLength){ ++thisGrid.cubeY; thisGrid.gridY = thisGrid.gridY - mapCubeArrayLength; }
                // If the sub-map that the checked grid belongs to is empty or not
                int mapInd = mapArrayInd[thisGrid.cubeX][thisGrid.cubeY];
                if (mapInd == -1) continue;
                // the neighbor cell
                mapCell_t *thisCell = grid2Cell(&thisGrid);
                // save neighbor cell for calculating traversability
                
                if (thisCell->elevation != -FLT_MAX && !std::isnan(thisCell->xyz->z)){
                    xyzVector.push_back(thisCell->xyz->x);
                    xyzVector.push_back(thisCell->xyz->y);
                    xyzVector.push_back(thisCell->xyz->z);
                }
            }
        }

        return xyzVector;
    }
    
    /*
    vector<mapCell_t*> findNeighborElevations(mapCell_t *centerCell){
        // new version of find neighbor function (return cell_t rather than x,y,z)
        vector<mapCell_t*> mapCellVector;

        grid_t centerGrid = centerCell->grid;
        grid_t thisGrid;

        int footprintRadiusLength = int(robotRadius / mapResolution);

        for (int k = -footprintRadiusLength; k <= footprintRadiusLength; ++k){
            for (int l = -footprintRadiusLength; l <= footprintRadiusLength; ++l){
                // skip grids too far
                if (std::sqrt(float(k*k + l*l)) * mapResolution > robotRadius)
                    continue;
                // the neighbor grid
                thisGrid.cubeX = centerGrid.cubeX;
                thisGrid.cubeY = centerGrid.cubeY;
                thisGrid.gridX = centerGrid.gridX + k;
                thisGrid.gridY = centerGrid.gridY + l;
                // If the checked grid is in another sub-map, update it's indexes
                if(thisGrid.gridX < 0){ --thisGrid.cubeX; thisGrid.gridX = thisGrid.gridX + mapCubeArrayLength;
                }else if(thisGrid.gridX >= mapCubeArrayLength){ ++thisGrid.cubeX; thisGrid.gridX = thisGrid.gridX - mapCubeArrayLength; }
                if(thisGrid.gridY < 0){ --thisGrid.cubeY; thisGrid.gridY = thisGrid.gridY + mapCubeArrayLength;
                }else if(thisGrid.gridY >= mapCubeArrayLength){ ++thisGrid.cubeY; thisGrid.gridY = thisGrid.gridY - mapCubeArrayLength; }
                // If the sub-map that the checked grid belongs to is empty or not
                int mapInd = mapArrayInd[thisGrid.cubeX][thisGrid.cubeY];
                if (mapInd == -1) continue;
                // the neighbor cell
                mapCell_t *thisCell = grid2Cell(&thisGrid);
                // save neighbor cell for calculating traversability
                
                if (thisCell->elevation != -FLT_MAX && !std::isnan(thisCell->xyz->z)){
                    mapCellVector.push_back(thisCell);
                }
            }
        }

        return mapCellVector;
    }
    */

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////// Occupancy Map (local) //////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void publishMap(){
        // Publish Occupancy Grid Map and Elevation Map
        pubCount++;
        if (pubCount > visualizationFrequency){
            pubCount = 1;
            publishLocalMap();
            publishTraversabilityMap();
        }
    }

    
    void publishLocalMap(){

        if (pubOccupancyMapLocal.getNumSubscribers() == 0 &&
            pubOccupancyMapLocalHeight.getNumSubscribers() == 0)
            return;

        // 1.3 Initialize local occupancy grid map to unknown, height to -FLT_MAX
        std::fill(occupancyMap2DHeight.occupancy.data.begin(), occupancyMap2DHeight.occupancy.data.end(), -1);
        std::fill(occupancyMap2DHeight.height.begin(), occupancyMap2DHeight.height.end(), -FLT_MAX);
        std::fill(occupancyMap2DHeight.costMap.begin(), occupancyMap2DHeight.costMap.end(), 0);
        
        // local map origin x and y
        localMapOriginPoint.x = robotPoint.x - localMapLength / 2;
        localMapOriginPoint.y = robotPoint.y - localMapLength / 2;
        localMapOriginPoint.z = robotPoint.z;
        // local map origin cube id (in global map)
        localMapOriginGrid.cubeX = int((localMapOriginPoint.x + mapCubeLength/2.0) / mapCubeLength) + rootCubeIndex;
        localMapOriginGrid.cubeY = int((localMapOriginPoint.y + mapCubeLength/2.0) / mapCubeLength) + rootCubeIndex;
        if (localMapOriginPoint.x + mapCubeLength/2.0 < 0)  --localMapOriginGrid.cubeX;
        if (localMapOriginPoint.y + mapCubeLength/2.0 < 0)  --localMapOriginGrid.cubeY;
        // local map origin grid id (in sub-map)
        float originCubeOriginX, originCubeOriginY; // the orign of submap that the local map origin belongs to (note the submap may not be created yet, cannot use originX and originY)
        originCubeOriginX = (localMapOriginGrid.cubeX - rootCubeIndex) * mapCubeLength - mapCubeLength/2.0;
        originCubeOriginY = (localMapOriginGrid.cubeY - rootCubeIndex) * mapCubeLength - mapCubeLength/2.0;
        localMapOriginGrid.gridX = int((localMapOriginPoint.x - originCubeOriginX) / mapResolution);
        localMapOriginGrid.gridY = int((localMapOriginPoint.y - originCubeOriginY) / mapResolution);

        // 2 Calculate local occupancy grid map root position
        occupancyMap2DHeight.header.stamp = ros::Time::now();
        occupancyMap2DHeight.occupancy.header.stamp = occupancyMap2DHeight.header.stamp;
        occupancyMap2DHeight.occupancy.info.origin.position.x = localMapOriginPoint.x;
        occupancyMap2DHeight.occupancy.info.origin.position.y = localMapOriginPoint.y;
        occupancyMap2DHeight.occupancy.info.origin.position.z = localMapOriginPoint.z + 10; // add 10, just for visualization

        // extract all info
        for (int i = 0; i < localMapArrayLength; ++i){
            for (int j = 0; j < localMapArrayLength; ++j){

                int indX = localMapOriginGrid.gridX + i;
                int indY = localMapOriginGrid.gridY + j;

                grid_t thisGrid;

                thisGrid.cubeX = localMapOriginGrid.cubeX + indX / mapCubeArrayLength;
                thisGrid.cubeY = localMapOriginGrid.cubeY + indY / mapCubeArrayLength;

                thisGrid.gridX = indX % mapCubeArrayLength;
                thisGrid.gridY = indY % mapCubeArrayLength;

                // if sub-map is not created yet
                if (mapArrayInd[thisGrid.cubeX][thisGrid.cubeY] == -1) {
                    continue;
                }
                
                mapCell_t *thisCell = grid2Cell(&thisGrid);

                // skip unknown grid
                if (thisCell->elevation != -FLT_MAX){
                    int index = i + j * localMapArrayLength; // index of the 1-D array 
                    occupancyMap2DHeight.height[index] = thisCell->elevation;
                    occupancyMap2DHeight.occupancy.data[index] = thisCell->occupancy > 80 ? 100 : 0;
                }
            }
        }

        pubOccupancyMapLocalHeight.publish(occupancyMap2DHeight);
        pubOccupancyMapLocal.publish(occupancyMap2DHeight.occupancy);
    }
    
    

    void initializeLocalOccupancyMap(){
        // initialization of customized map message
        occupancyMap2DHeight.header.frame_id = "map";
        occupancyMap2DHeight.occupancy.info.width = localMapArrayLength;
        occupancyMap2DHeight.occupancy.info.height = localMapArrayLength;
        occupancyMap2DHeight.occupancy.info.resolution = mapResolution;
        
        occupancyMap2DHeight.occupancy.info.origin.orientation.x = 0.0;
        occupancyMap2DHeight.occupancy.info.origin.orientation.y = 0.0;
        occupancyMap2DHeight.occupancy.info.origin.orientation.z = 0.0;
        occupancyMap2DHeight.occupancy.info.origin.orientation.w = 1.0;

        occupancyMap2DHeight.occupancy.data.resize(occupancyMap2DHeight.occupancy.info.width * occupancyMap2DHeight.occupancy.info.height);
        occupancyMap2DHeight.height.resize(occupancyMap2DHeight.occupancy.info.width * occupancyMap2DHeight.occupancy.info.height);
        occupancyMap2DHeight.costMap.resize(occupancyMap2DHeight.occupancy.info.width * occupancyMap2DHeight.occupancy.info.height);
    }    

    bool getRobotPosition(){
        try{listener.lookupTransform("map","base_link", ros::Time(0), transform); } 
        catch (tf::TransformException ex){ ROS_ERROR("Transfrom Failure in Traversaility Map."); return false; }

        robotPoint.x = transform.getOrigin().x();
        robotPoint.y = transform.getOrigin().y();
        robotPoint.z = transform.getOrigin().z();

        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////// Point Cloud /////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void publishTraversabilityMap(){

        if (pubElevationCloud.getNumSubscribers() == 0)
            return;
        // 1. Find robot current cube index
        int currentCubeX, currentCubeY;
        getPointCubeIndex(&currentCubeX, &currentCubeY, &robotPoint);
        // 2. Loop through all the sub-maps that are nearby
        int visualLength = int(visualizationRadius / mapCubeLength);
        for (int i = -visualLength; i <= visualLength; ++i){
            for (int j = -visualLength; j <= visualLength; ++j){

                if (sqrt(float(i*i+j*j)) >= visualLength) continue;

                int idx = i + currentCubeX;
                int idy = j + currentCubeY;

                if (idx < 0 || idx >= mapArrayLength ||  idy < 0 || idy >= mapArrayLength) continue;

                if (mapArrayInd[idx][idy] == -1) continue;

                *laserCloudElevation += mapArray[mapArrayInd[idx][idy]]->cloud;
            }
        }
        // 3. Publish elevation point cloud
        sensor_msgs::PointCloud2 laserCloudTemp;
        pcl::toROSMsg(*laserCloudElevation, laserCloudTemp);
        laserCloudTemp.header.frame_id = "/map";
        laserCloudTemp.header.stamp = ros::Time::now();
        pubElevationCloud.publish(laserCloudTemp);
        // 4. free memory
        laserCloudElevation->clear();
    }
};




int main(int argc, char** argv){

    ros::init(argc, argv, "traversability_mapping");
    
    TraversabilityMapping tMapping;

    std::thread predictionThread(&TraversabilityMapping::TraversabilityThread, &tMapping);

    ROS_INFO("\033[1;32m---->\033[0m Traversability Mapping Started.");
    ROS_INFO("\033[1;32m---->\033[0m Traversability Mapping Scenario: %s.", 
        urbanMapping == true ? "\033[1;31mUrban\033[0m" : "\033[1;31mTerrain\033[0m");

    // ros::AsyncSpinner asyncSpinner(2);
    // asyncSpinner.start();
    // ros::waitForShutdown();
    ros::spin();

    return 0;
}
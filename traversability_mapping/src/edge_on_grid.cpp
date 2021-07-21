#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <visualization_msgs/Marker.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include <numeric>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class localGoalExtraction
{
private:
    ros::NodeHandle nh_;
    std::string occupancyGridTopic_;
    MoveBaseClient ac;
    ros::Subscriber occupancySub_;
    ros::Publisher markerPub_;
    uint32_t shape_;
    visualization_msgs::Marker marker_;
    cv::Mat region_grow(cv::Mat& img, cv::Point seed)
    {
        cv::Mat retImg = cv::Mat::zeros(img.size(),CV_8UC1);
        if (img.at<uchar>(seed) != 255) {
            ROS_INFO_STREAM("INVALID SEED, OCCUPIED!");
        }
        else {
            std::vector<std::vector<int>> neighbor = {{-1,0},{1,0},{0,-1},{0,1}};
            std::queue<cv::Point> q;
            q.push(seed);
            while (!q.empty()) {
                cv::Point cPoint = q.front(); q.pop();
                retImg.at<uchar>(cPoint) = 255;
                int cx = cPoint.x;
                int cy = cPoint.y;
                for (auto n : neighbor) {
                    if (cx + n[0] >= 0 && cx + n[0] < img.rows && cy + n[1] >= 0 && cy + n[1] < img.cols) {
                        cv::Point neighborPoint(cx + n[0], cy + n[1]);
                        if (img.at<uchar>(neighborPoint) == 255) {
                            q.push(neighborPoint);
                            img.at<uchar>(neighborPoint) = 0;
                        }
                    }
                }
            }
        }
        return retImg;

    }
    bool polynomial_curve_fit(std::vector<cv::Point>& key_point, int n, cv::Mat& A)
    {
        //Number of key points
        int N = key_point.size();
    
        //构造矩阵X
        cv::Mat X = cv::Mat::zeros(n + 1, n + 1, CV_64FC1);
        for (int i = 0; i < n + 1; i++)
        {
            for (int j = 0; j < n + 1; j++)
            {
                for (int k = 0; k < N; k++)
                {
                    // X.at<double>(i, j) = X.at<double>(i, j) + std::pow(key_point[k].x, i + j);
                    X.at<double>(i, j) = X.at<double>(i, j) + std::pow(key_point[k].y, i + j);
                }
            }
        }
    
        //构造矩阵Y
        cv::Mat Y = cv::Mat::zeros(n + 1, 1, CV_64FC1);
        for (int i = 0; i < n + 1; i++)
        {
            for (int k = 0; k < N; k++)
            {
                // Y.at<double>(i, 0) = Y.at<double>(i, 0) + std::pow(key_point[k].x, i) * key_point[k].y;
                Y.at<double>(i, 0) = Y.at<double>(i, 0) + std::pow(key_point[k].y, i) * key_point[k].x;
            }
        }
    
        A = cv::Mat::zeros(n + 1, 1, CV_64FC1);
        //求解矩阵A
        cv::solve(X, Y, A, cv::DECOMP_LU);
        return true;
    }

public:
    localGoalExtraction(ros::NodeHandle nh);
    ~localGoalExtraction(){}
    void occupancyGridHandler(const nav_msgs::OccupancyGridConstPtr occupancyMapPtr);

};

localGoalExtraction::localGoalExtraction(ros::NodeHandle nh): nh_(nh), occupancyGridTopic_("/occupancy_map_local"), ac("move_base",true)
{
    // wait for move base action server
    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO_STREAM("waiting for the move_base action server to come up!");
    }
    occupancySub_ = nh_.subscribe(occupancyGridTopic_, 5, &localGoalExtraction::occupancyGridHandler, this);
    markerPub_ = nh_.advertise<visualization_msgs::Marker>("local_goal",1);
    shape_ = visualization_msgs::Marker::SPHERE;
}

void localGoalExtraction::occupancyGridHandler(const nav_msgs::OccupancyGridConstPtr occupancyMapPtr) {
    // under /map frame
    // map origin
    ROS_INFO_STREAM("Start Extraction!");
    double origin_x = occupancyMapPtr->info.origin.position.x;
    double origin_y = occupancyMapPtr->info.origin.position.y;
    double origin_z = occupancyMapPtr->info.origin.position.z - 10;
    // ROS_INFO_STREAM("Origin of Map = (" << origin_x << "," << origin_y << ")");
    // map size
    int width = occupancyMapPtr->info.width;
    int height = occupancyMapPtr->info.height;

    cv::Mat cvLocalMap(width, height, CV_8UC1);
    // for display
    cv::Mat dispImg(width,height,CV_8UC3);
    for (int i = 0; i < height; i++) {
        uchar* rowPtr = cvLocalMap.ptr<uchar>(i);
        cv::Vec3b* dispRowPtr = dispImg.ptr<cv::Vec3b>(i);
        for (int j = 0; j < width; j++) {
            int idx = i * width + j;
            auto occupancy = occupancyMapPtr->data[idx];
            if (occupancy == -1) {
                // unknown
                rowPtr[j] = 128;
                dispRowPtr[j] = cv::Vec3b(128,128,128);
            }
            else if (occupancy == 0) {
                // free
                rowPtr[j] = 255;
                dispRowPtr[j] = cv::Vec3b(255,255,255);
            }
            else if (occupancy == 100) {
                // occupied
                rowPtr[j] = 0;
                dispRowPtr[j] = cv::Vec3b(0,0,0);
            }
            else {
                ROS_INFO_STREAM("Invalid Data in Occupancy Map");
            }
        }
    }

    cv::Mat afterMorph;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5));
    cv::morphologyEx(cvLocalMap,afterMorph,cv::MORPH_OPEN,kernel);
    // cv::imshow("morphology", afterMorph);
    cv::Mat regionGrowImg = region_grow(afterMorph, cv::Point(height/2,width/2));
    // cv::imshow("regionGrow", regionGrowImg);
    cv::Mat edgeImg;
    cv::Canny(regionGrowImg,edgeImg,3,9,3);
    // cv::imshow("cannyEdge", edgeImg);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edgeImg,contours,cv::noArray(),cv::RETR_LIST,cv::CHAIN_APPROX_NONE);
    cv::Mat fittedCoff;
    // cv::Mat resImg = cv::Mat::zeros(cvLocalMap.size(),CV_8UC3);
    cv::Point robotOrigin(width/2, height/2);
    std::vector<cv::Point> intersect;
    for (auto contour : contours) {
        // std::cout << "coutour size = " << contour.size() << std::endl;
        if (contour.size() < 50) {continue;}
        if (polynomial_curve_fit(contour,2,fittedCoff) == false) {continue;}
        std::vector<cv::Point> pointFitted;
        std::vector<cv::Point> localIntersect;
        for (int y = 0; y < height; y++) {
            double x = fittedCoff.at<double>(0,0) + fittedCoff.at<double>(1,0) * y + fittedCoff.at<double>(2,0) * std::pow(y,2);
            cv::Point p(int(x),y);
            if (x >= 0 && x < width) {
                pointFitted.push_back(p);
                double pointDist = cv::norm(robotOrigin - p);
                if (fabs(pointDist - 40) < 3 && y > robotOrigin.y) {localIntersect.push_back(p);}

                // std::cout << "x = " << x << " y = " << y << std::endl; 
            }
        }
        cv::polylines(dispImg, pointFitted, false, cv::Scalar(0,0,255),1,8,0);
        int sum_x = 0, sum_y = 0;
        for (auto p : localIntersect) {
            sum_x += p.x;
            sum_y += p.y;
        }
        if (localIntersect.size() != 0) {
            intersect.push_back(cv::Point(sum_x / localIntersect.size(), sum_y / localIntersect.size()));
        }
    }
    cv::circle(dispImg,robotOrigin,3,cv::Scalar(255,0,0),1,8,0);
    cv::Point goalPoint;
    if (intersect.size() == 1) {
        goalPoint.x = intersect[0].x;
        goalPoint.y = intersect[0].y;
    }
    else if (intersect.size() == 2) {
        goalPoint.x = (intersect[0].x + intersect[1].x) / 2.0;
        goalPoint.y = (intersect[0].y + intersect[1].y) / 2.0;
    }
    else {
        ROS_INFO_STREAM("NO LOCAL GOAL FOUND!");
        return;
    }
    cv::circle(dispImg, goalPoint, 3, cv::Scalar(255,0,255), 1, 8, 0);
    cv::imshow("fittingResult",dispImg);
    
    cv::waitKey(1);

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = occupancyMapPtr->header.stamp;
    goal.target_pose.pose.position.x = origin_x + goalPoint.x / 10.0;
    goal.target_pose.pose.position.y = origin_y + goalPoint.y / 10.0;
    goal.target_pose.pose.position.z = origin_z;
    goal.target_pose.pose.orientation.w = 1.0;
    ROS_INFO_STREAM("Sending Goal as " << goal.target_pose.pose.position.x << " " << goal.target_pose.pose.position.y << " " << goal.target_pose.pose.position.z);
    ac.sendGoal(goal);

    marker_.header.frame_id = "map";
    marker_.header.stamp = occupancyMapPtr->header.stamp;
    marker_.ns = "goal";
    marker_.id = 0;
    marker_.type = shape_;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.pose.position.x = origin_x + goalPoint.x / 10.0;
    marker_.pose.position.y = origin_y + goalPoint.y / 10.0;
    marker_.pose.position.z = origin_z;
    marker_.pose.orientation.x = 0.0;
    marker_.pose.orientation.y = 0.0;
    marker_.pose.orientation.z = 0.0;
    marker_.pose.orientation.w = 1.0;
    marker_.scale.x = 0.5;
    marker_.scale.y = 0.5;
    marker_.scale.z = 0.5;
    marker_.color.r = 0.0f;
    marker_.color.g = 1.0f;
    marker_.color.b = 0.0f;
    marker_.color.a = 1.0;
    marker_.lifetime = ros::Duration();
    markerPub_.publish(marker_);
    // ac.waitForResult();
    // if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    //     ROS_INFO_STREAM("Move Success!");
    // }
    // else {
    //     ROS_INFO_STREAM()
    // }


    /* Edge Detection Based Method
    cv::Point localorigin(height/2,width/2);
    cv::Mat edge;
    cv::blur(cvLocalMap,edge,cv::Size(3,3));
    cv::Canny(edge,edge,3,9,3);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edge,contours,cv::noArray(),cv::RETR_LIST,cv::CHAIN_APPROX_SIMPLE);
    std::vector<int> indices(contours.size());
    std::iota(indices.begin(), indices.end(), 0); // iota::将数组按递增顺序赋值
    std::sort(indices.begin(), indices.end(), [&contours](int lhs, int rhs){return contours[lhs].size() > contours[rhs].size();}); // 按contour长度排序index
    int N = 3; // 5 largest contours
    N = std::min(N, int(contours.size()));
    std::vector<int> clipIndices(indices.begin(),indices.begin()+N);
    cv::Mat3b res(width,height,cv::Vec3b(0,0,0));
    cv::Mat fittedCoff;
    // std::vector<double> fittingErr;
    std::vector<double> distVec;
    std::vector<cv::Scalar> color = {{255,0,0},{0,255,0},{0,0,255},{128,128,0},{0,128,128}};
    for (int i = 0; i < N; i++) {
        
        if (polynomial_curve_fit(contours[indices[i]],2,fittedCoff) == false) {continue;}
        std::vector<cv::Point> pointFitted;
        for (int x = 0; x < height; x++) {
            double y = fittedCoff.at<double>(0,0) + fittedCoff.at<double>(1,0) * x + fittedCoff.at<double>(2,0) * std::pow(x,2);
            if (y >= 0 && y < width) {
                pointFitted.push_back(cv::Point(x,int(y)));
                std::cout << "x = " << x << " y = " << y << std::endl; 
            }
              
        }
        cv::polylines(res, pointFitted, false, cv::Scalar(0,255,255),1,8,0);
        // double a = fittedCoff.at<double>(2,0);
        // double b = fittedCoff.at<double>(1,0);
        // double c = fittedCoff.at<double>(0,0);
        // double x0 = height / 2.0;
        // double y0 = width / 2.0;
        // std::vector<double> solution;
        // ShengJin(4*a*a, 4*a*b, b*b+4*a*c-4*y0+2, 2*b*(c-y0)-2*x0, solution);
        // std::cout << "solution = " << solution[0] << " | " << solution[1] << " | " << solution[2] << std::endl;
        
        // double dist = cv::pointPolygonTest(contours[indices[i]],localorigin,1);
        // distVec.push_back(std::fabs(dist));

        // double std_err = 0;
        // for (auto p : contours[indices[i]]) {
        //     double y_est = fittedCoff.at<double>(0,0) + fittedCoff.at<double>(1,0) * p.x + fittedCoff.at<double>(2,0) * std::pow(p.x,2);
        //     cv::circle(res, p, 1, color[i], 1, 8, 0);
        //     std_err += std::pow(y_est-p.y,2);
        // }
        // std_err = std::sqrt(std_err / contours[i].size());
        // fittingErr.push_back(std_err);
        // cv::Scalar color(rand() & 255, rand() & 255, rand() & 255);
        // cv::Vec3b othercolor(color[2],color[0],color[1]);
        // cv::drawContours(res,contours,indices[i],color);
    }
    // std::sort(clipIndices.begin(),clipIndices.end(),[&distVec](int lhs, int rhs){return distVec[lhs] < distVec[rhs];});
    // // std::sort(clipIndices.begin(),clipIndices.end(),[&fittingErr](int lhs, int rhs){return fittingErr[lhs] > fittingErr[rhs];});
    // int finalN = std::min(2,N);
    // for (int i = 0; i < finalN; i++) {
    //     cv::Scalar color(rand() & 255, rand() & 255, rand() & 255);
    //     cv::Vec3b othercolor(color[2],color[0],color[1]);
    //     cv::drawContours(res,contours,clipIndices[i],color);
    // }
    


    // cv::Mat dst(cvLocalMap.size(),CV_8UC1,cv::Scalar::all(0));
    // cv::drawContours(dst,contours,-1,cv::Scalar::all(255));
    cv::imshow("Local Map in OpenCV", cvLocalMap);
    cv::imshow("Edge",edge);
    cv::imshow("Contours",res);
    cv::waitKey(1);
    */
}


















int main(int argc, char** argv) {
    ros::init(argc, argv, "edge_detection");
    ros::NodeHandle nh;
    localGoalExtraction localGoalExtraction(nh);
    ros::spin();
    return 0;
}
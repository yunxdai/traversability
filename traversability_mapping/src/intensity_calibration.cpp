#include "utility.h"
#include <boost/bind.hpp>
void intensityCalibration(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in){


    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(pc_in);
    int k = 3;

    for(int i=0;i<pc_in->points.size();i++) {
        // cout << " intensity before = " << pc_in->points[i].intensity;
        double distance = pc_in->points[i].x*pc_in->points[i].x+pc_in->points[i].y*pc_in->points[i].y+pc_in->points[i].z*pc_in->points[i].z;
        pc_in->points[i].intensity /= distance;
        // cout << " intensity after = " << pc_in->points[i].intensity;
        std::vector<int> pointIdx(k);
        std::vector<float> pointDist(k);
        Eigen::Vector3d pVec(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);

        if (kdtree.nearestKSearch(pc_in->points[i], k, pointIdx, pointDist) == k) { // knn search返回的第一个值是当前搜索点
            // auto p1 = pc_in->points[pointIdx[0]];
            auto p2 = pc_in->points[pointIdx[1]];
            auto p3 = pc_in->points[pointIdx[2]];
            // Eigen::Vector3d p1Vec(p1.x, p1.y, p1.z);
            Eigen::Vector3d p2Vec(p2.x, p2.y, p2.z);
            Eigen::Vector3d p3Vec(p3.x, p3.y, p3.z);
            p2Vec = pVec - p2Vec;
            p3Vec = pVec - p3Vec;
            // cout << " pvec =" << pVec.transpose() << endl << " p1vec =" << p1Vec.transpose() << " p2vec =" << p2Vec.transpose() << " p3vec =" << p3Vec.transpose() << endl;
            Eigen::Vector3d localNorm = p2Vec.cross(p3Vec) / (p2Vec.norm() * p3Vec.norm());
            double cos = pVec.transpose().dot(localNorm) / pVec.norm();
            // cout << "cos = " << cos << endl;
            pc_in->points[i].intensity *= int(fabs(cos) * 1e06);
            // cout << " intensity after = " << pc_in->points[i].intensity * 1e06 << endl;
        }
        
        
        
        /*
        if(pc_in->points[i].z<-1.1){
            if(pc_in->points[i].intensity <0.05)
                pc_in->points[i].intensity =0.05;
            double distance_temp = pc_in->points[i].x*pc_in->points[i].x+pc_in->points[i].y*pc_in->points[i].y;
            double distance_ratio = sqrt(pc_in->points[i].x*pc_in->points[i].x+pc_in->points[i].y*pc_in->points[i].y+pc_in->points[i].z*pc_in->points[i].z);
            double ratio = fabs(sqrt(distance_temp)/pc_in->points[i].z);
            double angle = atan(ratio); 
            if (angle<M_PI/18) angle = M_PI/18;
            if (angle>M_PI/7.5) angle = M_PI/7.5;
            double new_intensity = pc_in->points[i].intensity * cos(angle)/cos(M_PI/7.5);
            pc_in->points[i].intensity = new_intensity;
            if(pc_in->points[i].intensity > 1)
                pc_in->points[i].intensity = 1;
            //pc_in->points[i].intensity =1.0;
        }
        */
    }

    return;
}

void pointcloudHandler(const sensor_msgs::PointCloud2ConstPtr& pc_raw, ros::Publisher& pb) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_in(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*pc_raw,*pc_in);
    intensityCalibration(pc_in);
    pb.publish(pc_in);

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "intensity_calibration");
    ros::NodeHandle nh;
    ros::Publisher pcPub = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/calibrated_points",1);
    ros::Subscriber pcSub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points",5,boost::bind(&pointcloudHandler,_1,pcPub));
    ros::spin();
}
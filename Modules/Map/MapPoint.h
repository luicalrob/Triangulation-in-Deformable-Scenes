#ifndef SLAM_MAPPOINT_H
#define SLAM_MAPPOINT_H

#include <opencv2/opencv.hpp>
#include <Eigen/Core>

class MapPoint {
public:
    /*
     * Constructor with a given 3D position
     */
    MapPoint(Eigen::Vector3f& p3d);

    /*
     * Copy constructor
     */
    MapPoint(const MapPoint& other);
    
    /*
     * Clone method
     */
    MapPoint* clone() const;

    /*
     * Gets the position of the MapPoint in the world reference
     */
    Eigen::Vector3f getWorldPosition();

    /*
     * Sets the new position of the MapPoint in the world reference
     */
    void setWorldPosition(Eigen::Vector3f& p3d);

    /*
     * Sets the most distinctive descriptor of the MapPoint
     */
    void setDescriptor(cv::Mat& desc);

    /*
     * Gets the most distinctive descriptor of the MapPoint
     */
    cv::Mat& getDescriptor();

    /*
     * Sets the orientation of the MapPoint
     */
    void setNormalOrientation(Eigen::Vector3f& v);

    /*
     * Gets the orientation of the MapPoint
     */
    Eigen::Vector3f getNormalOrientation();

    /*
     * Setters and getters for the Scale invariance distances
     */
    void setMinDistanceInvariance(float minDistance);
    void setMaxDistanceInvariance(float maxDistance);
    float getMinDistanceInvariance();
    float getMaxDistanceInvariance();

    /*
     * Gets the unique id of the MapPoint
     */
    long unsigned int getId();

private:
    // 3D position of the point in the world reference
    Eigen::Vector3f position3D_;

    // Normal orientation of all observations of the MapPoint
    Eigen::Vector3f normalOrientation_;

    // Most distinctive descriptor of the MapPoint for fast matching
    cv::Mat mDescriptor_;

    float fMinDistance_, fMaxDistance_;

    // Unique id
    long unsigned int nId_;
    static long unsigned int nNextId_;
};

#endif //SLAM_MAPPOINT_H

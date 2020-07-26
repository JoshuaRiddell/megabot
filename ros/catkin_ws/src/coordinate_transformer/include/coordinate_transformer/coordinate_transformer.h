#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>

class CoordinateTransformer {
public:
    CoordinateTransformer();

    void setGroundPlaneOffset(double offset);
    void setCameraParameters(double cx, double cy, double fx, double fy);
    void setCameraTransform(tf2::Transform transform);

    tf2::Vector3 projectImagePointToGroundPlane(cv::Point point);
    std::vector<tf2::Vector3> getCameraBoundsOnPlane(const cv::Mat &img);

private:
    tf2::Vector3 getUnitVectorFromCameraTo(cv::Point point);
    tf2::Vector3 rotateCameraVectorToWorld(tf2::Vector3 vector);
    tf2::Vector3 projectVectorToGroundPlane(tf2::Vector3 vector);
    tf2::Vector3 transformLocalToGlobal(tf2::Vector3 vector);

    tf2::Transform cameraTransform;

    double planeOffset = 0;
    double cx = 0;
    double cy = 0;
    double fx = 0;
    double fy = 0;
};

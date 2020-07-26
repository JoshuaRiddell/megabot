#include <coordinate_transformer/coordinate_transformer.h>

CoordinateTransformer::CoordinateTransformer() {

}

void CoordinateTransformer::setGroundPlaneOffset(double offset) {
    planeOffset = offset;
}

void CoordinateTransformer::setCameraParameters(double cx, double cy, double fx, double fy) {
    this->cx = cx;
    this->cy = cy;
    this->fx = fx;
    this->fy = fy;
}

void CoordinateTransformer::setCameraTransform(tf2::Transform transform) {
    cameraTransform = transform;
}

tf2::Vector3 CoordinateTransformer::projectImagePointToGroundPlane(cv::Point point) {
    tf2::Vector3 cameraVector = getUnitVectorFromCameraTo(point);
    tf2::Vector3 worldVector = rotateCameraVectorToWorld(cameraVector);
    tf2::Vector3 projectedGroundVector = projectVectorToGroundPlane(worldVector);
    tf2::Vector3 globalLocation = transformLocalToGlobal(projectedGroundVector);
    return globalLocation;
}

tf2::Vector3 CoordinateTransformer::getUnitVectorFromCameraTo(cv::Point point) {
    return tf2::Vector3(
        1.0,
        -(point.x - cx)/fx,
        -(point.y - cy)/fy
    );
}

tf2::Vector3 CoordinateTransformer::rotateCameraVectorToWorld(tf2::Vector3 vector) {
    return tf2::quatRotate(cameraTransform.getRotation(), vector);
}

tf2::Vector3 CoordinateTransformer::projectVectorToGroundPlane(tf2::Vector3 vector) {
    double rise = -(cameraTransform.getOrigin().getZ() - planeOffset) / vector.getZ();

    return tf2::Vector3(
        rise * vector.getX(),
        rise * vector.getY(),
        0
    );
}

tf2::Vector3 CoordinateTransformer::transformLocalToGlobal(tf2::Vector3 vector) {
    return tf2::Vector3(
        cameraTransform.getOrigin().getX() + vector.getX(),
        cameraTransform.getOrigin().getY() + vector.getY(),
        planeOffset + vector.getZ()
    );
}

std::vector<tf2::Vector3> CoordinateTransformer::getCameraBoundsOnPlane(const cv::Mat &img) {
    std::vector<cv::Point> image_bounds;
    image_bounds.push_back(cv::Point(0.1*img.cols, 0.1*img.rows));
    image_bounds.push_back(cv::Point(0.9*img.cols, 0.1*img.rows));
    image_bounds.push_back(cv::Point(0.9*img.cols, 0.9*img.rows));
    image_bounds.push_back(cv::Point(0.1*img.cols, 0.9*img.rows));

    std::vector<tf2::Vector3> viewBounds;

    for (int i = 0; i < image_bounds.size(); ++i) {
        viewBounds.push_back(projectImagePointToGroundPlane(image_bounds.at(i)));
    }

    return viewBounds;
}



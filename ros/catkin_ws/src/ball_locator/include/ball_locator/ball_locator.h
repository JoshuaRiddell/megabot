#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>

typedef std::vector<cv::Point> contour_t;
typedef std::vector<contour_t> contours_t;
typedef std::vector<cv::Point> centres_t;

class BallLocator {
public:
    BallLocator();

    void setHsvLowThreshold(uint8_t h, uint8_t s, uint8_t v);
    void setHsvHighThreshold(uint8_t h, uint8_t s, uint8_t v);
    void setAreaBounds(float low, float high);
    void setCircularityBounds(float low, float high);

    std::vector<cv::Point> getBallImageCentres(const cv::Mat &img);

    bool generateDebugImage = false;
    cv::Mat debugImage;

private:
    contours_t getContours(const cv::Mat &img);
    contours_t filterContours(const contours_t &contours);
    std::vector<cv::Point> getCircleCentres(const contours_t &contours);

    void updateDebugImage(const cv::Mat &img, const contours_t &contours, const centres_t centres);

    cv::Scalar hsvLowThreshold = cv::Scalar(0,0,0);
    cv::Scalar hsvHighThreshold = cv::Scalar(255,255,255);
    cv::Mat thresholdedImage;
    float areaLowThreshold = 0;
    float areaHighThreshold = 1000000;
    float circularityLowThreshold = 0;
    float circularityHighThreshold = 1.;
};


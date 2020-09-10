#include <ball_locator/ball_locator.h>
#include <opencv2/imgproc.hpp>

BallLocator::BallLocator() {

}

std::vector<cv::Point> BallLocator::getBallImageCentres(const cv::Mat &img) {
    contours_t contours = getContours(img);
    contours_t filteredContours = filterContours(contours);
    centres_t circleCentres = getCircleCentres(filteredContours);

    if (generateDebugImage) {
        updateDebugImage(img, filteredContours, circleCentres);
    }

    return circleCentres;
}

contours_t BallLocator::getContours(const cv::Mat &img) {
    cv::Mat hsv;

    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, hsvLowThreshold, hsvHighThreshold, thresholdedImage);

    cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(morphSize, morphSize));
    cv::erode(thresholdedImage, thresholdedImage, element, cv::Point(-1,-1), erodeCount);
    cv::dilate(thresholdedImage, thresholdedImage, element, cv::Point(-1,-1), dilateCount);

    cv::erode(thresholdedImage, thresholdedImage, element, cv::Point(-1,-1), erode2Count);
    cv::dilate(thresholdedImage, thresholdedImage, element, cv::Point(-1,-1), dilate2Count);

    contours_t contours;
    cv::findContours(thresholdedImage, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    return contours;
}

contours_t BallLocator::filterContours(const contours_t &contours) {
    contours_t filteredContours;

    for (int i = 0; i< contours.size(); i++)
    {
        contour_t contour = contours.at(i);

        double area = cv::contourArea(contour);
        if (area < areaLowThreshold)
            continue;
        if (area > areaHighThreshold)
            continue;

        double perimeter = cv::arcLength(contour, true);
        double circularity = 4*area*M_PI/perimeter/perimeter;
        if (circularity < circularityLowThreshold)
            continue;
        if (circularity > circularityHighThreshold)
            continue;

        filteredContours.push_back(contour);
    }

    return filteredContours;
}

std::vector<cv::Point> BallLocator::getCircleCentres(const contours_t &contours) {
    std::vector<cv::Point> centres;

    for (int i = 0; i < contours.size(); i++)
    {
        cv::RotatedRect ellipse;
        ellipse = cv::fitEllipse(contours[i]);
        cv::Point centre = ellipse.center;
        centres.push_back(centre);
    }

    return centres;
}

void BallLocator::updateDebugImage(const cv::Mat &img, const contours_t &contours, const centres_t centres) {
    img.copyTo(debugImage);

    cv::Mat colourThresholdedImage;
    cv::cvtColor(thresholdedImage, colourThresholdedImage, cv::COLOR_GRAY2BGR);
    cv::addWeighted(debugImage, 0.7, colourThresholdedImage, 0.3, 0, debugImage);

    cv::Scalar drawColour = cv::Scalar(0, 0, 255);

    for (int i = 0; i < contours.size(); ++i) {
        cv::drawContours(debugImage, contours, i, drawColour, 2, 8);
    }

    for (int i = 0; i < centres.size(); ++i) {
        cv::circle(debugImage, centres.at(i), 2, drawColour, 3);
    }
}

void BallLocator::setHsvLowThreshold(uint8_t h, uint8_t s, uint8_t v) {
    hsvLowThreshold = cv::Scalar(h, s, v);
}

void BallLocator::setHsvHighThreshold(uint8_t h, uint8_t s, uint8_t v) {
    hsvHighThreshold = cv::Scalar(h, s, v);
}

void BallLocator::setAreaBounds(float low, float high) {
    areaLowThreshold = low;
    areaHighThreshold = high;
}

void BallLocator::setCircularityBounds(float low, float high) {
    circularityLowThreshold = low;
    circularityHighThreshold = high;
}

void BallLocator::setMorphSize(int size) {
    morphSize = size;
}

void BallLocator::setErodeCount(int count) {
    erodeCount = count;
}

void BallLocator::setDilateCount(int count) {
    dilateCount = count;
}

void BallLocator::setErode2Count(int count) {
    erode2Count = count;
}

void BallLocator::setDilate2Count(int count) {
    dilate2Count = count;
}

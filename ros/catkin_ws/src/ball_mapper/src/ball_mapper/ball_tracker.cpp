#include <ball_mapper/ball_tracker.h>
#include <ros/time.h>

BallTracker::BallTracker(geometry_msgs::PointStamped initial_point,
            std::string frame_id,
            int max_samples,
            int num_samples_valid_threshold,
            int samples_distance_valid_threshold,
            double expiry_timeout) {
    // initialise id to invalid since ball is not valid on startup
    _id = -1;

    // save frame id
    _frame_id = frame_id;

    // maximum samples we will track for this ball
    _max_samples = max_samples;

    // conditions for valid ball location
    _num_samples_valid_threshold = num_samples_valid_threshold;
    _samples_distance_valid_threshold = samples_distance_valid_threshold;
    _expiry_timeout = expiry_timeout;

    // add the initial sample to our list
    tf2::Vector3 point_tf = tf2::Vector3(
        initial_point.point.x,
        initial_point.point.y,
        initial_point.point.z
    );
    samples.push_back(point_tf);
    _last_sample_time = initial_point.header.stamp;

    // sample mean is now just the added point
    sample_mean = point_tf;
}

bool BallTracker::is_valid() {
    // check we have enough samples
    if (samples.size() < _num_samples_valid_threshold) {
        return false;
    }

    // we are valid, if the id isn't set then set it now
    if (_id == -1) {
        _id = _next_id++;
    }

    return true;
}

bool BallTracker::add_sample(geometry_msgs::PointStamped point) {
    tf2::Vector3 point_tf = tf2::Vector3(
        point.point.x,
        point.point.y,
        point.point.z
    );

    if (sample_mean.distance(point_tf) > _samples_distance_valid_threshold) {
        // point is too far to be considered
        return false;
    }
    
    // update samples
    samples.push_back(point_tf);
    _last_sample_time = point.header.stamp;

    // remove oldest sample if required
    if (samples.size() > _max_samples) {
        samples.pop_front();
    }

    // update the location since samples have now changed
    calculate_location();
}

bool BallTracker::expired() {
    // check if this ball has not had enough data to be considered
    if (is_valid()) {
        // valid balls have unlimited expiry
        return false;
    }

    ros::Duration expiry_duration(_expiry_timeout);
    if (ros::Time::now() - _last_sample_time < expiry_duration) {
        // it's not been long enough for it to expire
        return false;
    }

    return true;
}

void BallTracker::calculate_location() {
    // varaibles to store sums for averaging
    double x_sum = 0;
    double y_sum = 0;
    double z_sum = 0;

    // number of samples we have
    int n = samples.size();

    // sum x and y coordinates
    for (auto sample = samples.begin(); sample != samples.end(); ++sample) {
        x_sum += sample->getX();
        y_sum += sample->getY();
        z_sum += sample->getZ();
    }

    // get mean coordinates
    sample_mean = tf2::Vector3(
        x_sum/n,
        y_sum/n,
        z_sum/n
    );
}

geometry_msgs::Point BallTracker::get_location() {
    // pack calculated location into stamped point
    geometry_msgs::Point msg;

    msg.x = sample_mean.getX();
    msg.y = sample_mean.getY();
    msg.z = sample_mean.getZ();

    return msg;
}

int BallTracker::_next_id = 0;

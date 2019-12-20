#include <ball_mapper/ball_tracker.h>

BallTracker::BallTracker(geometry_msgs::PointStamped point,
            std::string frame_id = "map",
            int max_samples = 10,
            int num_samples_valid_threshold = 5,
            int samples_distance_valid_threshold = 0.01) {
    // initialise id to invalid since ball is not valid on startup
    _id = -1;

    // save frame id
    _frame_id = frame_id;

    // maximum samples we will track for this ball
    _max_samples = max_samples;

    // conditions for valid ball location
    _num_samples_valid_threshold = num_samples_valid_threshold;
    _samples_distance_valid_threshold = samples_distance_valid_threshold;

    // add the initial sample to our array
    tf2::Vector3 point_tf = tf2::Vector3(
        point.point.x,
        point.point.y,
        point.point.z
    );
    samples.push_back(point_tf);
    _last_sample_time = point.header.stamp;

    // sample mean is now just the added point
    sample_mean = point_tf;
}

bool BallTracker::is_valid() {
    // check we have enough samples and the stdev of them is low enough
    if (samples.size() < _num_samples_valid_threshold) {
        return false;
    }


}



// // set out constant id and max samples
//         _id = _next_id++;


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

    // if is valid and the id isn't set, set our id
    if (is_valid() && _id == -1) {
        _id = _next_id++;
    }
}

bool BallTracker::expired(ros::Time time) {
    // check if this ball has not had enough data to be considered
    return false;
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

geometry_msgs::PointStamped BallTracker::get_location() {
    // pack calculated location into stamped point
    geometry_msgs::PointStamped msg;
    msg.header.stamp = _last_sample_time;
    msg.header.frame_id = _frame_id;

    msg.point.x = sample_mean.getX();
    msg.point.y = sample_mean.getY();
    msg.point.z = sample_mean.getZ();

    return msg;
}

int BallTracker::_next_id = 0;

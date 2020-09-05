#include <ros/time.h>
#include <ball_mapper/ball_tracker.h>

BallTracker::BallTracker(ros::Time stamp,
                tf2::Vector3 initial_sample,
                std::string frame_id,
                int max_samples,
                int num_samples_valid_threshold,
                double samples_distance_valid_threshold,
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
    samples.push_back(initial_sample);
    _last_sample_time = stamp;

    // sample mean is now just the added point
    sample_mean = initial_sample;

    // initialise to not in view
    _was_in_view = false;
}

bool BallTracker::update(ros::Time stamp,
                        std::vector<tf2::Vector3> &locations,
                        std::vector<tf2::Vector3> camera_view) {
    
    // if we're not in view then none of the points will be for us
    bool currently_in_view = in_view(camera_view);
    bool sample_found = add_sample(stamp, locations);

    if (expired_by_timeout(stamp)) {
        return true;
    }

    if (expired_by_view(stamp, currently_in_view, sample_found)) {
        return true;
    }
    return false;
}

bool BallTracker::in_view(std::vector<tf2::Vector3> camera_view) {
    bool ret = false;
    int i, j;
    for (i = 0, j = camera_view.size()-1; i < camera_view.size(); j = i++) {
        if ((camera_view[i].getY() > sample_mean.getY()) != (camera_view[j].getY() > sample_mean.getY()) &&
            (sample_mean.getX() < (camera_view[j].getX() - camera_view[i].getX()) * (sample_mean.getY() - camera_view[i].getY()) / (camera_view[j].getY()-camera_view[i].getY()) + camera_view[i].getX())) {
            ret = !ret;
        }
    }
    return ret;
}

bool BallTracker::add_sample(ros::Time stamp, std::vector<tf2::Vector3> &locations) {
    // loop through all passed detected locations
    for (int i = 0; i < locations.size(); ++i) {

        // check point is close enough to current mean to be considered
        if (sample_mean.distance(locations[i]) < _samples_distance_valid_threshold) {

            // add sample to our list
            samples.push_back(locations[i]);
            _last_sample_time = stamp;

            // remove this sample from the locations list since we have found the ball
            locations.erase(locations.begin() + i);

            // remove our oldest sample to keep the max length if required
            if (samples.size() > _max_samples) {
                samples.pop_front();
            }

            // update the location since samples have now changed
            calculate_location();

            // check location validity and add id if required
            if (_id == -1 && location_valid()) {
                _id = _next_id++;
            }

            // there can't be two samples of the same ball so we can stop here
            return true;
        }
    }

    // no samples corresponded to our ball
    return false;
}

bool BallTracker::expired_by_timeout(ros::Time stamp) {
    // check if ball's location has been invalid for enough time
    ros::Duration expiry_duration(_expiry_timeout);
    if (!location_valid()
            && stamp - _last_sample_time > expiry_duration) {
        // ball has not been valid for enough time
        return true;
    }

    return false;
}

bool BallTracker::expired_by_view(ros::Time stamp, bool currently_in_view, bool sample_found) {
    if (!currently_in_view) {
        _was_in_view = false;
        return true;   // delete any ball that's not in view
    }

    if (!_was_in_view) {
        _enter_view_time = stamp;
        _was_in_view = true;
    }

    ros::Duration expiry_duration(_expiry_timeout);
    if (!sample_found &&
        stamp - _enter_view_time > expiry_duration) {
        return true;
    }

    return false;
}

bool BallTracker::location_valid() {
    // check we have enough samples
    if (samples.size() < _num_samples_valid_threshold) {
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

tf2::Vector3 BallTracker::get_location() {
    // pack calculated location into stamped point
    return sample_mean;
}

int BallTracker::_next_id = 0;

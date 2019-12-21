#include <ros/time.h>
#include <tf2/utils.h>

class BallTracker {
public:
    // initialise the tracker with first sample
    BallTracker(ros::Time stamp,
                tf2::Vector3 initial_sample,
                std::string frame_id = "map",
                int max_samples = 30,
                int num_samples_valid_threshold = 5,
                double samples_distance_valid_threshold = 0.03,
                double expiry_timeout = 0.5);

    // update ball tracker with another sample
    // returns true if the tracker has expired
    bool update(ros::Time stamp,
                std::vector<tf2::Vector3> &locations,
                std::vector<tf2::Vector3> camera_view);

    // check if the current location is within valid threshold
    bool location_valid();

    // get the location based on measured samples
    tf2::Vector3 get_location();

private:
    bool add_sample(ros::Time stamp, std::vector<tf2::Vector3> &locations);
    bool expired_by_timeout(ros::Time stamp);
    bool expired_by_view(ros::Time stamp, bool currently_in_view, bool sample_found);
    bool in_view(std::vector<tf2::Vector3> camera_view);
    void calculate_location();
    
    // id of this ball tracker
    int _id;

    // id of the frame the tracker is operating in
    std::string _frame_id;

    // maximum samples stored inside the tracker array
    int _max_samples;

    // number of samples required before a valid location is reported
    int _num_samples_valid_threshold;

    // maximum distance for a sample to be considered part of this tracker
    double _samples_distance_valid_threshold;

    // timeout for tracker to be marked as removed
    double _expiry_timeout;

    // array of samples to be used to calculate the location
    std::list<tf2::Vector3> samples;

    // mean location of all the samples
    tf2::Vector3 sample_mean;

    // last time a sample was added
    ros::Time _last_sample_time;

    // last time this ball entered into the camera view
    ros::Time _enter_view_time;

    // if the ball location is currently inside the cameras view
    bool _was_in_view;

protected:
    // unique sequential ID iterator
    static int _next_id;
};

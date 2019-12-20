#include <geometry_msgs/PointStamped.h>
#include <tf2/utils.h>

class BallTracker {
    BallTracker(geometry_msgs::PointStamped point,
                std::string frame_id,
                int max_samples,
                int num_samples_valid_threshold,
                int samples_distance_valid_threshold);
    bool is_valid();
    bool add_sample(geometry_msgs::PointStamped point);
    bool expired(ros::Time time);
    void calculate_location();
    geometry_msgs::PointStamped get_location();

    int _id;
    std::string _frame_id;
    int _max_samples;
    int _num_samples_valid_threshold;
    int _samples_distance_valid_threshold;
    std::list<tf2::Vector3> samples;
    tf2::Vector3 sample_mean;
    ros::Time _last_sample_time;

protected:
    static int _next_id;
};

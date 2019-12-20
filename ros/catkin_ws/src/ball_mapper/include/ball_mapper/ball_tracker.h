#include <geometry_msgs/PointStamped.h>
#include <tf2/utils.h>

class BallTracker {
public:
    BallTracker(geometry_msgs::PointStamped initial_point,
                std::string frame_id = "map",
                int max_samples = 10,
                int num_samples_valid_threshold = 5,
                double samples_distance_valid_threshold = 0.01,
                double expiry_timeout = 5);
    bool is_valid();
    bool add_sample(geometry_msgs::PointStamped point);
    bool expired();
    void calculate_location();
    geometry_msgs::Point get_location();

private:
    int _id;
    std::string _frame_id;
    int _max_samples;
    int _num_samples_valid_threshold;
    double _samples_distance_valid_threshold;
    double _expiry_timeout;
    std::list<tf2::Vector3> samples;
    tf2::Vector3 sample_mean;
    ros::Time _last_sample_time;

protected:
    static int _next_id;
};

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <memory>
#include <vector>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdexcept>
#include <cmath>

/**
 * Finite impulse response filter. The precision is double.
 * Supports LMS adaptation and dynamic coefficient initialization.
 **/
class Fir1 {
public:
    template <unsigned nTaps> Fir1(const double (&_coefficients)[nTaps]) :
        coefficients(new double[nTaps]),
        buffer(new double[nTaps]()),
        taps(nTaps) {
        for(unsigned i=0; i<nTaps; i++) {
            coefficients[i] = _coefficients[i];
            buffer[i] = 0;
        }
    }

    Fir1(std::vector<double> _coefficients) {
        initWithVector(_coefficients);
    }

    Fir1(const double *coefficients, const unsigned number_of_taps);

    Fir1(const char* coeffFile, unsigned number_of_taps = 0);

    Fir1(unsigned number_of_taps, double value = 0);

    ~Fir1();

    inline double filter(double input) {
        const double *coeff = coefficients;
        const double *const coeff_end = coefficients + taps;
        double *buf_val = buffer + offset;
        *buf_val = input;
        double output_ = 0;
        while(buf_val >= buffer)
            output_ += *buf_val-- * *coeff++;
        buf_val = buffer + taps-1;
        while(coeff < coeff_end)
            output_ += *buf_val-- * *coeff++;
        if(++offset >= taps)
            offset = 0;
        return output_;
    }

    inline void lms_update(double error) {
        double *coeff = coefficients;
        const double *const coeff_end = coefficients + taps;
        double *buf_val = buffer + offset;
        while(buf_val >= buffer) {
            *coeff++ += *buf_val-- * error * mu;
        }
        buf_val = buffer + taps-1;
        while(coeff < coeff_end) {
            *coeff++ += *buf_val-- * error * mu;
        }
    }

    void setLearningRate(double _mu) { mu = _mu; };
    double getLearningRate() { return mu; };
    void reset();
    void zeroCoeff();
    void getCoeff(double* coeff_data, unsigned number_of_taps) const;
    void setCoeff(const double *coeff_data, const unsigned number_of_taps);
    std::vector<double> getCoeffVector() const {
        return std::vector<double>(coefficients, coefficients+taps);
    }
    unsigned getTaps() { return taps; };
    inline double getTapInputPower() {
        double *buf_val = buffer;
        double p = 0;
        for(unsigned i = 0; i < taps; i++) {
            p += (*buf_val) * (*buf_val);
            buf_val++;
        }
        return p;
    }

private:
    void initWithVector(std::vector<double> _coefficients);
    double *coefficients;
    double *buffer;
    unsigned taps;
    unsigned offset = 0;
    double mu = 0;
};

Fir1::Fir1(const double *_coefficients, const unsigned number_of_taps) :
    coefficients(new double[number_of_taps]),
    buffer(new double[number_of_taps]()),
    taps(number_of_taps) {
    for(unsigned int i=0; i<number_of_taps; i++) {
        coefficients[i] = _coefficients[i];
        buffer[i] = 0;
    }
}

Fir1::Fir1(unsigned number_of_taps, double value) :
    coefficients(new double[number_of_taps]),
    buffer(new double[number_of_taps]),  
    taps(number_of_taps) {
    for(unsigned int i=0; i<number_of_taps; i++) {
        coefficients[i] = value;
        buffer[i] = 0;
    }
}

void Fir1::initWithVector(std::vector<double> _coefficients) {
    coefficients = new double[_coefficients.size()];
    buffer = new double[_coefficients.size()]();
    taps = ((unsigned int)_coefficients.size());
    for(unsigned long i=0; i<_coefficients.size(); i++) {
        coefficients[i] = _coefficients[i];
        buffer[i] = 0;
    }
}

Fir1::Fir1(const char* coeffFile, unsigned number_of_taps) {
    std::vector<double> tmpCoefficients;
    FILE* f = fopen(coeffFile, "rt");
    if (!f) {
        throw std::invalid_argument("Could not open file.");
    }
    for(unsigned int i=0; (i<number_of_taps) || (number_of_taps==0); i++) {
        double v = 0;
        int r = fscanf(f, "%lf\n", &v);
        if (r < 1) break;
        tmpCoefficients.push_back(v);
    }
    fclose(f);
    initWithVector(tmpCoefficients);
}

Fir1::~Fir1() {
    delete[] buffer;
    delete[] coefficients;
}

void Fir1::reset() {
    memset(buffer, 0, sizeof(double)*taps);
    offset = 0;
}

void Fir1::zeroCoeff() {
    memset(coefficients, 0, sizeof(double)*taps);
    offset = 0;
}

void Fir1::getCoeff(double* coeff_data, unsigned number_of_taps) const {
    if (number_of_taps < taps)
        throw std::out_of_range("Fir1: target of getCoeff: too many weights to copy into target");
    memcpy(coeff_data, coefficients, taps * sizeof(double));
    if (number_of_taps > taps)
        memset(&coeff_data[taps], 0, (number_of_taps - taps)*sizeof(double));
}

void Fir1::setCoeff(const double* coeff_data, const unsigned number_of_taps) {
    if (number_of_taps != taps) {
        throw std::runtime_error("Invalid number of taps in new coefficient array");
    }
    for (unsigned int i = 0; i < number_of_taps; i++) {
        coefficients[i] = coeff_data[i];
    }
}

class ImuFilter {
public:
    ImuFilter() : nh_("~") {
        // Load parameters from params.yaml
        nh_.param("num_taps", num_taps_, 21);
        nh_.param("cutoff_frequency", cutoff_frequency_, 10.0);
        nh_.param("sampling_rate", sampling_rate_, 100.0);
        nh_.param("learning_rate", learning_rate_, 0.01);
        nh_.param("imu_topic", imu_topic_, std::string("/imu/data_newIMU"));
        nh_.param("filter_gyroscope", filter_gyroscope_, true);

        // Validate parameters
        if (num_taps_ <= 0) {
            ROS_ERROR("Invalid num_taps: %d, must be positive", num_taps_);
            throw std::invalid_argument("Invalid num_taps");
        }
        if (cutoff_frequency_ <= 0 || cutoff_frequency_ >= sampling_rate_ / 2.0) {
            ROS_ERROR("Invalid cutoff_frequency: %f, must be in (0, %f]", 
                      cutoff_frequency_, sampling_rate_ / 2.0);
            throw std::invalid_argument("Invalid cutoff_frequency");
        }
        if (sampling_rate_ <= 0) {
            ROS_ERROR("Invalid sampling_rate: %f, must be positive", sampling_rate_);
            throw std::invalid_argument("Invalid sampling_rate");
        }
        if (learning_rate_ <= 0) {
            ROS_ERROR("Invalid learning_rate: %f, must be positive", learning_rate_);
            throw std::invalid_argument("Invalid learning_rate");
        }
        if (imu_topic_.empty()) {
            ROS_ERROR("Invalid imu_topic: empty string");
            throw std::invalid_argument("Invalid imu_topic");
        }

        // Log parameters
        ROS_INFO("FIR filter parameters: num_taps=%d, cutoff_frequency=%f Hz, "
                 "sampling_rate=%f Hz, learning_rate=%f, imu_topic=%s, filter_gyroscope=%d",
                 num_taps_, cutoff_frequency_, sampling_rate_, learning_rate_, 
                 imu_topic_.c_str(), filter_gyroscope_);

        // Initialize FIR filters with low-pass coefficients
        std::vector<double> coeffs = computeFirCoefficients(num_taps_, cutoff_frequency_, sampling_rate_);
        accel_filter_x_ = std::make_unique<Fir1>(coeffs);
        accel_filter_y_ = std::make_unique<Fir1>(coeffs);
        accel_filter_z_ = std::make_unique<Fir1>(coeffs);
        gyro_filter_x_ = std::make_unique<Fir1>(coeffs);
        gyro_filter_y_ = std::make_unique<Fir1>(coeffs);
        gyro_filter_z_ = std::make_unique<Fir1>(coeffs);

        // Set learning rate for gyroscope filters (for LMS adaptation)
        gyro_filter_x_->setLearningRate(learning_rate_);
        gyro_filter_y_->setLearningRate(learning_rate_);
        gyro_filter_z_->setLearningRate(learning_rate_);

        // Initialize subscribers and publishers
        imu_sub_ = nh_.subscribe(imu_topic_, 10, &ImuFilter::imuCallback, this);
        stationary_sub_ = nh_.subscribe("/imu/stationary", 10, &ImuFilter::stationaryCallback, this);
        imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu/data_filtered", 10);
    }

private:
    // Compute FIR low-pass coefficients (simplified firwin with Hamming window)
    std::vector<double> computeFirCoefficients(int num_taps, double cutoff, double fs) {
        std::vector<double> coeffs(num_taps, 0.0);
        double nyquist = fs / 2.0;
        double normalized_cutoff = cutoff / nyquist;
        int M = num_taps - 1;
        double sum = 0.0;
        for (int n = 0; n < num_taps; n++) {
            if (n == M / 2) {
                coeffs[n] = normalized_cutoff;
            } else {
                coeffs[n] = normalized_cutoff * sin(M_PI * normalized_cutoff * (n - M / 2)) /
                            (M_PI * (n - M / 2));
            }
            // Apply Hamming window
            coeffs[n] *= 0.54 - 0.46 * cos(2 * M_PI * n / M);
            sum += coeffs[n];
        }
        // Normalize coefficients to ensure unity gain
        for (int n = 0; n < num_taps; n++) {
            coeffs[n] /= sum;
        }
        return coeffs;
    }

    // Callback for stationary state (to enable LMS adaptation)
    void stationaryCallback(const std_msgs::Bool::ConstPtr& msg) {
        is_stationary_ = msg->data;
    }

    // IMU callback with optional gyroscope filtering and LMS adaptation
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        sensor_msgs::Imu filtered_msg = *msg;
        // Filter accelerometer data
        filtered_msg.linear_acceleration.x = accel_filter_x_->filter(msg->linear_acceleration.x);
        filtered_msg.linear_acceleration.y = accel_filter_y_->filter(msg->linear_acceleration.y);
        filtered_msg.linear_acceleration.z = accel_filter_z_->filter(msg->linear_acceleration.z);
        // Filter gyroscope data if enabled
        if (filter_gyroscope_) {
            filtered_msg.angular_velocity.x = gyro_filter_x_->filter(msg->angular_velocity.x);
            filtered_msg.angular_velocity.y = gyro_filter_y_->filter(msg->angular_velocity.y);
            filtered_msg.angular_velocity.z = gyro_filter_z_->filter(msg->angular_velocity.z);
            // LMS adaptation for gyroscope (assume zero angular velocity when stationary)
            if (is_stationary_) {
                gyro_filter_x_->lms_update(-filtered_msg.angular_velocity.x); // Error = desired (0) - output
                gyro_filter_y_->lms_update(-filtered_msg.angular_velocity.y);
                gyro_filter_z_->lms_update(-filtered_msg.angular_velocity.z);
            }
        }
        // Publish filtered data
        imu_pub_.publish(filtered_msg);
    }

    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;
    ros::Subscriber stationary_sub_;
    ros::Publisher imu_pub_;
    std::unique_ptr<Fir1> accel_filter_x_, accel_filter_y_, accel_filter_z_;
    std::unique_ptr<Fir1> gyro_filter_x_, gyro_filter_y_, gyro_filter_z_;
    int num_taps_;
    double cutoff_frequency_, sampling_rate_, learning_rate_;
    std::string imu_topic_;
    bool filter_gyroscope_, is_stationary_ = false;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_filter_node");
    try {
        ImuFilter imu_filter;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in imu_filter_node: %s", e.what());
        return 1;
    }
    return 0;
}
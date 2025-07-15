#include "fir_lms_imu.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdexcept>
#include <cmath>

Fir1::Fir1(const double *_coefficients, const unsigned number_of_taps) :
    coefficients(new double[number_of_taps]),
    buffer(new double[number_of_taps]()),
    taps(number_of_taps) {
    for(unsigned int i=0; i<number_of_taps; i++) {
        coefficients[i] = _coefficients[i];
        buffer[i] = 0;
    }
}

template <unsigned nTaps>
Fir1::Fir1(const double (&_coefficients)[nTaps]) :
    coefficients(new double[nTaps]),
    buffer(new double[nTaps]()),
    taps(nTaps) {
    for(unsigned i=0; i<nTaps; i++) {
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

void Fir1::setOffset(int _offset) {
    if (_offset < 0) {
        throw std::invalid_argument("Offset must be non-negative");
    }
    if (_offset >= static_cast<int>(taps)) {
        throw std::invalid_argument("Offset must be less than number of taps");
    }
    offset = _offset;
}

ImuFilter::ImuFilter() : nh_("~") {
    nh_.param("num_taps", num_taps_, 21);
    nh_.param("cutoff_frequency", cutoff_frequency_, 10.0);
    nh_.param("sampling_rate", sampling_rate_, 100.0);
    nh_.param("learning_rate", learning_rate_, 0.01);
    nh_.param("imu_topic", imu_topic_, std::string("/imu/data_newIMU"));
    nh_.param("filter_gyroscope", filter_gyroscope_, true);
    nh_.param("mu", mu_, 0.01);
    nh_.param("offset", offset_, 0);

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
    if (mu_ <= 0) {
        ROS_ERROR("Invalid mu: %f, must be positive", mu_);
        throw std::invalid_argument("Invalid mu");
    }
    if (offset_ < 0) {
        ROS_ERROR("Invalid offset: %d, must be non-negative", offset_);
        throw std::invalid_argument("Invalid offset");
    }
    if (offset_ >= num_taps_) {
        ROS_ERROR("Invalid offset: %d, must be less than num_taps (%d)", offset_, num_taps_);
        throw std::invalid_argument("Invalid offset");
    }

    // Log parameters
    ROS_INFO("FIR filter parameters: num_taps=%d, cutoff_frequency=%f Hz, "
             "sampling_rate=%f Hz, learning_rate=%f, mu=%f, offset=%d, imu_topic=%s, filter_gyroscope=%d",
             num_taps_, cutoff_frequency_, sampling_rate_, learning_rate_, mu_, offset_, 
             imu_topic_.c_str(), filter_gyroscope_);

    // Initialize FIR filters with low-pass coefficients
    std::vector<double> coeffs = computeFirCoefficients(num_taps_, cutoff_frequency_, sampling_rate_);
    accel_filter_x_ = std::make_unique<Fir1>(coeffs);
    accel_filter_y_ = std::make_unique<Fir1>(coeffs);
    accel_filter_z_ = std::make_unique<Fir1>(coeffs);
    gyro_filter_x_ = std::make_unique<Fir1>(coeffs);
    gyro_filter_y_ = std::make_unique<Fir1>(coeffs);
    gyro_filter_z_ = std::make_unique<Fir1>(coeffs);

    // Set learning rate and offset for all filters
    accel_filter_x_->setLearningRate(mu_);
    accel_filter_y_->setLearningRate(mu_);
    accel_filter_z_->setLearningRate(mu_);
    gyro_filter_x_->setLearningRate(mu_);
    gyro_filter_y_->setLearningRate(mu_);
    gyro_filter_z_->setLearningRate(mu_);

    accel_filter_x_->setOffset(offset_);
    accel_filter_y_->setOffset(offset_);
    accel_filter_z_->setOffset(offset_);
    gyro_filter_x_->setOffset(offset_);
    gyro_filter_y_->setOffset(offset_);
    gyro_filter_z_->setOffset(offset_);

    // Initialize subscribers and publishers
    imu_sub_ = nh_.subscribe(imu_topic_, 10, &ImuFilter::imuCallback, this);
    stationary_sub_ = nh_.subscribe("/imu/stationary", 10, &ImuFilter::stationaryCallback, this);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu/data_filtered", 10);
}

std::vector<double> ImuFilter::computeFirCoefficients(int num_taps, double cutoff, double fs) {
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
        coeffs[n] *= 0.54 - 0.46 * cos(2 * M_PI * n / M);
        sum += coeffs[n];
    }
    for (int n = 0; n < num_taps; n++) {
        coeffs[n] /= sum;
    }
    return coeffs;
}

void ImuFilter::stationaryCallback(const std_msgs::Bool::ConstPtr& msg) {
    is_stationary_ = msg->data;
}

void ImuFilter::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    sensor_msgs::Imu filtered_msg = *msg;
    filtered_msg.linear_acceleration.x = accel_filter_x_->filter(msg->linear_acceleration.x);
    filtered_msg.linear_acceleration.y = accel_filter_y_->filter(msg->linear_acceleration.y);
    filtered_msg.linear_acceleration.z = accel_filter_z_->filter(msg->linear_acceleration.z);
    if (filter_gyroscope_) {
        filtered_msg.angular_velocity.x = gyro_filter_x_->filter(msg->angular_velocity.x);
        filtered_msg.angular_velocity.y = gyro_filter_y_->filter(msg->angular_velocity.y);
        filtered_msg.angular_velocity.z = gyro_filter_z_->filter(msg->angular_velocity.z);
        if (is_stationary_) {
            gyro_filter_x_->lms_update(-filtered_msg.angular_velocity.x);
            gyro_filter_y_->lms_update(-filtered_msg.angular_velocity.y);
            gyro_filter_z_->lms_update(-filtered_msg.angular_velocity.z);
        }
    }
    // Print filtered IMU data
    ROS_INFO("Filtered IMU Data: accel_x=%.6f, accel_y=%.6f, accel_z=%.6f",
             filtered_msg.linear_acceleration.x,
             filtered_msg.linear_acceleration.y,
             filtered_msg.linear_acceleration.z);
    if (filter_gyroscope_) {
        ROS_INFO("Filtered IMU Data: gyro_x=%.6f, gyro_y=%.6f, gyro_z=%.6f",
                 filtered_msg.angular_velocity.x,
                 filtered_msg.angular_velocity.y,
                 filtered_msg.angular_velocity.z);
    }
    imu_pub_.publish(filtered_msg);
}

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
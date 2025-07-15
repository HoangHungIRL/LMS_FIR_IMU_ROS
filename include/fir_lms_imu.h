#ifndef FIR_LMS_IMU_H
#define FIR_LMS_IMU_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <memory>
#include <vector>
#include <string>

/**
 * Finite impulse response filter. The precision is double.
 * Supports LMS adaptation and dynamic coefficient initialization.
 */
class Fir1 {
public:
    template <unsigned nTaps> 
    Fir1(const double (&_coefficients)[nTaps]);

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
        if(++offset >= static_cast<int>(taps))
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

    void setLearningRate(double _mu) { mu = _mu; }
    double getLearningRate() { return mu; }
    void setOffset(int _offset);
    int getOffset() { return offset; }
    void reset();
    void zeroCoeff();
    void getCoeff(double* coeff_data, unsigned number_of_taps) const;
    void setCoeff(const double *coeff_data, const unsigned number_of_taps);
    std::vector<double> getCoeffVector() const {
        return std::vector<double>(coefficients, coefficients+taps);
    }
    unsigned getTaps() { return taps; }
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
    int offset = 0;
    double mu = 0;
};

class ImuFilter {
public:
    ImuFilter();

private:
    std::vector<double> computeFirCoefficients(int num_taps, double cutoff, double fs);
    void stationaryCallback(const std_msgs::Bool::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;
    ros::Subscriber stationary_sub_;
    ros::Publisher imu_pub_;
    std::unique_ptr<Fir1> accel_filter_x_, accel_filter_y_, accel_filter_z_;
    std::unique_ptr<Fir1> gyro_filter_x_, gyro_filter_y_, gyro_filter_z_;
    int num_taps_;
    double cutoff_frequency_, sampling_rate_, learning_rate_, mu_;
    std::string imu_topic_;
    bool filter_gyroscope_, is_stationary_ = false;
    int offset_;
};

#endif // FIR_LMS_IMU_H
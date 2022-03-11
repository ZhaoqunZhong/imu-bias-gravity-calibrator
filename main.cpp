#include "acc_bias_calibrator.h"
#include "imu_tk/imu_tk.h"
#include <experimental/filesystem>
#include <iostream>

using namespace std;
using namespace imu_tk;

int main(int argc, char const *argv[]) {
    vector<TriadData> acc_data, gyro_data;
    // std::experimental::filesystem::path cwd = std::experimental::filesystem::current_path() / "imu.csv";
    std::experimental::filesystem::path cwd = std::experimental::filesystem::current_path();
    string imufile = cwd.string() + "/imu.csv";
//    cout << "imufile " << imufile << endl;
    importAsciiData(imufile.c_str(), acc_data, gyro_data, TIMESTAMP_UNIT_NSEC, DATASET_COMMA_SEPARATED);
    DataInterval init_static_interval = DataInterval::initialInterval(acc_data, 2.0);
    Eigen::Vector3d acc_variance = dataVariance(acc_data, init_static_interval);
    double norm_th = acc_variance.norm();
    vector<DataInterval> static_intervals;
    vector<TriadData> static_samples;
    staticIntervalsDetector(acc_data, 2 * norm_th, static_intervals, 400);
    vector<DataInterval> extracted_intervals;
    extractIntervalsSamples(acc_data, static_intervals,
                            static_samples, extracted_intervals,
                            400, true);
    //Perform here a quality test
/*	if (extracted_intervals.size() < 6) {
		cout << "Not enough intervals, calibration is not possible" << endl;
		return -1;
	}*/
    for (int i = 0; i < static_samples.size(); i++) {
        cout << "acc sample " << static_samples[i].data().transpose() << endl;
    }
    std::ofstream of_result;
    of_result.open(cwd.string() + "/static_detect_visual.csv", std::ofstream::out);
    for (auto v : static_intervals_detect_result) {
        of_result << v << std::endl;
    }
    of_result.close();

    Eigen::Vector3d ba(0, 0, 0);
    double g_norm = 9.8;
    ceres::Problem problem;
    for (auto &it : static_samples) {
        Eigen::Vector3d meas = it.data();
        //AccMeasureFactor *factor = new AccMeasureFactor(meas);
        AccMeasureFactorSquared * factor = new AccMeasureFactorSquared(meas);
        problem.AddResidualBlock(factor, nullptr, ba.data(), &g_norm);
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    cout << summary.FullReport() << std::endl;
    cout << " acc offset " << ba.transpose() << " g_norm " << g_norm << endl << endl;

    extractIntervalsSamples(gyro_data, static_intervals,
                            static_samples, extracted_intervals,
                            400, true);
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (int i = 0; i < static_samples.size(); i++) {
        cout << "gyr sample " << static_samples[i].data().transpose() << endl;
        sum += static_samples[i].data();
    }
    sum = sum / static_samples.size();
    cout << "\n gyr offset " << sum.transpose() << endl;
}
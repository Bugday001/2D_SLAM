#include "slam2d.h"


slam2d::slam2d(int width, int height, double resolution)
{   
    curr_scan_.reset(new CloudType());
    prev_scan_.reset(new CloudType());
    version_ = "0.1.1";
    //end temp
    width_ = width;
    height_ = height;
    resolution_ = resolution;
    state.t = Vector2d::Zero();
    state.theta = 0;
    map2d_vec.reset(new Map2d(width_, height_, resolution_));
    firstTimeStamp_ = -1;

    char CPUBrandString[0x40];
    memset(CPUBrandString, 0, sizeof(CPUBrandString));
    cpu_type_ = "";
    #ifdef HAS_CPUID
    unsigned int CPUInfo[4] = {0,0,0,0};
    __cpuid(0x80000000, CPUInfo[0], CPUInfo[1], CPUInfo[2], CPUInfo[3]);
    unsigned int nExIds = CPUInfo[0];
    for (unsigned int i = 0x80000000; i <= nExIds; ++i) {
        __cpuid(i, CPUInfo[0], CPUInfo[1], CPUInfo[2], CPUInfo[3]);
        if (i == 0x80000002)
        memcpy(CPUBrandString, CPUInfo, sizeof(CPUInfo));
        else if (i == 0x80000003)
        memcpy(CPUBrandString + 16, CPUInfo, sizeof(CPUInfo));
        else if (i == 0x80000004)
        memcpy(CPUBrandString + 32, CPUInfo, sizeof(CPUInfo));
    }
    cpu_type_ = CPUBrandString;
    boost::trim(cpu_type_);
    #endif

    FILE* file;
    struct tms timeSample;
    char line[128];

    this->lastCPU = times(&timeSample);
    this->lastSysCPU = timeSample.tms_stime;
    this->lastUserCPU = timeSample.tms_utime;

    file = fopen("/proc/cpuinfo", "r");
    this->numProcessors = 0;
    while(fgets(line, 128, file) != nullptr) {
        if (strncmp(line, "processor", 9) == 0) this->numProcessors++;
    }
    fclose(file);
}

slam2d::~slam2d() {
}


void slam2d::scan_match() {
    double pose[3] = {0};
    if (curr_scan_->size() && prev_scan_->size())
    {
        pcl::PointCloud<PointType> scan, scan_prev;
        vector2pcl(curr_scan_, scan);
        vector2pcl(prev_scan_, scan_prev);
        //solve delta with ceres constraints
        pcl::KdTreeFLANN<PointType> kdtree;
        kdtree.setInputCloud(scan.makeShared());
        int K = 5; // K nearest neighbor search
        std::vector<int> index(K);
        std::vector<float> distance(K);
        Eigen::VectorXd B(K);
        B.setOnes();
        //1. project scan_prev to scan
        for(int iter=0; iter<2; iter++) {
            Problem problem;
            pcl::PointCloud<PointType> scan_predict;
            Eigen::Matrix4f T = delta.toMatrix4f();
            pcl::transformPointCloud(scan_prev, scan_predict, T);
            //find nearest neighur
            for (size_t i = 0; i < scan_predict.points.size(); i++)
            {
                PointType search_point = scan_predict.points[i];
                //project search_point to current frame
                if (kdtree.nearestKSearch(search_point, K, index, distance) == K)
                {
                    //add constraints
                    Eigen::Vector2d p = point2eigen(search_point);
                    Eigen::MatrixXd A(K, 2);
                    for(int i=0; i<K; i++) 
                        A.row(i) = curr_scan_->points[index[i]];
                    Eigen::VectorXd res = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);
                    ceres::CostFunction *cost_function = lidar_edge_error2::Create(p, res);
                    problem.AddResidualBlock(cost_function,
                                            new CauchyLoss(0.5),
                                            pose);
                }
            }

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_SCHUR;
            options.minimizer_progress_to_stdout = false;

            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            // std::cout << summary.FullReport() << "\n";
            // printf("result: %lf, %lf, %lf\n", pose[0], pose[1], pose[2]);

            delta.theta += pose[0];
            delta.t(0) += pose[1];
            delta.t(1) += pose[2];
        }
    }
}

void slam2d::s2sGaussNewton() {
    if (curr_scan_->size() && prev_scan_->size())
    {
        pcl::PointCloud<PointType> scan, scan_prev;
        vector2pcl(curr_scan_, scan);
        vector2pcl(prev_scan_, scan_prev);
        //solve delta with ceres constraints
        pcl::KdTreeFLANN<PointType> kdtree;
        kdtree.setInputCloud(scan.makeShared());
        int K = 5; // K nearest neighbor search
        std::vector<int> index(K);
        std::vector<float> distance(K);
        double cost = 0, lastCost = 0;
        delta.setZero();
        Eigen::VectorXd B(K);
        B.setOnes();
        for(int iter=0; iter<50; iter++) {
            Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
            Eigen::Vector3d b = Eigen::Vector3d::Zero();
            cost = 0;
            int effective_num = 0;
            pcl::PointCloud<PointType> scan_predict;
            Eigen::Matrix4f T = delta.toMatrix4f();
            pcl::transformPointCloud(scan_prev, scan_predict, T);
            //find nearest neighur
            for (size_t i = 0; i < scan_predict.points.size(); i++) {
                PointType search_point = scan_predict.points[i];
                //project search_point to current frame
                if (kdtree.nearestKSearch(search_point, K, index, distance) == K) {
                    effective_num++;
                    //add constraints
                    Eigen::Vector2d p = point2eigen(search_point);
                               
                    double angle = atan(scan_prev.points[i].y/(scan_prev.points[i].x+1e-10));
                    double r = sqrt(p(1)*p(1)+p(0)*p(0));
                    Eigen::MatrixXd A(K, 2);
                    for(int i=0; i<K; i++) 
                        A.row(i) = curr_scan_->points[index[i]];
                    Eigen::Vector2d res = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);
                    //point2line
                    Eigen::Matrix<double,3,1> J;
                    J << res(0), res(1), 
                        -res(0) * r * std::sin(angle + delta.theta) + res(1) * r * std::cos(angle + delta.theta);
                    double e = (res(0) * p(0) + res(1) * p(1) - 1)/res.norm();
                    H += J * J.transpose();
                    b += -J * e;
                    cost += e*e;
                //     Eigen::Matrix<double,3,2> J;
                //     J <<    1,                            0, 
                //             0,                            1, 
                //             -r * std::sin(angle + delta.theta)*5, r * std::cos(angle + delta.theta)*5;
                //     H += J * J.transpose();
                //     Eigen::Vector2d e(p(0) - curr_scan_->points[index[0]](0), p(1) - curr_scan_->points[index[0]](1));
                //     b += -J * e;
                //     cost += e.dot(e);
                }
            }
            // solve for dx
            Eigen::Vector3d dx = H.ldlt().solve(b);
            if (isnan(dx(0))) {
                std::cout<<"error"<<std::endl;
                break;
            }

            cost /= effective_num;
            if (iter > 0 && cost >= lastCost) {
                break;
            }
            delta.theta += dx(2)*0.1;
            delta.t(0) += dx(0)*0.1;
            delta.t(1) += dx(1)*0.1;
            lastCost = cost;
        }
    }
}

/**
* 插值帧图匹配
*/
void slam2d::scan_map_interpolation() {
    double pose[3] = {0};
    CloudType::Ptr scan_w = std::make_shared<CloudType>(curr_scan_->size());
    transfromCloudVec(curr_scan_, scan_w, state.toMatrix3d());
    Problem problem;
    ceres::CostFunction *cost_function = new OccupiedError(map2d_vec, scan_w);
    problem.AddResidualBlock(cost_function, new CauchyLoss(0.5), pose);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // state.theta += pose[0];
    state.t(0) += pose[1];
    state.t(1) += pose[2];
}

/**
* 被视作是一种不精确的方法
* 未使用
*/
void slam2d::scan_map_violent() {
    //xy方向
    int lenSearchxy = 1;
    std::vector<double> x_drift, y_drift, theta_drift;
    double searchResolutionxy = 2 * resolution_;
    int numsSearchxy = round(lenSearchxy * 2 / searchResolutionxy) + 1;
    x_drift.resize(numsSearchxy);   y_drift.resize(numsSearchxy);
    for(int i=0; i<numsSearchxy; i++) {
        x_drift[i] = -lenSearchxy + i * searchResolutionxy;
        y_drift[i] = -lenSearchxy + i * searchResolutionxy;
    }
    //角度
    double lenSearchTheta = 10.0 / 180.0 * M_PI;
    double searchResolutionTheta = 2.0 / 180.0 * M_PI;
    int numsSearchTheta = round(lenSearchTheta * 2 / searchResolutionTheta) + 1;
    theta_drift.resize(numsSearchTheta);
    for(int i=0; i<numsSearchTheta; i++) {
        theta_drift[i] = -lenSearchTheta + i * searchResolutionTheta;
    }
    //3层循环
    Eigen::Vector3d bestPose(state.theta, state.t(0), state.t(1));
    int score = map2d_vec->scan_map_match_score(curr_scan_, bestPose);
    mapScore_ = (double)score/(double)curr_scan_->size();
    Eigen::Vector3d pose(state.theta, state.t(0), state.t(1));
    //x
    for(int i=0; i<numsSearchxy; i++) {
        double& dx = x_drift[i];
        //y
        for(int j=0; j<numsSearchxy; j++) {
            double& dy = y_drift[j];
            //theta
            for(int k=0; k<numsSearchTheta; k++) {
                double& dTheta = theta_drift[k];
                const Eigen::Vector3d dtxy(dTheta, dx, dy);
                int curr_score = map2d_vec->scan_map_match_score(curr_scan_, pose + dtxy);
                double curr_pScore = (double)curr_score/(double)curr_scan_->size();
                if(mapScore_ < curr_pScore) {
                    bestPose = pose + dtxy;
                    mapScore_ = curr_pScore;
                }
                if(mapScore_ > 0.95) break;
            }
            if(mapScore_ > 0.95) break;
        }
        if(mapScore_ > 0.95) break;
    }
    state.theta = bestPose(0);
    state.t = bestPose.bottomRows(2);
}

/**
* 被视作是一种随机的方法
* 效果好过暴力法
* 未使用
*/
void slam2d::scan_map_match_random() {
    Vector3d pose(state.theta, state.t(0), state.t(1));
    double eps = 1e-5;
    //search best mattch
    int N = 100;
    for (int i = 0; i < N; i++) {
        //random direction
        Vector3d d = Vector3d::Random();
        d(0) /= 10.0;
        d.normalize();
        double min_len = 0;
        double max_len = 0.2;
        //search best len
        while((max_len - min_len) > eps) {
            int score1 = map2d_vec->scan_map_match_score(curr_scan_, pose + d * min_len);
            int score2 = map2d_vec->scan_map_match_score(curr_scan_, pose + d * max_len);
            if (score1 >= score2) {
                max_len = (min_len + max_len) / 2.0;
            } 
            else {
                min_len = (min_len + max_len) / 2.0;
            }
        }
        pose += d * min_len;
        // Vector3d dx = d * min_len;
        int score = map2d_vec->scan_map_match_score(curr_scan_, pose);
        mapScore_ = (double)score/(double)curr_scan_->size();
        if(mapScore_>0.95) break;
    }
    //update to state
    state.theta = pose(0);
    state.t = pose.bottomRows(2);
}

void slam2d::update_transform()
{

    Eigen::Matrix2d dR;
    dR(0, 0) = cos(delta.theta); dR(0, 1) = -sin(delta.theta);
    dR(1, 0) = sin(delta.theta); dR(1, 1) = cos(delta.theta);


    Eigen::Vector2d dt_inv = -dR.transpose() * delta.t;
    // Eigen::Matrix2d dR_inv = dR.transpose();

    Eigen::Matrix2d R;
    R(0, 0) = cos(state.theta); R(0, 1) = -sin(state.theta);
    R(1, 0) = sin(state.theta); R(1, 1) =  cos(state.theta);
    state.theta += (-delta.theta);
    state.t += R * dt_inv;
}

void slam2d::update_map() {
    Eigen::Vector2i origin = map2d_vec->world2map_i(state.t);
    CloudType::Ptr scan_w = std::make_shared<CloudType>(curr_scan_->size());
    transfromCloudVec(curr_scan_, scan_w, state.toMatrix3d());
    map2d_vec->update_map(origin, scan_w);
}

/**
* 主函数
*/
void slam2d::update()
{
    if (curr_scan_->size() && prev_scan_->size())
    {
        static clock_t startT, endT;
        startT = clock();
        scan_match();
        // s2sGaussNewton();
        endT = clock();
        timeLog_.s2sMatchT = (double) (endT - startT) / CLOCKS_PER_SEC;
        update_transform();
        startT = clock();
        // scan_map_match_random();
        // scan_map_violent();
        scan_map_interpolation();
        endT = clock();
        timeLog_.s2mMatchT = (double) (endT - startT) / CLOCKS_PER_SEC;
        startT = clock();
        update_map();
        endT = clock();
        timeLog_.updateMapT = (double) (endT - startT) / CLOCKS_PER_SEC;
    }
    if (curr_scan_->size()) {
        prev_scan_ = curr_scan_;
        
    }
    debug_thread_ = std::thread(&slam2d::debug, this);
    debug_thread_.detach();
}

/**
* 更新当前点云
*/
void slam2d::setCurrentCloud(CloudType cloud) {
    curr_scan_.reset(new CloudType(cloud));
}

/**
* 终端打印信息
*/
void slam2d::debug() {
    if(firstTimeStamp_==-1) firstTimeStamp_ = timestamp_;
    //Odom Info
    elapsed_time_ = timestamp_ - firstTimeStamp_;

    // RAM Usage
    // double vm_usage = 0.0;
    double resident_set = 0.0;
    std::ifstream stat_stream("/proc/self/stat", std::ios_base::in); //get info from proc directory
    std::string pid, comm, state, ppid, pgrp, session, tty_nr;
    std::string tpgid, flags, minflt, cminflt, majflt, cmajflt;
    std::string utime, stime, cutime, cstime, priority, nice;
    std::string num_threads, itrealvalue, starttime;
    unsigned long vsize;
    long rss;
    stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr
                >> tpgid >> flags >> minflt >> cminflt >> majflt >> cmajflt
                >> utime >> stime >> cutime >> cstime >> priority >> nice
                >> num_threads >> itrealvalue >> starttime >> vsize >> rss; // don't care about the rest
    stat_stream.close();
    long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024; // for x86-64 is configured to use 2MB pages
    // vm_usage = vsize / 1024.0;
    resident_set = rss * page_size_kb;

    // CPU Usage
    struct tms timeSample;
    clock_t now;
    double cpu_percent;
    now = times(&timeSample);
    if (now <= lastCPU || timeSample.tms_stime < lastSysCPU ||
        timeSample.tms_utime < lastUserCPU) {
        cpu_percent = -1.0;
    } else {
        cpu_percent = (timeSample.tms_stime - lastSysCPU) + (timeSample.tms_utime - lastUserCPU);
        cpu_percent /= (now - lastCPU);
        cpu_percent /= numProcessors;
        cpu_percent *= 100.;
    }
    lastCPU = now;
    lastSysCPU = timeSample.tms_stime;
    lastUserCPU = timeSample.tms_utime;
    cpu_percents.push_back(cpu_percent);
    double avg_cpu_usage =
        std::accumulate(cpu_percents.begin(), cpu_percents.end(), 0.0) / cpu_percents.size();

    // Print to terminal
    printf("\033[2J\033[1;1H");

    std::cout << std::endl
                << "+-------------------------------------------------------------------+" << std::endl;
    std::cout << "|                    2D LiDAR Odometry v" << version_  << "                       |"
                << std::endl;
    std::cout << "+-------------------------------------------------------------------+" << std::endl;

    std::time_t curr_time = timestamp_;
    std::string asc_time = std::asctime(std::localtime(&curr_time)); asc_time.pop_back();
    std::cout << "| " << std::left << asc_time;
    std::cout << std::right << std::setfill(' ') << std::setw(42)
        << "Elapsed Time: " + to_string_with_precision(elapsed_time_, 2) + " seconds "
        << "|" << std::endl;

    if ( !cpu_type_.empty() ) {
        std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
        << cpu_type_ + " x " + std::to_string(numProcessors)
        << "|" << std::endl;
    }

    std::cout << "|===================================================================|" << std::endl;
    double totalT = timeLog_.s2mMatchT + timeLog_.s2sMatchT + timeLog_.updateMapT;
    std::cout << std::right << std::setprecision(2) << std::fixed;
    std::cout << "| Computation Time :: "
        << std::setfill(' ') << std::setw(6) << totalT*1000. << " ms    // Score: "
        << std::setw(6) << mapScore_*100.
        << "                 |" << std::endl;
    std::cout << "| Detail Time   S2S::" << " "
        << std::setfill(' ') << std::setw(6) << timeLog_.s2sMatchT*1000. << " ms    // S2M: "
        << std::setw(5) << timeLog_.s2mMatchT*1000. << " ms/ Map: "
        << std::setw(5) << timeLog_.updateMapT*1000.
        << " ms  |" << std::endl;
    std::cout << "| Cores Utilized   :: "
        << std::setfill(' ') << std::setw(6) << (cpu_percent/100.) * numProcessors << " cores // Avg: "
        << std::setw(6) << (avg_cpu_usage/100.) * numProcessors << " / Max: "
        << std::setw(6) << (*std::max_element(cpu_percents.begin(), cpu_percents.end()) / 100.)
                        * numProcessors
        << "     |" << std::endl;
    std::cout << "| CPU Load         :: "
        << std::setfill(' ') << std::setw(6) << cpu_percent << " %     // Avg: "
        << std::setw(6) << avg_cpu_usage << " / Max: "
        << std::setw(6) << *std::max_element(cpu_percents.begin(), cpu_percents.end())
        << "     |" << std::endl;
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
        << "RAM Allocation   :: " + to_string_with_precision(resident_set/1000., 2) + " MB"
        << "|" << std::endl;

    std::cout << "+-------------------------------------------------------------------+" << std::endl;

}
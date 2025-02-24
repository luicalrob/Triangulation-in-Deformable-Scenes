#include "Utils/Geometry.h"
#include "Utils/Measurements.h"

#include <random> 

using namespace std;

void measureSimAbsoluteMapErrors(const std::shared_ptr<Map> pMap, 
                                const std::vector<Eigen::Vector3f> originalPoints, const std::vector<Eigen::Vector3f> movedPoints, 
                                const std::string filePath) {
    std::ofstream outFile;
    outFile.imbue(std::locale("es_ES.UTF-8"));

    // 3D Error measurement in map points
    std::unordered_map<ID, std::shared_ptr<MapPoint>> mapPoints_corrected = pMap->getMapPoints();

    float total_movement = 0.0f;
    float total_error_original = 0.0f;
    float total_error_moved = 0.0f;
    float total_error = 0.0f;
    float total_squared_error_original = 0.0f;
    float total_squared_error_moved = 0.0f;
    float total_squared_error = 0.0f;
    int point_count = mapPoints_corrected.size();
    int point_count_in_kf = mapPoints_corrected.size() / 2.0;
    
    for(size_t i = 0, j = 0; j < mapPoints_corrected.size()/2; i += 2, j++) {
        std::shared_ptr<MapPoint> mapPoint1 = pMap->getMapPoint(i);
        std::shared_ptr<MapPoint> mapPoint2 = pMap->getMapPoint(i+1);
        Eigen::Vector3f opt_original_position = mapPoint1->getWorldPosition();
        Eigen::Vector3f opt_moved_position = mapPoint2->getWorldPosition();

        Eigen::Vector3f original_position = originalPoints[j];
        Eigen::Vector3f moved_position = movedPoints[j];

        Eigen::Vector3f movement = original_position - moved_position;
        Eigen::Vector3f original_error = opt_original_position - original_position;
        Eigen::Vector3f moved_error = opt_moved_position - moved_position;
        float error_magnitude_movement = movement.norm();
        float error_magnitude_original = original_error.norm();
        float error_magnitude_moved = moved_error.norm();

        float squared_error_magnitude_original = original_error.squaredNorm();
        float squared_error_magnitude_moved = moved_error.squaredNorm();
        
        total_movement += error_magnitude_movement;
        total_error_original += error_magnitude_original;
        total_error_moved += error_magnitude_moved;
        total_error += error_magnitude_moved + error_magnitude_original;
        
        total_squared_error_original += squared_error_magnitude_original;
        total_squared_error_moved += squared_error_magnitude_moved;
        total_squared_error += squared_error_magnitude_original + squared_error_magnitude_moved;

        // std::cout << "\nError for point: " << mapPoint1->getId() << " and " << mapPoint2->getId() << "\n";
        // std::cout << "Position " << mapPoints_corrected[j] << "\n";
        // std::cout << "Mappoint x: " << opt_original_position.x() << " y: " << opt_original_position.y() << " z: " << opt_original_position.z() << std::endl;
        // std::cout << "point x: " << original_position.x() << " y: " << original_position.y() << " z: " << original_position.z() << std::endl;
        // std::cout << "moved Mappoint x: " << opt_moved_position.x() << " y: " << opt_moved_position.y() << " z: " << opt_moved_position.z() << std::endl;
        // std::cout << "moved point x: " << moved_position.x() << " y: " << moved_position.y() << " z: " << moved_position.z() << std::endl;
        // std::cout << "x: " << total_error << std::endl;
        //point_count++;
    }

    // std::cout << "point_count: " << point_count << std::endl;
    if (point_count > 0) {
        std::cout << "\nABSOLUTE MEASUREMENTS: \n";

        float average_movement = total_movement / point_count_in_kf;
        //std::cout << "\nTotal movement: " << total_movement << std::endl;
        std::cout << "Average movement: " << average_movement * 1000 << std::endl;
        float average_error_original = total_error_original / point_count_in_kf;
        //std::cout << "\nTotal error in ORIGINAL 3D: " << total_error_original << std::endl;
        //std::cout << "Average error in ORIGINAL 3D: " << average_error_original * 1000 << std::endl;
        float average_error_moved = total_error_moved / point_count_in_kf;
        //std::cout << "\nTotal error in MOVED 3D: " << total_error_moved << std::endl;
        //std::cout << "Average error in MOVED 3D: " << average_error_moved * 1000 << std::endl;
        float average_error = total_error / point_count;
        //std::cout << "\nTotal error in 3D: " << total_error << std::endl;
        float rmse = std::sqrt(total_squared_error / point_count);
        std::cout << "Average error in 3D: " << average_error * 1000 << std::endl;
        std::cout << "RMSE in 3D: " << rmse * 1000 << "\n" << std::endl;
        
        outFile.open(filePath, std::ios::app);
        if (outFile.is_open()) {
            outFile << "Av. movement: " << average_movement * 1000 << '\n';
            outFile << "Av. error: " << average_error * 1000 << '\n';
            outFile << "RMSE: " << rmse * 1000 << "\n\n";

            outFile.close();
            std::cout << "Data has been written to Experiment.txt" << std::endl;
        } else {
            std::cerr << "Unable to open file for writing" << std::endl;
        }
    } else {
        std::cout << "No points to compare." << std::endl;
    } 
}


void measureRealAbsoluteMapErrors(const std::shared_ptr<Map> pMap, const std::string filePath) {
    std::ofstream outFile;
    outFile.imbue(std::locale("es_ES.UTF-8"));

    // 3D Error measurement in map points
    std::unordered_map<ID, std::shared_ptr<MapPoint>> mapPoints_corrected = pMap->getMapPoints();

    float total_movement = 0.0f;
    float total_error_original = 0.0f;
    float total_error_moved = 0.0f;
    float total_error = 0.0f;
    float total_squared_error_original = 0.0f;
    float total_squared_error_moved = 0.0f;
    float total_squared_error = 0.0f;

    float total_upToScale_error_original = 0.0f;
    float total_upToScale_error_moved = 0.0f;
    float total_upToScale_error = 0.0f;
    float total_upToScale_squared_error_original = 0.0f;
    float total_upToScale_squared_error_moved = 0.0f;
    float total_upToScale_squared_error = 0.0f;

    int point_count = mapPoints_corrected.size();
    int point_count_in_kf = mapPoints_corrected.size() / 2.0;
    int n_points = 0;
    float scale1 = 0, scale2 = 0;
    
    std::unordered_map<ID,KeyFrame_>&  mKeyFrames = pMap->getKeyFrames();

    for (auto k1 = mKeyFrames.begin(); k1 != mKeyFrames.end(); ++k1) {
        for (auto k2 = std::next(k1); k2 != mKeyFrames.end(); ++k2) {
            std::shared_ptr<KeyFrame> pKF1 = k2->second;
            std::shared_ptr<KeyFrame> pKF2 = k1->second;
            int kf1ID = k2->first;
            int kf2ID = k1->first;
            std::cout << "Pair: (" << k1->first << ", " << k2->first<< ")\n";

            vector<std::shared_ptr<MapPoint>>& v1MPs = pKF1->getMapPoints();
            vector<std::shared_ptr<MapPoint>>& v2MPs = pKF2->getMapPoints();
            
            std::shared_ptr<CameraModel> pCamera1 = pKF1->getCalibration();
            g2o::SE3Quat camera1Pose = g2o::SE3Quat(pKF1->getPose().unit_quaternion().cast<double>(),pKF1->getPose().translation().cast<double>());
            std::shared_ptr<CameraModel> pCamera2 = pKF2->getCalibration();
            g2o::SE3Quat camera2Pose = g2o::SE3Quat(pKF2->getPose().unit_quaternion().cast<double>(),pKF2->getPose().translation().cast<double>());
            Sophus::SE3f T1w = pKF1->getPose();
            Sophus::SE3f T2w = pKF2->getPose();

            for (size_t i = 0; i < v1MPs.size(); i++) {
                std::shared_ptr<MapPoint> pMPi1, pMPi2;
                pMPi1 = v1MPs[i];
                pMPi2 = v2MPs[i];
                if (!pMPi1) continue;
                if (!pMPi2) continue;

                Eigen::Vector3f opt_original_position = pMPi1->getWorldPosition();
                Eigen::Vector3f opt_moved_position = pMPi2->getWorldPosition();

                // Eigen::Vector3f p3Dc1 = camera1Pose.map(opt_original_position);
                // Eigen::Vector3f p3Dc2 = camera2Pose.map(opt_moved_position);
                // Eigen::Vector3f p3Dc1 = T1w * opt_original_position;
                // Eigen::Vector3f p3Dc2 = T2w * opt_moved_position;

                int index_in_kf1 = pMap->isMapPointInKeyFrame(pMPi1->getId(), kf1ID);
                int index_in_kf2 = pMap->isMapPointInKeyFrame(pMPi2->getId(), kf2ID);

                if(index_in_kf1 < 0 || index_in_kf2 < 0) continue;
                size_t idx1 = (size_t)index_in_kf1;
                size_t idx2 = (size_t)index_in_kf2;

                cv::Point2f x1 = pKF1->getKeyPoint(idx1).pt;
                cv::Point2f x2 = pKF2->getKeyPoint(idx2).pt;

                // Eigen::Vector2f p_p1;
                // Eigen::Vector2f p_p2;
                // pCamera1->project(p3Dc1, p_p1);
                // pCamera2->project(p3Dc2, p_p2);

                double d1 = pKF1->getDepthMeasure(x1.x, x1.y);
                double d2 = pKF2->getDepthMeasure(x2.x, x2.y);
                // double d1 = pKF1->getDepthMeasure(p_p1[0], p_p1[1]);
                // double d2 = pKF2->getDepthMeasure(p_p2[0], p_p2[1]);

                // Eigen::Vector3f original_position_c(p3Dc1[0], p3Dc1[1], d1);
                // Eigen::Vector3f moved_position_c(p3Dc2[0], p3Dc2[1], d2);
                
                // cv::Point2f p_p1_cv(p_p1.x(), p_p1.y());
                // cv::Point2f p_p2_cv(p_p2.x(), p_p2.y());

                // Eigen::Matrix<float,1,3> x3D1 = pCamera1->unproject(x1, d1);
                // Eigen::Matrix<float,1,3> x3D2 = pCamera2->unproject(x2, d2);

                Eigen::Vector3f m_pos_c1_aux = pCamera1->unproject(x1);
                m_pos_c1_aux /= m_pos_c1_aux.z();
                Eigen::Matrix<float,1,3> x3D1 = m_pos_c1_aux * d1;

                Eigen::Vector3f m_pos_c2_aux = pCamera2->unproject(x2);
                m_pos_c2_aux /= m_pos_c2_aux.z();
                Eigen::Matrix<float,1,3> x3D2 = m_pos_c2_aux * d2;

                Eigen::Matrix<float, 1, 4> x3D1_h;
                Eigen::Matrix<float, 1, 4> x3D2_h;
                x3D1_h << x3D1[0], x3D1[1], x3D1[2], 1;
                x3D2_h << x3D2[0], x3D2[1], x3D2[2], 1; 
                Eigen::Vector4f original_position_h = T1w.inverse().matrix() * x3D1_h.transpose();
                Eigen::Vector4f moved_position_h = T2w.inverse().matrix() * x3D2_h.transpose();

                Eigen::Vector3f original_position;
                Eigen::Vector3f moved_position;
                original_position << original_position_h[0], original_position_h[1], original_position_h[2];
                moved_position << moved_position_h[0], moved_position_h[1], moved_position_h[2];
                
                Eigen::Vector3f mp_c1 = T1w * opt_original_position;
                Eigen::Vector3f mp_c2 = T2w * opt_moved_position;
                scale1 += x3D1[2] / mp_c1[2];
                scale2 += x3D2[2] / mp_c2[2];

                Eigen::Vector3f movement = original_position - moved_position;
                Eigen::Vector3f original_error = opt_original_position - original_position;
                Eigen::Vector3f moved_error = opt_moved_position - moved_position;
                float error_magnitude_movement = movement.norm();
                float error_magnitude_original = original_error.norm();
                float error_magnitude_moved = moved_error.norm();

                float squared_error_magnitude_original = original_error.squaredNorm();
                float squared_error_magnitude_moved = moved_error.squaredNorm();
                
                total_movement += error_magnitude_movement;
                total_error_original += error_magnitude_original;
                total_error_moved += error_magnitude_moved;
                total_error += error_magnitude_moved + error_magnitude_original;
                
                total_squared_error_original += squared_error_magnitude_original;
                total_squared_error_moved += squared_error_magnitude_moved;
                total_squared_error += squared_error_magnitude_original + squared_error_magnitude_moved;

                // std::cout << "\nError for point: " << mapPoint1->getId() << " and " << mapPoint2->getId() << "\n";
                // std::cout << "Position " << mapPoints_corrected[j] << "\n";
                // std::cout << "Mappoint x: " << opt_original_position.x() << " y: " << opt_original_position.y() << " z: " << opt_original_position.z() << std::endl;
                // std::cout << "point x: " << original_position.x() << " y: " << original_position.y() << " z: " << original_position.z() << std::endl;
                // std::cout << "moved Mappoint x: " << opt_moved_position.x() << " y: " << opt_moved_position.y() << " z: " << opt_moved_position.z() << std::endl;
                // std::cout << "moved point x: " << moved_position.x() << " y: " << moved_position.y() << " z: " << moved_position.z() << std::endl;
                // std::cout << "x: " << total_error << std::endl;
                //point_count++;
                n_points++;
            }
            scale1 = scale1 / n_points;
            scale2 = scale2 / n_points;

            for (size_t i = 0; i < v1MPs.size(); i++) {
                std::shared_ptr<MapPoint> pMPi1, pMPi2;
                pMPi1 = v1MPs[i];
                pMPi2 = v2MPs[i];
                if (!pMPi1) continue;
                if (!pMPi2) continue;

                Eigen::Vector3f opt_original_position = pMPi1->getWorldPosition();
                Eigen::Vector3f opt_moved_position = pMPi2->getWorldPosition();

                int index_in_kf1 = pMap->isMapPointInKeyFrame(pMPi1->getId(), kf1ID);
                int index_in_kf2 = pMap->isMapPointInKeyFrame(pMPi2->getId(), kf2ID);

                if(index_in_kf1 < 0 || index_in_kf2 < 0) continue;
                size_t idx1 = (size_t)index_in_kf1;
                size_t idx2 = (size_t)index_in_kf2;

                cv::Point2f x1 = pKF1->getKeyPoint(idx1).pt;
                cv::Point2f x2 = pKF2->getKeyPoint(idx2).pt;

                double d1 = pKF1->getDepthMeasure(x1.x, x1.y);
                double d2 = pKF2->getDepthMeasure(x2.x, x2.y);

                Eigen::Vector3f m_pos_c1_aux = pCamera1->unproject(x1);
                m_pos_c1_aux /= m_pos_c1_aux.z();
                Eigen::Matrix<float,1,3> x3D1 = m_pos_c1_aux * d1 / scale1;

                Eigen::Vector3f m_pos_c2_aux = pCamera2->unproject(x2);
                m_pos_c2_aux /= m_pos_c2_aux.z();
                Eigen::Matrix<float,1,3> x3D2 = m_pos_c2_aux * d2 / scale2;

                Eigen::Matrix<float, 1, 4> x3D1_h;
                Eigen::Matrix<float, 1, 4> x3D2_h;
                x3D1_h << x3D1[0], x3D1[1], x3D1[2], 1;
                x3D2_h << x3D2[0], x3D2[1], x3D2[2], 1; 
                Eigen::Vector4f original_position_h = T1w.inverse().matrix() * x3D1_h.transpose();
                Eigen::Vector4f moved_position_h = T2w.inverse().matrix() * x3D2_h.transpose();

                Eigen::Vector3f original_position;
                Eigen::Vector3f moved_position;
                original_position << original_position_h[0], original_position_h[1], original_position_h[2];
                moved_position << moved_position_h[0], moved_position_h[1], moved_position_h[2];

                Eigen::Vector3f movement = original_position - moved_position;
                Eigen::Vector3f original_error = opt_original_position - original_position;
                Eigen::Vector3f moved_error = opt_moved_position - moved_position;
                float error_magnitude_original = original_error.norm();
                float error_magnitude_moved = moved_error.norm();

                float squared_error_magnitude_original = original_error.squaredNorm();
                float squared_error_magnitude_moved = moved_error.squaredNorm();
                
                total_upToScale_error_original += error_magnitude_original;
                total_upToScale_error_moved += error_magnitude_moved;
                total_upToScale_error += error_magnitude_moved + error_magnitude_original;
                
                total_upToScale_squared_error_original += squared_error_magnitude_original;
                total_upToScale_squared_error_moved += squared_error_magnitude_moved;
                total_upToScale_squared_error += squared_error_magnitude_original + squared_error_magnitude_moved;
            }
        }
    }

    // std::cout << "point_count: " << point_count << std::endl;
    if (point_count > 0) {
        std::cout << "\nABSOLUTE MEASUREMENTS: \n";

        float average_movement = total_movement / point_count_in_kf;
        //std::cout << "\nTotal movement: " << total_movement << std::endl;
        std::cout << "Average movement: " << average_movement * 1000 << std::endl;
        float average_error_original = total_error_original / point_count_in_kf;
        //std::cout << "Average error in ORIGINAL 3D: " << average_error_original * 1000 << std::endl;
        float average_error_moved = total_error_moved / point_count_in_kf;
        //std::cout << "Average error in MOVED 3D: " << average_error_moved * 1000 << std::endl;
        float average_error = total_error / point_count;
        //std::cout << "\nTotal error in 3D: " << total_error << std::endl;
        float rmse = std::sqrt(total_squared_error / point_count);
        std::cout << "Average error in 3D: " << average_error * 1000 << std::endl;
        std::cout << "RMSE in 3D: " << rmse * 1000 << std::endl;

        float average_upToScale_error = total_upToScale_error / point_count;
        std::cout << "Average up-to-scale error in 3D: " << average_upToScale_error * 1000 << "\n" << std::endl;
        
        outFile.open(filePath, std::ios::app);
        if (outFile.is_open()) {
            outFile << "Av. movement: " << average_movement * 1000 << '\n';
            outFile << "Av. error: " << average_error * 1000 << '\n';
            outFile << "RMSE: " << rmse * 1000 << '\n';
            outFile << "Av. up-to-scale error in 3D: " << average_upToScale_error * 1000 << "\n\n";

            outFile.close();
            std::cout << "Data has been written to Experiment.txt" << std::endl;
        } else {
            std::cerr << "Unable to open file for writing" << std::endl;
        }
    } else {
        std::cout << "No points to compare." << std::endl;
    } 
}

void measureRelativeMapErrors(std::shared_ptr<Map> pMap, std::string filePath){
    std::ofstream outFile;
    outFile.imbue(std::locale("es_ES.UTF-8"));
    
    std::vector<Eigen::Vector3d> relativeErrors;
    std::vector<double> squaredNormRelativeErrors;
    Eigen::Vector3d meanRelativeError = Eigen::Vector3d::Zero();
    double depthError = 0;
    double gloablTError = 0;
    double meanSquaredNormRelativeError = 0;
    
    size_t nMatches = 0;
    size_t validPairs = 0;

    std::unordered_map<ID,KeyFrame_>&  mKeyFrames = pMap->getKeyFrames();

    std::cout << "\nKEYFRAMES k AND k+1 MEASUREMENTS: \n";
    for (auto k1 = mKeyFrames.begin(); k1 != mKeyFrames.end(); ++k1) {
        for (auto k2 = std::next(k1); k2 != mKeyFrames.end(); ++k2) {
            KeyFrame_ pKF1 = k2->second;
            KeyFrame_ pKF2 = k1->second;
            int kf1ID = k2->first;
            int kf2ID = k1->first;
            std::cout << "Pair: (" << k1->first << ", " << k2->first<< ")\n";

            Eigen::Matrix3f Rs_global = Eigen::Matrix3f::Identity();
            Eigen::Vector3f Ts = Eigen::Vector3f::Zero();
            Sophus::SE3f transformation = pMap->getGlobalKeyFramesTransformation(k2->first, k1->first);

            Rs_global = transformation.so3().matrix();
            Ts = transformation.translation();

            std::cout << "Global rotation: " << Rs_global<< "\n";
            std::cout << "Global translation: " << Ts<< "\n";

            double s1 = pKF1->getEstimatedDepthScale();
            double s2 = pKF2->getEstimatedDepthScale();

            std::cout << "C1 depth scale: " << s1 << std::endl;
            std::cout << "C2 depth scale: " << s2 << std::endl;

            vector<MapPoint_>& v1MPs = pKF1->getMapPoints();
            vector<MapPoint_>& v2MPs = pKF2->getMapPoints();
            
            // MESH CREATION
            std::vector<Eigen::Vector3d> v1Positions = extractPositions(v1MPs);
            std::vector<Eigen::Vector3d> v2Positions = extractPositions(v2MPs);

            std::shared_ptr<open3d::geometry::TriangleMesh> mesh;

            std::vector<Eigen::Matrix3d> Rs(v1Positions.size(), Eigen::Matrix3d::Identity());

            // Perform Delaunay Reconstruction
            mesh = ComputeDelaunayTriangulation3D(v1Positions);
            mesh->ComputeAdjacencyList();

            auto posIndexes = createVectorMap(mesh->vertices_, v1Positions);

            std::map<size_t, size_t> invertedPosIndexes;
            for (const auto& pair : posIndexes) {
                invertedPosIndexes[pair.second] = pair.first;
            } 

            Sophus::SE3f kfPose1 = pKF1->getPose();
            g2o::SE3Quat cameraPose1 = g2o::SE3Quat(kfPose1.unit_quaternion().cast<double>(),kfPose1.translation().cast<double>());
            Sophus::SE3f kfPose2 = pKF2->getPose();
            g2o::SE3Quat cameraPose2 = g2o::SE3Quat(kfPose2.unit_quaternion().cast<double>(),kfPose2.translation().cast<double>());

            for (size_t i = 0; i < v1MPs.size(); i++) {
                //Depth error 
                MapPoint_ pMPi1 = v1MPs[i];
                MapPoint_ pMPi2 = v2MPs[i];
                if (!pMPi1) continue;
                if (!pMPi2) continue;
                int index_in_kf1 = pMap->isMapPointInKeyFrame(pMPi1->getId(), kf1ID);
                int index_in_kf2 = pMap->isMapPointInKeyFrame(pMPi2->getId(), kf2ID);

                if(index_in_kf1 < 0 || index_in_kf2 < 0) continue;
                size_t idx1 = (size_t)index_in_kf1;
                size_t idx2 = (size_t)index_in_kf2;

                cv::Point2f uv1 = pKF1->getKeyPoint(idx1).pt;
                cv::Point2f uv2 = pKF1->getKeyPoint(idx2).pt;

                Eigen::Vector3d p3Dw1 = pMPi1->getWorldPosition().cast<double>();
                Eigen::Vector3d p3Dw2 = pMPi2->getWorldPosition().cast<double>();
                Eigen::Vector3d p3Dc1 = cameraPose1.map(p3Dw1);
                Eigen::Vector3d p3Dc2 = cameraPose2.map(p3Dw2);

                double d1 = pKF1->getDepthMeasure(uv1.x, uv1.y, false);
                double d2 = pKF2->getDepthMeasure(uv2.x, uv2.y, false);
                depthError += pow((d1 - p3Dc1[2] * static_cast<double>(s1)),2);
                depthError += pow((d2 - p3Dc2[2] * static_cast<double>(s2)),2);;

                //ARAP error
                auto it = invertedPosIndexes.find(i);
                size_t meshIndex = 0;
                if (it != invertedPosIndexes.end()) {
                    meshIndex = it->second;
                } else {
                    continue;
                }

                if (mesh->adjacency_list_[meshIndex].empty()) continue;

                std::unordered_set<int> jIndexes = mesh->adjacency_list_[meshIndex];

                for (int j : jIndexes) {
                    Eigen::Vector3d pi1, pj1, pi2, pj2;

                    pi1 = v1Positions[i];
                    pj1 = v1Positions[posIndexes[j]];
                    pi2 = v2Positions[i];
                    pj2 = v2Positions[posIndexes[j]];
                    if (!pi1.allFinite() || !pj1.allFinite() || !pi2.allFinite() || !pj2.allFinite()) continue;

                    Eigen::Vector3d d1 = pi1 - pj1;
                    Eigen::Vector3d d2 = pi2 - pj2;
                    Eigen::Vector3d diff = d2 - d1;
                    double squaredNorm = diff.squaredNorm();
                      
                    meanRelativeError += diff;
                    meanSquaredNormRelativeError += squaredNorm;
                    validPairs++;
                    
                    //Global error
                    gloablTError += (((Rs_global.cast<double>() * pi2 - Ts.cast<double>()) - pi1) + ((Rs_global.cast<double>() * pj2 - Ts.cast<double>()) - pj1)).squaredNorm();
                }

                nMatches++;
            }

            if (validPairs > 1) {
                PixelsError pixelsErrors;
                calculatePixelsStandDev(pMap, pixelsErrors);

                std::cout << "Pixels C1 error (average): " << pixelsErrors.avgc1 << std::endl;
                std::cout << "Pixels C1 error (standard desv): " << pixelsErrors.desvc1 << std::endl;
                std::cout << "Pixels C2 error (average): " << pixelsErrors.avgc2 << std::endl;
                std::cout << "Pixels C2 error (standard desv): " << pixelsErrors.desvc2 << std::endl;
                std::cout << "Pixels error (average): " << pixelsErrors.avg << std::endl;
                std::cout << "Pixels error (standard desv): " << pixelsErrors.desv << std::endl;
                
                std::cout << "Average squared norm relative error: " << meanSquaredNormRelativeError / mesh->GetSurfaceArea() << std::endl;
                std::cout << "depthError: " << depthError << std::endl;
                std::cout << "gloablTError: " << gloablTError  / mesh->GetSurfaceArea()<< std::endl;

                outFile.open(filePath, std::ios::app);
                if (outFile.is_open()) {
                    outFile << "C1 standard desv: " << pixelsErrors.desvc1 << '\n';
                    outFile << "C2 standard desv: " << pixelsErrors.desvc2 << '\n';
                    outFile << "Rel. error: " << meanSquaredNormRelativeError  / mesh->GetSurfaceArea() << '\n';
                    outFile << "depthError: " << depthError << '\n';
                    outFile << "gloablTError: " << gloablTError  / mesh->GetSurfaceArea()<< '\n';
                    outFile << "Global rotation: " << Rs_global << '\n';
                    outFile << "Global translation: " << Ts << '\n';

                    outFile.close();
                    std::cout << "Data has been written to Experiment.txt" << std::endl;
                } else {
                    std::cerr << "Unable to open file for writing" << std::endl;
                }

            } else {
                std::cout << "No points to compare." << std::endl;
            }
        }
    }
}
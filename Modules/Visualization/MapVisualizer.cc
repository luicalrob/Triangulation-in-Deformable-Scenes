/**
* This file is part of Mini-SLAM
*
* Copyright (C) 2021 Juan J. Gómez Rodríguez and Juan D. Tardós, University of Zaragoza.
*
* Mini-SLAM is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Mini-SLAM is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with Mini-SLAM.
* If not, see <http://www.gnu.org/licenses/>.
*/



#include "MapVisualizer.h"
#include "Utils/Geometry.h"
#include "Utils/Measurements.h"

#include <Eigen/Core>

using namespace std;

MapVisualizer::MapVisualizer(shared_ptr<Map> pMap, const PoseData initialPose, const bool showScene) : pMap_(pMap), showScene_(showScene){
    if (showScene_) {
        pangolin::CreateWindowAndBind("SLAM",1024,768);

        // 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);

        // Issue specific OpenGl we might need
        glEnable (GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        // Define Camera Render Object (for view / scene browsing)
        if(initialPose.isValid) {
            Eigen::Vector3f keyframe_position = Eigen::Vector3f(initialPose.tx, initialPose.ty, initialPose.tz);

            Eigen::Quaternionf quaternion(initialPose.qw, initialPose.qx, initialPose.qy, initialPose.qz);
            Eigen::Matrix3f rotation_matrix = quaternion.toRotationMatrix();
            Eigen::Vector3f camera_offset(0.4f, 0.0f, -10.0f);  // offset behind the keyframe
            Eigen::Vector3f camera_position = keyframe_position + rotation_matrix * camera_offset;
            s_cam = pangolin::OpenGlRenderState(
                    pangolin::ProjectionMatrix(1024,768,500,500,512,389,0.1,1000),
                    pangolin::ModelViewLookAt(
                        camera_position[0], camera_position[1], camera_position[2],
                        keyframe_position[0], keyframe_position[1], keyframe_position[2],
                        0.0, -1.0, 0.0));
        } else {
            s_cam = pangolin::OpenGlRenderState(
                    pangolin::ProjectionMatrix(1024,768,500,500,512,389,0.1,1000),
                    pangolin::ModelViewLookAt(0,-0.7,-1.8, 0,0,0,0.0,-1.0, 0.0));
        }

        // Add named OpenGL viewport to window and provide 3D Handler
        d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
                .SetHandler(new pangolin::Handler3D(s_cam));
    }

}

void MapVisualizer::update(bool drawRaysSelection) {
    if (showScene_) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Add named OpenGL viewport to window and provide 3D Handler
        d_cam.Activate(s_cam);

        glClearColor(1.0f,1.0f,1.0f,1.0f);

        drawMapPoints();
        drawKeyFrames();
        if (drawRaysSelection) drawRays();
        drawCurrentPose();

        pangolin::FinishFrame();
    }
}

void MapVisualizer::updateCurrentPose(Sophus::SE3f &currPose) {
    currPose_ = currPose;
}

void MapVisualizer::drawMapPoints() {
    std::unordered_map<ID,std::shared_ptr<KeyFrame>> mKeyFrames = pMap_->getKeyFrames();

    glPointSize(6);
    glBegin(GL_POINTS);
    //glColor3f(0.0,0.0,0.0);

    for (auto k1 = mKeyFrames.begin(); k1 != mKeyFrames.end(); ++k1) {
        for (auto k2 = std::next(k1); k2 != mKeyFrames.end(); ++k2) {
            std::shared_ptr<KeyFrame> pKF1 = k2->second;
            std::shared_ptr<KeyFrame> pKF2 = k1->second;
            int kf1ID = k2->first;
            int kf2ID = k1->first;

            vector<std::shared_ptr<MapPoint>>& v1MPs = pKF1->getMapPoints();
            vector<std::shared_ptr<MapPoint>>& v2MPs = pKF2->getMapPoints();

            std::shared_ptr<CameraModel> pCamera1 = pKF1->getCalibration();
            std::shared_ptr<CameraModel> pCamera2 = pKF2->getCalibration();
            Sophus::SE3f T1w = pKF1->getPose();
            Sophus::SE3f T2w = pKF2->getPose();
            double estimatedScale1 = pKF1->getEstimatedDepthScale();
            double estimatedScale2 = pKF2->getEstimatedDepthScale();
            // for (size_t col = 1; col <= 625; col++) {
            //     for (size_t row = 1; row <= 500; row++) {
            //         double d1 = pKF1->getDepthMeasure(col, row) / estimatedScale1;
            //         double d2 = pKF2->getDepthMeasure(col, row) / estimatedScale2;

            //         cv::Point2f x1(col, row);
            //         cv::Point2f x2(col, row);

            //         if(d1 != -1 || d1 < 0.2 || d2 != -1 || d2 < 0.2) {
            //             Eigen::Matrix<float,1,3> m_pos_c1 = pCamera1->unproject(x1, d1);
            //             Eigen::Matrix<float,1,3> m_pos_c2 = pCamera2->unproject(x2, d2);

            //             Eigen::Matrix<float,1,4> m_pos_c1_h, m_pos_c2_h;
            //             m_pos_c1_h << m_pos_c1[0], m_pos_c1[1], m_pos_c1[2], 1;
            //             m_pos_c2_h << m_pos_c2[0], m_pos_c2[1], m_pos_c2[2], 1;
                        
            //             Eigen::Vector4f m_pos_1_h = T1w.inverse().matrix() * m_pos_c1_h.transpose();
            //             Eigen::Vector4f m_pos_2_h = T2w.inverse().matrix() * m_pos_c2_h.transpose();
            //             Eigen::Vector3f m_pos_1, m_pos_2;
            //             m_pos_1 << m_pos_1_h[0], m_pos_1_h[1], m_pos_1_h[2];
            //             m_pos_2 << m_pos_2_h[0], m_pos_2_h[1], m_pos_2_h[2];
                        
            //             glColor3f(0.0,0.7,0.0);
            //             glVertex3f(m_pos_1(0),m_pos_1(1),m_pos_1(2));

            //             glColor3f(0.0,1.0,0.0);
            //             glVertex3f(m_pos_2(0),m_pos_2(1),m_pos_2(2));
            //         }
            //     }
            // }

            for (size_t i = 0; i < v1MPs.size(); i++) {
                std::shared_ptr<MapPoint> pMPi1, pMPi2;
                pMPi1 = v1MPs[i];
                pMPi2 = v2MPs[i];
                if (!pMPi1) continue;
                if (!pMPi2) continue;

                Eigen::Vector3f pos1 = pMPi1->getWorldPosition(); 
                Eigen::Vector3f pos2 = pMPi2->getWorldPosition(); 

                int index_in_kf1 = pMap_->isMapPointInKeyFrame(pMPi1->getId(), kf1ID);
                int index_in_kf2 = pMap_->isMapPointInKeyFrame(pMPi2->getId(), kf2ID);

                if(index_in_kf1 < 0 || index_in_kf2 < 0) continue;
                size_t idx1 = (size_t)index_in_kf1;
                size_t idx2 = (size_t)index_in_kf2;

                cv::Point2f x1 = pKF1->getKeyPoint(idx1).pt;
                cv::Point2f x2 = pKF2->getKeyPoint(idx2).pt;

                // double d1 = pKF1->getDepthMeasure(x1.x, x1.y);
                // double d2 = pKF2->getDepthMeasure(x2.x, x2.y);

                // if(d1 != -1 && d2 != -1) {
                //     Eigen::Matrix<float,1,3> m_pos_c1 = pCamera1->unproject(x1, d1);
                //     Eigen::Matrix<float,1,3> m_pos_c2 = pCamera2->unproject(x2, d2);

                //     Eigen::Matrix<float,1,4> m_pos_c1_h, m_pos_c2_h;
                //     m_pos_c1_h << m_pos_c1[0], m_pos_c1[1], m_pos_c1[2], 1;
                //     m_pos_c2_h << m_pos_c2[0], m_pos_c2[1], m_pos_c2[2], 1;
                    
                //     Eigen::Vector4f m_pos_1_h = T1w.inverse().matrix() * m_pos_c1_h.transpose();
                //     Eigen::Vector4f m_pos_2_h = T2w.inverse().matrix() * m_pos_c2_h.transpose();
                //     Eigen::Vector3f m_pos_1, m_pos_2;
                //     m_pos_1 << m_pos_1_h[0], m_pos_1_h[1], m_pos_1_h[2];
                //     m_pos_2 << m_pos_2_h[0], m_pos_2_h[1], m_pos_2_h[2];
                    
                //     glColor3f(0.0,0.7,0.0);
                //     glVertex3f(m_pos_1(0),m_pos_1(1),m_pos_1(2));

                //     glColor3f(0.0,1.0,0.0);
                //     glVertex3f(m_pos_2(0),m_pos_2(1),m_pos_2(2));
                // }
                
                glColor3f(0.0,0.0,0.0);
                glVertex3f(pos2(0),pos2(1),pos2(2));

                glColor3f(1.0,0.3,0.3);
                glVertex3f(pos1(0),pos1(1),pos1(2));
            }
        }
    }

    glEnd();
}

void MapVisualizer::drawRays() {
    auto keyFrames = pMap_->getKeyFrames();

    glLineWidth(0.5);
    glBegin(GL_LINES);

    for(auto kf : keyFrames){
        auto kps = kf.second->getKeyPoints();  
        
        if(kf.second->getId()%2 == 0){
            glColor3f(1.0f,0.0f,0.0f);
        }
        else{
            glColor3f(0.0f,0.0f,1.0f);
        }

        Sophus::SE3f Twc = kf.second->getPose().inverse();
        Eigen::Vector3f cameraPos = Twc.translation();    

        std::shared_ptr<CameraModel> calib = kf.second->getCalibration();

        for (auto& kp : kps) {
            if (kp.pt.x <= 0 || kp.pt.y <= 0) continue;

            Eigen::Vector3f xn = calib->unproject(kp.pt).normalized();

            Eigen::Vector3f xnWorld = Twc.so3() * xn;

            Eigen::Vector3f endPoint = cameraPos + xnWorld * 10.0f;

            glVertex3f(cameraPos(0), cameraPos(1), cameraPos(2));  // Start point (camera position)
            glVertex3f(endPoint(0), endPoint(1), endPoint(2));      // End point (in direction of keypoint)
        }
    }

    glEnd();
}

void MapVisualizer::drawKeyFrames() {
    const float &w = 0.05*80;
    const float h = w*0.75;
    const float z = w*0.6;

    auto keyFrames = pMap_->getKeyFrames();
    for(auto kf : keyFrames){
        if(kf.second->getId() == 0){
            glColor3f(1.0f,0.0f,0.0f);
            glLineWidth(5);
        }
        else{
            glColor3f(0.0f,0.0f,1.0f);
            glLineWidth(2);
        }
        Sophus::SE3f Twc = kf.second->getPose().inverse();

        glPushMatrix();
        glMultMatrixf(Twc.matrix().data());

        glBegin(GL_LINES);
        glVertex3f(0,0,0);
        glVertex3f(w,h,z);
        glVertex3f(0,0,0);
        glVertex3f(w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,h,z);

        glVertex3f(w,h,z);
        glVertex3f(w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);

        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);
        glEnd();

        glPopMatrix();
    }
}

void MapVisualizer::drawCurrentPose() {
    const float &w = 0.05*80;
    const float h = w*0.75;
    const float z = w*0.6;

    glColor3f(0.0f,1.0f,0.0f);

    glPushMatrix();
    glMultMatrixf(currPose_.inverse().matrix().data());

    glLineWidth(5);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}
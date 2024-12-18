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

MapVisualizer::MapVisualizer(shared_ptr<Map> pMap) : pMap_(pMap){
    pangolin::CreateWindowAndBind("SLAM",1024,768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Define Camera Render Object (for view / scene browsing)
    s_cam = pangolin::OpenGlRenderState(
            pangolin::ProjectionMatrix(1024,768,500,500,512,389,0.1,1000),
            pangolin::ModelViewLookAt(0,-0.7,-1.8, 0,0,0,0.0,-1.0, 0.0));

    // Add named OpenGL viewport to window and provide 3D Handler
    d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));
}

void MapVisualizer::update(bool drawRaysSelection) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    d_cam.Activate(s_cam);
    glClearColor(1.0f,1.0f,1.0f,1.0f);

    drawMapPoints();
    drawKeyFrames();
    if (drawRaysSelection) drawRays();
    drawCurrentPose();

    pangolin::FinishFrame();
}

void MapVisualizer::updateCurrentPose(Sophus::SE3f &currPose) {
    currPose_ = currPose;
}

void MapVisualizer::drawMapPoints() {
    std::unordered_map<ID,std::shared_ptr<KeyFrame>> keyFrames = pMap_->getKeyFrames();

    glPointSize(6);
    glBegin(GL_POINTS);
    //glColor3f(0.0,0.0,0.0);

    for(auto kf_info : keyFrames){
        int id = kf_info.first; 
        std::shared_ptr<KeyFrame> kf = kf_info.second;
        if (!kf) {
            continue;
        }
        std::vector<std::shared_ptr<MapPoint>> mapPoints = kf->getMapPoints();

        std::shared_ptr<CameraModel> pCamera = kf->getCalibration();
        g2o::SE3Quat cameraPose = g2o::SE3Quat(kf->getPose().unit_quaternion().cast<double>(),kf->getPose().translation().cast<double>());
        Sophus::SE3f Tcw = kf->getPose();

        for(std::shared_ptr<MapPoint> mp: mapPoints) {
            if (!mp) {
                continue;
            }

            Eigen::Vector3f pos = mp->getWorldPosition(); 

            Eigen::Vector3d p3Dc = cameraPose.map(pos.cast<double>());
            Eigen::Vector2f projection;
            pCamera->project(p3Dc.cast<float>(), projection);
            float d = kf->getDepthMeasure(projection[0], projection[1]);
            Eigen::Vector3f m_pos_c(p3Dc[0], p3Dc[1], d);
            Eigen::Vector3f m_pos = Tcw.inverse() * m_pos_c;
            
            glColor3f(0.0,1.0,0.0);
            glVertex3f(m_pos(0),m_pos(1),m_pos(2));
            
            if(id%2 == 0) {
                glColor3f(1.0,0.3,0.3);
            } else {
                glColor3f(0.0,0.0,0.0);
            }
            glVertex3f(pos(0),pos(1),pos(2));
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
    const float &w = 0.05;
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
    const float &w = 0.05;
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
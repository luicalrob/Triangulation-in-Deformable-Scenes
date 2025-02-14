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

#include "FAST.h"

#include <opencv2/opencv.hpp>

using namespace std;

void ExtractorNode::DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4){
    const int halfX = ceil(static_cast<float>(UR.x-UL.x)/2);
    const int halfY = ceil(static_cast<float>(BR.y-UL.y)/2);

    //Define boundaries of childs
    n1.UL = UL;
    n1.UR = cv::Point2i(UL.x+halfX,UL.y);
    n1.BL = cv::Point2i(UL.x,UL.y+halfY);
    n1.BR = cv::Point2i(UL.x+halfX,UL.y+halfY);
    n1.vKeys.reserve(vKeys.size());

    n2.UL = n1.UR;
    n2.UR = UR;
    n2.BL = n1.BR;
    n2.BR = cv::Point2i(UR.x,UL.y+halfY);
    n2.vKeys.reserve(vKeys.size());

    n3.UL = n1.BL;
    n3.UR = n1.BR;
    n3.BL = BL;
    n3.BR = cv::Point2i(n1.BR.x,BL.y);
    n3.vKeys.reserve(vKeys.size());

    n4.UL = n3.UR;
    n4.UR = n2.BR;
    n4.BL = n3.BR;
    n4.BR = BR;
    n4.vKeys.reserve(vKeys.size());

    //Associate points to childs
    for(size_t i=0;i<vKeys.size();i++)
    {
        const cv::KeyPoint &kp = vKeys[i];
        if(kp.pt.x<n1.UR.x)
        {
            if(kp.pt.y<n1.BR.y)
                n1.vKeys.push_back(kp);
            else
                n3.vKeys.push_back(kp);
        }
        else if(kp.pt.y<n1.BR.y)
            n2.vKeys.push_back(kp);
        else
            n4.vKeys.push_back(kp);
    }

    if(n1.vKeys.size()==1)
        n1.bNoMore = true;
    if(n2.vKeys.size()==1)
        n2.bNoMore = true;
    if(n3.vKeys.size()==1)
        n3.bNoMore = true;
    if(n4.vKeys.size()==1)
        n4.bNoMore = true;

}

void FAST::extract(const cv::Mat &im, std::vector<cv::KeyPoint> &vKeys) {

    if(im.empty())
        return;

    assert(im.type() == CV_8UC1);

    // Pre-compute the scale pyramid

    computePyramid(im);

    vector<vector<cv::KeyPoint>> allKeypoints;
    std::vector<cv::Mat> reflectionMasks = GenerateMasks(im, cv::Mat(), nScales_, false, true);

    computeKeyPointsOctTree(allKeypoints, reflectionMasks);

    int nExtracted = 0;
    for(auto vExtracedLevel : allKeypoints){
        nExtracted += vExtracedLevel.size();

    }

    vKeys.clear();
    //vKeys.reserve(nExtracted);
    int extracted = 0;
    for(auto vExtracedLevel : allKeypoints){
        for(auto kp : vExtracedLevel){
            if(extracted < vKeys.capacity()){
                kp.pt *= vScaleFactor_[kp.octave];
                vKeys.push_back(kp);
                extracted++;
            }
        }
    }
}

void FAST::computePyramid(const cv::Mat& im){
    for (int level = 0; level < nScales_; ++level){
        float scale = vInvScaleFactor_[level];
        cv::Size sz(cvRound((float)im.cols*scale), cvRound((float)im.rows*scale));
        cv::Size wholeSize(sz.width + EDGE_THRESHOLD*2, sz.height + EDGE_THRESHOLD*2);
        cv::Mat temp(wholeSize, im.type()), masktemp;
        vImagePyramid_[level] = temp(cv::Rect(EDGE_THRESHOLD, EDGE_THRESHOLD, sz.width, sz.height));

        // Compute the resized image
        if( level != 0 ){
            resize(vImagePyramid_[level-1], vImagePyramid_[level], sz, 0, 0, cv::INTER_LINEAR);

            copyMakeBorder(vImagePyramid_[level], temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                           cv::BORDER_REFLECT_101+cv::BORDER_ISOLATED);}
        else{
            copyMakeBorder(im, temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                           cv::BORDER_REFLECT_101);
        }
    }
}

void FAST::computeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint>>& allKeypoints, const std::vector<cv::Mat>& reflectionMasks)
{
    allKeypoints.resize(nScales_);

    const float W = 30;

    for (int level = 0; level < nScales_; ++level){
        const cv::Mat& mask = reflectionMasks[level];
        cv::imshow("Máscara", mask);
        cv::waitKey(0); // Espera una tecla para cerrar

        const int minBorderX = EDGE_THRESHOLD-3;
        const int minBorderY = minBorderX;
        const int maxBorderX = vImagePyramid_[level].cols-EDGE_THRESHOLD+3;
        const int maxBorderY = vImagePyramid_[level].rows-EDGE_THRESHOLD+3;

        vector<cv::KeyPoint> vToDistributeKeys;
        vToDistributeKeys.reserve(nMaxFeatures_*10);

        const float width = (maxBorderX-minBorderX);
        const float height = (maxBorderY-minBorderY);

        const int nCols = width/W;
        const int nRows = height/W;
        const int wCell = ceil(width/nCols);
        const int hCell = ceil(height/nRows);

        for(int i=0; i<nRows; i++){
            const float iniY =minBorderY+i*hCell;
            float maxY = iniY+hCell+6;

            if(iniY>=maxBorderY-3)
                continue;
            if(maxY>maxBorderY)
                maxY = maxBorderY;

            for(int j=0; j<nCols; j++){
                const float iniX =minBorderX+j*wCell;
                float maxX = iniX+wCell+6;
                if(iniX>=maxBorderX-6)
                    continue;
                if(maxX>maxBorderX)
                    maxX = maxBorderX;

                vector<cv::KeyPoint> vKeysCell;
                cv::FAST(vImagePyramid_[level].rowRange(iniY,maxY).colRange(iniX,maxX),
                     vKeysCell,thFast_,true);

                if(vKeysCell.empty())
                {
                    cv::FAST(vImagePyramid_[level].rowRange(iniY,maxY).colRange(iniX,maxX),
                         vKeysCell,minThFAST_,true);
                }

                if(!vKeysCell.empty()){
                    for(vector<cv::KeyPoint>::iterator vit=vKeysCell.begin(); vit!=vKeysCell.end();vit++){
                        (*vit).pt.x+=j*wCell;
                        (*vit).pt.y+=i*hCell;
                        vToDistributeKeys.push_back(*vit);
                    }
                }

                vKeysCell.erase(
                remove_if(vKeysCell.begin(), vKeysCell.end(),
                            [&](const cv::KeyPoint& kp) {
                                int x = cvRound(kp.pt.x);
                                int y = cvRound(kp.pt.y);
            
                                int mask_value = mask.at<uchar>(y, x);
                                std::cout << "Checking KeyPoint at (" << x << ", " << y << "): mask value = " << mask_value << std::endl;
            
                                return mask_value == 255; // Keep only keypoints where mask is NOT 255
                            }),
                vKeysCell.end());
            }
        }

        vector<cv::KeyPoint> & keypoints = allKeypoints[level];
        keypoints.reserve(nMaxFeatures_);

        keypoints = distributeOctTree(vToDistributeKeys, minBorderX, maxBorderX,
                                      minBorderY, maxBorderY,vFeaturesPerLevel_[level], level);

        const int scaledPatchSize = PATCH_SIZE*vScaleFactor_[level];

        // Add border to coordinates and scale information
        const int nkps = keypoints.size();
        for(int i=0; i<nkps ; i++)
        {
            keypoints[i].pt.x+=minBorderX;
            keypoints[i].pt.y+=minBorderY;
            keypoints[i].octave=level;
            keypoints[i].size = scaledPatchSize;
        }
    }

    // compute orientations
    for (int level = 0; level < nScales_; ++level)
        computeOrientation(vImagePyramid_[level], allKeypoints[level], umax_);
}

vector<cv::KeyPoint> FAST::distributeOctTree(const vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                                     const int &maxX, const int &minY, const int &maxY, const int &N, const int &level){
    // Compute how many initial nodes
    const int nIni = round(static_cast<float>(maxX-minX)/(maxY-minY));

    const float hX = static_cast<float>(maxX-minX)/nIni;

    list<ExtractorNode> lNodes;

    vector<ExtractorNode*> vpIniNodes;
    vpIniNodes.resize(nIni);

    for(int i=0; i<nIni; i++){
        ExtractorNode ni;
        ni.UL = cv::Point2i(hX*static_cast<float>(i),0);
        ni.UR = cv::Point2i(hX*static_cast<float>(i+1),0);
        ni.BL = cv::Point2i(ni.UL.x,maxY-minY);
        ni.BR = cv::Point2i(ni.UR.x,maxY-minY);
        ni.vKeys.reserve(vToDistributeKeys.size());

        lNodes.push_back(ni);
        vpIniNodes[i] = &lNodes.back();
    }

    //Associate points to childs
    for(size_t i=0;i<vToDistributeKeys.size();i++){
        const cv::KeyPoint &kp = vToDistributeKeys[i];
        vpIniNodes[kp.pt.x/hX]->vKeys.push_back(kp);
    }

    list<ExtractorNode>::iterator lit = lNodes.begin();

    while(lit!=lNodes.end()){
        if(lit->vKeys.size()==1){
            lit->bNoMore=true;
            lit++;
        }
        else if(lit->vKeys.empty())
            lit = lNodes.erase(lit);
        else
            lit++;
    }

    bool bFinish = false;

    int iteration = 0;

    vector<pair<int,ExtractorNode*> > vSizeAndPointerToNode;
    vSizeAndPointerToNode.reserve(lNodes.size()*4);

    while(!bFinish){
        iteration++;

        int prevSize = lNodes.size();

        lit = lNodes.begin();

        int nToExpand = 0;

        vSizeAndPointerToNode.clear();

        while(lit!=lNodes.end()){
            if(lit->bNoMore){
                // If node only contains one point do not subdivide and continue
                lit++;
                continue;
            }
            else{
                // If more than one point, subdivide
                ExtractorNode n1,n2,n3,n4;
                lit->DivideNode(n1,n2,n3,n4);

                // Add childs if they contain points
                if(n1.vKeys.size()>0){
                    lNodes.push_front(n1);
                    if(n1.vKeys.size()>1){
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(),&lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if(n2.vKeys.size()>0){
                    lNodes.push_front(n2);
                    if(n2.vKeys.size()>1){
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(),&lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if(n3.vKeys.size()>0){
                    lNodes.push_front(n3);
                    if(n3.vKeys.size()>1){
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(),&lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if(n4.vKeys.size()>0){
                    lNodes.push_front(n4);
                    if(n4.vKeys.size()>1){
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(),&lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }

                lit=lNodes.erase(lit);
                continue;
            }
        }

        // Finish if there are more nodes than required features
        // or all nodes contain just one point
        if((int)lNodes.size()>=N || (int)lNodes.size()==prevSize){
            bFinish = true;
        }
        else if(((int)lNodes.size()+nToExpand*3)>N){

            while(!bFinish){

                prevSize = lNodes.size();

                vector<pair<int,ExtractorNode*> > vPrevSizeAndPointerToNode = vSizeAndPointerToNode;
                vSizeAndPointerToNode.clear();

                sort(vPrevSizeAndPointerToNode.begin(),vPrevSizeAndPointerToNode.end());
                for(int j=vPrevSizeAndPointerToNode.size()-1;j>=0;j--){
                    ExtractorNode n1,n2,n3,n4;
                    vPrevSizeAndPointerToNode[j].second->DivideNode(n1,n2,n3,n4);

                    // Add childs if they contain points
                    if(n1.vKeys.size()>0){
                        lNodes.push_front(n1);
                        if(n1.vKeys.size()>1){
                            vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n2.vKeys.size()>0){
                        lNodes.push_front(n2);
                        if(n2.vKeys.size()>1){
                            vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n3.vKeys.size()>0){
                        lNodes.push_front(n3);
                        if(n3.vKeys.size()>1){
                            vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n4.vKeys.size()>0){
                        lNodes.push_front(n4);
                        if(n4.vKeys.size()>1){
                            vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }

                    lNodes.erase(vPrevSizeAndPointerToNode[j].second->lit);

                    if((int)lNodes.size()>=N)
                        break;
                }

                if((int)lNodes.size()>=N || (int)lNodes.size()==prevSize)
                    bFinish = true;
            }
        }
    }

    // Retain the best point in each node
    vector<cv::KeyPoint> vResultKeys;
    vResultKeys.reserve(nMaxFeatures_);
    for(list<ExtractorNode>::iterator lit=lNodes.begin(); lit!=lNodes.end(); lit++){
        vector<cv::KeyPoint> &vNodeKeys = lit->vKeys;
        cv::KeyPoint* pKP = &vNodeKeys[0];
        float maxResponse = pKP->response;

        for(size_t k=1;k<vNodeKeys.size();k++){
            if(vNodeKeys[k].response>maxResponse){
                pKP = &vNodeKeys[k];
                maxResponse = vNodeKeys[k].response;
            }
        }

        vResultKeys.push_back(*pKP);
    }

    return vResultKeys;
}

void FAST::computeOrientation(const cv::Mat& image, vector<cv::KeyPoint>& keypoints, const vector<int>& umax){
    for (vector<cv::KeyPoint>::iterator keypoint = keypoints.begin(),
                 keypointEnd = keypoints.end(); keypoint != keypointEnd; ++keypoint){
        keypoint->angle = IC_Angle(image, keypoint->pt, umax);
    }
}

float FAST::IC_Angle(const cv::Mat& image, cv::Point2f pt,  const vector<int> & u_max){
    int m_01 = 0, m_10 = 0;

    const uchar* center = &image.at<uchar> (cvRound(pt.y), cvRound(pt.x));

    // Treat the center line differently, v=0
    for (int u = -HALF_PATCH_SIZE; u <= HALF_PATCH_SIZE; ++u)
        m_10 += u * center[u];

    // Go line by line in the circuI853lar patch
    int step = (int)image.step1();
    for (int v = 1; v <= HALF_PATCH_SIZE; ++v){
        // Proceed over the two lines
        int v_sum = 0;
        int d = u_max[v];
        for (int u = -d; u <= d; ++u){
            int val_plus = center[u + v*step], val_minus = center[u - v*step];
            v_sum += (val_plus - val_minus);
            m_10 += u * (val_plus + val_minus);
        }
        m_01 += v * v_sum;
    }

    return cv::fastAtan2((float)m_01, (float)m_10);
}


std::vector<cv::Mat> FAST::GenerateMasks(const cv::Mat &image, const cv::Mat &borderMask, int numOctaves, bool bBorder, bool bReflexion)
{
    vector<cv::Mat> vMasks;

    cv::Mat baseMask = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);
    std::cout << "image: " << image.size << "; type: " << image.type() << std::endl;
    //std::cout << "borderMask: " << borderMask.size << "; type: " << borderMask.type() << std::endl;
    //std::cout << "baseMask before border: " << baseMask.size << "; type: " << baseMask.type() << std::endl;
    if(bBorder)
    {
        baseMask = borderMask.clone();
    }
    //std::cout << "baseMask after border: " << baseMask.size << "; type: " << baseMask.type() << std::endl;
    if(bReflexion)
    {
        double nColorThreshold = 240;
        cv::Mat mask_reflexion;
        cv::threshold(image, mask_reflexion, nColorThreshold, 255, cv::THRESH_BINARY);
        std::cout << "mask_reflexion: " << mask_reflexion.size << "; type: " << mask_reflexion.type() << std::endl;

        if(baseMask.empty())
        {
            baseMask = mask_reflexion.clone();
        }
        else
        {
            cv::bitwise_or(mask_reflexion, baseMask, baseMask);
        }
    }
    std::cout << "baseMask after reflexion: " << baseMask.size << "; type: " << baseMask.type() << std::endl;
    double minVal, maxVal;
    cv::minMaxLoc(baseMask, &minVal, &maxVal);
    std::cout << "baseMask values: min=" << minVal << ", max=" << maxVal << std::endl;

    if(baseMask.empty())
    {
        return vMasks;
    }

    vMasks.reserve(numOctaves);
    float factor = 2.5/1.5; // factor es 1.66666666
    int max_scale = 1;
    for (int i = 0; i < numOctaves; ++i)
    {
        max_scale *= 2;
        int side = ceil(max_scale * factor) * 2 + 5;
        //std::cout << "MaskByOct --> octave: " << i << "; side: " << side << std::endl;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(side, side));
        cv::Mat mask_i;
        cv::dilate(baseMask, mask_i, kernel);

        vMasks.push_back(mask_i);

    }

    //std::cout << "There are " << vMasks.size() << " masks generated" << std::endl;

    return vMasks;
}
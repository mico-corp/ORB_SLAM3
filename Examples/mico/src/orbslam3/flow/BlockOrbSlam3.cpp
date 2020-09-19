//---------------------------------------------------------------------------------------------------------------------
//  mico
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2020 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------

#include <mico/orbslam3/flow/BlockOrbSlam3.h>
#include <flow/Policy.h>
#include <flow/Outpipe.h>

namespace mico{

    BlockOrbSlam3::BlockOrbSlam3(){
        
        createPolicy({    {"image", "image"} });

        createPipe("pose", "mat44");
        
        registerCallback({  "image" }, 
                                [this](flow::DataFlow _data){
                                    this->callbackOdometry(_data);
                                });

    }

    BlockOrbSlam3::~BlockOrbSlam3(){
        if(slam_ != nullptr){
            slam_->Shutdown();
        }
    }



    bool BlockOrbSlam3::configure(std::unordered_map<std::string, std::string> _params){
        std::string configFile = "";
        std::string vocabFile = "";
        for(auto &param: _params){
            if(param.first == "vocabulary" && param.second != ""){
                vocabFile = param.second;
            }else if(param.first == "configuration" && param.second != ""){
                configFile = param.second;
            }
        }

        if(configFile == "" || vocabFile == ""){
            return false;
        }

        slam_ = new ORB_SLAM3::System(vocabFile, configFile, ORB_SLAM3::System::MONOCULAR, false);
        configured_ = true;
        t0_ = std::chrono::high_resolution_clock::now();

        return true;
    }
    
    std::vector<std::string> BlockOrbSlam3::parameters(){
        return {"vocabulary", "configuration"};
    }


    void BlockOrbSlam3::callbackOdometry(flow::DataFlow _data){
        if(idle_){
            idle_ = false;
            if(configured_){
                // Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
                try{
                    cv::Mat image = _data.get<cv::Mat>("image");
                    auto t1 = std::chrono::high_resolution_clock::now();
                    float incT = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0_).count();
                    cv::Mat result = slam_->TrackMonocular(image, incT/1000.0f);
                    if(getPipe("pose")->registrations()){
                        Eigen::Matrix4f pose;
                        if(result.empty()){
                            pose = Eigen::Matrix4f::Identity();
                        }else{
                            for(unsigned i = 0; i<4; i++){
                                for(unsigned j = 0; j<4; j++){
                                    pose(i,j) = result.at<float>(i,j);
                                }
                            }
                        }
                        
                        getPipe("pose")->flush(pose);
                    }
                }catch(std::exception& e){
                    std::cout << "Failure OdometryRGBD. " <<  e.what() << std::endl;
                    idle_ = true;
                    return;
                }

            }else{
                std::cout << "Please, configure OrbSlam3 block" << std::endl;
            }
            idle_ = true;
        }
    }

}

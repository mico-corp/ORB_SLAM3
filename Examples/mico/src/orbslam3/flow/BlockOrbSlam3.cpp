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

#include <QVBoxLayout>
#include <QComboBox>

namespace mico{

    BlockOrbSlam3::BlockOrbSlam3(){
        createPipe("pose", "mat44");
        this->preparePolicy();
    }

    BlockOrbSlam3::~BlockOrbSlam3(){
        if(slam_ != nullptr){
            slam_->Shutdown();
        }
    }

    QBoxLayout * BlockOrbSlam3::creationWidget() { 
        layout_ = new QVBoxLayout;
        slamSelector_ = new QComboBox();
        slamSelector_->addItem("Monocular");
        slamSelector_->addItem("Stereo");
        slamSelector_->addItem("Mono-Inertial");
        slamSelector_->addItem("RGBD");
        slamSelector_->addItem("Stereo-Inertial");
        
        QObject::connect(slamSelector_, &QComboBox::currentTextChanged, [&](const QString &_text){
            type_ = parseType(_text.toStdString());
            this->preparePolicy();
        });

        layout_->addWidget(slamSelector_);

        return layout_; 
    };       

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
                    auto t1 = std::chrono::high_resolution_clock::now();


                    float incT = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0_).count();
                    cv::Mat result;
                    
                    dataLock_.lock();
                    auto vImu = vImu_;
                    vImu_.clear();
                    dataLock_.unlock();

                    if(type_ == eSlamType::MONOCULAR){
                        cv::Mat image = _data.get<cv::Mat>("image");
                        result = slam_->TrackMonocular(image, incT/1000.0f);
                    } else if(type_ == eSlamType::STEREO){
                        cv::Mat left = _data.get<cv::Mat>("left");
                        cv::Mat right = _data.get<cv::Mat>("right");
                        result = slam_->TrackStereo(left, right, incT);
                    } else if(type_ == eSlamType::RGBD){
                        cv::Mat image = _data.get<cv::Mat>("image");
                        cv::Mat depth = _data.get<cv::Mat>("depth");
                        result = slam_->TrackRGBD(image, depth, incT);
                    } else if(type_ == eSlamType::MONO_INER){
                        cv::Mat image = _data.get<cv::Mat>("image");
                        result = slam_->TrackMonocular(image, incT, vImu);
                    } else if(type_ == eSlamType::STEREO_INER){
                        cv::Mat left = _data.get<cv::Mat>("left");
                        cv::Mat right = _data.get<cv::Mat>("right");
                        result = slam_->TrackStereo(left, right, incT, vImu_);
                    }
                    
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

    BlockOrbSlam3::eSlamType BlockOrbSlam3::parseType(const std::string &_type){
        if(_type == "Monocular"){
            return eSlamType::MONOCULAR;
        }else if(_type == "RGBD"){
            return eSlamType::RGBD;
        }else if(_type == "Stereo"){
            return eSlamType::STEREO;
        }else if(_type == "Mono-Inertial"){
            return eSlamType::MONO_INER;
        }else if(_type == "Stereo-Inertial"){
            return eSlamType::STEREO_INER;
        }

        return eSlamType::MONOCULAR;
    }

    void BlockOrbSlam3::preparePolicy(){
        removePolicy();
        flow::Policy::PolicyMask mask;

        if(type_ == eSlamType::MONOCULAR){
            createPolicy({{"image", "image"}});
            mask = {"image"};
        }else if(type_ == eSlamType::STEREO){
            createPolicy({{"left", "image"}, {"right", "image"}});
            mask = {"left", "right"};
        }else if(type_ == eSlamType::RGBD){
            createPolicy({{"image", "image"}, {"depth", "image"}});
            mask = {"image", "depth"};
        }else if(type_ == eSlamType::MONO_INER){   
           createPolicy({{"image", "image"}, {"acc", "vec3"}, {"gyro", "vec3"}});
           mask = {"image", "acc", "gyro"};
        }else if(type_ == eSlamType::STEREO_INER){   
           createPolicy({{"left", "image"}, {"right", "image"}, {"acc", "vec3"}, {"gyro", "vec3"}});
           mask = {"left", "right"};
        }else{
            return;
        }

        registerCallback(   mask, 
                            [&](flow::DataFlow  _data){
                                this->callbackOdometry(_data);
                            });

        if(type_ == eSlamType::MONO_INER || type_ == eSlamType::STEREO_INER){
            registerCallback(   {"acc", "gyro"}, 
                            [&](flow::DataFlow  _data){
                                dataLock_.lock();
                                auto t1 = std::chrono::high_resolution_clock::now();
                                float incT = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0_).count();            
                                Eigen::Vector3f acc = _data.get<Eigen::Vector3f>("acc");
                                Eigen::Vector3f gyro = _data.get<Eigen::Vector3f>("gyro");

                                ORB_SLAM3::IMU::Point data(  acc[0], acc[1], acc[2],
                                                            gyro[0], gyro[1], gyro[2],
                                                            incT);
                                vImu_.push_back(data);
                                dataLock_.unlock();
                            });
        }
    }
}

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


#ifndef MICO_FLOW_STREAMERS_BLOCKS_PROCESSORS_BLOCKORBSLAM3_H_
#define MICO_FLOW_STREAMERS_BLOCKS_PROCESSORS_BLOCKORBSLAM3_H_

#include <flow/Block.h>
#include <System.h>
#include <chrono>
#include <vector>
#include <mutex>


class QBoxLayout;
class QComboBox;

namespace mico{

    class BlockOrbSlam3: public flow::Block{
    public:
        enum eSlamType {MONOCULAR, RGBD, STEREO, MONO_INER, STEREO_INER};
        virtual std::string name() const override {return "OrbSlam3";}

        BlockOrbSlam3();
        ~BlockOrbSlam3();

        QBoxLayout * creationWidget() override;        

        bool configure(std::unordered_map<std::string, std::string> _params) override;
        std::vector<std::string> parameters() override;

        std::string description() const override {return    "Block for visual odometry using ORBSLAM3.\n"
                                                            "   - Inputs: \n"
                                                            "   - Outputs: \n";};

        QIcon icon() const override {return QIcon("/usr/share/icons/Humanity/emblems/48/emblem-cool.svg");};

    private:
        void callbackOdometry(flow::DataFlow _data);
        eSlamType parseType(const std::string &_type);
        void preparePolicy();
        
    private:
        bool idle_ = true;
        bool configured_ = false;
        ORB_SLAM3::System *slam_ = nullptr;
        std::vector<ORB_SLAM3::IMU::Point>  vImu_;
        std::mutex dataLock_;
        std::chrono::time_point<std::chrono::high_resolution_clock> t0_;

        QBoxLayout *layout_;
        QComboBox *slamSelector_;

        eSlamType type_ = eSlamType::MONOCULAR;
        
    };

}

#endif
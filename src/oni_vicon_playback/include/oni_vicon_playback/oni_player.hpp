/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014 Max-Planck-Institute for Intelligent Systems,
 *                     University of Southern California,
 *                     Karlsruhe Institute of Technology
 *    Jan Issac (jan.issac@gmail.com)
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @date 05/04/2014
 * @author Jan Issac (jan.issac@gmail.com)
 * Max-Planck-Institute for Intelligent Systems, University of Southern California (USC),
 *   Karlsruhe Institute of Technology (KIT)
 */


#ifndef oni_vicon_playback_ONI_PLAYER_HPP
#define oni_vicon_playback_ONI_PLAYER_HPP

#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <ni/XnCppWrapper.h>

#include <oni_vicon_common/types.hpp>

namespace oni_vicon_playback
{
    class OniPlayer
    {
    public:
        typedef boost::shared_ptr<OniPlayer> Ptr;

    public:
        OniPlayer();
        virtual ~OniPlayer();

        /**
         * @brief open
         * @param source_file
         * @param camera_intrinsics
         *
         * @throw OpenOniFileException
         */
        void open(const std::string &source_file, const
                  oni_vicon::CameraIntrinsics& camera_intrinsics);
        bool processNextFrame();
        bool close();

        bool seekToFrame(uint32_t frameID);
        bool setPlaybackSpeed(double speed);

        /**
         * @brief isEOF
         * @return
         */
        bool isEOF() const;

        uint32_t currentFrameID() const;
        const xn::DepthMetaData& currentDepthMetaData() const;
        sensor_msgs::ImagePtr currentDepthImageMsg();
        sensor_msgs::PointCloud2Ptr currentPointCloud2Msg();

        uint32_t countFrames() const;

    public:
        /**
         * @brief toMsgImage converts an ONI depth image into a ros imag e message
         *
         * @param [in]   depth_meta_data   OpenNI DepthMetaData
         * @param [out]  image             Ros image message
         */
        void toMsgImage(const xn::DepthMetaData& depth_meta_data,
                        sensor_msgs::ImagePtr image) const;

        float toMeter(const XnDepthPixel& depth_pixel) const;
        float toMillimeter(const XnDepthPixel& depth_pixel) const;

    private:
        /* OpenNI player */
        xn::Context context_;
        xn::Player player_;
        xn::DepthGenerator depth_generator_;
        XnUInt64 no_sample_value_;
        XnUInt64 shadow_value_;

        oni_vicon::CameraIntrinsics camera_intrinsics_;
        boost::mutex read_write_mutex_;

        /* frame data */
        xn::DepthMetaData depth_meta_data_;
        XnUInt32 frames_;

        /* cache */
        sensor_msgs::PointCloud2Ptr cache_msg_pointcloud_;
        sensor_msgs::ImagePtr cache_msg_image_;
        bool msg_image_dirty_;
        bool msg_pointcloud_dirty_;
    };
}

#endif

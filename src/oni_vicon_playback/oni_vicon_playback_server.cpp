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


// boost
#include <boost/filesystem.hpp>
#include <boost/function.hpp>

#include <oni_vicon_common/global_calibration.hpp>
#include <oni_vicon_common/local_calibration.hpp>
#include <oni_vicon_common/calibration_reader.hpp>

#include "oni_vicon_playback/exceptions.hpp"
#include "oni_vicon_playback/oni_vicon_playback_server.hpp"

using namespace oni_vicon;
using namespace oni_vicon_playback;


// c++/std
#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>

// boost
#include <boost/filesystem.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/regex.hpp>

// ros
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <oni_vicon_common/global_calibration.hpp>
#include <oni_vicon_common/local_calibration.hpp>
#include <oni_vicon_common/calibration_reader.hpp>

#include <oni_vicon_playback/exceptions.hpp>
#include <oni_vicon_playback/oni_vicon_player.hpp>
#include <state_filtering/run_spkf_filter.hpp>
#include <oni_vicon_common/type_conversion.hpp>


#include <state_filtering/filter/kalman/spkf/spkf_state_filter.hpp>
#include <state_filtering/filter/kalman/spkf/sigma_point_kalman_filter.hpp>
#include <state_filtering/filter/kalman/spkf/validation_gate_base.hpp>
#include <state_filtering/filter/kalman/spkf/zero_validation_gate.hpp>
#include <state_filtering/filter/kalman/spkf/euclidean_validation_gate.hpp>
#include <state_filtering/filter/kalman/spkf/ellipsoidal_validation_gate.hpp>
#include <state_filtering/filter/kalman/spkf/ukf/ukf_distribution_descriptor.hpp>

#include <state_filtering/observation_models/spkf/depth_measurement_model.hpp>
#include <state_filtering/filter/kalman/spkf/ukf/factorized_unscented_kalman_filter.hpp>


// filter
#include <state_filtering/filter/particle/coordinate_filter.hpp>
#include <state_filtering/filter/particle/particle_filter_context.hpp>

// observation model
#include <state_filtering/observation_models/cpu_image_observation_model/gaussian_pixel_observation_model.hpp>
#include <state_filtering/observation_models/image_observation_model.hpp>
#include <state_filtering/observation_models/cpu_image_observation_model/cpu_image_observation_model.hpp>
// tools
#include <state_filtering/tools/object_file_reader.hpp>
#include <state_filtering/tools/helper_functions.hpp>
#include <state_filtering/tools/pcl_interface.hpp>
#include <state_filtering/tools/ros_interface.hpp>
#include <state_filtering/tools/macros.hpp>
//#include "cloud_visualizer.hpp"

// distributions
#include <state_filtering/distribution/distribution.hpp>
#include <state_filtering/distribution/implementations/gaussian_distribution.hpp>
#include <state_filtering/process_model/stationary_process_model.hpp>
#include <state_filtering/process_model/composed_stationary_process_model.hpp>
#include <state_filtering/process_model/brownian_process_model.hpp>

#include <state_filtering/system_states/rigid_body_system.hpp>
#include <state_filtering/system_states/full_rigid_body_system.hpp>

using namespace boost;
using namespace std;
using namespace Eigen;
using namespace filter;

typedef filter::FactorizedUkfInternals FilterInternals;
typedef distr::DepthMeasurementModel<-1,-1,-1> UsedMeasurementModel;

std::string uirToPath(std::string uri)
{
    std::string re_str = "package://([\\w\\-]+)(/.+)";
    boost::cmatch matches;
    std::string package_path;
    std::string sub_path;

    try
    {
       boost:regex re(re_str);
       if (!boost::regex_match(uri.c_str(), matches, re))
       {
          std::cout << "Your URL is not formatted correctly!" << std::endl;
          throw "Your URL is not formatted correctly!";
       }

       package_path = std::string(matches[1].first, matches[1].second);
       sub_path = std::string(matches[2].first, matches[2].second);
    }
    catch (boost::regex_error& e)
    {
       cerr << "The regexp " << re_str << " is invalid!" << endl;
       throw(e);
    }

    return ros::package::getPath(package_path) + sub_path;
}

std::string recordName(std::string uri)
{
    std::string re_str = "/([\\w\\-]+/)+([\\w\\-]+)";
    boost::cmatch matches;

    try
    {
       boost:regex re(re_str);
       if (!boost::regex_match(uri.c_str(), matches, re))
       {
          std::cout << "Your URL is not formatted correctly!" << std::endl;
          throw "Your URL is not formatted correctly!";
       }

       return std::string(matches[2].first, matches[2].second);
    }
    catch (boost::regex_error& e)
    {
       cerr << "The regexp " << re_str << " is invalid!" << endl;
       throw(e);
    }
}
class EvalRunSpkfFilter
{
    template<typename T>
    void ReadParameter(const string& path, T& parameter)
    {
        ReadParam<T>(path, parameter, node_handle_);
    }

    template <typename Type>
    Type getParamValue(const std::string& name)
    {
        XmlRpc::XmlRpcValue param_ros;
        node_handle_.getParam(name, param_ros);
        return param_ros;
    }

public:
    EvalRunSpkfFilter():
        node_handle_("~")
    {

    }

    virtual void initialize(std::string object_model_path, Matrix3d camera_matrix,
                            VectorXd init_state)
    {
        ReadParameter("downsampling_factor", downsampling_factor_);

        std::cout << "downsampling_factor_ =  " << downsampling_factor_ << std::endl;


        camera_matrix_ = camera_matrix;
        camera_matrix_.topLeftCorner(2,3) /= float(downsampling_factor_);

        object_model_path_ = object_model_path;
        validate_gate_type_ = getParamValue<string>("validation_gate_type");

        ut_alpha_ = getParamValue<double>("/spkf_filter/ut_alpha");
        ut_beta_ = getParamValue<double>("/spkf_filter/ut_beta");
        ut_kappa_ = getParamValue<double>("/spkf_filter/ut_kappa");
        mean_depth_ = getParamValue<double>("/spkf_filter/mean_depth");
        uncertain_depth_sigma_ = getParamValue<double>("/spkf_filter/uncertain_depth_sigma");
        measurement_NA_sigma_ = getParamValue<double>("/spkf_filter/measurement_NA_sigma");
        depth_noise_sigma_ = getParamValue<double>("/spkf_filter/depth_noise_sigma");
        init_process_sigma_ = getParamValue<double>("/spkf_filter/init_process_sigma");
        some_c_ = getParamValue<double>("/spkf_filter/some_c");
        some_s_ = getParamValue<double>("/spkf_filter/some_s");

        occ_q_ = getParamValue<double>("/spkf_filter/occ_q");
        occ_r_ = getParamValue<double>("/spkf_filter/occ_r");

        stateDesc.timestamp(0);
        predictedStateDesc.timestamp(0);


        // load object mesh ------------------------------------------------------------------------
        boost::shared_ptr<RigidBodySystem<> > rigid_body_system(new FullRigidBodySystem<>(1));
        ObjectFileReader file_reader;


        file_reader.set_filename(uirToPath(object_model_path));
        file_reader.Read();

        vector<vector<Vector3d> > object_vertices;
        vector<vector<vector<int> > > object_triangle_indices;

        object_vertices.push_back(*file_reader.get_vertices());
        object_triangle_indices.push_back(*file_reader.get_indices());

        object_renderer_ = boost::shared_ptr<obj_mod::RigidBodyRenderer>(
                    new obj_mod::RigidBodyRenderer(object_vertices,
                                                   object_triangle_indices,
                                                   rigid_body_system));

        // initialize process model ========================================================================================================================================================================================================================================================================================================================================================================================================================
        double free_damping; ReadParameter("free_damping", free_damping);
        double free_linear_acceleration_sigma; ReadParameter("free_linear_acceleration_sigma", free_linear_acceleration_sigma);
        MatrixXd free_linear_acceleration_covariance =
                MatrixXd::Identity(3, 3) * pow(double(free_linear_acceleration_sigma), 2);

        double free_angular_acceleration_sigma; ReadParameter("free_angular_acceleration_sigma", free_angular_acceleration_sigma);
        MatrixXd free_angular_acceleration_covariance =
                MatrixXd::Identity(3, 3) * pow(double(free_angular_acceleration_sigma), 2);

        vector<boost::shared_ptr<StationaryProcessModel<> > > partial_process_models(1);

        for(size_t i = 0; i < partial_process_models.size(); i++)
        {
            boost::shared_ptr<BrownianProcessModel<> > partial_process_model(new BrownianProcessModel<>);

            partial_process_model->parameters(
                        object_renderer_->get_coms(i).cast<double>(),
                        free_damping,
                        free_linear_acceleration_covariance,
                        free_angular_acceleration_covariance);

            partial_process_models[i] = partial_process_model;
        }
        boost::shared_ptr<ComposedStationaryProcessModel> process_model
                (new ComposedStationaryProcessModel(partial_process_models));

        spkfInternals = FilterInternals::Ptr(new FilterInternals());
        spkf = SpkfStateFilter::Ptr(new SpkfStateFilter(spkfInternals));

//        // set process model
//        boost::shared_ptr<StationaryProcessModel<> > process_model;
        spkf->processModel(process_model);

        // set measurement model
        measurementModel = UsedMeasurementModel::Ptr(new UsedMeasurementModel(object_renderer_));
        spkf->measurementModel(measurementModel);

        // set sigma point transform
        sigmaPointTransform = UnscentedTransform::Ptr(new UnscentedTransform(ut_alpha_,
                                                                             ut_beta_,
                                                                             ut_kappa_));

        spkf->sigmaPointTransform(sigmaPointTransform);

//        // set validation gate type
//        if (validate_gate_type_.compare("none") == 0)
//        {
//            validationGate_ = ZeroValidationGate::Ptr(new ZeroValidationGate());
//        }
//        else if(validate_gate_type_.compare("euclidean") == 0)
//        {
//            double t = getParamValue<double>("/spkf_filter/validation_gate_distance_thresh");
//            PRINT << "Info: Using Euclidean Validation Gate with" << CR;
//            validationGate_ = EuclideanValidationGate::Ptr(new EuclideanValidationGate(t));
//        }
//        else if(validate_gate_type_.compare("ellipsoidal") == 0)
//        {
//            double a = getParamValue<double>("/spkf_filter/validation_gate_pass_prob");
//            PRINT << "Info: Using Ellipsoidal Validation Gate" << CR;
//            validationGate_ = EllipsoidalValidationGate::Ptr(new EllipsoidalValidationGate(a));
//        }
//        else
//        {
//            PRINT << "Error: Unknown validation gate type "  << validate_gate_type_ << CR;
//            exit(-1);
//        }
        double t = getParamValue<double>("/spkf_filter/validation_gate_distance_thresh");
        PRINT << "Info: Using Euclidean Validation Gate with" << CR;
        validationGate_ = EuclideanValidationGate::Ptr(new EuclideanValidationGate(t));
        spkf->validationGate(validationGate_);

        spkfInternals->occTest = OccTest::Ptr(new OccTest(occ_q_, occ_r_));
        spkfInternals->occTest->useOcc = getParamValue<bool>("/spkf_filter/apply_occlusion");

        stateDesc.mean() = init_state;
        stateDesc.covariance() = ProcessModel::CovarianceType::Identity(
                    BrownianProcessModel<>::Base::VariableSize,
                    BrownianProcessModel<>::Base::VariableSize)
                * (init_process_sigma_ * init_process_sigma_);

        std::cout << "initial state" << stateDesc.mean().transpose() << std::endl;
    }

    filter::UkfDistributionDescriptor filter(sensor_msgs::PointCloud2Ptr ros_cloud)
    {
        ProcessModel::ControlType control = ProcessModel::ControlType::Zero(6);

        Matrix4f H;
        VectorXd output_state;

        //INIT_PROFILING

        // convert ros cloud to std::vector --------------------------------------------------------
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg (*ros_cloud, *point_cloud);

        int n_rows = point_cloud->height/downsampling_factor_;
        int n_cols = point_cloud->width/downsampling_factor_;

        vector<int> availableIndices(n_rows*n_cols, 0);
        measurement.resize(n_rows*n_cols, 1);
        int newSize = 0;

        for(int row = 0; row < n_rows; row++)
        {
            for(int col = 0; col < n_cols; col++)
            {
                measurement(row*n_cols + col, 0) = point_cloud->at(col * downsampling_factor_,
                                                                   row * downsampling_factor_).z;
                if (!std::isnan(measurement(row*n_cols + col, 0)))
                {
                    availableIndices[newSize++] = row*n_cols + col;
                }
                else
                {
                    // value Nan, so we know nothing about that pixel and hence set to a mean
                    measurement(row*n_cols + col, 0) = mean_depth_;
                }
            }
        }

        // shrink vector to the number of available pixels
        availableIndices.resize(newSize);

        if (stateDesc.timestamp() == 0)
        {
            stateDesc.timestamp(ros_cloud->header.stamp.toSec());

            spkfInternals->occTest->init(n_rows*n_cols);
        }

        double deltaT = (ros_cloud->header.stamp.toSec() - stateDesc.timestamp());

        //MEASURE("---------------------------\nPreparation")

        spkf->predict(stateDesc, control,  deltaT, predictedStateDesc);
        stateDesc.timestamp(ros_cloud->header.stamp.toSec());

        //MEASURE("Prediction")

//        // publish predicted object state
//        output_state = predictedStateDesc.mean();
//        hf::Vector2Hom(hf::NewState2OldState(output_state), H);
//        ri::PublishMarker(H, ros_cloud->header, object_model_path_, object2_publisher_, 2, 1, 0, 0);

        measurementModel->parameters(camera_matrix_,
                                     n_rows,
                                     n_cols,
                                     availableIndices,
                                     depth_noise_sigma_,
                                     mean_depth_,
                                     uncertain_depth_sigma_,
                                     measurement_NA_sigma_,
                                     measurement,
                                     some_c_,
                                     some_s_);

        //MEASURE("Update preparation")

        spkf->update(measurement, predictedStateDesc, stateDesc);

        // estimate occ
        measurementModel->parameters(camera_matrix_,
                                     n_rows,
                                     n_cols,
                                     availableIndices,
                                     depth_noise_sigma_,
                                     mean_depth_,
                                     uncertain_depth_sigma_,
                                     measurement_NA_sigma_,
                                     measurement,
                                     some_c_,
                                     some_s_);

        measurementModel->conditionals(stateDesc.mean());
        spkfInternals->occTest->filter(measurement, measurementModel->predict());
        spkfInternals->occTest->o_t_ = spkfInternals->occTest->o_t_.array() * measurementModel->mask().array();


//        std::cout << "e "
//                  << stateDesc.mean()(0, 0) << " "
//                  << stateDesc.mean()(1, 0) << " "
//                  << stateDesc.mean()(2, 0) << " "
//                  << stateDesc.mean()(3, 0) << " "
//                  << stateDesc.mean()(4, 0) << " "
//                  << stateDesc.mean()(5, 0) << " "
//                  << stateDesc.mean()(6, 0) << " " << std::endl;


        //MEASURE("Update")

//        stateDesc.mean().middleRows(3, 4).normalize();
//        output_state = stateDesc.mean();
//        hf::Vector2Hom(hf::NewState2OldState(output_state), H);
//        ri::PublishMarker(H, ros_cloud.header, object_model_path_, object_publisher_, 1, 0, 1, 0);

        //MEASURE("Predict & Update ")

        return stateDesc;
    }

protected:
    ros::NodeHandle node_handle_;

    ros::Publisher chi2Publisher;
    ros::Publisher object2_publisher_;
//    tools::DepthImagePublisher viz;

    filter::SpkfStateFilter::Ptr spkf;
    //filter::SpkfInternals::Ptr spkfInternals;
    FilterInternals::Ptr spkfInternals;
    filter::ValidationGate::Ptr validationGate_;
    filter::SigmaPointTransform::Ptr sigmaPointTransform;
    UsedMeasurementModel::Ptr measurementModel;
    filter::UkfDistributionDescriptor stateDesc;
    filter::UkfDistributionDescriptor predictedStateDesc;
    filter::DynamicVector measurement;

    double ut_alpha_;
    double ut_beta_;
    double ut_kappa_;
    double mean_depth_;
    double uncertain_depth_sigma_;
    double measurement_NA_sigma_;
    double depth_noise_sigma_;
    double init_process_sigma_;
    double some_c_;
    double some_s_;

    double occ_r_;
    double occ_q_;

    uint8_t rainbow[0x10000][3];

    std::string validate_gate_type_;
    std::string sigma_point_transform_type_;

    std::string object_name_;
    std::string object_model_path_;

    Matrix3d camera_matrix_;
    int downsampling_factor_;

    boost::shared_ptr<obj_mod::RigidBodyRenderer> object_renderer_;
    boost::shared_ptr<StationaryProcessModel<> > process_model_;
};

class EvalRunRbpfFilter
{
    template<typename T>
    void ReadParameter(const string& path, T& parameter)
    {
        ReadParam<T>(path, parameter, node_handle_);
    }

    template <typename Type>
    Type getParamValue(const std::string& name)
    {
        XmlRpc::XmlRpcValue param_ros;
        node_handle_.getParam(name, param_ros);
        return param_ros;
    }

public:
    EvalRunRbpfFilter():
        node_handle_("~")
    {

    }

    virtual void initialize(std::string object_model_path, Matrix3d camera_matrix,
                            VectorXd init_state,
                            sensor_msgs::PointCloud2Ptr ros_cloud)
    {
        ReadParameter("dependencies", dependencies_);

        ReadParameter("downsampling_factor", downsampling_factor_);
        ReadParameter("cpu_sample_count", cpu_sample_count_);
        ReadParameter("p_visible_init", p_visible_init_);
        ReadParameter("p_visible_visible", p_visible_visible_);
        ReadParameter("p_visible_occluded", p_visible_occluded_);
        int initial_sample_count;
        ReadParam("initial_sample_count", initial_sample_count, node_handle_);

        camera_matrix_ = camera_matrix;
        camera_matrix_.topLeftCorner(2,3) /= float(downsampling_factor_);

        // init states
        Vector3d t_mean;
        t_mean(0,0) = init_state(0,0);
        t_mean(1,0) = init_state(1,0);
        t_mean(2,0) = init_state(2,0);

        double standard_deviation_translation = 0.0003;
        double standard_deviation_rotation = 100.00;
        GaussianDistribution<double, 1> unit_gaussian;
        unit_gaussian.setNormal();

        // sample around mean
        for(size_t i = 0; i < size_t(initial_sample_count); i++)
        {
            FullRigidBodySystem<-1> state(1);
            state.translation() =
                    t_mean +
                    standard_deviation_translation * unit_gaussian.sample()(0) * Vector3d::Ones();
            state.orientation() = Quaterniond(init_state(6,0), init_state(3,0), init_state(4,0), init_state(5,0)).coeffs();

            initial_states.push_back(state);
        }

        // initialize observation model ========================================================================================================================================================================================================================================================================================================================================================================================================================
        // load object mesh ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        boost::shared_ptr<RigidBodySystem<> > rigid_body_system(new FullRigidBodySystem<>(1));

        vector<vector<Vector3d> > object_vertices(1);
        vector<vector<vector<int> > > object_triangle_indices(1);
        for(size_t i = 0; i < 1; i++)
        {
            ObjectFileReader file_reader;
            file_reader.set_filename(uirToPath(object_model_path));
            file_reader.Read();

            object_vertices[i] = *file_reader.get_vertices();
            object_triangle_indices[i] = *file_reader.get_indices();
        }

        object_renderer_ = boost::shared_ptr<obj_mod::RigidBodyRenderer>(
                    new obj_mod::RigidBodyRenderer(object_vertices,
                                                   object_triangle_indices,
                                                   rigid_body_system));
        // pixel_observation_model -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        double tail_weight; ReadParameter("tail_weight", tail_weight);
        double model_sigma; ReadParameter("model_sigma", model_sigma);
        double sigma_factor; ReadParameter("sigma_factor", sigma_factor);
        boost::shared_ptr<obs_mod::PixelObservationModel>
                pixel_observation_model(new obs_mod::GaussianPixelObservationModel(tail_weight, model_sigma, sigma_factor));

        // initialize occlusion process model -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        boost::shared_ptr<proc_mod::OcclusionProcessModel>
                occlusion_process_model(new proc_mod::OcclusionProcessModel(p_visible_visible_, p_visible_occluded_));

        // cpu obseration model -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        boost::shared_ptr<obs_mod::ImageObservationModel> cpu_observation_model(new obs_mod::CPUImageObservationModel(
                                                                                    camera_matrix_,
                                                                                    ros_cloud->height/downsampling_factor_,
                                                                                    ros_cloud->width/downsampling_factor_,
                                                                                    initial_states.size(),
                                                                                    rigid_body_system,
                                                                                    object_renderer_,
                                                                                    pixel_observation_model,
                                                                                    occlusion_process_model,
                                                                                    p_visible_init_));

        // initialize process model ========================================================================================================================================================================================================================================================================================================================================================================================================================
        double free_damping; ReadParameter("free_damping", free_damping);

        double free_linear_acceleration_sigma; ReadParameter("free_linear_acceleration_sigma", free_linear_acceleration_sigma);
        MatrixXd free_linear_acceleration_covariance =
                MatrixXd::Identity(3, 3) * pow(double(free_linear_acceleration_sigma), 2);

        double free_angular_acceleration_sigma; ReadParameter("free_angular_acceleration_sigma", free_angular_acceleration_sigma);
        MatrixXd free_angular_acceleration_covariance =
                MatrixXd::Identity(3, 3) * pow(double(free_angular_acceleration_sigma), 2);

        vector<boost::shared_ptr<StationaryProcessModel<> > > partial_process_models(1);
        for(size_t i = 0; i < partial_process_models.size(); i++)
        {
            boost::shared_ptr<BrownianProcessModel<> > partial_process_model(new BrownianProcessModel<>);
            partial_process_model->parameters(
                        object_renderer_->get_coms(i).cast<double>(),
                        free_damping,
                        free_linear_acceleration_covariance,
                        free_angular_acceleration_covariance);
            partial_process_models[i] = partial_process_model;
        }

        boost::shared_ptr<ComposedStationaryProcessModel> process_model
                (new ComposedStationaryProcessModel(partial_process_models));

        // initialize coordinate_filter ============================================================================================================================================================================================================================================================
        cpu_filter_ = boost::shared_ptr<filter::CoordinateFilter>
                (new filter::CoordinateFilter(cpu_observation_model, process_model, dependencies_));


        filter_context_ =
                boost::shared_ptr<filter::ParticleFilterContext<double, -1> >
                (new filter::ParticleFilterContext<double, -1>(cpu_filter_) );



        start_time_ = ros_cloud->header.stamp.toSec();
        previous_time_ = ros_cloud->header.stamp.toSec();
        is_first_iteration_ = false;


        // create the multi body initial samples ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        FullRigidBodySystem<> default_state(1);
        for(size_t object_index = 0; object_index < 1; object_index++)
            default_state.translation(object_index) = Vector3d(0, 0, 1.5); // outside of image

        cout << "creating intiial stuff" << endl;
        vector<VectorXd> multi_body_samples(initial_states.size());
        for(size_t state_index = 0; state_index < multi_body_samples.size(); state_index++)
            multi_body_samples[state_index] = default_state;

        // the point cloud is converted and downsampled ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        vector<float> observations; size_t n_rows, n_cols;
        pi::Ros2Std(*ros_cloud, downsampling_factor_, observations, n_rows, n_cols);

        cout << "doing evaluations " << endl;
        for(size_t body_index = 0; body_index < 1; body_index++)
        {
            for(size_t state_index = 0; state_index < multi_body_samples.size(); state_index++)
            {
                FullRigidBodySystem<> full_initial_state(multi_body_samples[state_index]);
                full_initial_state[body_index] = initial_states[state_index];
                multi_body_samples[state_index] = full_initial_state;
            }

            cpu_filter_->set_states(multi_body_samples);
//            cpu_filter_->Evaluate(observations);
//            cpu_filter_->Resample(multi_body_samples.size());
//            cpu_filter_->get(multi_body_samples);
        }

        // we evaluate the initial particles and resample ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//        cout << "evaluating initial particles cpu ..." << endl;
//        cpu_filter_->set_states(multi_body_samples);
//        cpu_filter_->Evaluate(observations);
//        cpu_filter_->Resample(cpu_sample_count_);
    }

    VectorXd filter(sensor_msgs::PointCloud2& ros_cloud)
    {
        // the point cloud is converted and downsampled ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        vector<float> observations; size_t n_rows, n_cols;
        pi::Ros2Std(ros_cloud, downsampling_factor_, observations, n_rows, n_cols);

        filter_context_->predictAndUpdate(observations,
                                          0.03,//ros_cloud.header.stamp.toSec() - previous_time_,
                                          VectorXd::Zero(1*6));

        previous_time_ = ros_cloud.header.stamp.toSec();

        // we visualize the likeliest state -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        vector<float> cpu_loglikes; cpu_filter_->get(cpu_loglikes);
        FullRigidBodySystem<> cpu_likeliest_state = cpu_filter_->get_state(hf::BoundIndex(cpu_loglikes, true));
        return cpu_likeliest_state[0];
    }

protected:
    vector<VectorXd> initial_states;

    boost::shared_ptr<filter::ParticleFilterContext<double, -1> > filter_context_;

    ros::NodeHandle node_handle_;
    bool is_first_iteration_;
    double start_time_;
    double previous_time_;

    ros::Publisher object_publisher_;

    vector<vector<size_t> > dependencies_;

    boost::mutex mutex_;

    //	filter::StateFilter standard_filter_;
    boost::shared_ptr<filter::CoordinateFilter> cpu_filter_;
    boost::shared_ptr<obj_mod::RigidBodyRenderer> object_renderer_;
    boost::shared_ptr<StationaryProcessModel<> > process_model_;

    // parameters
    Matrix3d camera_matrix_;
    int downsampling_factor_;
    int cpu_sample_count_;
    double p_visible_init_;
    double p_visible_visible_;
    double p_visible_occluded_;
};



OniViconPlaybackServer::OniViconPlaybackServer(ros::NodeHandle& node_handle,
                               OniViconPlayer::Ptr playback,
                               const std::string& depth_frame_id,
                               const std::string& camera_info_topic,
                               const std::string& point_cloud_topic):
    playback_(playback),
    image_transport_(node_handle),
    depth_frame_id_(depth_frame_id),
    camera_info_topic_(camera_info_topic),
    point_cloud_topic_(point_cloud_topic),
    open_as_(OpenGoal::ACTION_NAME,
             boost::bind(&OniViconPlaybackServer::openCb, this, _1),
             false),
    play_as_(PlayGoal::ACTION_NAME,
             boost::bind(&OniViconPlaybackServer::playCb, this, _1),
             false),
    paused_(false)
{
    pause_srv_ = node_handle.advertiseService(Pause::Request::SERVICE_NAME,
                                              &OniViconPlaybackServer::pauseCb,
                                              this);

    seek_frame_srv_ = node_handle.advertiseService(SeekFrame::Request::SERVICE_NAME,
                                                   &OniViconPlaybackServer::seekFrameCb,
                                                   this);

    set_playback_speed_srv_ = node_handle.advertiseService(SetPlaybackSpeed::Request::SERVICE_NAME,
                                                           &OniViconPlaybackServer::setPlaybackSpeedCb,
                                                           this);

    set_time_offset_srv_ = node_handle.advertiseService(SetTimeOffset::Request::SERVICE_NAME,
                                                        &OniViconPlaybackServer::setTimeOffset,
                                                        this);

    pub_depth_image_ = image_transport_.advertise("depth/image", 100);
    pub_depth_info_ = node_handle.advertise<sensor_msgs::CameraInfo>("depth/camera_info", 100);
    pub_point_cloud_ = node_handle.advertise<sensor_msgs::PointCloud2>("depth/points", 100);

    vicon_object_pose_publisher_ =
            node_handle.advertise<visualization_msgs::Marker>("vicon_object_pose", 0);
}


void OniViconPlaybackServer::run()
{
    open_as_.start();
    play_as_.start();

    ros::spin();
}

void OniViconPlaybackServer::playCb(const PlayGoalConstPtr& goal)
{
    PlayFeedback feedback;
    feedback.playing = true;
    feedback.current_time = 0;
    feedback.current_vicon_frame = 0;
    feedback.current_depth_sensor_frame = goal->starting_frame;

    playback_->play(goal->starting_frame);

    play_as_.publishFeedback(feedback);


    // this is due to ros, which complains if the time stamps are small and therefore interpreted as
    // too old. ros merely throws them away. how cruel is that?!
    ros::Time startup_time = ros::Time::now();

    EvalRunSpkfFilter evalRunSpkf;
    EvalRunRbpfFilter evalRunRbpf;
    bool init = false;

    while (ros::ok() && !play_as_.isPreemptRequested() && playback_->isPlaying())
    {
        if (paused_)
        {
            playback_->seekToFrame(seeking_frame_);
        }

        boost::mutex::scoped_lock lock(player_lock_);
        uint32_t frame_id = playback_->nextFrame();

        // get depth sensor frame and corresponding vicon frame
        sensor_msgs::ImagePtr depth_msg = playback_->oniPlayer()->currentDepthImageMsg();
        ViconPlayer::PoseRecord vicon_pose_record_dts = playback_->currentViconPose();
        ViconPlayer::PoseRecord vicon_pose_record = playback_->viconPlayer()->pose(frame_id);

//        std::cout << "*dts:";
//        std::cout <<vicon_pose_record_dts.stamp << " ";
//        std::cout <<vicon_pose_record_dts.vicon_frame << " ";
//        std::cout <<vicon_pose_record_dts.pose.getOrigin().getX() << " ";
//        std::cout <<vicon_pose_record_dts.pose.getOrigin().getY() << " ";
//        std::cout <<vicon_pose_record_dts.pose.getOrigin().getZ() << " ";
//        std::cout <<vicon_pose_record_dts.pose.getRotation().getW() << " ";
//        std::cout <<vicon_pose_record_dts.pose.getRotation().getX() << " ";
//        std::cout <<vicon_pose_record_dts.pose.getRotation().getY() << " ";
//        std::cout <<vicon_pose_record_dts.pose.getRotation().getZ() << std::endl;

        if (vicon_pose_record.stamp.isZero())
        {
            break;
        }

        ros::Time stamp;
        // this is the shifted time
        // stamp.fromNSec(startup_time.toNSec() + vicon_pose_record.stamp.toNSec());

        // for display puposes
        stamp = ros::Time::now();

        // set meta data and publish
        depth_msg->header.stamp = stamp;
        depth_msg->header.frame_id = depth_frame_id_;

        // publish visualization data (depth image, point cloud, vicon pose marker)
        publish(depth_msg);
        publish(vicon_pose_record.pose,
                depth_msg,
                playback_->transformer().localCalibration().objectDisplay(),
                "_system_stamp",
                0, 0.8, 1);

        publish(vicon_pose_record_dts.pose,
                depth_msg,
                playback_->transformer().localCalibration().objectDisplay(),
                "_device_stamp",
                1, 0, 0);

        tf_broadcaster_.sendTransform(
                    tf::StampedTransform(
                        playback_->transformer().globalCalibration().viconToCameraTransform(),
                        depth_msg->header.stamp,
                        depth_frame_id_,
                        "vicon_global_frame"));

        // publish evaluation data
        feedback.current_time = frame_id / 30.;
        feedback.current_vicon_frame = 0;
        feedback.current_depth_sensor_frame = frame_id;
        play_as_.publishFeedback(feedback);

        if (!paused_)
        {
            if (!init)
            {
                VectorXd init_state = VectorXd::Zero(BrownianProcessModel<>::Base::VariableSize);
                init_state[0] = vicon_pose_record_dts.pose.getOrigin().getX();
                init_state[1] = vicon_pose_record_dts.pose.getOrigin().getY();
                init_state[2] = vicon_pose_record_dts.pose.getOrigin().getZ();
                init_state[3] = vicon_pose_record_dts.pose.getRotation().getX();
                init_state[4] = vicon_pose_record_dts.pose.getRotation().getY();
                init_state[5] = vicon_pose_record_dts.pose.getRotation().getZ();
                init_state[6] = vicon_pose_record_dts.pose.getRotation().getW();

                sensor_msgs::CameraInfoPtr camera_info =
                        oni_vicon::toCameraInfo(playback_->transformer().cameraIntrinsics());
                Matrix3d camera_matrix;
                for(unsigned int col = 0; col < 3; col++)
                    for(unsigned int row = 0; row < 3; row++)
                        camera_matrix(row,col) = camera_info->K[col+row*3];

                evalRunSpkf.initialize(playback_->transformer().localCalibration().object(),
                                       camera_matrix,
                                       init_state);

                evalRunRbpf.initialize(playback_->transformer().localCalibration().object(),
                                       camera_matrix,
                                       init_state,
                                       points_msg_);

                init = true;
            }
            else
            {
                filter::UkfDistributionDescriptor stateDesc = evalRunSpkf.filter(points_msg_);
                VectorXd rbpfState = evalRunRbpf.filter(*points_msg_);

                tf::Pose tracked_pose;
                tf::Vector3 t;
                tf::Quaternion q;

                t.setX(stateDesc.mean()(0, 0));
                t.setY(stateDesc.mean()(1, 0));
                t.setZ(stateDesc.mean()(2, 0));

                q.setW(stateDesc.mean()(6, 0));
                q.setX(stateDesc.mean()(3, 0));
                q.setY(stateDesc.mean()(4, 0));
                q.setZ(stateDesc.mean()(5, 0));

                tracked_pose.setOrigin(t);
                tracked_pose.setRotation(q);

                publish(tracked_pose,
                        depth_msg,
                        playback_->transformer().localCalibration().objectDisplay(),
                        "_spkf",
                        0, 1, 0);

                t.setX(rbpfState(0, 0));
                t.setY(rbpfState(1, 0));
                t.setZ(rbpfState(2, 0));

                q.setW(rbpfState(6, 0));
                q.setX(rbpfState(3, 0));
                q.setY(rbpfState(4, 0));
                q.setZ(rbpfState(5, 0));

                tracked_pose.setOrigin(t);
                tracked_pose.setRotation(q);

                publish(tracked_pose,
                        depth_msg,
                        playback_->transformer().localCalibration().objectDisplay(),
                        "_rbpf",
                        0, 0, 1);
            }
        }
    }

    paused_ = false;
    playback_->seekToFrame(0);
    play_as_.setSucceeded();
}

void OniViconPlaybackServer::openCb(const OpenGoalConstPtr& goal)
{
    OpenResult result;

    feedback_.open = false;
    feedback_.progress = 0;
    feedback_.progress_max = 0;
    feedback_.total_time = 0;
    feedback_.total_vicon_frames = 0;
    feedback_.total_depth_sensor_frames = 0;

    try
    {
        playback_->open(goal->record_path,
                       boost::bind(&OniViconPlaybackServer::loadUpdateCb, this, _1, _2));
    }
    catch(oni_vicon_playback::OpenRecordException& e)
    {
        result.message = e.what();
        open_as_.setAborted(result);
        return;
    }

    feedback_.progress = feedback_.progress_max;
    feedback_.open = true;
    feedback_.total_time = playback_->oniPlayer()->countFrames() / 30;
    feedback_.total_vicon_frames = feedback_.total_time * 100;
    feedback_.total_depth_sensor_frames = playback_->oniPlayer()->countFrames();
    feedback_.time_offet = playback_->viconCameraTimeOffset();
    open_as_.publishFeedback(feedback_);

    // wait until stopped
    ros::Rate rate(30);
    while (ros::ok() && !open_as_.isPreemptRequested())
    {
        rate.sleep();
    }

    // stop player if playing
    playback_->stop();
    paused_ = false;
    feedback_ .open = false;
    feedback_ .progress = 0;
    feedback_ .total_time = 0;
    feedback_ .total_vicon_frames = 0;
    feedback_ .total_depth_sensor_frames = 0;
    open_as_.publishFeedback(feedback_);

    // wait till oni player processing is over (if the controller is implemented correctly, this
    // should not be necessary)
    boost::mutex::scoped_lock lock(player_lock_);
    playback_->close();

    result.message = "Record closed";
    open_as_.setSucceeded(result);
}

bool OniViconPlaybackServer::pauseCb(Pause::Request& request, Pause::Response& response)
{
    seeking_frame_ = playback_->oniPlayer()->currentFrameID();
    paused_ = request.paused;

    return true;
}

bool OniViconPlaybackServer::seekFrameCb(SeekFrame::Request& request, SeekFrame::Response &response)
{
    //boost::mutex::scoped_lock lock(player_lock_);
    seeking_frame_ = request.frame;
    return playback_->isPlaying() && paused_ && playback_->oniPlayer()->seekToFrame(request.frame);
}

bool OniViconPlaybackServer::setPlaybackSpeedCb(SetPlaybackSpeed::Request& request,
                                        SetPlaybackSpeed::Response& response)
{
    //return playing_ && paused_ && playback_->oniPlayer()->setPlaybackSpeed(request.speed);
    return playback_->oniPlayer()->setPlaybackSpeed(request.speed);
}

bool OniViconPlaybackServer::setTimeOffset(SetTimeOffset::Request& request,
                                           SetTimeOffset::Response& response)
{
    playback_->viconCameraTimeOffset(request.time_offset);

    return true;
}

void OniViconPlaybackServer::loadUpdateCb(uint32_t total_frames, uint32_t frames_loaded)
{
    if (total_frames > 0)
    {
        feedback_.progress_max = total_frames;
    }

    feedback_.progress = frames_loaded;
    open_as_.publishFeedback(feedback_);
}

void OniViconPlaybackServer::publish(sensor_msgs::ImagePtr depth_msg)
{
    depth_msg->header.frame_id = depth_frame_id_;

    if (pub_depth_info_.getNumSubscribers() > 0)
    {
        sensor_msgs::CameraInfoPtr camera_info =
                oni_vicon::toCameraInfo(playback_->transformer().cameraIntrinsics());
        camera_info->header.frame_id = depth_msg->header.frame_id;
        camera_info->header.stamp = depth_msg->header.stamp;
        camera_info->height = depth_msg->height;
        camera_info->width = depth_msg->width;

        pub_depth_info_.publish(camera_info);
    }

    if (pub_depth_image_.getNumSubscribers () > 0)
    {
        pub_depth_image_.publish(depth_msg);
    }

    points_msg_ = boost::make_shared<sensor_msgs::PointCloud2>();
    oni_vicon::toMsgPointCloud(depth_msg,
                               playback_->transformer().cameraIntrinsics(),
                               points_msg_);

    if (pub_point_cloud_.getNumSubscribers () > 0)
    {


        pub_point_cloud_.publish(points_msg_);
    }
}

void OniViconPlaybackServer::publish(const tf::Pose& pose,
                             sensor_msgs::ImagePtr corresponding_image,
                             const std::string& object_display,
                             const std::string& suffix,
                             double r, double g, double b)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = corresponding_image->header.frame_id;
    marker.header.stamp =  corresponding_image->header.stamp;
    marker.ns = "vicon_object_pose" + suffix;
    marker.id = 0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1;

    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = pose.getOrigin().getX();
    marker.pose.position.y = pose.getOrigin().getY();
    marker.pose.position.z = pose.getOrigin().getZ();

    tf::Quaternion orientation = pose.getRotation();
    marker.pose.orientation.w = orientation.getW();
    marker.pose.orientation.x = orientation.getX();
    marker.pose.orientation.y = orientation.getY();
    marker.pose.orientation.z = orientation.getZ();

    marker.mesh_resource = object_display;

    vicon_object_pose_publisher_.publish(marker);
}

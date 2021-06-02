// http://wiki.ros.org/image_transport/Tutorials/SubscribingToImages
// https://blog.okikiolu.com/posts/jetson-nano-ros-vision-ai-part-2/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <fstream>
#include <chrono>
#include <math.h>
#include "cuda_runtime_api.h"
#include "NvInfer.h"

#include "logging.h"
// namespace ddrnet {
// #include "common.hpp"
// }
// #include "calibrator.h"


//#define USE_INT8  // comment out this if want to use INT8
#define USE_FP16  // comment out this if want to use FP32
#define DEVICE 0  // GPU id
static const int INPUT_H = 1024;
static const int INPUT_W = 1024;
static const int OUT_MAP_H = 128;
static const int OUT_MAP_W = 128;
// static const char* INPUT_BLOB_NAME = "input_0";
// static const char* OUTPUT_BLOB_NAME = "output_0";
static Logger gLogger;

using namespace nvinfer1;

class DDRNetNode;

DDRNetNode* node_instance = NULL;

void _imageCallback(const sensor_msgs::ImageConstPtr& input);
void doInference(IExecutionContext& context, float* input, float* output);
void APIToModel(std::string wts_filename, unsigned int maxBatchSize, IHostMemory** modelStream);
cv::Mat map2cityscape(cv::Mat real_out,cv::Mat real_out_);
cv::Mat read2mat(float * prob, cv::Mat out);

class DDRNetNode {
public:
    ros::Publisher mask_color_pub;
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    image_transport::Subscriber sub;
    image_transport::ImageTransport *image_transport;
    IExecutionContext* context;
    std::vector<float> mean_value{ 0.406, 0.456, 0.485 };  // BGR
    std::vector<float> std_value{ 0.225, 0.224, 0.229 };
    std::string weights_filename;

    DDRNetNode():pnh("~") {
        node_instance = this;
        pnh.param("weights_file", weights_filename, std::string("DDRNet.engine"));
        mask_color_pub = pnh.advertise<sensor_msgs::Image>("mask_color", 2);
        image_transport = new image_transport::ImageTransport(nh);

        cudaSetDevice(DEVICE);
        char *trtModelStream{ nullptr };
        size_t size{ 0 };

        auto wts_filename = replace_ext(weights_filename, ".wts");
        auto engine_filename = replace_ext(weights_filename, ".engine");

        std::ifstream file(engine_filename, std::ios::binary);
        if (file.good()) {
            file.seekg(0, file.end);
            size = file.tellg();
            file.seekg(0, file.beg);
            trtModelStream = new char[size];
            assert(trtModelStream);
            file.read(trtModelStream, size);
            file.close();
        } else {
            std::ifstream wts_file(wts_filename, std::ios::binary);
            if (wts_file.good()) {
                IHostMemory* modelStream{ nullptr };
                APIToModel(wts_filename, 1, &modelStream);
                assert(modelStream != nullptr);
                std::ofstream p(engine_filename, std::ios::binary);
                if (!p) {
                    throw std::runtime_error("could not open .engine output file");
                }
                size = modelStream->size();
                trtModelStream = new char[modelStream->size()];
                std::memcpy(trtModelStream, reinterpret_cast<const char*>(modelStream->data()), modelStream->size());
                p.write(reinterpret_cast<const char*>(modelStream->data()), modelStream->size());
                p.close();
                modelStream->destroy();
            } else {
                std::string out;
                out.append("neither ");
                out.append(engine_filename);
                out.append(" nor ");
                out.append(wts_filename);
                out.append(" files exist");
                throw std::runtime_error(out);
            }
        }

        IRuntime* runtime = createInferRuntime(gLogger);
        assert(runtime != nullptr);
        ICudaEngine* engine = runtime->deserializeCudaEngine(trtModelStream, size);
        assert(engine != nullptr);
        context = engine->createExecutionContext();
        assert(context != nullptr);
        delete[] trtModelStream;

        sub = image_transport->subscribe("camera/image", 1, _imageCallback);
    }

    std::string replace_ext(std::string filename, std::string ext) {
        return filename.substr(0,filename.find_last_of('.')) + ext;
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& input) {

        cv_bridge::CvImagePtr cv_ptr;

        try
        {    
            cv_ptr = cv_bridge::toCvCopy(input, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }

        cv::Mat pr_img = cv_ptr->image;
        int orig_w = pr_img.cols;
        int orig_h = pr_img.rows;

        cv::resize(pr_img, pr_img, cv::Size(INPUT_W,INPUT_H));

        float* data = new float[3 * pr_img.rows * pr_img.cols];
        int i = 0;
        for (int row = 0; row < pr_img.rows; ++row) {
            uchar* uc_pixel = pr_img.data + row * pr_img.step;
            for (int col = 0; col < pr_img.cols; ++col) {
                data[i] = (uc_pixel[2] / 255.0 - mean_value[2]) / std_value[2];
                data[i + pr_img.rows * pr_img.cols] = (uc_pixel[1] / 255.0 - mean_value[1]) / std_value[1];
                data[i + 2 * pr_img.rows * pr_img.cols] = (uc_pixel[0] / 255.0 - mean_value[0]) / std_value[0];
                uc_pixel += 3;
                ++i;
            }
        }
        float* prob = new float[ 19* OUT_MAP_H* OUT_MAP_W];
        // Run inference
        auto start = std::chrono::system_clock::now();
        doInference(*context, data, prob);
        auto end = std::chrono::system_clock::now();        
        ROS_INFO_STREAM_THROTTLE(5, "ddrnet inference took " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" );

        // show mask
        cv::Mat out;
        out.create(OUT_MAP_H, OUT_MAP_W, CV_32FC(19));
        out = read2mat(prob, out);
//        cv::resize(out, real_out, real_out.size());
        cv::Mat mask;
        mask.create(OUT_MAP_H, OUT_MAP_W, CV_8UC3);
        mask = map2cityscape(out, mask);
        cv::resize(mask,mask,cv::Size(orig_w,orig_h));

        delete[] prob;
        delete[] data;

        cv_bridge::CvImage out_msg;
        out_msg.header   = input->header; // Same timestamp and tf frame as input image
        out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
        out_msg.image    = mask;

        mask_color_pub.publish(out_msg.toImageMsg());
    }
};

void _imageCallback(const sensor_msgs::ImageConstPtr& input) {
    if (node_instance) {
        node_instance->imageCallback(input);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ddrseg_node");
    DDRNetNode node;
    ros::spin();
    node_instance = NULL;
}

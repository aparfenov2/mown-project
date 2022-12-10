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

#define CHECK(status)                                          \
    do                                                         \
    {                                                          \
        auto ret = (status);                                   \
        if (ret != 0)                                          \
        {                                                      \
            std::cerr << "Cuda failure: " << ret << std::endl; \
            abort();                                           \
        }                                                      \
    } while (0)


cv::Mat createLTU(int len);
//#define USE_INT8  // comment out this if want to use INT8
// #define USE_FP16  // comment out this if want to use FP32
#define USE_FP32
#define DEVICE 0     // GPU id
#define BATCH_SIZE 1 //

static const int INPUT_H = 1024;
static const int INPUT_W = 1024;
static const int OUTPUT_H = 1024;
static const int OUTPUT_W = 1024;
static const int NUM_CLASSES = 19;
// static const int OUTPUT_SIZE = INPUT_H * INPUT_W;
static const char* INPUT_BLOB_NAME = "input_0";
static const char* OUTPUT_BLOB_NAME = "output_0";
extern Logger gLogger;

using namespace nvinfer1;

void doInference(IExecutionContext &context, cudaStream_t &stream, void **buffers, int batchSize);
void APIToModel(unsigned int maxBatchSize, IHostMemory **modelStream, std::string wtsPath, int width);

class DDRNetNode {
public:
    ros::Publisher mask_color_pub;
    ros::Publisher overlay_pub;
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    image_transport::Subscriber sub;
    image_transport::ImageTransport *image_transport;
    IExecutionContext* context;
    std::string weights_filename;
    cudaStream_t stream;
    float *data;
    float *prob;
    int grass_id_;
    int road_id_;
    void *buffers[2];

    DDRNetNode():pnh("~") {
        pnh.param("weights_file", weights_filename, std::string("DDRNet.engine"));
        pnh.param("grass_id", grass_id_, 1);
        pnh.param("road_id", road_id_, 2);

        ROS_INFO("using %d for grass id, %d for road id", grass_id_, road_id_);
        
        mask_color_pub = pnh.advertise<sensor_msgs::Image>("mask_color", 2);
        overlay_pub = pnh.advertise<sensor_msgs::Image>("overlay", 2);
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
                APIToModel(BATCH_SIZE, &modelStream, wts_filename, 48);
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

        // prepare input data ---------------------------
        cudaSetDeviceFlags(cudaDeviceMapHost);
        // float *data;
        // float *prob;
        CHECK(cudaHostAlloc((void **)&data, BATCH_SIZE * 3           * INPUT_H * INPUT_W * sizeof(float), cudaHostAllocMapped));
        CHECK(cudaHostAlloc((void **)&prob, BATCH_SIZE * NUM_CLASSES * OUTPUT_H * OUTPUT_W * sizeof(float), cudaHostAllocMapped));

        IRuntime *runtime = createInferRuntime(gLogger);
        assert(runtime != nullptr);
        ICudaEngine *engine = runtime->deserializeCudaEngine(trtModelStream, size);
        assert(engine != nullptr);
        context = engine->createExecutionContext();
        assert(context != nullptr);
        delete[] trtModelStream;
        // void *buffers[2];
        // In order to bind the buffers, we need to know the names of the input and output tensors.
        // Note that indices are guaranteed to be less than IEngine::getNbBindings()
        const int inputIndex = engine->getBindingIndex(INPUT_BLOB_NAME);
        const int outputIndex = engine->getBindingIndex(OUTPUT_BLOB_NAME);
        assert(inputIndex == 0);
        assert(outputIndex == 1);
        // cudaStream_t stream;
        CHECK(cudaStreamCreate(&stream));

        cudaHostGetDevicePointer((void **)&buffers[inputIndex], (void *)data, 0);  // buffers[inputIndex]-->data
        cudaHostGetDevicePointer((void **)&buffers[outputIndex], (void *)prob, 0); // buffers[outputIndex] --> prob

        sub = image_transport->subscribe("camera/image", 1, boost::bind(&DDRNetNode::imageCallback, this, _1));
    }

    std::string replace_ext(std::string filename, std::string ext) {
        return filename.substr(0,filename.find_last_of('.')) + ext;
    }

    cv::Mat resizeKeepAspectRatio(const cv::Mat &input, const cv::Size &dstSize, const cv::Scalar &bgcolor)
    {
        cv::Mat output;

        double h1 = dstSize.width * (input.rows/(double)input.cols);
        double w2 = dstSize.height * (input.cols/(double)input.rows);
        if( h1 <= dstSize.height) {
            cv::resize( input, output, cv::Size(dstSize.width, h1));
        } else {
            cv::resize( input, output, cv::Size(w2, dstSize.height));
        }

        int top = (dstSize.height-output.rows) / 2;
        int down = (dstSize.height-output.rows+1) / 2;
        int left = (dstSize.width - output.cols) / 2;
        int right = (dstSize.width - output.cols+1) / 2;

        cv::copyMakeBorder(output, output, top, down, left, right, cv::BORDER_CONSTANT, bgcolor );

        return output;
    }

    cv::Mat resizeKeepAspectRatioBack(const cv::Mat &input, const cv::Size &srcSize, const cv::Scalar &bgcolor)
    {
        const cv::Size dstSize = input.size();

        double h1 = dstSize.width * (srcSize.height/(double)srcSize.width);
        double w2 = dstSize.height * (srcSize.width/(double)srcSize.height);
        cv::Size outSize;
        if( h1 <= dstSize.height) {
            // cv::resize( input, output, cv::Size(dstSize.width, h1));
            outSize = cv::Size(dstSize.width, h1);
        } else {
            // cv::resize( input, output, cv::Size(w2, dstSize.height));
            outSize = cv::Size(w2, dstSize.height);
        }

        int top = (dstSize.height-outSize.height) / 2;
        int down = (dstSize.height-outSize.height+1) / 2;
        int left = (dstSize.width - outSize.width) / 2;
        int right = (dstSize.width - outSize.width+1) / 2;
        // std::cout << left << " " << top << " " << down << " " << right << "\n";
        cv::Mat output(input, cv::Rect(cv::Point(left, top), cv::Point(input.cols - right, input.rows - down)));
        cv::resize( output, output, srcSize);
        return output;
    }

    void dumpProbs(std::string fn, float *prob) {
        std::ofstream wf(fn, std::ios::out | std::ios::binary);
        if(!wf) {
            std::cout << "Cannot open file!" << std::endl;
            return;
        }
        wf.write((char *) prob, BATCH_SIZE * NUM_CLASSES * OUTPUT_H * OUTPUT_W * sizeof(float));
        wf.close();
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& input) {

        cv_bridge::CvImagePtr cv_ptr;

        try
        {    
            cv_ptr = cv_bridge::toCvCopy(input, sensor_msgs::image_encodings::RGB8);
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }

        cv::Mat pr_img = cv_ptr->image;

        int orig_w = pr_img.cols;
        int orig_h = pr_img.rows;

        cv::Mat img = pr_img.clone(); // for img show
        // cv::resize(pr_img, pr_img, cv::Size(INPUT_W,INPUT_H));
        pr_img = resizeKeepAspectRatio(pr_img, cv::Size(INPUT_W,INPUT_H), 0 );

        pr_img.convertTo(pr_img, CV_32FC3);

        if (!pr_img.isContinuous())
        {
            pr_img = pr_img.clone();
        }
        std::memcpy(data, pr_img.data, BATCH_SIZE * 3 * INPUT_W * INPUT_H * sizeof(float));

        // cudaHostGetDevicePointer((void **)&buffers[inputIndex], (void *)data, 0);  // buffers[inputIndex]-->data
        // cudaHostGetDevicePointer((void **)&buffers[outputIndex], (void *)prob, 0); // buffers[outputIndex] --> prob

        // Run inference
        auto start = std::chrono::high_resolution_clock::now();
        doInference(*context, stream, buffers, BATCH_SIZE);
        auto end = std::chrono::high_resolution_clock::now();
        ROS_INFO_STREAM_THROTTLE(10, "ddrnet inference took " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" );


        cv::Mat outimg(OUTPUT_H, OUTPUT_W, CV_32FC3);

        float *prob_grass = &prob[grass_id_ * OUTPUT_H * OUTPUT_W];
        float *prob_road  = &prob[road_id_  * OUTPUT_H * OUTPUT_W];
        const int grass_channel = 0;
        const int road_channel = 1;
        
        for (int row = 0; row < OUTPUT_H; ++row)
        {
            for (int col = 0; col < OUTPUT_W; ++col)
            {
                auto vec = cv::Vec3f();
                vec[grass_channel] = prob_grass[row * OUTPUT_W + col];
                vec[road_channel] = prob_road[row * OUTPUT_W + col];
                outimg.at<cv::Vec3f>(cv::Point(col, row)) = vec;
            }
        }

        dumpProbs("probs.bin", prob);

        // double minVal; 
        // double maxVal; 
        // cv::minMaxLoc( outimg, &minVal, &maxVal);
        // ROS_INFO_STREAM_THROTTLE(10, "minVal " << minVal << " maxVal " << maxVal );

        // cv::Mat im_color;
        // cv::cvtColor(outimg, im_color, cv::COLOR_GRAY2RGB);
        // cv::Mat lut = createLTU(NUM_CLASSES);
        // cv::LUT(im_color, lut, im_color);
        // // false color
        // cv::cvtColor(im_color, im_color, cv::COLOR_RGB2GRAY);
        // cv::applyColorMap(im_color, im_color, cv::COLORMAP_HOT);

        // cv::Mat fusionImg;
        // cv::addWeighted(img, 1, im_color, 0.8, 1, im_color);

        // cv::resize(im_color,im_color,cv::Size(orig_w,orig_h));
        
        auto im_color = resizeKeepAspectRatioBack(outimg, cv::Size(orig_w,orig_h), 0 );

        // cv::Mat debug_color;
        // cv::cvtColor(outimg, debug_color, cv::COLOR_RGB2GRAY);

        // std::cout << im_color.rows << " " << im_color.cols << " " << im_color.channels() << "\n";
        // std::cout << img.rows << " " << img.cols << " " << img.channels() << "\n";


        {
            cv_bridge::CvImage out_msg;
            out_msg.header   = input->header; // Same timestamp and tf frame as input image
            out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC3; // Or whatever
            out_msg.image    = im_color;
            mask_color_pub.publish(out_msg.toImageMsg());
        }


        // cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
        // cv::addWeighted(img, 1, im_color, 0.8, 1, im_color);

        // {
        //     cv_bridge::CvImage out_msg;
        //     out_msg.header   = input->header; // Same timestamp and tf frame as input image
        //     out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
        //     out_msg.image    = debug_color;
        //     overlay_pub.publish(out_msg.toImageMsg());
        // }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "ddrseg_node");
    DDRNetNode node;
    ros::spin();
}

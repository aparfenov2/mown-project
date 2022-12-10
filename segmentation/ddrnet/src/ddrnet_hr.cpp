#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>
#include <chrono>
#include "common_ddr.hpp"
#include "common.hpp"
#include "logging.h"

Logger gLogger;
#define USE_FP32
#define DEVICE 0     // GPU id
#define BATCH_SIZE 1 //

// const char *INPUT_BLOB_NAME = "data";
// const char *OUTPUT_BLOB_NAME = "output";
const char* INPUT_BLOB_NAME = "input_0";
const char* OUTPUT_BLOB_NAME = "output_0";
// static const int INPUT_H = 512;
static const int INPUT_H = 1024;
static const int INPUT_W = 1024;
static const int NUM_CLASSES = 19;
static const int OUTPUT_SIZE = INPUT_H * INPUT_W;

// Creat the engine using only the API and not any parser.
ICudaEngine *createEngine(unsigned int maxBatchSize, IBuilder *builder, IBuilderConfig *config, DataType dt, std::string wtsPath, int width)
{
    INetworkDefinition *network = builder->createNetworkV2(0U);
    // Create input tensor of shape {3, INPUT_H, INPUT_W} with name INPUT_BLOB_NAME
    ITensor *data = network->addInput(INPUT_BLOB_NAME, dt, Dims3{INPUT_H, INPUT_W, 3});
    assert(data);

    // {
    //     // ITensor& input = *layer5_input->getOutput(0);
    //     ITensor& input = *data;
    //     int input_n = input.getDimensions().d[0];
    //     int input_c = input.getDimensions().d[1];
    //     int input_h = input.getDimensions().d[2];
    //     int input_w = input.getDimensions().d[3];
    //     std::cout << "DEBUG:" << input_n << " " << input_c << " " << input_h << " " << input_w << std::endl;
    // }

    // hwc to chw
    auto ps = network->addShuffle(*data);
    ps->setFirstTranspose(nvinfer1::Permutation{2, 0, 1});
    float mean[3] = {0.485, 0.456, 0.406};
    float std[3] = {0.229, 0.224, 0.225};
    ITensor *preinput = MeanStd(network, ps->getOutput(0), mean, std, true);

    std::map<std::string, Weights> weightMap = loadWeights(wtsPath);

    Weights emptywts{ DataType::kFLOAT, nullptr, 0 };

    IConvolutionLayer* conv1 = network->addConvolution(*preinput, 32, DimsHW{ 3, 3 }, weightMap["conv1.0.weight"], weightMap["conv1.0.bias"]);
    assert(conv1);
    conv1->setStride(DimsHW{ 2, 2 });
    conv1->setPadding(DimsHW{ 1, 1 });

    IScaleLayer* bn1 = addBatchNorm2d(network, weightMap, *conv1->getOutput(0), "conv1.1", 1e-5);

    IActivationLayer* relu1 = network->addActivation(*bn1->getOutput(0), ActivationType::kRELU);
    assert(relu1);

    IConvolutionLayer* conv2 = network->addConvolution(*relu1->getOutput(0), 32, DimsHW{ 3, 3 }, weightMap["conv1.3.weight"], weightMap["conv1.3.bias"]);
    assert(conv2);
    conv2->setStride(DimsHW{ 2, 2 });
    conv2->setPadding(DimsHW{ 1, 1 });

    IScaleLayer* bn2 = addBatchNorm2d(network, weightMap, *conv2->getOutput(0), "conv1.4", 1e-5);

    IActivationLayer* relu2 = network->addActivation(*bn2->getOutput(0), ActivationType::kRELU);
    assert(relu2);

    // layer1
    ILayer* layer1_0 = basicBlock(network, weightMap, *relu2->getOutput(0), 32, 32, 1, false, false, "layer1.0.");
    ILayer* layer1_1 = basicBlock(network, weightMap, *layer1_0->getOutput(0), 32, 32, 1, false, true, "layer1.1.");
    IActivationLayer* layer1_relu = network->addActivation(*layer1_1->getOutput(0), ActivationType::kRELU);
    assert(layer1_relu);

    // layer2
    ILayer* layer2_0 = basicBlock(network, weightMap, *layer1_relu->getOutput(0), 32, 64, 2, true, false, "layer2.0.");
    ILayer* layer2_1 = basicBlock(network, weightMap, *layer2_0->getOutput(0), 64, 64, 1, false, true, "layer2.1."); // 1/8
    IActivationLayer* layer2_relu = network->addActivation(*layer2_1->getOutput(0), ActivationType::kRELU);
    assert(layer2_relu);

    // layer3
    ILayer* layer3_0 = basicBlock(network, weightMap, *layer2_relu->getOutput(0), 64, 128, 2, true, false, "layer3.0.");
    ILayer* layer3_1 = basicBlock(network, weightMap, *layer3_0->getOutput(0), 128, 128, 1, false, true, "layer3.1."); // 1/16
    IActivationLayer* layer3_relu = network->addActivation(*layer3_1->getOutput(0), ActivationType::kRELU);
    assert(layer3_relu);   // layer[2]

    // x_ = self.layer3_(self.relu(layers[1]))
    // layer3_
    ILayer* layer3_10 = basicBlock(network, weightMap, *layer2_relu->getOutput(0), 64, 64, 1, false, false, "layer3_.0.");
    ILayer* layer3_11 = basicBlock(network, weightMap, *layer3_10->getOutput(0), 64, 64, 1, false, true, "layer3_.1."); // x_ = self.layer3_(self.relu(layers[1]))
    //

    // down3
    IActivationLayer* down3_input_relu = network->addActivation(*layer3_11->getOutput(0), ActivationType::kRELU);
    assert(down3_input_relu);

    ILayer* down3_out = down3(network, weightMap, *down3_input_relu->getOutput(0), 128, "down3.");
    //  x = x + self.down3(self.relu(x_))

    IElementWiseLayer* down3_add = network->addElementWise(*layer3_1->getOutput(0), *down3_out->getOutput(0), ElementWiseOperation::kSUM);

    //x_ = x_ + F.interpolate(self.compression3(self.relu(layers[2])), size=[height_output, width_output], mode='bilinear',align_corners=True)
    ILayer* compression3_input = compression3(network, weightMap, *layer3_relu->getOutput(0), 64, "compression3.");

    float *deval = reinterpret_cast<float*>(malloc(sizeof(float) * 64 * 2 * 2));
    for (int i = 0; i < 64 * 2 * 2; i++) {
        deval[i] = 1.0;
    }
    Weights deconvwts1{ DataType::kFLOAT, deval, 64 * 2 * 2 };
    IDeconvolutionLayer* compression3_up = network->addDeconvolutionNd(*compression3_input->getOutput(0), 64, DimsHW{ 2, 2 }, deconvwts1, emptywts);
    compression3_up->setStrideNd(DimsHW{ 2, 2 });
    compression3_up->setNbGroups(64);
    IElementWiseLayer* compression3_add = network->addElementWise(*layer3_11->getOutput(0), *compression3_up->getOutput(0), ElementWiseOperation::kSUM);
//  x_ = self.layer4_(self.relu(x_))
    // layer4
    IActivationLayer* layer4_input = network->addActivation(*down3_add->getOutput(0), ActivationType::kRELU);
    ILayer* layer4_0 = basicBlock(network, weightMap, *layer4_input->getOutput(0), 128, 256, 2, true, false, "layer4.0.");
    //  x = self.layer4(self.relu(x))
    ILayer* layer4_1 = basicBlock(network, weightMap, *layer4_0->getOutput(0), 256, 256, 1, false, true, "layer4.1."); // 1/32
    IActivationLayer* layer4_relu = network->addActivation(*layer4_1->getOutput(0), ActivationType::kRELU);
    assert(layer4_relu);

    // layer4_
    IActivationLayer* layer4_1_input = network->addActivation(*compression3_add->getOutput(0), ActivationType::kRELU);
    ILayer* layer4_10 = basicBlock(network, weightMap, *layer4_1_input->getOutput(0), 64, 64, 1, false, false, "layer4_.0.");
    //  x_ = self.layer4_(self.relu(x_))
    ILayer* layer4_11 = basicBlock(network, weightMap, *layer4_10->getOutput(0), 64, 64, 1, false, true, "layer4_.1."); // 1/8
    // down4
    IActivationLayer* down4_input_relu = network->addActivation(*layer4_11->getOutput(0), ActivationType::kRELU);
    assert(down4_input_relu);
    ILayer* down4_out = down4(network, weightMap, *down4_input_relu->getOutput(0), 128, "down4.");

    IElementWiseLayer* down4_add = network->addElementWise(*layer4_1->getOutput(0), *down4_out->getOutput(0), ElementWiseOperation::kSUM);
//         x_ = x_ + F.interpolate(self.compression4(self.relu(layers[3])),size=[height_output, width_output],mode='bilinear',align_corners=True)
    ILayer* compression4_input = compression4(network, weightMap, *layer4_relu->getOutput(0), 64, "compression4.");

    float *deval2 = reinterpret_cast<float*>(malloc(sizeof(float) * 64 * 4 * 4));
    for (int i = 0; i < 64 * 4 * 4; i++) {
        deval2[i] = 1.0;
    }
    Weights deconvwts2{ DataType::kFLOAT, deval2, 64 * 4 * 4 };
    IDeconvolutionLayer* compression4_up = network->addDeconvolutionNd(*compression4_input->getOutput(0), 64, DimsHW{ 4, 4 }, deconvwts2, emptywts);
    compression4_up->setStrideNd(DimsHW{ 4, 4 });
    compression4_up->setNbGroups(64);

    IElementWiseLayer* compression4_add = network->addElementWise(*layer4_11->getOutput(0), *compression4_up->getOutput(0), ElementWiseOperation::kSUM);
    IActivationLayer* compression4_add_relu = network->addActivation(*compression4_add->getOutput(0), ActivationType::kRELU);
    assert(compression4_add_relu);
    // layer5_
    //  x_ = self.layer5_(self.relu(x_))
    ILayer* layer5_ = Bottleneck(network, weightMap, *compression4_add_relu->getOutput(0), 64, 64, 1, true, true, "layer5_.0.");

    // layer5
    IActivationLayer* layer5_input = network->addActivation(*down4_add->getOutput(0), ActivationType::kRELU);
    assert(layer5_input);

    ILayer* layer5 = Bottleneck(network, weightMap, *layer5_input->getOutput(0), 256, 256, 2, true, true, "layer5.0.");

    // {
    //     ITensor& input = *layer5->getOutput(0);
    //     // ITensor& input = *data;
    //     int input_n = input.getDimensions().d[0];
    //     int input_c = input.getDimensions().d[1];
    //     int input_h = input.getDimensions().d[2];
    //     int input_w = input.getDimensions().d[3];
    //     std::cout << "DEBUG:" << input_n << " " << input_c << " " << input_h << " " << input_w << std::endl;
    // }
    // layer5->getOutput(0): 512 16 16
    ILayer* ssp = DAPPM(network, weightMap, *layer5->getOutput(0), 512, 128, 128, "spp.");

    float *deval3 = reinterpret_cast<float*>(malloc(sizeof(float) * 128 * 8 * 8));
    for (int i = 0; i < 128 * 8 * 8; i++) {
        deval3[i] = 1.0;
    }
    Weights deconvwts3{ DataType::kFLOAT, deval3, 128 * 8 * 8 };
    IDeconvolutionLayer* spp_up = network->addDeconvolutionNd(*ssp->getOutput(0), 128, DimsHW{ 8, 8 }, deconvwts3, emptywts);
    spp_up->setStrideNd(DimsHW{ 8, 8 });
    spp_up->setNbGroups(128);
    // x_ = self.final_layer(x + x_)

    IElementWiseLayer* final_in = network->addElementWise(*spp_up->getOutput(0), *layer5_->getOutput(0), ElementWiseOperation::kSUM);

    ILayer* seg_out= segmenthead(network, weightMap, *final_in->getOutput(0), 64, NUM_CLASSES, "final_layer.");

//    IActivationLayer* thresh = network->addActivation(*seg_out->getOutput(0), ActivationType::kSIGMOID);
//    assert(thresh);

    // y = F.interpolate(y, size=(H, W)) 
    // seg_out->getOutput(0)->setName(OUTPUT_BLOB_NAME);
    // network->markOutput(*seg_out->getOutput(0));


    // auto conv_1785 = network->addConvolutionNd(*relu_1784->getOutput(0), NUM_CLASSES, DimsHW{1, 1}, weightMap["cls_head.weight"], weightMap["cls_head.bias"]);
    auto conv_1785 = seg_out;
    debug_print(conv_1785->getOutput(0), "final_layer");
    nvinfer1::Dims dim;
    dim.nbDims = 3;
    dim.d[0] = NUM_CLASSES;
    dim.d[1] = INPUT_H;
    dim.d[2] = INPUT_W;
    auto feature_map = netAddUpsampleBi(network, conv_1785->getOutput(0), dim);
    debug_print(feature_map->getOutput(0), "upsample");

    // auto topk = network->addTopK(*feature_map->getOutput(0), TopKOperation::kMAX, 1, 0X01);
    // debug_print(topk->getOutput(0), "topk");

    std::cout << "set name out" << std::endl;
    // topk->getOutput(1)->setName(OUTPUT_BLOB_NAME);
    // network->markOutput(*topk->getOutput(1));

    feature_map->getOutput(1)->setName(OUTPUT_BLOB_NAME);
    network->markOutput(*feature_map->getOutput(1));

    builder->setMaxBatchSize(maxBatchSize);
    config->setMaxWorkspaceSize((1 << 30)); // 1G
#ifdef USE_FP16
    std::cout << "use fp16" << std::endl;
    config->setFlag(BuilderFlag::kFP16);
#endif
    ICudaEngine *engine = builder->buildEngineWithConfig(*network, *config);
    std::cout << "build success!" << std::endl;
    network->destroy();
    for (auto &mem : weightMap)
    {
        free((void *)(mem.second.values));
    }
    return engine;
}
void APIToModel(unsigned int maxBatchSize, IHostMemory **modelStream, std::string wtsPath, int width)
{
    IBuilder *builder = createInferBuilder(gLogger);
    IBuilderConfig *config = builder->createBuilderConfig();
    ICudaEngine *engine = createEngine(maxBatchSize, builder, config, DataType::kFLOAT, wtsPath, width);
    assert(engine != nullptr);
    (*modelStream) = engine->serialize();
    engine->destroy();
    builder->destroy();
}

bool parse_args(int argc, char **argv, std::string &wts, std::string &engine, int &width, std::string &img_dir)
{
    if (std::string(argv[1]) == "-s" && argc == 5)
    {
        wts = std::string(argv[2]);
        engine = std::string(argv[3]);
        width = std::stoi(argv[4]);
    }
    else if (std::string(argv[1]) == "-d" && argc == 4)
    {
        engine = std::string(argv[2]);
        img_dir = std::string(argv[3]);
    }
    else
    {
        return false;
    }
    return true;
}
void doInference(IExecutionContext &context, cudaStream_t &stream, void **buffers, int batchSize)
{
    context.enqueue(batchSize, buffers, stream, nullptr);
    cudaStreamSynchronize(stream);
    cudaDeviceSynchronize();
}

int ddr_main(int argc, char **argv)
{
    cudaSetDevice(DEVICE);
    std::string wtsPath = "";
    std::string engine_name = "";
    int width;
    std::string img_dir;
    // parse args
    if (!parse_args(argc, argv, wtsPath, engine_name, width, img_dir))
    {
        std::cerr << "arguments not right!" << std::endl;
        std::cerr << "./hrnet_ocr -s [.wts] [.engine] [18 or 32 or 48]  // serialize model to plan file" << std::endl;
        std::cerr << "./hrnet_ocr -d [.engine] ../samples  // deserialize plan file and run inference" << std::endl;
        return -1;
    }
    // create a model using the API directly and serialize it to a stream
    if (!wtsPath.empty())
    {
        IHostMemory *modelStream{nullptr};
        APIToModel(BATCH_SIZE, &modelStream, wtsPath, width);
        assert(modelStream != nullptr);
        std::ofstream p(engine_name, std::ios::binary);
        if (!p)
        {
            std::cerr << "could not open plan output file" << std::endl;
            return -1;
        }
        p.write(reinterpret_cast<const char *>(modelStream->data()), modelStream->size());
        modelStream->destroy();
        return 0;
    }

    // deserialize the .engine and run inference
    char *trtModelStream{nullptr};
    size_t size{0};
    std::ifstream file(engine_name, std::ios::binary);
    if (file.good())
    {
        file.seekg(0, file.end);
        size = file.tellg();
        file.seekg(0, file.beg);
        trtModelStream = new char[size];
        assert(trtModelStream);
        file.read(trtModelStream, size);
        file.close();
    }
    else
    {
        std::cerr << "could not open plan file" << std::endl;
    }

    std::vector<std::string> file_names;
    if (read_files_in_dir(img_dir.c_str(), file_names) < 0)
    {
        std::cout << "read_files_in_dir failed." << std::endl;
        return -1;
    }
    // prepare input data ---------------------------
    cudaSetDeviceFlags(cudaDeviceMapHost);
    float *data;
    int *prob; // using int. output is index
    CHECK(cudaHostAlloc((void **)&data, BATCH_SIZE * 3 * INPUT_H * INPUT_W * sizeof(float), cudaHostAllocMapped));
    CHECK(cudaHostAlloc((void **)&prob, BATCH_SIZE * OUTPUT_SIZE * sizeof(int), cudaHostAllocMapped));

    IRuntime *runtime = createInferRuntime(gLogger);
    assert(runtime != nullptr);
    ICudaEngine *engine = runtime->deserializeCudaEngine(trtModelStream, size);
    assert(engine != nullptr);
    IExecutionContext *context = engine->createExecutionContext();
    assert(context != nullptr);
    delete[] trtModelStream;
    void *buffers[2];
    // In order to bind the buffers, we need to know the names of the input and output tensors.
    // Note that indices are guaranteed to be less than IEngine::getNbBindings()
    const int inputIndex = engine->getBindingIndex(INPUT_BLOB_NAME);
    const int outputIndex = engine->getBindingIndex(OUTPUT_BLOB_NAME);
    assert(inputIndex == 0);
    assert(outputIndex == 1);
    cudaStream_t stream;
    CHECK(cudaStreamCreate(&stream));

    for (int f = 0; f < (int)file_names.size(); f++)
    {
        std::cout << file_names[f] << std::endl;
        cv::Mat pr_img;
        cv::Mat img_BGR = cv::imread(img_dir + "/" + file_names[f], 1); // BGR
        cv::Mat img;
        cv::cvtColor(img_BGR, img, cv::COLOR_BGR2RGB);
        if (img.empty())
            continue;
        cv::resize(img, pr_img, cv::Size(INPUT_W, INPUT_H));
        img = pr_img.clone(); // for img show
        pr_img.convertTo(pr_img, CV_32FC3);
        if (!pr_img.isContinuous())
        {
            pr_img = pr_img.clone();
        }
        std::memcpy(data, pr_img.data, BATCH_SIZE * 3 * INPUT_W * INPUT_H * sizeof(float));

        cudaHostGetDevicePointer((void **)&buffers[inputIndex], (void *)data, 0);  // buffers[inputIndex]-->data
        cudaHostGetDevicePointer((void **)&buffers[outputIndex], (void *)prob, 0); // buffers[outputIndex] --> prob

        // Run inference
        auto start = std::chrono::high_resolution_clock::now();
        doInference(*context, stream, buffers, BATCH_SIZE);
        auto end = std::chrono::high_resolution_clock::now();
        std::cout << "infer time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;

        cv::Mat outimg(INPUT_H, INPUT_W, CV_8UC1);
        for (int row = 0; row < INPUT_H; ++row)
        {
            uchar *uc_pixel = outimg.data + row * outimg.step;
            for (int col = 0; col < INPUT_W; ++col)
            {
                uc_pixel[col] = (uchar)prob[row * INPUT_W + col];
            }
        }
        cv::Mat im_color;
        cv::cvtColor(outimg, im_color, cv::COLOR_GRAY2RGB);
        cv::Mat lut = createLTU(NUM_CLASSES);
        cv::LUT(im_color, lut, im_color);
        // false color
        cv::cvtColor(im_color, im_color, cv::COLOR_RGB2GRAY);
        cv::applyColorMap(im_color, im_color, cv::COLORMAP_HOT);
        // cv::imshow("False Color Map", im_color);
        cv::imwrite(std::to_string(f) + "_false_color_map.png", im_color);
        //fusion
        cv::Mat fusionImg;
        cv::addWeighted(img, 1, im_color, 0.8, 1, fusionImg);
        // cv::imshow("Fusion Img", fusionImg);
        // cv::waitKey(0);
        cv::imwrite(std::to_string(f) + "_fusion_img.png", fusionImg);
    }

    // Release stream and buffers
    cudaStreamDestroy(stream);
    CHECK(cudaFreeHost(buffers[inputIndex]));
    CHECK(cudaFreeHost(buffers[outputIndex]));
    // Destroy the engine
    context->destroy();
    engine->destroy();
    runtime->destroy();
    return 0;
}

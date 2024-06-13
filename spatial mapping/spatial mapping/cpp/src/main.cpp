#include <sl/Camera.hpp>
#include "GLViewer.hpp"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace sl;

void parse_args(int argc, char** argv, InitParameters& param, sl::Mat& roi);
void print(std::string msg_prefix, sl::ERROR_CODE err_code = sl::ERROR_CODE::SUCCESS, std::string msg_suffix = "");

int main(int argc, char** argv) {
    Camera zed;
    InitParameters init_parameters;
    init_parameters.depth_mode = DEPTH_MODE::NEURAL;
    init_parameters.coordinate_units = UNIT::METER;
    init_parameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
    init_parameters.depth_maximum_distance = 8.0;
    sl::Mat roi;
    parse_args(argc, argv, init_parameters, roi);

    auto returned_state = zed.open(init_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        print("Open Camera", returned_state, "\nExit program.");
        zed.close();
        return EXIT_FAILURE;
    }

    if (roi.isInit()) {
        auto state = zed.setRegionOfInterest(roi, { sl::MODULE::POSITIONAL_TRACKING, sl::MODULE::SPATIAL_MAPPING });
        std::cout << "Applied ROI " << state << "\n";
    }

    std::cout << "Shortcuts\n";
    std::cout << "\t- 'l' to enable/disable current live point cloud display\n";
    std::cout << "\t- 'w' to switch mesh display from faces to triangles\n";
    std::cout << "\t- 'd' to switch background color from dark to light\n";

    auto camera_infos = zed.getCameraInformation();
    Pose pose;
    POSITIONAL_TRACKING_STATE tracking_state = POSITIONAL_TRACKING_STATE::OFF;

    returned_state = zed.enablePositionalTracking();
    if (returned_state != ERROR_CODE::SUCCESS) {
        print("Enabling positional tracking failed: ", returned_state);
        zed.close();
        return EXIT_FAILURE;
    }

    SpatialMappingParameters spatial_mapping_parameters;
    spatial_mapping_parameters.map_type = SpatialMappingParameters::SPATIAL_MAP_TYPE::MESH;
    Mesh map;
    spatial_mapping_parameters.set(SpatialMappingParameters::MAPPING_RANGE::SHORT);
    spatial_mapping_parameters.set(SpatialMappingParameters::MAPPING_RESOLUTION::HIGH);
    spatial_mapping_parameters.use_chunk_only = true;
    spatial_mapping_parameters.stability_counter = 4;

    chrono::high_resolution_clock::time_point ts_last;
    RuntimeParameters runtime_parameters;
    runtime_parameters.confidence_threshold = 50;

    auto resolution = camera_infos.camera_configuration.resolution;
    float image_aspect_ratio = resolution.width / (1.f * resolution.height);
    int requested_low_res_w = min(720, (int)resolution.width);
    sl::Resolution display_resolution(requested_low_res_w, requested_low_res_w / image_aspect_ratio);

    Mat image(display_resolution, MAT_TYPE::U8_C4, sl::MEM::GPU);
    Mat point_cloud(display_resolution, MAT_TYPE::F32_C4, sl::MEM::GPU);

    GLViewer viewer;
    viewer.init(argc, argv, image, point_cloud, zed.getCUDAStream());

    bool request_new_mesh = true;
    bool wait_for_mapping = true;
    sl::Timestamp timestamp_start;
    timestamp_start.data_ns = 0;

    while (viewer.isAvailable()) {
        if (zed.grab(runtime_parameters) == ERROR_CODE::SUCCESS) {
            zed.retrieveImage(image, VIEW::LEFT, MEM::GPU, display_resolution);
            zed.retrieveMeasure(point_cloud, MEASURE::XYZBGRA, MEM::GPU, display_resolution);
            tracking_state = zed.getPosition(pose);
            viewer.updateCameraPose(pose.pose_data, tracking_state);

            if (tracking_state == POSITIONAL_TRACKING_STATE::OK) {
                if (wait_for_mapping) {
                    zed.enableSpatialMapping(spatial_mapping_parameters);
                    wait_for_mapping = false;
                }
                else {
                    if (request_new_mesh) {
                        auto duration = chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - ts_last).count();
                        if (duration > 100) {
                            zed.requestSpatialMapAsync();
                            request_new_mesh = false;
                        }
                    }

                    if (zed.getSpatialMapRequestStatusAsync() == ERROR_CODE::SUCCESS && !request_new_mesh) {
                        zed.retrieveSpatialMapAsync(map);
                        viewer.updateMap(map);
                        request_new_mesh = true;
                        ts_last = chrono::high_resolution_clock::now();
                    }
                }
            }
        }
    }

    map.save("MyMap", sl::MESH_FILE_FORMAT::PLY);
    image.free();
    point_cloud.free();
    zed.close();
    return 0;
}

void parse_args(int argc, char** argv, InitParameters& param, sl::Mat& roi) {
    if (argc == 1) return;
    for (int id = 1; id < argc; id++) {
        std::string arg(argv[id]);
        if (arg.find(".svo") != string::npos) {
            param.input.setFromSVOFile(arg.c_str());
            param.svo_real_time_mode = true;
            cout << "[Sample] Using SVO File input: " << arg << endl;
        }

        unsigned int a, b, c, d, port;
        if (sscanf(arg.c_str(), "%u.%u.%u.%u:%d", &a, &b, &c, &d, &port) == 5) {
            string ip_adress = to_string(a) + "." + to_string(b) + "." + to_string(c) + "." + to_string(d);
            param.input.setFromStream(String(ip_adress.c_str()), port);
            cout << "[Sample] Using Stream input, IP: " << ip_adress << ", port: " << port << endl;
        }
        else if (sscanf(arg.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d) == 4) {
            param.input.setFromStream(String(argv[1]));
            cout << "[Sample] Using Stream input, IP: " << argv[1] << endl;
        }
        else if (arg.find("HD2K") != string::npos) {
            param.camera_resolution = RESOLUTION::HD2K;
            cout << "[Sample] Using Camera in resolution HD2K" << endl;
        }
        else if (arg.find("HD1200") != string::npos) {
            param.camera_resolution = RESOLUTION::HD1200;
            cout << "[Sample] Using Camera in resolution HD1200" << endl;
        }
        else if (arg.find("HD1080") != string::npos) {
            param.camera_resolution = RESOLUTION::HD1080;
            cout << "[Sample] Using Camera in resolution HD1080" << endl;
        }
        else if (arg.find("HD720") != string::npos) {
            param.camera_resolution = RESOLUTION::HD720;
            cout << "[Sample] Using Camera in resolution HD720" << endl;
        }
        else if (arg.find("SVGA") != string::npos) {
            param.camera_resolution = RESOLUTION::SVGA;
            cout << "[Sample] Using Camera in resolution SVGA" << endl;
        }
        else if (arg.find("VGA") != string::npos) {
            param.camera_resolution = RESOLUTION::VGA;
            cout << "[Sample] Using Camera in resolution VGA" << endl;
        }
        else if ((arg.find(".png") != string::npos) || ((arg.find(".jpg") != string::npos))) {
            roi.read(arg.c_str());
            cout << "[Sample] Using Region of interest from " << arg << endl;
        }
    }
}

void print(std::string msg_prefix, sl::ERROR_CODE err_code, std::string msg_suffix) {
    cout << "[Sample]";
    if (err_code != sl::ERROR_CODE::SUCCESS)
        cout << "[Error] ";
    else
        cout << " ";
    cout << msg_prefix << " ";
    if (err_code != sl::ERROR_CODE::SUCCESS) {
        cout << " | " << toString(err_code) << " : ";
        cout << toVerbose(err_code);
    }
    if (!msg_suffix.empty())
        cout << " " << msg_suffix;
    cout << endl;
}

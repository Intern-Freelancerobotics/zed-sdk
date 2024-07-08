#include <sl/Camera.hpp>
#include "HalconCpp.h"
#include <thread>
#include <chrono>
#include <fstream>
#include <iostream>
#include <sysinfoapi.h>

#ifndef __APPLE__
#include <hdevengine/HDevEngineCpp.h>
#else
#include <HDevEngineCpp/HDevEngineCpp.h>
#endif

using namespace std;
using namespace sl;
using namespace HalconCpp;
using namespace HDevEngineCpp;

size_t getAvailableMemory() {
    MEMORYSTATUSEX memStatus;
    memStatus.dwLength = sizeof(MEMORYSTATUSEX);
    GlobalMemoryStatusEx(&memStatus);
    return memStatus.ullAvailPhys;
}

void waitForMemory(size_t requiredMemory) {
    while (getAvailableMemory() < requiredMemory) {
        cout << "Not enough memory, waiting...\n";
        this_thread::sleep_for(chrono::seconds(5));
    }
}

bool initCamera(Camera& zed) {
    InitParameters init_parameters;
    init_parameters.camera_resolution = RESOLUTION::AUTO;
    init_parameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
    init_parameters.coordinate_units = UNIT::METER;

    auto returned_state = zed.open(init_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        cout << "Error " << returned_state << ", exit program.\n";
        return false;
    }
    cout << "Camera initialized successfully.\n";
    return true;
}

bool enablePositionalTracking(Camera& zed) {
    sl::PositionalTrackingParameters tracking_parameters;
    auto returned_state = zed.enablePositionalTracking(tracking_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        cout << "Error " << returned_state << ", exit program.\n";
        return false;
    }
    cout << "Positional tracking enabled successfully.\n";
    return true;
}

bool enableObjectDetection(Camera& zed) {
    ObjectDetectionParameters detection_parameters;
    detection_parameters.detection_model = OBJECT_DETECTION_MODEL::MULTI_CLASS_BOX_ACCURATE;
    detection_parameters.enable_tracking = true;

    auto returned_state = zed.enableObjectDetection(detection_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        cout << "Error enabling object detection: " << returned_state << "\n";
        return false;
    }
    cout << "Object detection enabled successfully.\n";
    return true;
}

bool enableSpatialMapping(Camera& zed) {
    SpatialMappingParameters mapping_parameters;
    mapping_parameters.range_meter = 3.0; // Adjust range as necessary
    mapping_parameters.resolution_meter = 0.05; // Adjust resolution as necessary
    auto spatial_mapping_state = zed.enableSpatialMapping(mapping_parameters);

    if (spatial_mapping_state != ERROR_CODE::SUCCESS) {
        cout << "Error enabling spatial mapping: " << spatial_mapping_state << "\n";
        return false;
    }
    cout << "Spatial mapping enabled successfully.\n";
    return true;
}

bool extractAndSaveMesh(Camera& zed, const sl::ObjectData& obj, const string& file_name) {
    sl::Mesh object_mesh;

    // Define the region of interest based on the object's bounding box
    sl::Transform object_transform;
    object_transform.setTranslation(obj.position);

    int frame_count = 0;
    while (frame_count < 1000) { // Ensure some frames are captured
        if (zed.grab() == ERROR_CODE::SUCCESS) {
            frame_count++;
            cout << "\rLoaded " << frame_count << " frame(s) out of 1000" << flush;
            auto spatial_mapping_state = zed.extractWholeSpatialMap(object_mesh);

            if (spatial_mapping_state != ERROR_CODE::SUCCESS) {
                cout << "Error extracting spatial map: " << spatial_mapping_state << "\n";
                return false;
            }
        }
        else {
            cout << "\rError capturing frame " << frame_count + 1 << "\n";
        }
    }

    // Check if the mesh has vertices
    if (object_mesh.vertices.empty()) {
        cout << "\nNo vertices found in the mesh for object " << obj.id << ". Skipping...\n";
        return false;
    }

    // Filter the mesh
    cout << "\nFiltering mesh for object " << obj.id << "...\n";
    object_mesh.filter(sl::MeshFilterParameters::MESH_FILTER::LOW);

    // Apply texture to the mesh
    object_mesh.applyTexture();

    // Save the mesh to a file
    cout << "\nSaving mesh for object " << obj.id << "...\n";
    if (!object_mesh.save(file_name.c_str())) {
        cout << "Error saving mesh\n";
        return false;
    }
    else {
        cout << "Mesh saved successfully to " << file_name << "\n";
        return true;
    }
}

int main(int argc, char** argv) {
    Camera zed;

    if (!initCamera(zed)) return EXIT_FAILURE;
    if (!enablePositionalTracking(zed)) return EXIT_FAILURE;
    if (!enableObjectDetection(zed)) return EXIT_FAILURE;
    if (!enableSpatialMapping(zed)) return EXIT_FAILURE;

    ObjectDetectionRuntimeParameters detection_runtime_parameters;
    detection_runtime_parameters.detection_confidence_threshold = 40;

    Objects objects;

    while (true) {
        if (zed.grab() == ERROR_CODE::SUCCESS) {
            if (zed.retrieveObjects(objects, detection_runtime_parameters) == ERROR_CODE::SUCCESS) {
                cout << "Detected " << objects.object_list.size() << " objects.\n";
                for (const auto& obj : objects.object_list) {
                    cout << "Object ID: " << obj.id << "\n";
                    cout << "Label: " << obj.label << "\n";
                    cout << "Confidence: " << obj.confidence << "\n";
                    cout << "Position: " << obj.position << "\n";
                    cout << "Dimensions: " << obj.dimensions << "\n";
                    cout << "Bounding Box 3D: ";
                    for (const auto& point : obj.bounding_box) {
                        cout << point << " ";
                    }
                    cout << "\n";

                    string file_name = "mesh_obj_" + to_string(obj.id) + ".obj";
                    if (!extractAndSaveMesh(zed, obj, file_name)) {
                        cout << "Failed to extract and save mesh for object " << obj.id << ".\n";
                    }
                }
            }
            else {
                cout << "Error retrieving objects.\n";
            }
        }
        else {
            cout << "Error capturing frame.\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Adjust sleep time as necessary
    }

    zed.disableSpatialMapping();
    zed.disablePositionalTracking();
    zed.close();

    cout << "Program completed successfully.\n";
    return EXIT_SUCCESS;
}
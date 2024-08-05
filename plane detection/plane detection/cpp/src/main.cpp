///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2024, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/*************************************************************************
** This sample shows how to capture a real-time 3D reconstruction      **
** of the scene using the Spatial Mapping API. The resulting mesh      **
** is displayed as a wireframe on top of the left image using OpenGL.  **
** Spatial Mapping can be started and stopped with the Space Bar key   **
*************************************************************************/

// ZED includes
#include <sl/Camera.hpp>
#include <cmath>
#include <vector>
#include <algorithm>

// Sample includes
#include "GLViewer.hpp"

// Using std and sl namespaces
using namespace std;
using namespace sl;

void parseArgs(int argc, char** argv, sl::InitParameters& param);

void addPlaneToMesh(Mesh& global_mesh, const Mesh& new_mesh) {
    // Adds new_mesh data to global_mesh by creating a new chunk.
    Chunk new_chunk;
    new_chunk.vertices = new_mesh.vertices;
    new_chunk.triangles = new_mesh.triangles;
    new_chunk.normals = new_mesh.normals;
    new_chunk.colors = new_mesh.colors;

    global_mesh.chunks.push_back(new_chunk);
    global_mesh.updateMeshFromChunkList();
}

float dot(const sl::float3& a, const sl::float3& b) {
    // Returns the dot product of vectors a and b.
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

float norm(const sl::float3& a) {
    // Returns the Euclidean norm of vector a.
    return sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

sl::float3 cross(const sl::float3& a, const sl::float3& b) {
    // Returns the cross product of vectors a and b.
    return sl::float3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

bool isOverlapping(Plane& plane, vector<Plane>& detected_planes, float threshold) {
    // Checks if the plane overlaps with any detected planes within a given threshold.
    for (auto& detected_plane : detected_planes) {
        if (dot(plane.getNormal(), detected_plane.getNormal()) > threshold &&
            norm(plane.getCenter() - detected_plane.getCenter()) < threshold) {
            return true;
        }
    }
    return false;
}

bool isPointInPolygon(const sl::float3& point, const vector<sl::float3>& polygon) {
    // Checks if a point is inside a polygon (convex and coplanar).
    int n = polygon.size();
    for (int i = 0; i < n; i++) {
        sl::float3 v1 = polygon[i];
        sl::float3 v2 = polygon[(i + 1) % n];
        sl::float3 edge = v2 - v1;
        sl::float3 to_point = point - v1;
        if (dot(cross(edge, to_point), polygon[0]) < 0) {
            return false;
        }
    }
    return true;
}

void markPlaneArea(vector<sl::float3>& restricted_areas, Plane& plane) {
    // Adds the bounds of a plane to the restricted areas.
    auto bounds = plane.getBounds();
    restricted_areas.insert(restricted_areas.end(), bounds.begin(), bounds.end());
}

bool isWithinRestrictedArea(const sl::float3& point, const vector<sl::float3>& restricted_areas) {
    // Checks if a point is within any of the restricted areas.
    for (size_t i = 0; i < restricted_areas.size(); i += 4) {
        vector<sl::float3> polygon(restricted_areas.begin() + i, restricted_areas.begin() + i + 4);
        if (isPointInPolygon(point, polygon)) {
            return true;
        }
    }
    return false;
}

void removeTrianglesInRestrictedAreas(Mesh& mesh, const vector<sl::float3>& restricted_areas) {
    // Removes triangles in restricted areas from the mesh.
    auto& vertices = mesh.vertices;
    auto& triangles = mesh.triangles;
    vector<sl::uint3> filtered_triangles;

    for (const auto& tri : triangles) {
        bool in_restricted_area = false;
        if (isWithinRestrictedArea(vertices[tri.x], restricted_areas) ||
            isWithinRestrictedArea(vertices[tri.y], restricted_areas) ||
            isWithinRestrictedArea(vertices[tri.z], restricted_areas)) {
            in_restricted_area = true;
        }
        if (!in_restricted_area) {
            filtered_triangles.push_back(tri);
        }
    }

    mesh.triangles = filtered_triangles;
}

bool isPlaneVerticalOrHorizontal(const sl::float3& normal) {
    // Checks if a plane is vertical or horizontal based on its normal.
    return fabs(normal.z) < 0.1 || fabs(normal.y) < 0.1;
}

#include <unordered_map>
#include <set>

void simplifyMesh(Mesh& mesh, float merge_threshold) {
    // Create a map to store merged vertices
    std::unordered_map<int, int> vertex_map;
    std::vector<sl::float3> new_vertices;
    std::set<int> merged_indices;

    for (size_t i = 0; i < mesh.vertices.size(); ++i) {
        if (merged_indices.find(i) != merged_indices.end()) {
            continue; // Skip already merged vertices
        }

        sl::float3 v = mesh.vertices[i];
        int new_index = static_cast<int>(new_vertices.size());
        new_vertices.push_back(v);
        vertex_map[i] = new_index;

        for (size_t j = i + 1; j < mesh.vertices.size(); ++j) {
            if (merged_indices.find(j) != merged_indices.end()) {
                continue;
            }

            sl::float3 v2 = mesh.vertices[j];
            if (norm(v - v2) < merge_threshold) {
                vertex_map[j] = new_index;
                merged_indices.insert(j);
            }
        }
    }

    std::vector<sl::uint3> new_triangles;
    for (const auto& tri : mesh.triangles) {
        int v0 = vertex_map[tri.x];
        int v1 = vertex_map[tri.y];
        int v2 = vertex_map[tri.z];

        if (v0 != v1 && v1 != v2 && v2 != v0) {
            new_triangles.push_back(sl::uint3{ static_cast<unsigned int>(v0), static_cast<unsigned int>(v1), static_cast<unsigned int>(v2) });
        }
    }

    mesh.vertices = new_vertices;
    mesh.triangles = new_triangles;
    mesh.updateMeshFromChunkList();
}


int main(int argc, char** argv) {
    // Initialize the ZED camera and other necessary components.
    Camera zed;
    InitParameters init_parameters;
    init_parameters.depth_mode = sl::DEPTH_MODE::NEURAL;
    init_parameters.coordinate_units = UNIT::METER;
    init_parameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
    parseArgs(argc, argv, init_parameters);

    // Open the ZED camera.
    ERROR_CODE zed_open_state = zed.open(init_parameters);
    if (zed_open_state != ERROR_CODE::SUCCESS) {
        print("Camera Open", zed_open_state, "Exit program.");
        return EXIT_FAILURE;
    }

    // Get camera information and check if IMU is available.
    auto camera_infos = zed.getCameraInformation();
    auto has_imu = camera_infos.sensors_configuration.isSensorAvailable(SENSOR_TYPE::GYROSCOPE);

    // Initialize the OpenGL viewer.
    GLViewer viewer;
    bool error_viewer = viewer.init(argc, argv, camera_infos.camera_configuration.calibration_parameters.left_cam, has_imu);
    if (error_viewer) {
        viewer.exit();
        zed.close();
        return EXIT_FAILURE;
    }

    // Initialize matrices and planes for spatial mapping.
    Mat image;
    Pose pose;
    Plane plane;
    Mesh mesh;
    Mesh global_mesh;

    vector<Plane> detected_planes;
    vector<sl::float3> restricted_areas;
    Plane floor_plane;
    bool floor_detected = false;
    bool roof_detected = false;
    Plane roof_plane;

    ERROR_CODE find_plane_status = ERROR_CODE::SUCCESS;
    POSITIONAL_TRACKING_STATE tracking_state = POSITIONAL_TRACKING_STATE::OFF;

    chrono::high_resolution_clock::time_point ts_last;

    UserAction user_action;
    user_action.clear();

    // Enable positional tracking.
    zed.enablePositionalTracking();

    // Set runtime parameters.
    RuntimeParameters runtime_parameters;
    runtime_parameters.measure3D_reference_frame = REFERENCE_FRAME::WORLD;

    // Set plane detection parameters.
    PlaneDetectionParameters plane_parameters;

    // Adjust spatial mapping resolution to generate larger triangles.
    SpatialMappingParameters mapping_parameters;
    mapping_parameters.resolution_meter = 0.2; // Max resolution
    zed.enableSpatialMapping(mapping_parameters);

    while (viewer.isAvailable()) {
        // Grab new frame data from the ZED camera.
        if (zed.grab(runtime_parameters) == ERROR_CODE::SUCCESS) {
            zed.retrieveImage(image, VIEW::LEFT, MEM::GPU);
            tracking_state = zed.getPosition(pose);

            if (tracking_state == POSITIONAL_TRACKING_STATE::OK) {
                auto duration = chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - ts_last).count();
                if (user_action.hit) {
                    auto image_click = sl::uint2(user_action.hit_coord.x * camera_infos.camera_configuration.resolution.width, user_action.hit_coord.y * camera_infos.camera_configuration.resolution.height);
                    find_plane_status = zed.findPlaneAtHit(image_click, plane, plane_parameters);
                }

                if ((duration > 500) && user_action.press_space) {
                    Transform resetTrackingFloorFrame;
                    find_plane_status = zed.findFloorPlane(plane, resetTrackingFloorFrame);
                    if (find_plane_status != ERROR_CODE::SUCCESS) {
                        std::cout << "No plane found" << std::endl;
                    }
                    else if (!floor_detected) {
                        floor_plane = plane;
                        floor_detected = true;
                        markPlaneArea(restricted_areas, floor_plane);
                        std::cout << "Floor detected and excluded from meshing process." << std::endl;
                    }
                    ts_last = chrono::high_resolution_clock::now();
                }

                if (find_plane_status == ERROR_CODE::SUCCESS) {
                    // Check if the detected plane overlaps with any existing planes.
                    if (isPlaneVerticalOrHorizontal(plane.getNormal()) && !isOverlapping(plane, detected_planes, 0.01f)) {
                        detected_planes.push_back(plane);
                        markPlaneArea(restricted_areas, plane);
                        mesh = plane.extractMesh();
                        removeTrianglesInRestrictedAreas(mesh, restricted_areas);
                        // Simplify the mesh to generate larger triangles.
                        simplifyMesh(mesh, 0.2); // Adjust threshold as needed
                        addPlaneToMesh(global_mesh, mesh);
                        viewer.updateMesh(global_mesh, plane.type);
                        global_mesh.save("AccumulatedMesh", sl::MESH_FILE_FORMAT::PLY);
                    }

                    if (user_action.hit && !roof_detected) {
                        auto image_click = sl::uint2(user_action.hit_coord.x * camera_infos.camera_configuration.resolution.width, user_action.hit_coord.y * camera_infos.camera_configuration.resolution.height);
                        find_plane_status = zed.findPlaneAtHit(image_click, plane, plane_parameters);
                        if (find_plane_status == ERROR_CODE::SUCCESS) {
                            roof_plane = plane;
                            roof_detected = true;
                            markPlaneArea(restricted_areas, roof_plane);
                            std::cout << "Roof detected and excluded from meshing process." << std::endl;
                        }
                    }
                }
            }

            user_action = viewer.updateImageAndState(image, pose.pose_data, tracking_state);
        }
    }

    // Clean up resources.
    image.free();
    mesh.clear();
    global_mesh.clear();

    zed.disablePositionalTracking();
    zed.close();
    return EXIT_SUCCESS;
}

void parseArgs(int argc, char** argv, sl::InitParameters& param) {
    // Parse command line arguments to set the input source for the ZED camera.
    if (argc > 1 && string(argv[1]).find(".svo") != string::npos) {
        param.input.setFromSVOFile(argv[1]);
        cout << "[Sample] Using SVO File input: " << argv[1] << endl;
    }
    else if (argc > 1 && string(argv[1]).find(".svo") == string::npos) {
        string arg = string(argv[1]);
        unsigned int a, b, c, d, port;
        if (sscanf(arg.c_str(), "%u.%u.%u.%u:%d", &a, &b, &c, &d, &port) == 5) {
            string ip_adress = to_string(a) + "." + to_string(b) + "." + to_string(c) + "." + to_string(d);
            param.input.setFromStream(sl::String(ip_adress.c_str()), port);
            cout << "[Sample] Using Stream input, IP : " << ip_adress << ", port : " << port << endl;
        }
        else if (sscanf(arg.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d) == 4) {
            param.input.setFromStream(sl::String(argv[1]));
            cout << "[Sample] Using Stream input, IP : " << argv[1] << endl;
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
    }
    else {
        // Default
    }
}